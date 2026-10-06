/**
 * @file tasks.cpp
 *
 * @brief Implementação das tarefas, mailboxes, filas e interrupções.
 */

#include "app/tasks.h"

#include <Arduino.h>

#include "app/supervisor.h"
#include "config/board_pins.h"
#include "config/features.h"
#include "config/robot_config.h"
#include "control/line_follower.h"
#include "control/mark_detector.h"
#include "hal/imu.h"
#include "hal/ir_array.h"
#include "hal/motor_driver.h"
#include "hal/status_led.h"
#include "test/teleplot.h"

namespace {

/*****************************************
 * Instâncias
 *
 * Estáticas de arquivo: nada no caminho de controle usa alocação dinâmica.
 *****************************************/

IrArray      g_ir;
Imu          g_imu;
MotorDriver  g_motors;
LineFollower g_follower;
Supervisor   g_supervisor;
StatusLed    g_led;
MarkDetector g_marks;

/*****************************************
 * Primitivas
 *****************************************/

QueueHandle_t g_mb_line = nullptr;
QueueHandle_t g_mb_imu = nullptr;
QueueHandle_t g_mb_control = nullptr;
QueueHandle_t g_mb_mode = nullptr;

QueueHandle_t g_q_commands = nullptr;

EventGroupHandle_t g_eg_mode = nullptr;

TaskHandle_t g_task_imu = nullptr;
TaskHandle_t g_task_line = nullptr;

hw_timer_t* g_ir_timer = nullptr;

/**
 * @brief Progresso da inicialização da IMU.
 *
 * Escrito só pela TaskImu e lido só pela TaskSupervisor. Um byte, portanto
 * atômico no Xtensa: não precisa de mutex.
 */
volatile ImuInitStatus g_imu_status = ImuInitStatus::PENDING;

#if !FEATURE_AUTOSTART
/**
 * @brief Instante do último acionamento aceito do botão, em ticks.
 *
 * Em escopo de arquivo, e não estático dentro da ISR: variável local estática em
 * C++ ganha guarda de inicialização, e a guarda chama função de runtime que não
 * está em IRAM — proibido em contexto de interrupção.
 */
volatile TickType_t g_last_button_tick = 0;
#endif

/**
 * @brief Mede a taxa de repetição de um laço, em Hz, por janela de 1 s.
 *
 * Instrumentação permanente, e não temporária de depuração: a taxa efetiva de
 * cada tarefa é a única forma de ver privação de CPU acontecendo. Uma tarefa
 * privada não gera erro nem log — ela simplesmente não roda.
 */
class RateMeter {
 public:
   void tick(uint32_t now_ms)
   {
      count_++;

      const uint32_t elapsed = now_ms - window_start_ms_;

      if (elapsed >= 1000) {
         hz_ = (count_ * 1000.0f) / elapsed;
         count_ = 0;
         window_start_ms_ = now_ms;
      }
   }

   float hz() const { return hz_; }

 private:
   uint32_t count_ = 0;
   uint32_t window_start_ms_ = 0;
   float    hz_ = 0.0f;
};

/*****************************************
 * Interrupções — seção 4.3
 *
 * Todas mínimas: IRAM_ATTR, sem alocação, sem Serial, sem I2C.
 *****************************************/

/**
 * @brief Timer de hardware 0 — cadência de amostragem do array IR.
 *
 * Só acorda a TaskLineSensor; o ADC é lido na tarefa. Isso desacopla a taxa de
 * amostragem do escalonador, que é o que o delay() dos testes não conseguia.
 */
void IRAM_ATTR irTimerIsr()
{
   BaseType_t woken = pdFALSE;
   vTaskNotifyGiveFromISR(g_task_line, &woken);
   portYIELD_FROM_ISR(woken);
}

#if !FEATURE_AUTOSTART
/**
 * @brief Botão de start/stop, borda de descida com debounce por timestamp.
 *
 * O debounce é feito na própria ISR, comparando o relógio: um botão mecânico
 * gera dezenas de bordas por acionamento, e cada uma enfileiraria um comando,
 * fazendo o robô alternar de estado várias vezes num único toque.
 */
void IRAM_ATTR buttonIsr()
{
   /* xTaskGetTickCountFromISR, e não millis(): millis() resolve para uma função
      que pode não estar em IRAM, e chamá-la de dentro de uma ISR trava a placa
      se a flash estiver ocupada — o que passa a acontecer de verdade quando a
      etapa 6 gravar parâmetros em NVS. */
   const TickType_t now = xTaskGetTickCountFromISR();

   if (now - g_last_button_tick < pdMS_TO_TICKS(cfg::BTN_DEBOUNCE_MS)) {
      return;
   }

   g_last_button_tick = now;

   const Command command = Command::TOGGLE_RUN;
   BaseType_t woken = pdFALSE;
   xQueueSendFromISR(g_q_commands, &command, &woken);
   portYIELD_FROM_ISR(woken);
}
#endif  // !FEATURE_AUTOSTART

/*****************************************
 * Tarefas
 *****************************************/

/**
 * @brief Núcleo 1, prioridade 5 — amostragem da IMU.
 *
 * Detém o barramento I2C de ponta a ponta: inicialização, estimativa de bias e
 * leitura periódica acontecem todas aqui. É o que torna o mutex de I2C
 * desnecessário em vez de apenas bem usado — não existe segundo acesso.
 */
void taskImu(void*)
{
   if (!g_imu.begin()) {
      g_imu_status = ImuInitStatus::FAILED;

      /* A tarefa não morre: ela é o dono do pino de interrupção e do status, e
         encerrá-la deixaria o supervisor sem fonte de verdade. */
      for (;;) {
         vTaskDelay(pdMS_TO_TICKS(500));
      }
   }

   /* A partir daqui a ISR de data-ready acorda esta tarefa diretamente. */
   g_imu.notifyTask(xTaskGetCurrentTaskHandle());

   g_imu_status = ImuInitStatus::CALIBRATING;

   if (!g_imu.calibrateGyroBias()) {
      g_imu_status = ImuInitStatus::FAILED;
      for (;;) {
         vTaskDelay(pdMS_TO_TICKS(500));
      }
   }

   g_imu_status = ImuInitStatus::READY;

   for (;;) {
      /* Espera a notificação da ISR. O timeout existe para que a perda do sinal
         de data-ready seja detectável em vez de travar a tarefa para sempre. */
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(cfg::IMU_TIMEOUT_MS)) == 0) {
         continue;
      }

      g_imu.read();

      ImuState state;
      state.yaw_rate = g_imu.yawRate();
      state.yaw_rate_raw = g_imu.yawRateRaw();
      state.accel_x = g_imu.accelX();
      state.accel_y = g_imu.accelY();
      state.accel_z = g_imu.accelZ();
      state.measured_rate_hz = g_imu.measuredRateHz();
      state.timestamp_ms = millis();

      xQueueOverwrite(g_mb_imu, &state);
   }
}

/**
 * @brief Núcleo 1, prioridade 4 — amostragem do array IR.
 */
void taskLineSensor(void*)
{
   g_ir.begin();

   RateMeter rate;

   for (;;) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)) == 0) {
         continue;
      }

      const uint32_t sample_start_us = micros();
      g_ir.sample();
      const uint32_t sample_us = micros() - sample_start_us;

      rate.tick(millis());

      LineState state;
      state.position = g_ir.position();
      state.error = g_ir.error();
      state.detected_count = g_ir.detectedCount();
      state.line_detected = g_ir.lineDetected();

      for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
         state.raw[i] = g_ir.raw(i);
      }

      state.sample_hz = rate.hz();
      state.sample_us = sample_us;

      /* Carga medida, não estimada: é o que transforma o orçamento de tempo
         desta tarefa em algo verificável na placa. */
      state.load_pct = (sample_us * cfg::IR_SAMPLE_HZ) / 10000.0f;

      state.timestamp_ms = millis();

      xQueueOverwrite(g_mb_line, &state);
   }
}

/**
 * @brief Núcleo 1, prioridade 3 — malha de controle a 250 Hz.
 *
 * É a única tarefa que toca no MotorDriver, e só arma os motores com o bit
 * EV_MODE_RUN ativo. A parada acontece aqui, na camada que aciona, e não por
 * acordo entre tarefas.
 */
void taskControl(void*)
{
   g_motors.begin();
   g_follower.reset();

   TickType_t last_wake = xTaskGetTickCount();
   bool was_running = false;

   RateMeter rate;

   for (;;) {
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS));

      rate.tick(millis());

      const bool running = (xEventGroupGetBits(g_eg_mode) & EV_MODE_RUN) != 0;

      /* Transições de armamento tratadas na borda, não a cada ciclo: entrar em
         RUNNING precisa zerar a malha, senão o integral acumulado antes da
         parada é aplicado de uma vez na retomada (RF04). */
      if (running && !was_running) {
         g_follower.reset();
         g_motors.arm();
      } else if (!running && was_running) {
         g_motors.disarm();
      }

      was_running = running;

      LineState line;
      ImuState imu;

      const bool have_line = readLineState(&line);
      const bool have_imu = readImuState(&imu);

      /* Sem amostra do array não há malha a fechar: os motores ficam parados e a
         malha não é atualizada. Mas o ciclo **não** é abandonado — a publicação
         do estado acontece de qualquer forma, mais abaixo, para que a telemetria
         mostre `haveLine` em zero em vez de simplesmente não mostrar nada.
         Abandonar o ciclo aqui era o que tornava toda falha a montante
         indistinguível de "o controle calculou zero". */
      if (!have_line) {
         g_motors.stop();
      } else {
         const float yaw_rate = have_imu ? imu.yaw_rate : 0.0f;
         const float dt = cfg::CONTROL_PERIOD_MS / 1000.0f;

         g_follower.update(line.error, line.line_detected, yaw_rate, dt);

         if (running) {
            g_motors.setSpeed(g_follower.left(), g_follower.right());
         } else {
            g_motors.stop();
         }
      }

      ControlState state;
      state.action = g_follower.action();
      state.left = running ? g_motors.appliedLeft() : 0.0f;
      state.right = running ? g_motors.appliedRight() : 0.0f;
      state.base_speed = g_follower.baseSpeed();

      state.p_term = g_follower.pid().proportionalTerm();
      state.i_term = g_follower.pid().integralTerm();
      state.d_term = g_follower.pid().derivativeTerm();
      state.gyro_term = g_follower.damper().lastTerm();

      state.blend = g_follower.classifier().blend();
      state.regime = g_follower.classifier().regime();

      state.line_lost = g_follower.lineLost();
      state.line_lost_ms = g_follower.lineLostMs();
      state.pid_saturated = g_follower.pid().saturated();
      state.mix_saturated = g_follower.mixSaturated();
      state.motors_armed = g_motors.isArmed();

      state.have_line = have_line;
      state.have_imu = have_imu;
      state.loop_hz = rate.hz();

      state.timestamp_ms = millis();

      xQueueOverwrite(g_mb_control, &state);
   }
}

/**
 * @brief Núcleo 0, prioridade 3 — máquina de estados a 50 Hz.
 */
void taskSupervisor(void*)
{
   g_led.begin();
   g_supervisor.begin();

   TickType_t last_wake = xTaskGetTickCount();

   for (;;) {
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg::SUPERVISOR_PERIOD_MS));

      Command command = Command::NONE;
      xQueueReceive(g_q_commands, &command, 0);

      /* O tempo sem linha vem da malha, não é recontado aqui: a malha é quem
         sabe, no passo de 4 ms, há quanto tempo o array não vê a pista. */
      ControlState control = {};
      const uint32_t line_lost_ms = readControlState(&control) ? control.line_lost_ms : 0;

#if FEATURE_LATERAL_MARKS
      LineState line = {};
      if (readLineState(&line)) {
         if (g_marks.update(line.detected_count, cfg::SUPERVISOR_PERIOD_MS / 1000.0f)) {
            g_supervisor.notifyLap();
         }
      }
#endif

      g_supervisor.update(command, g_imu_status, line_lost_ms, millis());

      /* Publica o modo nos dois canais: o event group para a malha, que precisa
         da leitura mais barata possível, e a mailbox para a telemetria, que
         precisa do contexto inteiro. */
      EventBits_t bits = 0;

      switch (g_supervisor.mode()) {
         case RobotMode::RUNNING:     bits = EV_MODE_RUN; break;
         case RobotMode::STOPPED:     bits = EV_MODE_STOP; break;
         case RobotMode::CALIBRATING: bits = EV_MODE_CALIB; break;
         case RobotMode::FAULT:       bits = EV_MODE_FAULT; break;
         default:                     bits = 0; break;
      }

      xEventGroupClearBits(g_eg_mode, EV_MODE_ALL & ~bits);
      if (bits != 0) {
         xEventGroupSetBits(g_eg_mode, bits);
      }

      RobotState state = g_supervisor.state();
      xQueueOverwrite(g_mb_mode, &state);

      g_led.setMode(g_supervisor.mode());
      g_led.update();
   }
}

#if FEATURE_TELEMETRY
/**
 * @brief Núcleo 0, prioridade 2 — telemetria a 20 Hz.
 *
 * É a **única** tarefa que imprime. Nenhuma ISR e nenhuma tarefa do núcleo 1
 * toca na serial, o que dispensa mutex e mantém a malha livre do custo de
 * formatação de texto.
 *
 * Na etapa 5 esta tarefa passa a alimentar a fila do WebSocket; a saída serial
 * continua como caminho de depuração.
 */
void taskTelemetry(void*)
{
   TickType_t last_wake = xTaskGetTickCount();
   RobotMode last_mode = RobotMode::BOOT;
   bool first = true;

   for (;;) {
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg::TELEMETRY_PERIOD_MS));

      RobotState robot = {};
      ControlState control = {};
      LineState line = {};
      ImuState imu = {};

      const bool have_robot = readRobotState(&robot);
      readControlState(&control);
      readLineState(&line);
      readImuState(&imu);

      /* Mudança de estado vai como texto no log, não como série: é evento
         discreto e o que se quer é o histórico legível. */
      if (have_robot && (first || robot.mode != last_mode)) {
         Serial.printf("[%lu] modo: %s -> %s  (falha: %s, voltas: %u)\n",
                       (unsigned long)millis(),
                       Supervisor::modeName(last_mode),
                       Supervisor::modeName(robot.mode),
                       Supervisor::faultName(robot.fault),
                       robot.lap_count);
         Serial.printf("        ciclos de parada concluidos: %u\n", robot.stop_count);
         last_mode = robot.mode;
         first = false;
      }

      teleplot::send("mode", (int32_t)robot.mode);
      teleplot::send("modeMs", (int32_t)robot.mode_elapsed_ms);
      teleplot::send("laps", (int32_t)robot.lap_count);
      teleplot::send("stopCycles", (int32_t)robot.stop_count);

      teleplot::send("position", (int32_t)line.position);
      teleplot::send("error", (int32_t)line.error);
      teleplot::send("detectedCount", (int32_t)line.detected_count);

      teleplot::send("yawRate", imu.yaw_rate, "deg/s");
      teleplot::send("imuHz", imu.measured_rate_hz, "Hz");

      teleplot::send("action", control.action);
      teleplot::send("left", control.left);
      teleplot::send("right", control.right);
      teleplot::send("pTerm", control.p_term);
      teleplot::send("iTerm", control.i_term);
      teleplot::send("dTerm", control.d_term);
      teleplot::send("gyroTerm", control.gyro_term);
      teleplot::send("blend", control.blend);
      teleplot::send("lineLostMs", (int32_t)control.line_lost_ms);
      teleplot::send("armed", (int32_t)(control.motors_armed ? 1 : 0));

      /*****************************************
       * Diagnóstico da cadeia de tarefas
       *
       * Na ordem em que a cadeia pode quebrar. A primeira série que estiver em
       * zero é onde o problema está — não é preciso adivinhar.
       *****************************************/
      teleplot::send("imuStatus", (int32_t)g_imu_status);
      teleplot::send("lineHz", line.sample_hz, "Hz");
      teleplot::send("lineSampleUs", (int32_t)line.sample_us, "us");
      teleplot::send("lineLoadPct", line.load_pct, "%");
      teleplot::send("ctrlHz", control.loop_hz, "Hz");
      teleplot::send("haveLine", (int32_t)(control.have_line ? 1 : 0));
      teleplot::send("haveImu", (int32_t)(control.have_imu ? 1 : 0));

      teleplot::send("heap", (int32_t)ESP.getFreeHeap());
   }
}
#endif

}  // namespace

/*****************************************
 * Leitura das mailboxes
 *****************************************/

bool readLineState(LineState* out)
{
   return xQueuePeek(g_mb_line, out, 0) == pdTRUE;
}

bool readImuState(ImuState* out)
{
   return xQueuePeek(g_mb_imu, out, 0) == pdTRUE;
}

bool readControlState(ControlState* out)
{
   return xQueuePeek(g_mb_control, out, 0) == pdTRUE;
}

bool readRobotState(RobotState* out)
{
   return xQueuePeek(g_mb_mode, out, 0) == pdTRUE;
}

bool postCommand(Command command)
{
   return xQueueSend(g_q_commands, &command, 0) == pdTRUE;
}

/*****************************************
 * Partida
 *****************************************/

bool tasksStart()
{
   g_mb_line = xQueueCreate(1, sizeof(LineState));
   g_mb_imu = xQueueCreate(1, sizeof(ImuState));
   g_mb_control = xQueueCreate(1, sizeof(ControlState));
   g_mb_mode = xQueueCreate(1, sizeof(RobotState));

   g_q_commands = xQueueCreate(cfg::COMMAND_QUEUE_DEPTH, sizeof(Command));
   g_eg_mode = xEventGroupCreate();

   if (!g_mb_line || !g_mb_imu || !g_mb_control || !g_mb_mode || !g_q_commands ||
       !g_eg_mode) {
      return false;
   }

   /* Núcleo 1 — tempo real. Ordem de criação pela prioridade, para que as
      tarefas sensoriais já existam quando o controle começar a ler. */
   if (xTaskCreatePinnedToCore(taskImu, "TaskImu", 3072, nullptr, 5, &g_task_imu, 1) !=
       pdPASS) {
      return false;
   }

   if (xTaskCreatePinnedToCore(taskLineSensor, "TaskLine", 3072, nullptr, 4, &g_task_line,
                               1) != pdPASS) {
      return false;
   }

   if (xTaskCreatePinnedToCore(taskControl, "TaskControl", 4096, nullptr, 3, nullptr, 1) !=
       pdPASS) {
      return false;
   }

   /* Núcleo 0 — melhor esforço. */
   if (xTaskCreatePinnedToCore(taskSupervisor, "TaskSup", 4096, nullptr, 3, nullptr, 0) !=
       pdPASS) {
      return false;
   }

#if FEATURE_TELEMETRY
   if (xTaskCreatePinnedToCore(taskTelemetry, "TaskTelem", 4096, nullptr, 2, nullptr, 0) !=
       pdPASS) {
      return false;
   }
#endif

   /*****************************************
    * Interrupções
    *
    * Ligadas depois das tarefas: as ISRs notificam handles de tarefa, e notificar
    * um handle nulo travaria a placa no primeiro disparo.
    *****************************************/

#if !FEATURE_AUTOSTART
   pinMode(pins::BTN_START, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(pins::BTN_START), buttonIsr, FALLING);
#endif

   /* Divisor 80 sobre o clock de 80 MHz dá 1 MHz, ou seja, 1 µs por contagem. */
   g_ir_timer = timerBegin(0, 80, true);
   if (g_ir_timer == nullptr) {
      return false;
   }

   timerAttachInterrupt(g_ir_timer, &irTimerIsr, true);
   timerAlarmWrite(g_ir_timer, 1000000 / cfg::IR_SAMPLE_HZ, true);
   timerAlarmEnable(g_ir_timer);

   return true;
}
