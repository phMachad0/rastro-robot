/**
 * @file imu.cpp
 *
 * @brief Implementação da Imu sobre o MPU-6050.
 */

#include "hal/imu.h"

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>

#include "config/board_pins.h"
#include "config/robot_config.h"

namespace {

MPU6050 mpu;

/** @brief Fator de escala do acelerômetro em ±2 g, em LSB por g. */
constexpr float SCALE_ACCEL = 16384.0f;

/** @brief Fator de escala do giroscópio em ±500 °/s, em LSB por °/s. */
constexpr float SCALE_GYRO = 65.5f;

constexpr uint32_t I2C_FREQ_HZ = 400000;

/** @brief Janela de medição da taxa efetiva de amostragem. */
constexpr uint32_t RATE_WINDOW_MS = 1000;

/**
 * @brief Contador de interrupções de data-ready.
 *
 * `volatile` porque é escrito na ISR e lido na task. A leitura e a escrita de
 * um uint32_t são atômicas no Xtensa, então não é preciso seção crítica para
 * um contador simples.
 */
volatile uint32_t g_interrupt_count = 0;

/** @brief Valor do contador na última amostra consumida. */
volatile uint32_t g_consumed_count = 0;

/** @brief Task avisada pela ISR, quando houver uma (etapa 4). */
TaskHandle_t g_notify_task = nullptr;

/**
 * @brief ISR de data-ready do MPU-6050.
 *
 * Mínima de propósito: sem alocação, sem Serial e sem I2C. A leitura do sensor
 * custa dezenas de microssegundos no barramento e não pode acontecer aqui.
 */
void IRAM_ATTR imuDataReadyIsr()
{
   g_interrupt_count++;

   if (g_notify_task != nullptr) {
      BaseType_t higher_priority_woken = pdFALSE;
      vTaskNotifyGiveFromISR(g_notify_task, &higher_priority_woken);
      portYIELD_FROM_ISR(higher_priority_woken);
   }
}

}  // namespace

bool Imu::begin()
{
   Wire.begin(pins::I2C_SDA, pins::I2C_SCL, I2C_FREQ_HZ);

   mpu.initialize();

   // if (!mpu.testConnection()) {
   //    return false;
   // }

   mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

   /* O DLPF filtra antes da amostragem, o que é o que evita aliasing — um IIR
      no firmware não desfaz ruído que já entrou dobrado na amostra. Ligá-lo
      também baixa a taxa base do giroscópio de 8 kHz para 1 kHz. */
   mpu.setDLPFMode(cfg::GYRO_DLPF_MODE);

   /* Taxa de saída = base / (1 + divisor). */
   const uint8_t divider = (uint8_t)((cfg::IMU_BASE_RATE_HZ / cfg::IMU_SAMPLE_HZ) - 1);
   mpu.setRate(divider);

   /* Pulso ativo em nível alto, limpo automaticamente: a ISR não precisa
      tocar no I2C para rearmar a interrupção. */
   mpu.setInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
   mpu.setInterruptLatch(MPU6050_INTLATCH_50USPULSE);
   mpu.setIntDataReadyEnabled(true);

   pinMode(pins::IMU_INT, INPUT);
   attachInterrupt(digitalPinToInterrupt(pins::IMU_INT), imuDataReadyIsr, RISING);

   g_interrupt_count = 0;
   g_consumed_count = 0;

   rate_window_start_ms_ = millis();
   rate_window_count_ = 0;

   return true;
}

void Imu::notifyTask(TaskHandle_t task)
{
   g_notify_task = task;
}

uint32_t Imu::interruptCount() const
{
   return g_interrupt_count;
}

bool Imu::dataReady() const
{
   return g_interrupt_count != g_consumed_count;
}

bool Imu::waitForSample(uint32_t timeout_ms)
{
   const uint32_t deadline = millis() + timeout_ms;

   while (!dataReady()) {
      if ((int32_t)(millis() - deadline) >= 0) {
         return false;
      }

      /* vTaskDelay, e não taskYIELD: taskYIELD só cede para tarefas de
         prioridade igual ou maior, então a tarefa ociosa nunca rodaria e o
         watchdog de tarefa acabaria resetando a placa numa espera longa como a
         da estimativa de bias. Um tick de 1 ms é bem menor que o período de
         amostragem de 5 ms, então não custa latência perceptível. */
      vTaskDelay(1);
   }

   return true;
}

void Imu::read()
{
   g_consumed_count = g_interrupt_count;

   int16_t ax, ay, az;
   int16_t gx, gy, gz;

   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

   accel_x_ = ax / SCALE_ACCEL;
   accel_y_ = ay / SCALE_ACCEL;
   accel_z_ = az / SCALE_ACCEL;

   roll_rate_  = gx / SCALE_GYRO - bias_x_;
   pitch_rate_ = gy / SCALE_GYRO - bias_y_;

   /* A orientação é aplicada antes da remoção de bias, e isso é consistente: o
      bias é estimado com este mesmo código, então ele já vem no referencial
      corrigido e a subtração fecha. */
   const float yaw_oriented = cfg::GYRO_Z_REVERSED ? -gz / SCALE_GYRO : gz / SCALE_GYRO;
   yaw_rate_raw_ = yaw_oriented - bias_z_;

   /* IIR de primeira ordem. A primeira amostra inicializa o filtro em vez de
      ser promediada contra zero, o que evitaria um transitório de partida que
      o classificador de regime leria como movimento. */
   if (filter_primed_) {
      yaw_rate_ += cfg::GYRO_LPF_ALPHA * (yaw_rate_raw_ - yaw_rate_);
   } else {
      yaw_rate_ = yaw_rate_raw_;
      filter_primed_ = true;
   }

   /* Taxa efetiva, contada por janela de 1 s. */
   rate_window_count_++;
   const uint32_t elapsed = millis() - rate_window_start_ms_;

   if (elapsed >= RATE_WINDOW_MS) {
      measured_rate_hz_ = (rate_window_count_ * 1000.0f) / elapsed;
      rate_window_count_ = 0;
      rate_window_start_ms_ = millis();
   }
}

bool Imu::update()
{
   if (!waitForSample(cfg::IMU_TIMEOUT_MS)) {
      return false;
   }

   read();
   return true;
}

bool Imu::calibrateGyroBias()
{
   bias_x_ = 0.0f;
   bias_y_ = 0.0f;
   bias_z_ = 0.0f;

   double sum_x = 0.0;
   double sum_y = 0.0;
   double sum_z = 0.0;

   for (uint16_t i = 0; i < cfg::GYRO_BIAS_SAMPLES; i++) {
      if (!waitForSample(cfg::IMU_TIMEOUT_MS)) {
         return false;
      }

      read();

      /* Acumula o valor cru: bias_* ainda é zero, então yaw_rate_raw_ é a
         leitura direta do sensor. */
      sum_x += roll_rate_;
      sum_y += pitch_rate_;
      sum_z += yaw_rate_raw_;
   }

   bias_x_ = (float)(sum_x / cfg::GYRO_BIAS_SAMPLES);
   bias_y_ = (float)(sum_y / cfg::GYRO_BIAS_SAMPLES);
   bias_z_ = (float)(sum_z / cfg::GYRO_BIAS_SAMPLES);

   /* O filtro foi alimentado com leituras ainda enviesadas durante a
      estimativa; rearmá-lo faz a primeira amostra real reinicializá-lo. */
   filter_primed_ = false;

   return true;
}
