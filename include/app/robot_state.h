/**
 * @file robot_state.h
 *
 * @brief Estruturas compartilhadas entre as tarefas.
 *
 * Cada struct aqui é o conteúdo de uma mailbox da seção 4.2: escrita inteira
 * com `xQueueOverwrite` e lida inteira com `xQueuePeek`. É por isso que são
 * structs e não variáveis soltas — a cópia atômica do bloco é o que elimina a
 * possibilidade de um consumidor ler metade de uma amostra nova e metade da
 * anterior.
 *
 * Ver docs/ARQUITETURA.md, seções 4.1 e 4.2.
 */

#ifndef APP_ROBOT_STATE_H_
#define APP_ROBOT_STATE_H_

#include <stdint.h>

#include "config/board_pins.h"
#include "control/track_classifier.h"

/** @brief Estados da máquina de estados do supervisor. */
enum class RobotMode : uint8_t {
   BOOT,         /**< Auto-teste dos periféricos. */
   CALIBRATING,  /**< Estimativa do bias do giroscópio. Motores travados. */
   IDLE,         /**< Pronto e parado, esperando comando de partida. */
   RUNNING,      /**< Seguindo a linha. O único estado que arma os motores. */
   STOPPED,      /**< Parada programada do RF03. */
   FAULT,        /**< Falha; só sai por comando de reset. */
};

/** @brief Comandos aceitos pelo supervisor, via botão ou rede. */
enum class Command : uint8_t {
   NONE,
   START,
   STOP,
   RESET,
   /** @brief Alterna entre IDLE e RUNNING. É o que o botão físico envia. */
   TOGGLE_RUN,
};

/** @brief Motivo de uma entrada em FAULT. */
enum class FaultReason : uint8_t {
   NONE,
   IMU_ABSENT,   /**< MPU-6050 não respondeu no auto-teste. */
   IMU_SILENT,   /**< A interrupção de data-ready parou de chegar. */
   LINE_LOST,    /**< Linha fora do array por mais de LINE_LOST_TIMEOUT_MS. */
};

/** @brief Progresso da inicialização da IMU, publicado pela própria TaskImu. */
enum class ImuInitStatus : uint8_t {
   PENDING,
   CALIBRATING,
   READY,
   FAILED,
};

/** @brief Saída da TaskLineSensor. */
struct LineState {
   uint16_t position;        /**< 0 a 5000. */
   int16_t  error;           /**< -2500 a +2500. */
   uint8_t  detected_count;
   bool     line_detected;
   uint16_t raw[pins::IR_CHANNEL_COUNT];

   /** @brief Taxa efetiva de amostragem medida, em Hz. */
   float sample_hz;

   /** @brief Duração da última amostragem, em µs. */
   uint32_t sample_us;

   /**
    * @brief Fração do período de amostragem gasta amostrando, em porcentagem.
    *
    * É o número que diz se a tarefa cabe no próprio período. Acima de ~80% o
    * controle começa a ser privado de CPU; em 100% ele para de rodar, porque
    * `TaskLineSensor` tem prioridade maior e nunca chega a bloquear.
    */
   float load_pct;

   uint32_t timestamp_ms;
};

/** @brief Saída da TaskImu. */
struct ImuState {
   float    yaw_rate;        /**< °/s, sem bias e filtrado. */
   float    yaw_rate_raw;
   float    accel_x;
   float    accel_y;
   float    accel_z;
   float    measured_rate_hz;
   uint32_t timestamp_ms;
};

/** @brief Saída da TaskControl. */
struct ControlState {
   float action;
   float left;
   float right;
   float base_speed;

   float p_term;
   float i_term;
   float d_term;
   float gyro_term;

   float       blend;
   TrackRegime regime;

   bool     line_lost;
   uint32_t line_lost_ms;
   bool     pid_saturated;
   bool     mix_saturated;

   /** @brief true se os motores estavam armados neste ciclo. */
   bool motors_armed;

   /*****************************************
    * Diagnóstico
    *
    * Existe porque a primeira versão desta tarefa abandonava o ciclo quando
    * faltava amostra do array, sem publicar nada. Toda falha a montante aparecia
    * como zeros na telemetria, indistinguível de "o controle calculou zero".
    *****************************************/

   /** @brief false se a mailbox do array estava vazia neste ciclo. */
   bool have_line;

   /** @brief false se a mailbox da IMU estava vazia neste ciclo. */
   bool have_imu;

   /**
    * @brief Taxa efetiva da malha, em Hz.
    *
    * Se cair muito abaixo de cfg::CONTROL_HZ, esta tarefa está sendo privada de
    * CPU por alguma de prioridade maior no mesmo núcleo. Zero significa que ela
    * não roda — e aí nada do resto desta struct chega à telemetria.
    */
   float loop_hz;

   uint32_t timestamp_ms;
};

/**
 * @brief Estado da máquina de estados, publicado pelo supervisor.
 *
 * É também o que a StopPolicy recebe para decidir parada e retomada.
 */
struct RobotState {
   RobotMode   mode;
   uint32_t    mode_elapsed_ms;  /**< Tempo no estado corrente. */

   /**
    * @brief Marcações laterais reconhecidas — RF02.
    *
    * Alimentado apenas pelo MarkDetector, então fica **sempre em zero** com
    * FEATURE_LATERAL_MARKS desligada. Não é incrementado na retomada: ciclo de
    * parada e volta de pista são coisas diferentes, e misturá-las faria a
    * telemetria reportar voltas que o robô não deu.
    */
   uint8_t lap_count;

   /** @brief Ciclos de parada-retomada concluídos. Valida RF03 e RF04. */
   uint8_t stop_count;

   FaultReason fault;
   uint32_t    timestamp_ms;
};

#endif  // APP_ROBOT_STATE_H_
