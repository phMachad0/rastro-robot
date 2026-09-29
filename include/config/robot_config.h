/**
 * @file robot_config.h
 *
 * @brief Parâmetros de operação do robô.
 *
 * Os valores aqui são o padrão de fábrica. Em execução, os parâmetros
 * sintonizáveis vêm da NVS e podem ser reescritos pelo dashboard; estes
 * valores só valem no primeiro boot ou após um reset de configuração.
 *
 * Os ganhos são pontos de partida plausíveis, não valores sintonizados — a
 * sintonia de verdade acontece na pista, na etapa 6 do roteiro.
 *
 * Ver docs/ARQUITETURA.md, seção 9.
 */

#ifndef CONFIG_ROBOT_CONFIG_H_
#define CONFIG_ROBOT_CONFIG_H_

#include <stdint.h>

namespace cfg {

/*****************************************
 * Taxas das tarefas
 *****************************************/

constexpr uint32_t CONTROL_HZ   = 250;  /**< Malha de controle, TaskControl. */
constexpr uint32_t IR_SAMPLE_HZ = 500;  /**< Amostragem do array IR, via timer. */
constexpr uint32_t IMU_SAMPLE_HZ = 200; /**< Taxa do MPU-6050, via data-ready. */
constexpr uint32_t SUPERVISOR_HZ = 50;  /**< Máquina de estados. */
constexpr uint32_t TELEMETRY_HZ = 20;   /**< Envio de telemetria em regime normal. */

constexpr uint32_t CONTROL_PERIOD_MS    = 1000 / CONTROL_HZ;
constexpr uint32_t SUPERVISOR_PERIOD_MS = 1000 / SUPERVISOR_HZ;
constexpr uint32_t TELEMETRY_PERIOD_MS  = 1000 / TELEMETRY_HZ;

/*****************************************
 * Array IR e posição da linha
 *****************************************/

/** @brief Leituras por canal em cada amostragem, para média e redução de ruído. */
constexpr uint8_t IR_SAMPLES_PER_CHANNEL = 4;

/** @brief Escala da leitura normalizada de cada canal, após calibração. */
constexpr uint16_t IR_NORMALIZED_MAX = 1000;

/**
 * @brief Posição de setpoint, correspondente ao centro do array.
 *
 * A posição vai de 0 (extrema esquerda) a IR_NORMALIZED_MAX * (N - 1) com
 * N = 6 canais, ou seja, de 0 a 5000. O centro é 2500.
 */
constexpr uint16_t LINE_POSITION_CENTER = 2500;

/**
 * @brief Soma mínima dos pesos para considerar que a linha foi detectada.
 *
 * Abaixo disso nenhum sensor está sobre a linha e a posição calculada não
 * tem significado.
 */
constexpr uint16_t LINE_LOST_THRESHOLD = 300;

/** @brief Tempo sem linha após o qual o supervisor leva o robô para FAULT. */
constexpr uint32_t LINE_LOST_TIMEOUT_MS = 400;

/** @brief Duração da varredura de calibração do array IR. */
constexpr uint32_t IR_CALIBRATION_MS = 6000;

/*****************************************
 * Motores
 *****************************************/

constexpr float BASE_SPEED = 0.50f;  /**< Velocidade base em reta, de 0 a 1. */
constexpr float MAX_SPEED  = 0.95f;  /**< Teto de saída para cada lado. */

/**
 * @brief Duty mínimo em que o motor começa a girar, de 0 a 1.
 *
 * Abaixo da zona morta o motor apenas zumbe sem se mover, e correções
 * pequenas do PID não produzem movimento nenhum. Medir na bancada na
 * etapa 2 e ajustar.
 */
constexpr float MOTOR_DEADZONE = 0.12f;

/**
 * @brief Fator de compensação do descasamento entre os lados, de -1 a 1.
 *
 * Positivo favorece o lado direito. Com dois motores por canal é provável
 * que um lado seja mais rápido que o outro no mesmo duty. Medir na etapa 2.
 */
constexpr float MOTOR_TRIM = 0.0f;

/** @brief Frequência do PWM. A definir na bancada — ver docs, seção 12. */
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
constexpr uint8_t  MOTOR_PWM_RESOLUTION_BITS = 10;

/*****************************************
 * Ganhos do controlador — regime de reta
 *
 * Em reta o robô anda mais rápido com ganho proporcional menor, e o
 * amortecimento por giroscópio é mais forte, que é o que combate o
 * zigue-zague do RF06.
 *****************************************/

constexpr float KP_STRAIGHT = 0.60f;
constexpr float KI_STRAIGHT = 0.00f;
constexpr float KD_STRAIGHT = 0.02f;
constexpr float KG_STRAIGHT = 0.35f;  /**< Amortecimento pela taxa angular. */

/*****************************************
 * Ganhos do controlador — regime de curva
 *****************************************/

constexpr float KP_CURVE = 1.10f;
constexpr float KI_CURVE = 0.00f;
constexpr float KD_CURVE = 0.05f;
constexpr float KG_CURVE = 0.15f;

/** @brief Teto do integrador, para evitar windup quando a saída satura. */
constexpr float PID_INTEGRAL_LIMIT = 500.0f;

/*****************************************
 * Classificador reta × curva
 *
 * Dois limiares, não um: a histerese evita que o robô fique alternando entre
 * os dois conjuntos de ganhos quando a taxa angular oscila perto da fronteira.
 *****************************************/

constexpr float    GYRO_TH_LOW  = 25.0f;  /**< °/s — abaixo disso, entra em reta. */
constexpr float    GYRO_TH_HIGH = 60.0f;  /**< °/s — acima disso, entra em curva. */
constexpr uint32_t REGIME_WINDOW_MS = 150; /**< Janela da média móvel de |wz|. */
constexpr uint32_t REGIME_BLEND_MS  = 120; /**< Interpolação entre ganhos na troca. */

/*****************************************
 * IMU
 *****************************************/

/**
 * @brief Amostras usadas para estimar o bias do giroscópio na inicialização.
 *
 * O MPU-6050 tem offset significativo; sem remover o bias o robô desenvolve
 * uma tendência lateral constante.
 */
constexpr uint16_t GYRO_BIAS_SAMPLES = 500;

/** @brief Coeficiente do filtro IIR de primeira ordem aplicado a wz, de 0 a 1. */
constexpr float GYRO_LPF_ALPHA = 0.20f;

/*****************************************
 * Política de parada e retomada — RF03 e RF04
 *
 * Com FEATURE_LATERAL_MARKS desligada, TimedPolicy usa estes tempos. Com a
 * flag ligada, MarkCountPolicy usa LAPS_TARGET.
 *****************************************/

constexpr uint32_t RUN_DURATION_MS  = 30000;  /**< Tempo em RUNNING antes de parar. */
constexpr uint32_t STOP_DURATION_MS = 3000;   /**< Tempo parado antes de retomar. */
constexpr uint8_t  LAPS_TARGET      = 3;      /**< Marcações até a parada. */

/*****************************************
 * Rede
 *****************************************/

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000; /**< Depois disso, cai para modo AP. */
constexpr char     MDNS_HOSTNAME[]  = "rastro";     /**< Responde em rastro.local. */
constexpr char     AP_SSID[]        = "rastro-robot";
constexpr uint8_t  TELEMETRY_QUEUE_DEPTH = 4;
constexpr uint8_t  COMMAND_QUEUE_DEPTH   = 8;

/*****************************************
 * Diversos
 *****************************************/

constexpr uint32_t SERIAL_BAUD = 115200;

/** @brief Janela de debounce do botão de start, tratada na própria ISR. */
constexpr uint32_t BTN_DEBOUNCE_MS = 200;

}  // namespace cfg

#endif  // CONFIG_ROBOT_CONFIG_H_
