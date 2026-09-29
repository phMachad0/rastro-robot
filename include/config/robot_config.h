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

/**
 * @brief Amplitude mínima entre o mínimo e o máximo de um canal calibrado.
 *
 * Um canal cujo min e max ficaram próximos não viu contraste durante a
 * calibração — sensor solto, mal posicionado ou varredura curta demais.
 * Normalizar por uma amplitude minúscula amplificaria só ruído, então o canal
 * é marcado como não calibrado e a calibração inteira é rejeitada.
 */
constexpr uint16_t IR_CALIBRATION_MIN_SPAN = 200;

/*****************************************
 * Motores
 *****************************************/

constexpr float BASE_SPEED = 0.50f;  /**< Velocidade base em reta, de 0 a 1. */
constexpr float MAX_SPEED  = 0.95f;  /**< Teto de saída para cada lado. */

/**
 * @brief Duty mínimo em que o motor começa a girar, de 0 a 1.
 *
 * Medido na bancada com test_motors. Abaixo desse duty o motor apenas zumbe
 * sem se mover, e correções pequenas do PID não produziriam movimento nenhum.
 *
 * O MotorDriver compensa essa zona morta remapeando a velocidade comandada
 * para a faixa útil:
 *
 *     duty = MOTOR_DEADZONE + (1 - MOTOR_DEADZONE) * |velocidade|
 *
 * Por isso os valores de BASE_SPEED e MAX_SPEED continuam sendo velocidade
 * efetiva de 0 a 1, e não duty — quem chama setSpeed() não precisa saber que
 * a zona morta existe.
 *
 * @warning 0.65 é um valor alto: só 35% da faixa de PWM produz movimento. Isso
 *          é sintoma de tensão insuficiente chegando aos motores, coerente com
 *          a queda de ~2,5 V do L298N. Se a alimentação for revista (9 V na
 *          entrada do driver, ou troca por uma ponte de MOSFET), este valor cai
 *          bastante e a resolução de controle melhora junto. Remedir depois de
 *          qualquer mudança na alimentação.
 */
constexpr float MOTOR_DEADZONE = 0.65f;

/**
 * @brief Fator de compensação do descasamento entre os lados, de -1 a 1.
 *
 * Positivo favorece o lado direito. Verificado na bancada na etapa 2: não há
 * descasamento significativo entre os lados, então o fator fica em zero. O
 * mecanismo continua no MotorDriver porque o descasamento tende a aparecer com
 * o desgaste e com a queda de tensão da bateria, e aí é ajustável pelo
 * dashboard sem recompilar.
 */
constexpr float MOTOR_TRIM = 0.0f;

/**
 * @brief Frequência do PWM, decidida na bancada.
 *
 * Meio-termo entre o ~1 kHz (mais torque em baixa rotação, motor audível) e o
 * ~20 kHz (silencioso, perde torque): a 5 kHz o chiado sai da faixa mais
 * incômoda sem sacrificar torque na partida, que é o que importa com uma zona
 * morta tão alta.
 */
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 5000;

/**
 * @brief Resolução do PWM, em bits.
 *
 * A 5 kHz o LEDC do ESP32 (clock de 80 MHz) comporta até 13 bits. 10 bits dão
 * 1024 passos, dos quais ~358 caem na faixa útil acima da zona morta — folga
 * suficiente para a malha a 250 Hz.
 */
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 10;

/**
 * @brief Variação máxima de velocidade por segundo, em unidades de [0, 1].
 *
 * Limita a taxa de subida da saída dos motores. Com quatro motores partindo ao
 * mesmo tempo, o degrau de corrente é o que dispara a proteção de sobrecorrente
 * do powerbank e derruba o ESP32 junto; a rampa espalha esse pico no tempo.
 *
 * 4.0 significa ir de parado a toda velocidade em 250 ms — rápido o bastante
 * para não atrapalhar a malha de controle, que trabalha com correções pequenas
 * e contínuas, e lento o bastante para suavizar a partida.
 *
 * @note Ainda não validado na bancada. Se o robô ficar lerdo para responder a
 *       curvas fechadas, este é o primeiro valor a subir.
 */
constexpr float MOTOR_SLEW_PER_S = 4.0f;

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

/**
 * @brief Largura de banda do filtro digital interno do MPU-6050.
 *
 * O valor corresponde a MPU6050_DLPF_BW_42 (~44 Hz no giroscópio). Filtrar no
 * sensor, antes da amostragem, é o que evita aliasing — um IIR no firmware não
 * consegue desfazer ruído que já entrou dobrado na amostra.
 *
 * Ligar o DLPF também muda a taxa base do giroscópio de 8 kHz para 1 kHz, que
 * é a base do divisor de IMU_SAMPLE_HZ.
 */
constexpr uint8_t GYRO_DLPF_MODE = 3;

/** @brief Taxa base do giroscópio com o DLPF ligado, usada no divisor de taxa. */
constexpr uint32_t IMU_BASE_RATE_HZ = 1000;

/** @brief Tempo máximo esperando o data-ready antes de declarar a IMU muda. */
constexpr uint32_t IMU_TIMEOUT_MS = 100;

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
