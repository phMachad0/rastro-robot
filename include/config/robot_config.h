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

/**
 * @brief Limiar que separa fundo escuro de linha clara, na leitura bruta do ADC.
 *
 * Leitura **acima** do limiar é fundo escuro; abaixo ou igual é a linha branca.
 * Como o QTR-8A mede reflexão, a superfície clara reflete mais e lê mais baixo.
 *
 * O valor é fixo e dispensa calibração: a faixa de operação do array é
 * conhecida e estável o bastante para que 4000 separe os dois casos com folga
 * em qualquer ponto da pista.
 *
 * @note Sem histerese, de propósito — é o que mantém a leitura simples. Se
 *       algum canal oscilar com o robô parado e a leitura perto de 4000, o
 *       sintoma aparece como `detectedCount` piscando no test_ir.
 */
constexpr uint16_t IR_BLACK_THRESHOLD = 4000;

/**
 * @brief Distância de posição entre dois canais vizinhos.
 *
 * Define a escala de IrArray::position(): com N canais a posição vai de 0
 * (extrema esquerda) a IR_POSITION_UNIT * (N - 1), ou seja, de 0 a 5000 para
 * os 6 canais em uso.
 */
constexpr uint16_t IR_POSITION_UNIT = 1000;

/** @brief Posição de setpoint, o centro do array na escala de 0 a 5000. */
constexpr uint16_t LINE_POSITION_CENTER = 2500;

/** @brief Tempo sem linha após o qual o supervisor leva o robô para FAULT. */
constexpr uint32_t LINE_LOST_TIMEOUT_MS = 400;

/**
 * @brief Fator aplicado à velocidade base durante a busca por linha perdida.
 *
 * O robô continua virando para o lado em que a linha sumiu, mas mais devagar:
 * mais lento significa mais tempo de busca dentro do mesmo LINE_LOST_TIMEOUT_MS
 * e menos chance de se afastar de vez da pista.
 */
constexpr float LINE_LOST_SPEED_FACTOR = 0.6f;

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
 * @brief Inversão do sentido de rotação de cada lado.
 *
 * Os dois lados são montados espelhados no chassi, então a mesma polaridade
 * elétrica gira os motores em sentidos opostos em relação à frente do robô.
 * Medido na bancada com test_motors: com comando positivo, o lado esquerdo
 * girava para trás enquanto o direito girava para frente.
 *
 * A correção fica aqui, e não trocando MOTOR_L_IN1 com MOTOR_L_IN2 em
 * board_pins.h, porque o mapa de pinos precisa continuar descrevendo a fiação
 * de verdade — trocar lá esconderia o espelhamento e faria qualquer conferência
 * futura com o multímetro parecer errada.
 *
 * Com estas flags corretas, velocidade positiva significa **robô para frente**
 * nos dois lados, que é a convenção que a camada de controle assume.
 *
 * As duas flags existem, e não só a do lado invertido, para que o mapa fique
 * explícito: se a fiação for refeita, é um booleano de cada lado.
 */
constexpr bool MOTOR_L_REVERSED = true;
constexpr bool MOTOR_R_REVERSED = false;

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
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 1000;

/**
 * @brief Resolução do PWM, em bits.
 *
 * A 5 kHz o LEDC do ESP32 (clock de 80 MHz) comporta até 13 bits. 10 bits dão
 * 1024 passos, dos quais ~358 caem na faixa útil acima da zona morta — folga
 * suficiente para a malha a 250 Hz.
 */
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 10;

/*****************************************
 * Normalização dos sinais de entrada do controle
 *
 * Todos os ganhos do controlador são **adimensionais** e agem sobre sinais
 * normalizados em [-1, 1]. Sem isso os ganhos ficariam em unidades
 * incomparáveis entre si — Kp em "velocidade por unidade de posição de array" e
 * Kg em "velocidade por grau por segundo" —, os números não diriam nada a quem
 * sintoniza, e os sliders do dashboard não teriam faixa sensata.
 *****************************************/

/**
 * @brief Divisor que normaliza o erro de posição para [-1, 1].
 *
 * O erro cru vai de -2500 a +2500 (meia faixa do array), então o divisor é a
 * própria meia faixa. Erro normalizado de ±1 significa linha na extremidade.
 */
constexpr float LINE_ERROR_SCALE = (float)LINE_POSITION_CENTER;

/**
 * @brief Taxa angular, em °/s, que corresponde a amortecimento normalizado de 1.
 *
 * Não é o fundo de escala do sensor (±500 °/s), mas a ordem de grandeza da
 * guinada em curva normal do robô. Normalizar pelo fundo de escala deixaria o
 * termo de amortecimento sempre minúsculo e Kg teria que ser enorme para
 * compensar, o que é ruim de sintonizar.
 */
constexpr float GYRO_RATE_REFERENCE = 300.0f;

/**
 * @brief Inversão do sinal da taxa de guinada.
 *
 * Depende de como o MPU-6050 está orientado no chassi. Com o sinal errado o
 * termo de amortecimento deixa de ser realimentação negativa e passa a ser
 * **positiva**: em vez de frear a rotação, ela é reforçada, e o robô entra em
 * oscilação crescente até sair da pista.
 *
 * @warning Conferir no test_control antes de ligar os motores: girando o robô à
 *          mão no sentido horário, `yawRate` precisa sair positivo.
 */
constexpr bool GYRO_Z_REVERSED = false;

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

/**
 * @brief Velocidade base em curva.
 *
 * Menor que BASE_SPEED, que vale para a reta. O TrackClassifier interpola entre
 * as duas junto com os ganhos, para que a transição de regime não dê degrau de
 * aceleração.
 */
constexpr float BASE_SPEED_CURVE = 0.38f;

constexpr float KP_CURVE = 1.10f;
constexpr float KI_CURVE = 0.00f;
constexpr float KD_CURVE = 0.05f;
constexpr float KG_CURVE = 0.15f;

/**
 * @brief Teto do integrador, em unidades de erro normalizado vezes segundo.
 *
 * Com o erro normalizado em [-1, 1], um segundo de erro máximo acumula 1.0.
 * O teto de 2.0 dá margem para a integral corrigir uma tendência persistente
 * sem que ela domine a ação nos primeiros instantes de uma curva longa.
 */
constexpr float PID_INTEGRAL_LIMIT = 2.0f;

/**
 * @brief Limite da ação diferencial, em unidades de velocidade.
 *
 * 1.0 permite que a ação chegue a inverter um lado, que é o necessário para
 * curva fechada. Também é o limite usado pelo anti-windup para decidir que a
 * saída saturou.
 */
constexpr float PID_OUTPUT_LIMIT = 1.0f;

/**
 * @brief Tamanho da janela do termo derivativo, em amostras.
 *
 * A derivada é calculada sobre a diferença ao longo da janela inteira, e não
 * entre amostras consecutivas. Isso importa porque a posição da linha é
 * quantizada em passos de 500 (ver seção 5.1 da arquitetura): a diferença entre
 * duas amostras vizinhas é zero quase sempre e dá um salto enorme quando muda,
 * o que a derivada ponto a ponto transformaria em picos inúteis.
 *
 * A 250 Hz, 20 amostras são 80 ms de janela.
 */
constexpr uint8_t PID_DERIVATIVE_WINDOW = 20;

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
