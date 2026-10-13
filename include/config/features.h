/**
 * @file features.h
 *
 * @brief Flags de funcionalidade resolvidas em tempo de compilação.
 *
 * Cada flag desliga um subsistema inteiro sem deixar código morto ligado no
 * binário e sem que os outros módulos precisem saber que ela existe.
 *
 * Ver docs/ARQUITETURA.md, seções 6.1 e 6.2.
 */

#ifndef CONFIG_FEATURES_H_
#define CONFIG_FEATURES_H_

/**
 * @brief Detecção de marcações laterais na pista — RF02.
 *
 * Desligada por padrão: ainda não há certeza de que será implementada e a
 * prioridade é menor que a do restante.
 *
 * Com a flag em 0:
 *   - MarkDetector é um stub que nunca reporta marcação;
 *   - MarkCountPolicy não é compilada e TimedPolicy governa o ciclo de
 *     parada e retomada (RF03/RF04);
 *   - nenhum pino adicional é reservado;
 *   - a telemetria envia lapCount nulo e o dashboard esconde o painel de voltas.
 *
 * Ligar a flag é a única mudança necessária fora do próprio módulo.
 */
#ifndef FEATURE_LATERAL_MARKS
#define FEATURE_LATERAL_MARKS 0
#endif

/**
 * @brief Amortecimento da malha pela taxa angular do giroscópio — RF06.
 *
 * Mantida como flag para permitir comparar, na pista, o comportamento com e
 * sem o termo de amortecimento durante a sintonia.
 */
#ifndef FEATURE_GYRO_DAMPING
#define FEATURE_GYRO_DAMPING 1
#endif

/**
 * @brief Classificação reta × curva com ganhos adaptativos.
 *
 * Desligada, o robô usa um único conjunto de ganhos (o de curva, mais
 * conservador). Útil para sintonizar o PID base antes de introduzir a
 * adaptação.
 */
#ifndef FEATURE_ADAPTIVE_GAINS
#define FEATURE_ADAPTIVE_GAINS 1
#endif

/**
 * @brief Partida automática, sem comando nenhum.
 *
 * Com a flag em 1:
 *   - o supervisor sai de IDLE para RUNNING sozinho, depois de
 *     cfg::AUTOSTART_DELAY_MS;
 *   - a ISR do botão e o próprio pino não são compilados.
 *
 * Com a flag em 0 volta o comportamento manual: nada se move até um
 * Command::START ou um toque no botão de cfg / pins::BTN_START.
 *
 * É um interruptor único em vez de duas flags separadas porque os dois modos são
 * mutuamente exclusivos na prática — ou o robô espera comando, ou não espera.
 *
 * @warning Com a partida automática ligada, o robô **sai andando sozinho** alguns
 *          segundos depois de energizar, inclusive na bancada. Não há comando que
 *          impeça isso: o único jeito de parar é desligar a alimentação.
 */
#ifndef FEATURE_AUTOSTART
#define FEATURE_AUTOSTART 1
#endif

/**
 * @brief Telemetria e dashboard por Wi-Fi — RF05.
 *
 * Desligar libera memória e elimina qualquer atividade de rádio, o que ajuda
 * a isolar problemas de controle durante a depuração.
 */
#ifndef FEATURE_TELEMETRY
#define FEATURE_TELEMETRY 1
#endif

/** Telemetry transport: exactly one backend is compiled into the firmware. */
#define TELEMETRY_BACKEND_TELEPLOT 0
#define TELEMETRY_BACKEND_WIFI_MQTT 1
#ifndef FEATURE_TELEMETRY_BACKEND
#define FEATURE_TELEMETRY_BACKEND TELEMETRY_BACKEND_WIFI_MQTT
#endif
#if FEATURE_TELEMETRY_BACKEND != TELEMETRY_BACKEND_TELEPLOT && \
    FEATURE_TELEMETRY_BACKEND != TELEMETRY_BACKEND_WIFI_MQTT
#error "FEATURE_TELEMETRY_BACKEND must select TELEMETRY_BACKEND_TELEPLOT or TELEMETRY_BACKEND_WIFI_MQTT"
#endif

#endif  // CONFIG_FEATURES_H_
