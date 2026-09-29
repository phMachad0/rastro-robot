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
 * @brief Telemetria e dashboard por Wi-Fi — RF05.
 *
 * Desligar libera memória e elimina qualquer atividade de rádio, o que ajuda
 * a isolar problemas de controle durante a depuração.
 */
#ifndef FEATURE_TELEMETRY
#define FEATURE_TELEMETRY 1
#endif

#endif  // CONFIG_FEATURES_H_
