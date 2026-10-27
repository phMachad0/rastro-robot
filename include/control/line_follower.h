/**
 * @file line_follower.h
 *
 * @brief Malha de seguimento de linha: compõe PID, amortecimento e classificador.
 *
 * É o único módulo de controle que a camada de aplicação precisa conhecer. Ele
 * orquestra, nesta ordem:
 *
 *   1. `TrackClassifier` decide o regime e entrega os ganhos interpolados;
 *   2. `PidAlgorithm` calcula a ação sobre o erro normalizado de posição;
 *   3. `GyroDamper` subtrai o termo de amortecimento (RF06);
 *   4. a ação resultante é mixada nos dois lados, com saturação que preserva a
 *      diferença entre eles.
 *
 * **Convenção de sinal**, que é o ponto onde um seguidor de linha costuma sair
 * da pista em vez de voltar para ela:
 *
 *   - `line_error` positivo significa linha à **direita** do centro do array,
 *     ou seja, o robô está à esquerda da linha;
 *   - a ação `u` positiva acelera o lado **esquerdo** e desacelera o direito,
 *     virando o robô para a direita;
 *   - portanto erro positivo produz ação positiva, e o robô volta para a linha.
 *
 * Não produz PWM nem toca no `MotorDriver`: devolve velocidades em [-1, 1] e
 * quem chama decide se as aplica. É o que permite rodar a malha completa com os
 * motores desabilitados no `test_control`.
 *
 * Ver docs/ARQUITETURA.md, seções 5.2 a 5.5.
 */

#ifndef CONTROL_LINE_FOLLOWER_H_
#define CONTROL_LINE_FOLLOWER_H_

#include <stdint.h>

#include "control/gyro_damper.h"
#include "control/pid_algorithm.h"
#include "control/track_classifier.h"

class LineFollower {
 public:
   /** @brief Zera PID, classificador e o estado de linha perdida. */
   void reset();

   /**
    * @brief Roda um ciclo da malha.
    *
    * @param line_error    Erro cru do IrArray, de -2500 a +2500.
    * @param line_detected false quando nenhum canal vê a linha.
    * @param yaw_rate_dps  Taxa de guinada em °/s, sem bias e filtrada.
    * @param dt            Passo de tempo em segundos.
    */
   void update(int16_t line_error, bool line_detected, float yaw_rate_dps, float dt);

   /*****************************************
    * Saída
    *****************************************/

   /** @brief Velocidade do lado esquerdo, em [-1, 1]. */
   float left() const { return left_; }

   /** @brief Velocidade do lado direito, em [-1, 1]. */
   float right() const { return right_; }

   /** @brief Ação diferencial aplicada, já com amortecimento e saturação. */
   float action() const { return action_; }

   /** @brief Velocidade base em uso, já interpolada pelo regime. */
   float baseSpeed() const { return base_speed_; }

   /** @brief Erro normalizado em [-1, 1] usado no ciclo. */
   float normalizedError() const { return normalized_error_; }

   /** @brief true enquanto a linha está fora do array. */
   bool lineLost() const { return line_lost_; }

   /**
    * @brief Tempo contínuo sem linha, em ms.
    *
    * Quem decide o que fazer ao estourar cfg::LINE_LOST_TIMEOUT_MS é o
    * supervisor, na etapa 4 — a malha apenas reporta.
    */
   uint32_t lineLostMs() const { return line_lost_ms_; }

   /** @brief true se a mixagem teve de reduzir a diferença entre os lados. */
   bool mixSaturated() const { return mix_saturated_; }

   /*****************************************
    * Introspecção dos submódulos
    *****************************************/

   const PidAlgorithm&    pid() const { return pid_; }
   const GyroDamper&      damper() const { return damper_; }
   const TrackClassifier& classifier() const { return classifier_; }

 private:
   /** @brief Mixa a ação nos dois lados preservando a diferença. */
   void mix(float base, float action);

   PidAlgorithm    pid_;
   GyroDamper      damper_;
   TrackClassifier classifier_;

   float left_ = 0.0f;
   float right_ = 0.0f;
   float action_ = 0.0f;
   float base_speed_ = 0.0f;
   float normalized_error_ = 0.0f;

   bool     line_lost_ = false;
   uint32_t line_lost_ms_ = 0;
   bool     mix_saturated_ = false;
};

#endif  // CONTROL_LINE_FOLLOWER_H_
