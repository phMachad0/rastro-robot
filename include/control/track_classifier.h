/**
 * @file track_classifier.h
 *
 * @brief Classificador reta × curva e interpolação dos ganhos.
 *
 * Um seguidor de linha com ganho único é ou lento na reta ou instável na curva.
 * Com o giroscópio dá para saber em qual regime o robô está e trocar os
 * parâmetros: em reta, mais velocidade, menos Kp e mais amortecimento; em curva,
 * o inverso.
 *
 * Duas proteções contra o classificador virar fonte de instabilidade:
 *
 * - **Histerese** — dois limiares, não um. Com um único limiar, |ωz| oscilando
 *   em torno dele alternaria os regimes a cada ciclo.
 * - **Interpolação** — a troca de ganhos é distribuída ao longo de
 *   cfg::REGIME_BLEND_MS. Degrau de ganho em malha fechada gera solavanco.
 *
 * Ver docs/ARQUITETURA.md, seção 5.4.
 */

#ifndef CONTROL_TRACK_CLASSIFIER_H_
#define CONTROL_TRACK_CLASSIFIER_H_

#include <stdint.h>

/** @brief Regime de pista em que o robô se encontra. */
enum class TrackRegime : uint8_t {
   STRAIGHT,
   CURVE,
};

/**
 * @brief Conjunto de parâmetros que muda com o regime.
 */
struct ControlGains {
   float kp;
   float ki;
   float kd;
   float kg;
   float base_speed;
};

class TrackClassifier {
 public:
   /** @brief Zera a média de |ωz| e volta ao regime de curva. */
   void reset();

   /**
    * @brief Atualiza a classificação.
    *
    * @param yaw_rate_dps Taxa de guinada em °/s. Só o módulo é usado, então a
    *                     orientação do sensor no chassi não importa aqui.
    * @param dt           Passo de tempo em segundos.
    */
   void update(float yaw_rate_dps, float dt);

   /** @brief Regime alvo corrente, já com histerese aplicada. */
   TrackRegime regime() const { return regime_; }

   /**
    * @brief Posição na interpolação entre os dois conjuntos: 0 = reta, 1 = curva.
    *
    * Caminha em direção ao regime alvo, não salta para ele.
    */
   float blend() const { return blend_; }

   /** @brief Ganhos interpolados para o estado corrente. */
   ControlGains gains() const;

   /** @brief Média filtrada de |ωz| em °/s, a entrada da decisão. */
   float averageRate() const { return average_rate_; }

 private:
   /**
    * @brief Regime alvo.
    *
    * Nasce em CURVE de propósito: é o conjunto conservador, com menos velocidade
    * base. Partir do regime de reta faria o robô sair acelerado antes de ter
    * qualquer medida de |ωz| para justificar isso.
    */
   TrackRegime regime_ = TrackRegime::CURVE;

   float blend_ = 1.0f;
   float average_rate_ = 0.0f;
   bool  primed_ = false;
};

#endif  // CONTROL_TRACK_CLASSIFIER_H_
