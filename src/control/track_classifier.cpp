/**
 * @file track_classifier.cpp
 *
 * @brief Implementação do TrackClassifier.
 */

#include "control/track_classifier.h"

#include <math.h>

#include "config/robot_config.h"

namespace {

float clampf(float v, float lo, float hi)
{
   return v < lo ? lo : (v > hi ? hi : v);
}

float lerp(float a, float b, float t)
{
   return a + (b - a) * t;
}

}  // namespace

void TrackClassifier::reset()
{
   regime_ = TrackRegime::CURVE;
   blend_ = 1.0f;
   average_rate_ = 0.0f;
   primed_ = false;
}

void TrackClassifier::update(float yaw_rate_dps, float dt)
{
   if (dt <= 0.0f) {
      return;
   }

   const float magnitude = fabsf(yaw_rate_dps);

   /*****************************************
    * Média de |ωz|
    *
    * Filtro exponencial com constante de tempo de cfg::REGIME_WINDOW_MS, em vez
    * de média móvel com fila. Para esta finalidade os dois se comportam
    * praticamente igual, e o exponencial não precisa de buffer dimensionado pela
    * taxa de controle — trocar CONTROL_HZ não muda a janela de tempo efetiva,
    * que é o que se quer especificar.
    *****************************************/
   const float tau = cfg::REGIME_WINDOW_MS / 1000.0f;
   const float alpha = dt / (tau + dt);

   if (primed_) {
      average_rate_ += alpha * (magnitude - average_rate_);
   } else {
      average_rate_ = magnitude;
      primed_ = true;
   }

   /*****************************************
    * Histerese
    *
    * Só troca de regime quando cruza o limiar do lado oposto. Entre os dois
    * limiares, mantém o regime corrente.
    *****************************************/
   if (average_rate_ > cfg::GYRO_TH_HIGH) {
      regime_ = TrackRegime::CURVE;
   } else if (average_rate_ < cfg::GYRO_TH_LOW) {
      regime_ = TrackRegime::STRAIGHT;
   }

   /*****************************************
    * Interpolação
    *
    * O blend caminha em direção ao alvo a uma taxa tal que a travessia completa
    * leve cfg::REGIME_BLEND_MS.
    *****************************************/
   const float target = (regime_ == TrackRegime::CURVE) ? 1.0f : 0.0f;
   const float step = dt / (cfg::REGIME_BLEND_MS / 1000.0f);

   if (blend_ < target) {
      blend_ = clampf(blend_ + step, 0.0f, target);
   } else if (blend_ > target) {
      blend_ = clampf(blend_ - step, target, 1.0f);
   }
}

ControlGains TrackClassifier::gains() const
{
   ControlGains g;

   g.kp = lerp(cfg::KP_STRAIGHT, cfg::KP_CURVE, blend_);
   g.ki = lerp(cfg::KI_STRAIGHT, cfg::KI_CURVE, blend_);
   g.kd = lerp(cfg::KD_STRAIGHT, cfg::KD_CURVE, blend_);
   g.kg = lerp(cfg::KG_STRAIGHT, cfg::KG_CURVE, blend_);

   /* Em reta o robô anda mais rápido; a velocidade base acompanha o regime pela
      mesma interpolação, para não dar degrau de aceleração na transição. */
   g.base_speed = lerp(cfg::BASE_SPEED, cfg::BASE_SPEED_CURVE, blend_);

   return g;
}
