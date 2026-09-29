/**
 * @file line_follower.cpp
 *
 * @brief Implementação do LineFollower.
 */

#include "control/line_follower.h"

#include "config/features.h"
#include "config/robot_config.h"

namespace {

float clampf(float v, float lo, float hi)
{
   return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

void LineFollower::reset()
{
   pid_.reset();
   pid_.setOutputLimit(cfg::PID_OUTPUT_LIMIT);
   pid_.setIntegralLimit(cfg::PID_INTEGRAL_LIMIT);

   classifier_.reset();

   left_ = 0.0f;
   right_ = 0.0f;
   action_ = 0.0f;
   base_speed_ = 0.0f;
   normalized_error_ = 0.0f;

   line_lost_ = false;
   line_lost_ms_ = 0;
   mix_saturated_ = false;
}

void LineFollower::mix(float base, float action)
{
   float left = base + action;
   float right = base - action;

   /*****************************************
    * Saturação que preserva a diferença
    *
    * Cortar cada lado no seu limite, independentemente, reduziria a diferença
    * entre eles — e a diferença é exatamente a curva que o controle pediu. Em
    * vez disso os dois lados são deslocados juntos, o que sacrifica velocidade
    * para manter a curva.
    *****************************************/
   const float highest = (left > right) ? left : right;
   const float lowest = (left < right) ? left : right;

   if (highest > cfg::MAX_SPEED) {
      const float excess = highest - cfg::MAX_SPEED;
      left -= excess;
      right -= excess;
   } else if (lowest < -cfg::MAX_SPEED) {
      const float deficit = -cfg::MAX_SPEED - lowest;
      left += deficit;
      right += deficit;
   }

   /* Se a ação pedida é maior que a faixa inteira, não há deslocamento que
      preserve a diferença — aí o corte é inevitável e fica sinalizado. */
   left_ = clampf(left, -cfg::MAX_SPEED, cfg::MAX_SPEED);
   right_ = clampf(right, -cfg::MAX_SPEED, cfg::MAX_SPEED);

   mix_saturated_ = (left_ != left) || (right_ != right);
}

void LineFollower::update(int16_t line_error, bool line_detected, float yaw_rate_dps,
                          float dt)
{
   if (dt <= 0.0f) {
      return;
   }

   /* O classificador continua rodando mesmo sem linha: |ωz| é medida do chassi
      e não depende do array ver a pista. */
   classifier_.update(yaw_rate_dps, dt);

   const ControlGains gains = classifier_.gains();

#if FEATURE_ADAPTIVE_GAINS
   pid_.setGains(gains.kp, gains.ki, gains.kd);
   damper_.setGain(gains.kg);
   base_speed_ = gains.base_speed;
#else
   /* Sem adaptação, usa o conjunto de curva: é o conservador, e é o que se quer
      enquanto o PID base ainda não está sintonizado. */
   pid_.setGains(cfg::KP_CURVE, cfg::KI_CURVE, cfg::KD_CURVE);
   damper_.setGain(cfg::KG_CURVE);
   base_speed_ = cfg::BASE_SPEED_CURVE;
#endif

   normalized_error_ = line_error / cfg::LINE_ERROR_SCALE;

   if (line_detected) {
      line_lost_ = false;
      line_lost_ms_ = 0;

      pid_.holdIntegral(false);

      float action = pid_.update(normalized_error_, dt);

#if FEATURE_GYRO_DAMPING
      action += damper_.term(yaw_rate_dps);
      action = clampf(action, -cfg::PID_OUTPUT_LIMIT, cfg::PID_OUTPUT_LIMIT);
#else
      (void)damper_.term(yaw_rate_dps);  /* mantém a telemetria alimentada */
#endif

      action_ = action;

      mix(base_speed_, action_);
      return;
   }

   /*****************************************
    * Linha perdida — seção 5.5
    *
    * A ação diferencial é congelada no último valor conhecido: o robô estava
    * virando para algum lado quando perdeu a linha, então continua virando para
    * lá. A velocidade base cai, o que dá mais tempo de busca dentro do mesmo
    * timeout e reduz a chance de se afastar de vez da pista.
    *
    * O integrador é congelado, e não zerado: o IrArray mantém a última posição
    * válida, então o erro se repete a cada ciclo e seria integrado como se fosse
    * informação nova, saturando o integrador em poucos décimos de segundo.
    *****************************************/
   line_lost_ = true;
   line_lost_ms_ += (uint32_t)(dt * 1000.0f);

   pid_.holdIntegral(true);

   mix(base_speed_ * cfg::LINE_LOST_SPEED_FACTOR, action_);
}
