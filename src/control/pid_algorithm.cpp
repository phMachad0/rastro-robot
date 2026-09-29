/**
 * @file pid_algorithm.cpp
 *
 * @brief Implementação do PidAlgorithm.
 */

#include "control/pid_algorithm.h"

namespace {

float clampf(float v, float lo, float hi)
{
   return v < lo ? lo : (v > hi ? hi : v);
}

bool sameSign(float a, float b)
{
   return (a >= 0.0f) == (b >= 0.0f);
}

}  // namespace

PidAlgorithm::PidAlgorithm()
{
   reset();
}

void PidAlgorithm::setGains(float kp, float ki, float kd)
{
   kp_ = kp;
   ki_ = ki;
   kd_ = kd;
}

void PidAlgorithm::setOutputLimit(float limit)
{
   output_limit_ = limit;
}

void PidAlgorithm::setIntegralLimit(float limit)
{
   integral_limit_ = limit;
}

void PidAlgorithm::reset()
{
   integral_ = 0.0f;
   integral_held_ = false;

   for (uint8_t i = 0; i < cfg::PID_DERIVATIVE_WINDOW; i++) {
      error_history_[i] = 0.0f;
   }

   history_index_ = 0;
   history_filled_ = 0;

   p_term_ = 0.0f;
   i_term_ = 0.0f;
   d_term_ = 0.0f;

   output_ = 0.0f;
   saturated_ = false;
}

float PidAlgorithm::update(float error, float dt)
{
   if (dt <= 0.0f) {
      return output_;
   }

   /*****************************************
    * Derivada de janela longa
    *
    * A diferença é tomada contra o erro mais antigo da fila, e dividida pelo
    * tempo que a janela cobre de fato. Enquanto a fila não encheu, usa-se o
    * trecho já preenchido, senão os primeiros ciclos depois de um reset
    * derivariam contra zeros e produziriam um pico de partida.
    *****************************************/
   float derivative = 0.0f;

   if (history_filled_ > 0) {
      const uint8_t oldest_index =
          (uint8_t)((history_index_ + cfg::PID_DERIVATIVE_WINDOW - history_filled_) %
                    cfg::PID_DERIVATIVE_WINDOW);

      const float window_seconds = history_filled_ * dt;
      derivative = (error - error_history_[oldest_index]) / window_seconds;
   }

   error_history_[history_index_] = error;
   history_index_ = (uint8_t)((history_index_ + 1) % cfg::PID_DERIVATIVE_WINDOW);

   if (history_filled_ < cfg::PID_DERIVATIVE_WINDOW) {
      history_filled_++;
   }

   /*****************************************
    * Termos e anti-windup
    *
    * A integral é calculada como candidata e só é confirmada se não estiver
    * empurrando uma saída já saturada ainda mais para o limite. Esse é o
    * anti-windup por travamento condicional: sem ele, o robô que perde a linha
    * numa curva acumula integral durante toda a saturação e sai girando quando
    * o erro finalmente inverte.
    *****************************************/
   const float integral_candidate =
       integral_held_ ? integral_
                      : clampf(integral_ + error * dt, -integral_limit_, integral_limit_);

   p_term_ = kp_ * error;
   i_term_ = ki_ * integral_candidate;
   d_term_ = kd_ * derivative;

   const float unsaturated = p_term_ + i_term_ + d_term_;

   output_ = clampf(unsaturated, -output_limit_, output_limit_);
   saturated_ = (output_ != unsaturated);

   const bool pushing_further = saturated_ && sameSign(error, unsaturated);

   if (!pushing_further) {
      integral_ = integral_candidate;
   } else {
      /* O integrador congela, mas o termo relatado passa a refletir o valor
         que de fato entrou na conta, para a telemetria não mentir. */
      i_term_ = ki_ * integral_;
   }

   return output_;
}
