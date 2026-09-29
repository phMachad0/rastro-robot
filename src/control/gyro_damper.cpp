/**
 * @file gyro_damper.cpp
 *
 * @brief Implementação do GyroDamper.
 */

#include "control/gyro_damper.h"

#include "config/robot_config.h"

float GyroDamper::term(float yaw_rate_dps)
{
   const float oriented = cfg::GYRO_Z_REVERSED ? -yaw_rate_dps : yaw_rate_dps;

   normalized_rate_ = oriented / cfg::GYRO_RATE_REFERENCE;

   /* Sinal negativo: é o que torna o termo realimentação negativa da rotação. */
   last_term_ = -kg_ * normalized_rate_;

   return last_term_;
}
