/**
 * @file gyro_damper.cpp
 *
 * @brief Implementação do GyroDamper.
 */

#include "control/gyro_damper.h"

#include "config/robot_config.h"

float GyroDamper::term(float yaw_rate_dps)
{
   /* A orientação do sensor no chassi já foi resolvida na classe Imu, então aqui
      a taxa chega no referencial do robô: positiva é guinada no sentido horário.
      Há um único lugar que define esse sinal. */
   normalized_rate_ = yaw_rate_dps / cfg::GYRO_RATE_REFERENCE;

   /* Sinal negativo: é o que torna o termo realimentação negativa da rotação. */
   last_term_ = -kg_ * normalized_rate_;

   return last_term_;
}
