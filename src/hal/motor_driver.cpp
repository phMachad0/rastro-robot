/**
 * @file motor_driver.cpp
 *
 * @brief Implementação do MotorDriver sobre o periférico LEDC do ESP32.
 *
 * @note Usa a API de LEDC do core Arduino-ESP32 **2.x** (`ledcSetup`,
 *       `ledcAttachPin`, `ledcWrite`). O core 3.x trocou por `ledcAttach`,
 *       então uma atualização de plataforma exige revisar begin().
 */

#include "hal/motor_driver.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/robot_config.h"

namespace {

/** @brief Canais do LEDC reservados para a tração. */
constexpr uint8_t LEDC_CH_LEFT  = 0;
constexpr uint8_t LEDC_CH_RIGHT = 1;

/** @brief Duty máximo para a resolução configurada. */
constexpr uint32_t DUTY_MAX = (1u << cfg::MOTOR_PWM_RESOLUTION_BITS) - 1;

/**
 * @brief Abaixo disso a velocidade é tratada como zero.
 *
 * Sem esse piso, um resíduo numérico do PID manteria o motor energizado no
 * duty da zona morta — que é alto — zumbindo sem girar e consumindo corrente.
 */
constexpr float SPEED_EPSILON = 0.005f;

float clampf(float v, float lo, float hi)
{
   return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

void MotorDriver::begin()
{
   const uint8_t direction_pins[] = {
       pins::MOTOR_L_IN1, pins::MOTOR_L_IN2,
       pins::MOTOR_R_IN3, pins::MOTOR_R_IN4,
   };

   for (uint8_t i = 0; i < sizeof(direction_pins); i++) {
      pinMode(direction_pins[i], OUTPUT);
      digitalWrite(direction_pins[i], LOW);
   }

   ledcSetup(LEDC_CH_LEFT, cfg::MOTOR_PWM_FREQ_HZ, cfg::MOTOR_PWM_RESOLUTION_BITS);
   ledcSetup(LEDC_CH_RIGHT, cfg::MOTOR_PWM_FREQ_HZ, cfg::MOTOR_PWM_RESOLUTION_BITS);

   ledcAttachPin(pins::MOTOR_L_PWM, LEDC_CH_LEFT);
   ledcAttachPin(pins::MOTOR_R_PWM, LEDC_CH_RIGHT);

   trim_ = cfg::MOTOR_TRIM;

   /* Desarmado por padrão: o robô só se move depois de um arm() explícito. */
   armed_ = false;
   stop(StopMode::COAST);
}

void MotorDriver::arm()
{
   applied_left_ = 0.0f;
   applied_right_ = 0.0f;

   armed_ = true;
}

void MotorDriver::disarm()
{
   armed_ = false;
   stop(StopMode::COAST);
}

void MotorDriver::setTrim(float trim)
{
   trim_ = clampf(trim, -1.0f, 1.0f);
}

void MotorDriver::stop(StopMode mode)
{
   const bool brake = (mode == StopMode::BRAKE);
   const uint8_t level = brake ? HIGH : LOW;

   digitalWrite(pins::MOTOR_L_IN1, level);
   digitalWrite(pins::MOTOR_L_IN2, level);
   digitalWrite(pins::MOTOR_R_IN3, level);
   digitalWrite(pins::MOTOR_R_IN4, level);

   /* A frenagem do L298N é por curto-circuito do enrolamento: as duas entradas
      de direção no mesmo nível E a habilitação ativa. Com EN em zero a ponte
      fica em alta impedância e o motor gira livre, que é justamente o oposto —
      por isso o duty vai ao máximo no modo de frenagem. */
   const uint32_t duty = brake ? DUTY_MAX : 0;

   ledcWrite(LEDC_CH_LEFT, duty);
   ledcWrite(LEDC_CH_RIGHT, duty);

   applied_left_ = 0.0f;
   applied_right_ = 0.0f;
}

uint32_t MotorDriver::speedToDuty(float speed) const
{
   const float magnitude = fabsf(speed);

   if (magnitude < SPEED_EPSILON) {
      return 0;
   }

   /* Remapeia [0, 1] para [zona morta, 1]: a velocidade comandada passa a
      significar fração da faixa que realmente produz movimento. Sem isso,
      tudo abaixo de cfg::MOTOR_DEADZONE seria comando perdido — e com a zona
      morta medida em 0.65, isso seriam dois terços da faixa. */
   const float duty = cfg::MOTOR_DEADZONE + (1.0f - cfg::MOTOR_DEADZONE) * magnitude;

   return (uint32_t)(clampf(duty, 0.0f, 1.0f) * DUTY_MAX);
}

void MotorDriver::applySide(uint8_t channel, uint8_t in_a, uint8_t in_b, float speed,
                            bool reversed)
{
   const uint32_t duty = speedToDuty(speed);

   if (duty == 0) {
      /* Parada por roda livre: as duas entradas em nível baixo. */
      digitalWrite(in_a, LOW);
      digitalWrite(in_b, LOW);
      ledcWrite(channel, 0);
      return;
   }

   /* XOR entre o sinal do comando e a inversão do lado: é o que traduz
      "positivo = frente do robô" para a polaridade que aquele lado precisa. */
   const bool forward = (speed > 0.0f) != reversed;

   digitalWrite(in_a, forward ? HIGH : LOW);
   digitalWrite(in_b, forward ? LOW : HIGH);
   ledcWrite(channel, duty);
}

void MotorDriver::setSpeed(float left, float right)
{
   if (!armed_) {
      stop(StopMode::COAST);
      return;
   }

   /* Trim: em vez de somar um offset, que mudaria o ponto de parada e faria o
      robô andar sozinho com comando zero, atenua o lado mais rápido. */
   if (trim_ > 0.0f) {
      left *= (1.0f - trim_);
   } else if (trim_ < 0.0f) {
      right *= (1.0f + trim_);
   }

   applied_left_  = clampf(left, -cfg::MAX_SPEED, cfg::MAX_SPEED);
   applied_right_ = clampf(right, -cfg::MAX_SPEED, cfg::MAX_SPEED);

   /* Mapeia velocidade lógica para canal de hardware. É o último passo de
      propósito: tudo acima dele — saturação, trim, telemetria — trabalha em
      lados lógicos, que são os lados físicos do robô. Só aqui o fato de os
      canais do driver atenderem os lados trocados entra na conta. */
   const float cmd_channel_l =
       cfg::MOTOR_SIDES_SWAPPED ? applied_right_ : applied_left_;
   const float cmd_channel_r =
       cfg::MOTOR_SIDES_SWAPPED ? applied_left_ : applied_right_;

   applySide(LEDC_CH_LEFT, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, cmd_channel_l,
             cfg::MOTOR_L_REVERSED);
   applySide(LEDC_CH_RIGHT, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4, cmd_channel_r,
             cfg::MOTOR_R_REVERSED);
}
