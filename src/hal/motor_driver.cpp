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

/**
 * @brief Intervalo acima do qual o dt medido é descartado.
 *
 * Um intervalo muito longo significa que a malha ficou parada (boot, depuração,
 * ponto de parada no debugger). Usar esse dt daria uma rampa de passo enorme,
 * que é justamente o degrau de corrente que se quer evitar.
 */
constexpr uint32_t DT_SANITY_LIMIT_US = 200000;

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
   /* Zera a rampa antes de liberar a saída, para que o primeiro setSpeed()
      depois do arm() parta do repouso em vez de saltar para a última
      velocidade comandada antes do desarme. */
   applied_left_ = 0.0f;
   applied_right_ = 0.0f;
   last_update_us_ = micros();

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
   last_update_us_ = micros();
}

float MotorDriver::slew(float current, float target, float max_delta)
{
   const float delta = target - current;

   if (delta > max_delta) {
      return current + max_delta;
   }
   if (delta < -max_delta) {
      return current - max_delta;
   }

   return target;
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

void MotorDriver::applySide(uint8_t channel, uint8_t in_a, uint8_t in_b, float speed)
{
   const uint32_t duty = speedToDuty(speed);

   if (duty == 0) {
      /* Parada por roda livre: as duas entradas em nível baixo. */
      digitalWrite(in_a, LOW);
      digitalWrite(in_b, LOW);
      ledcWrite(channel, 0);
      return;
   }

   const bool forward = speed > 0.0f;

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

   left  = clampf(left, -cfg::MAX_SPEED, cfg::MAX_SPEED);
   right = clampf(right, -cfg::MAX_SPEED, cfg::MAX_SPEED);

   /* dt medido, e não assumido: setSpeed() é chamada pela malha a 250 Hz no
      firmware, mas as rotinas de teste chamam em cadências bem diferentes, e
      uma rampa calibrada em "por ciclo" mudaria de velocidade junto. */
   const uint32_t now_us = micros();
   const uint32_t elapsed_us = now_us - last_update_us_;
   last_update_us_ = now_us;

   const float dt = (elapsed_us > DT_SANITY_LIMIT_US)
                        ? (cfg::CONTROL_PERIOD_MS / 1000.0f)
                        : (elapsed_us / 1000000.0f);

   const float max_delta = cfg::MOTOR_SLEW_PER_S * dt;

   applied_left_  = slew(applied_left_, left, max_delta);
   applied_right_ = slew(applied_right_, right, max_delta);

   applySide(LEDC_CH_LEFT, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, applied_left_);
   applySide(LEDC_CH_RIGHT, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4, applied_right_);
}
