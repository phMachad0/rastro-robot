/**
 * @file test_motors.cpp
 *
 * @brief Teste dos dois lados de tração via driver L298N.
 *
 * Os dois motores de cada lado compartilham o mesmo canal do driver, então o
 * teste exercita um lado de cada vez, não um motor de cada vez.
 *
 * O que este teste precisa responder antes da etapa 2:
 *   1. Qual lado físico responde a qual canal do driver;
 *   2. Se o sentido de rotação bate com o esperado em cada lado;
 *   3. Qual o duty mínimo em que cada lado sai da inércia (a zona morta de
 *      cfg::MOTOR_DEADZONE) e se os dois lados andam igual no mesmo duty
 *      (o fator cfg::MOTOR_TRIM).
 *
 * @warning Rodar com o robô suspenso, com as rodas sem tocar a bancada.
 *
 * Visualização: Teleplot. A varredura emite a série `duty`, de forma que dá
 * para marcar no gráfico o instante em que cada roda começou a girar e ler o
 * valor correspondente — é assim que se mede cfg::MOTOR_DEADZONE. O restante
 * do teste é texto no painel de log.
 *
 * @note Ainda usa analogWrite. A etapa 2 substitui isto pelo MotorDriver
 *       sobre LEDC, com controle de frequência e resolução.
 */

#include "test/test_motors.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/robot_config.h"
#include "test/teleplot.h"

namespace {

/** @brief Resolução do analogWrite do core Arduino, de 0 a 255. */
constexpr uint8_t PWM_MAX = 255;

constexpr uint32_t STEP_MS  = 2000;
constexpr uint32_t PAUSE_MS = 1000;

/**
 * @brief Aciona um lado da tração.
 *
 * @param pwm_pin Pino de habilitação do canal (ENA ou ENB).
 * @param in_a    Primeiro pino de direção do canal.
 * @param in_b    Segundo pino de direção do canal.
 * @param duty    Velocidade de 0 a 255.
 * @param forward true para frente, false para ré.
 */
void driveSide(uint8_t pwm_pin, uint8_t in_a, uint8_t in_b,
               uint8_t duty, bool forward)
{
   digitalWrite(in_a, forward ? HIGH : LOW);
   digitalWrite(in_b, forward ? LOW : HIGH);
   analogWrite(pwm_pin, duty);
}

void stopSide(uint8_t pwm_pin, uint8_t in_a, uint8_t in_b)
{
   digitalWrite(in_a, LOW);
   digitalWrite(in_b, LOW);
   analogWrite(pwm_pin, 0);
}

/**
 * @brief Varre o duty de 0 a 255 num lado, para localizar a zona morta.
 */
void sweepSide(const char* label, uint8_t pwm_pin, uint8_t in_a, uint8_t in_b)
{
   Serial.printf("%s: varredura de duty — anote onde a roda comeca a girar\n", label);

   digitalWrite(in_a, HIGH);
   digitalWrite(in_b, LOW);

   for (uint16_t duty = 0; duty <= PWM_MAX; duty += 15) {
      analogWrite(pwm_pin, duty);
      teleplot::send("duty", (int32_t)duty);
      teleplot::send("dutyPercent", 100.0f * duty / PWM_MAX, "%");
      delay(400);
   }

   teleplot::send("duty", (int32_t)0);

   stopSide(pwm_pin, in_a, in_b);
   delay(PAUSE_MS);
}

}  // namespace

void testMotorsSetup()
{
   const uint8_t outputs[] = {
       pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2,
       pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4,
   };

   for (uint8_t i = 0; i < sizeof(outputs); i++) {
      pinMode(outputs[i], OUTPUT);
      digitalWrite(outputs[i], LOW);
   }

   Serial.begin(cfg::SERIAL_BAUD);
   Serial.println();
   Serial.println("=== Teste de motores (L298N) ===");
   Serial.printf("Esquerda: ENA=GPIO%u IN1=GPIO%u IN2=GPIO%u\n",
                 pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2);
   Serial.printf("Direita : ENB=GPIO%u IN3=GPIO%u IN4=GPIO%u\n",
                 pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4);
   Serial.println("ATENCAO: manter o robo suspenso, com as rodas livres.");
}

void testMotorsLoop()
{
   Serial.println("\n-- Lado esquerdo, frente --");
   driveSide(pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, PWM_MAX, true);
   delay(STEP_MS);

   Serial.println("-- Lado esquerdo, re --");
   driveSide(pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, PWM_MAX, false);
   delay(STEP_MS);

   Serial.println("-- Lado esquerdo, parado --");
   stopSide(pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2);
   delay(PAUSE_MS);

   Serial.println("-- Lado direito, frente --");
   driveSide(pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4, PWM_MAX, true);
   delay(STEP_MS);

   Serial.println("-- Lado direito, re --");
   driveSide(pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4, PWM_MAX, false);
   delay(STEP_MS);

   Serial.println("-- Lado direito, parado --");
   stopSide(pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4);
   delay(PAUSE_MS);

   sweepSide("Esquerda", pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2);
   sweepSide("Direita ", pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4);
}
