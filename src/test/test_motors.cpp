/**
 * @file test_motors.cpp
 *
 * @brief Teste dos dois lados de tração através do MotorDriver.
 *
 * Exercita a classe pela mesma interface que a malha de controle vai usar —
 * velocidade efetiva em [-1, 1] —, não o PWM cru. Assim o que este teste
 * valida é o caminho real: compensação de zona morta, trim, rampa e trava de
 * armamento.
 *
 * O que ele responde:
 *   1. Qual lado físico responde a qual canal do driver;
 *   2. Se o sentido de rotação bate com o esperado em cada lado;
 *   3. Se a compensação de zona morta está correta — a rampa lenta deve fazer
 *      a roda sair da inércia logo no começo, e não só perto do fim;
 *   4. Se os dois lados andam igual no mesmo comando (o fator MOTOR_TRIM).
 *
 * @warning Rodar com o robô suspenso, com as rodas sem tocar a bancada.
 *
 * Visualização: séries `cmdL`, `cmdR`, `appliedL` e `appliedR` no Teleplot. A
 * distância entre `cmd` e `applied` é a rampa de aceleração agindo.
 */

#include "test/test_motors.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/robot_config.h"
#include "hal/motor_driver.h"
#include "test/teleplot.h"

namespace {

MotorDriver motors;

constexpr uint32_t STEP_MS  = 2000;
constexpr uint32_t PAUSE_MS = 1000;

/** @brief Período com que setSpeed é chamada, imitando a malha de controle. */
constexpr uint32_t TICK_MS = 10;

/**
 * @brief Mantém um comando por um tempo, chamando setSpeed em cadência fixa.
 *
 * A cadência importa: a rampa do MotorDriver é calculada sobre o tempo real
 * entre chamadas, então comandar uma vez e dormir dois segundos nunca deixaria
 * a rampa progredir.
 */
void hold(const char* label, float left, float right, uint32_t duration_ms)
{
   Serial.printf("-- %s (L=%.2f R=%.2f) --\n", label, left, right);

   const uint32_t deadline = millis() + duration_ms;

   while ((int32_t)(millis() - deadline) < 0) {
      motors.setSpeed(left, right);

      teleplot::send("cmdL", left);
      teleplot::send("cmdR", right);
      teleplot::send("appliedL", motors.appliedLeft());
      teleplot::send("appliedR", motors.appliedRight());

      delay(TICK_MS);
   }
}

/**
 * @brief Sobe a velocidade em degraus, para localizar o ponto de partida real.
 *
 * Com a compensação de zona morta funcionando, a roda deve começar a girar
 * logo nos primeiros degraus. Se só girar perto de 0.6, a compensação não está
 * sendo aplicada ou cfg::MOTOR_DEADZONE está subestimada.
 */
void rampSide(const char* label, bool left_side)
{
   Serial.printf("%s: degraus de velocidade — anote onde a roda comeca a girar\n", label);

   for (float speed = 0.0f; speed <= cfg::MAX_SPEED; speed += 0.05f) {
      const uint32_t deadline = millis() + 500;

      while ((int32_t)(millis() - deadline) < 0) {
         motors.setSpeed(left_side ? speed : 0.0f, left_side ? 0.0f : speed);

         teleplot::send("speedCmd", speed);
         teleplot::send("appliedL", motors.appliedLeft());
         teleplot::send("appliedR", motors.appliedRight());

         delay(TICK_MS);
      }

      Serial.printf("  velocidade = %.2f\n", speed);
   }

   motors.stop();
   teleplot::send("speedCmd", 0.0f);
   delay(PAUSE_MS);
}

}  // namespace

void testMotorsSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   motors.begin();

   Serial.println();
   Serial.println("=== Teste de motores (MotorDriver / L298N) ===");
   Serial.printf("Esquerda: ENA=GPIO%u IN1=GPIO%u IN2=GPIO%u\n",
                 pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2);
   Serial.printf("Direita : ENB=GPIO%u IN3=GPIO%u IN4=GPIO%u\n",
                 pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4);
   Serial.printf("PWM: %lu Hz, %u bits | zona morta: %.2f | trim: %.2f\n",
                 (unsigned long)cfg::MOTOR_PWM_FREQ_HZ,
                 cfg::MOTOR_PWM_RESOLUTION_BITS,
                 cfg::MOTOR_DEADZONE, cfg::MOTOR_TRIM);
   Serial.println("ATENCAO: manter o robo suspenso, com as rodas livres.");

   /* Prova a trava de segurança antes de qualquer outra coisa: com o driver
      desarmado, comando de velocidade máxima não pode mover nada. */
   Serial.println("\n-- Trava de seguranca: desarmado, comandando 1.0 --");
   motors.setSpeed(1.0f, 1.0f);
   delay(1500);
   Serial.printf("   aplicado L=%.2f R=%.2f (esperado 0.00)\n",
                 motors.appliedLeft(), motors.appliedRight());

   motors.arm();
   Serial.println("Driver armado.");
}

void testMotorsLoop()
{
   hold("Ambos, frente", cfg::BASE_SPEED, cfg::BASE_SPEED, STEP_MS);
   hold("Ambos, parado", 0.0f, 0.0f, PAUSE_MS);

   hold("Esquerdo, frente", cfg::BASE_SPEED, 0.0f, STEP_MS);
   hold("Esquerdo, re", -cfg::BASE_SPEED, 0.0f, STEP_MS);
   hold("Parado", 0.0f, 0.0f, PAUSE_MS);

   hold("Direito, frente", 0.0f, cfg::BASE_SPEED, STEP_MS);
   hold("Direito, re", 0.0f, -cfg::BASE_SPEED, STEP_MS);
   hold("Parado", 0.0f, 0.0f, PAUSE_MS);

   /* Giro no próprio eixo: é o teste mais sensível ao descasamento entre os
      lados, porque qualquer diferença aparece como deriva do eixo de rotação. */
   hold("Giro no eixo", cfg::BASE_SPEED, -cfg::BASE_SPEED, STEP_MS);
   hold("Parado", 0.0f, 0.0f, PAUSE_MS);

   rampSide("Esquerda", true);
   rampSide("Direita ", false);
}
