/**
 * @file test_motors.cpp
 *
 * @brief Teste dos dois lados de tração através do MotorDriver.
 *
 * Exercita a classe pela mesma interface que a malha de controle vai usar —
 * velocidade efetiva em [-1, 1] —, não o PWM cru. Assim o que este teste
 * valida é o caminho real: compensação de zona morta, trim e trava de
 * armamento.
 *
 * O que ele responde:
 *   1. Qual lado físico responde a qual canal do driver;
 *   2. Se o sentido de rotação bate com o esperado em cada lado;
 *   3. Se a compensação de zona morta está correta — na varredura em degraus, a
 *      roda deve sair da inércia nos primeiros degraus. Se só girar perto de
 *      0,6, a compensação não está sendo aplicada ou cfg::MOTOR_DEADZONE está
 *      subestimada;
 *   4. Se os dois lados andam igual no mesmo comando (o fator MOTOR_TRIM).
 *
 * @warning Rodar com o robô suspenso, com as rodas sem tocar a bancada.
 *
 * Visualização: séries `cmdL`, `cmdR`, `appliedL` e `appliedR` no Teleplot.
 * `cmd` e `applied` só divergem quando a saturação em cfg::MAX_SPEED ou o trim
 * entram em ação — o driver não filtra nem atrasa o comando.
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

/** @brief Comanda os dois lados e mantém por um tempo. */
void hold(const char* label, float left, float right, uint32_t duration_ms)
{
   motors.setSpeed(left, right);

   Serial.printf("-- %s (L=%.2f R=%.2f) --\n", label, left, right);

   teleplot::send("cmdL", left);
   teleplot::send("cmdR", right);
   teleplot::send("appliedL", motors.appliedLeft());
   teleplot::send("appliedR", motors.appliedRight());

   delay(duration_ms);
}

/**
 * @brief Sobe a velocidade em degraus num lado, para achar o ponto de partida.
 */
void stepSide(const char* label, bool left_side)
{
   Serial.printf("%s: degraus de velocidade — anote onde a roda comeca a girar\n", label);

   for (float speed = 0.0f; speed <= cfg::MAX_SPEED; speed += 0.05f) {
      motors.setSpeed(left_side ? speed : 0.0f, left_side ? 0.0f : speed);

      teleplot::send("speedCmd", speed);
      teleplot::send("appliedL", motors.appliedLeft());
      teleplot::send("appliedR", motors.appliedRight());

      Serial.printf("  velocidade = %.2f\n", speed);
      delay(500);
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

   stepSide("Esquerda", true);
   stepSide("Direita ", false);
}
