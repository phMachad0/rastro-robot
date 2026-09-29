/**
 * @file robot_app.cpp
 *
 * @brief Esqueleto da aplicação principal.
 *
 * Existe para que o ambiente `firmware` compile e rode desde a etapa 1,
 * servindo de âncora para as etapas seguintes. Ainda não cria nenhuma tarefa
 * e não aciona os motores.
 *
 * Ver docs/ARQUITETURA.md, seção 11 (roteiro de implementação).
 */

#include "app/robot_app.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/features.h"
#include "config/robot_config.h"

void robotAppSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   /* Os pinos de direção do L298N ficam em nível baixo e o PWM em zero, para
      garantir que os motores não se movam enquanto a aplicação não existe. */
   const uint8_t motor_pins[] = {
       pins::MOTOR_L_PWM, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2,
       pins::MOTOR_R_PWM, pins::MOTOR_R_IN3, pins::MOTOR_R_IN4,
   };

   for (uint8_t i = 0; i < sizeof(motor_pins); i++) {
      pinMode(motor_pins[i], OUTPUT);
      digitalWrite(motor_pins[i], LOW);
   }

   pinMode(pins::LED_STATUS, OUTPUT);

   Serial.println();
   Serial.println("=== rastro-robot ===");
   Serial.println("Etapa 1: fundacao. Aplicacao principal ainda nao implementada.");
   Serial.printf("Canais IR: %u | controle: %lu Hz | telemetria: %lu Hz\n",
                 pins::IR_CHANNEL_COUNT,
                 (unsigned long)cfg::CONTROL_HZ,
                 (unsigned long)cfg::TELEMETRY_HZ);
   Serial.printf("Marcacao lateral (RF02): %s\n",
                 FEATURE_LATERAL_MARKS ? "habilitada" : "desabilitada");
   Serial.println("Use os ambientes de teste: pio run -e test_ir -t upload");
}

void robotAppLoop()
{
   /* Pulso lento no LED apenas para indicar que a placa está viva. */
   digitalWrite(pins::LED_STATUS, HIGH);
   delay(100);
   digitalWrite(pins::LED_STATUS, LOW);
   delay(900);
}
