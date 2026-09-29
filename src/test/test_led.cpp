/**
 * @file test_led.cpp
 *
 * @brief Pisca o LED da placa.
 *
 * O teste mais simples da suíte, e o primeiro a rodar quando algo dá errado:
 * prova que a placa está sendo gravada e que o firmware está executando, antes
 * de suspeitar de qualquer periférico.
 *
 * Visualização: o próprio LED.
 */

#include "test/test_led.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/robot_config.h"

void testLedSetup()
{
   pinMode(pins::LED_STATUS, OUTPUT);

   Serial.begin(cfg::SERIAL_BAUD);
   Serial.println();
   Serial.printf("Teste de LED iniciado no GPIO%u\n", pins::LED_STATUS);
}

void testLedLoop()
{
   digitalWrite(pins::LED_STATUS, HIGH);
   delay(1000);
   digitalWrite(pins::LED_STATUS, LOW);
   delay(1000);
}
