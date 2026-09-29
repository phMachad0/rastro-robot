/**
 * @file test_ir.cpp
 *
 * @brief Leitura bruta dos canais do QTR-8A.
 *
 * Lê os canais analógicos com analogRead() e imprime no formato do Serial
 * Plotter: `rótulo:valor` separado por tabulação, uma amostra por linha.
 *
 * Ligação (ESP32 DevKit v1):
 *   VCC   -> 3V3 (NÃO usar 5V: o ADC do ESP32 suporta no máximo ~3.3 V)
 *   GND   -> GND
 *   LEDON -> 3V3 ou desconectado (emissores sempre ligados)
 *   canais-> pinos de pins::IR_CHANNELS
 *
 * @note Este teste usava oito canais, quatro deles no ADC2. O ADC2 fica
 *       inutilizável com o Wi-Fi ligado, e o Wi-Fi é requisito do projeto
 *       (RF05) — por isso o antigo `WiFi.mode(WIFI_OFF)` era necessário aqui.
 *       Com todos os seis canais em ADC1, a leitura convive com o rádio ligado
 *       e o desligamento do Wi-Fi deixou de ser preciso.
 *       Ver docs/ARQUITETURA.md, seção 2.
 *
 * @note A calibração e o cálculo de posição da linha chegam na etapa 2, com a
 *       classe IrArray. Aqui a leitura é crua de propósito: serve para
 *       conferir ligação, alcance e ruído de cada canal.
 */

#include "test/test_ir.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/robot_config.h"

namespace {

uint16_t sensor_values[pins::IR_CHANNEL_COUNT];

constexpr uint32_t REFRESH_MS = 100;

}  // namespace

void testIrSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   /* ADC de 12 bits (0 a 4095) com atenuação para a faixa completa de 0 a 3.3 V. */
   analogReadResolution(12);
   analogSetAttenuation(ADC_11db);

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      pinMode(pins::IR_CHANNELS[i], INPUT);
   }

   Serial.println();
   Serial.printf("Teste do array IR: %u canais em ADC1\n", pins::IR_CHANNEL_COUNT);
   Serial.println("Formato do Serial Plotter: S0..S5, de 0 (claro / linha) a 4095 (escuro / fundo)");
}

void testIrLoop()
{
   /* Leitura bruta: valores altos significam superfície escura (pouca
      reflexão) e valores baixos, superfície clara (muita reflexão). Como a
      pista é linha branca sobre fundo escuro, a linha aparece como um vale. */
   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      uint32_t sum = 0;

      for (uint8_t s = 0; s < cfg::IR_SAMPLES_PER_CHANNEL; s++) {
         sum += analogRead(pins::IR_CHANNELS[i]);
      }

      sensor_values[i] = sum / cfg::IR_SAMPLES_PER_CHANNEL;
   }

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      Serial.printf("S%u:%u%c", i, sensor_values[i],
                    i < pins::IR_CHANNEL_COUNT - 1 ? '\t' : '\n');
   }

   delay(REFRESH_MS);
}
