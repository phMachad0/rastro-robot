/**
 * @file test_ir.cpp
 *
 * @brief Leitura bruta dos canais do QTR-8A.
 *
 * Lê os canais analógicos com analogRead() e mostra no Serial Monitor como
 * gráfico de barras. Com BAR_CHART em false, imprime no formato do Serial
 * Plotter.
 *
 * O gráfico usa códigos ANSI para redesenhar no mesmo lugar, então o monitor
 * precisa do filtro "direct" (`monitor_filters = direct`, já configurado no
 * ambiente test_ir), senão os códigos aparecem como lixo na tela.
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

/** @brief true = gráfico de barras no terminal, false = formato do Serial Plotter. */
constexpr bool BAR_CHART = true;

constexpr uint8_t  BAR_WIDTH = 50;
constexpr uint16_t ADC_MAX   = 4095;
constexpr uint32_t REFRESH_MS = 100;

void printPlotter()
{
   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      Serial.print("S");
      Serial.print(i);
      Serial.print(":");
      Serial.print(sensor_values[i]);
      Serial.print(i < pins::IR_CHANNEL_COUNT - 1 ? '\t' : '\n');
   }
}

void printBarChart()
{
   /* Monta o quadro inteiro antes de enviar, para reduzir o piscar da tela. */
   String frame = "\033[H";  // cursor para o topo
   frame += "QTR-8A  (0 = claro / linha, 4095 = escuro / fundo)\n\n";

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      uint8_t filled = (uint32_t)sensor_values[i] * BAR_WIDTH / ADC_MAX;

      char label[20];
      snprintf(label, sizeof(label), "S%u (GPIO%2u) ", i, pins::IR_CHANNELS[i]);
      frame += label;

      for (uint8_t b = 0; b < BAR_WIDTH; b++) {
         frame += b < filled ? "█" : "░";
      }

      char value[8];
      snprintf(value, sizeof(value), " %4u\n", sensor_values[i]);
      frame += value;
   }

   frame += "\033[J";  // apaga o que sobrar abaixo do quadro
   Serial.print(frame);
}

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

   if (BAR_CHART) {
      Serial.print("\033[2J");  // limpa a tela
   }
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

   if (BAR_CHART) {
      printBarChart();
   } else {
      printPlotter();
   }

   delay(REFRESH_MS);
}
