/*
 * QTR-8A Sensor IR Test
 * Lê os valores analógicos brutos dos 8 canais do sensor QTR-8A
 * direto com analogRead() e mostra no Serial Monitor como gráfico de barras.
 * Com BAR_CHART = false imprime no formato do Serial Plotter.
 *
 * O gráfico usa códigos ANSI para redesenhar no mesmo lugar, então o
 * monitor precisa do filtro "direct" (monitor_filters = direct no
 * platformio.ini), senão os códigos aparecem como lixo na tela.
 *
 * Ligação (ESP32 DevKit v1):
 *   VCC   -> 3V3 (NÃO usar 5V: o ADC da ESP32 suporta no máximo ~3.3V)
 *   GND   -> GND
 *   LEDON -> 3V3 ou desconectado (emissores sempre ligados)
 *   1..8  -> pinos em SENSOR_PINS
 *
 * Pinos 34, 35, 32, 33 são do ADC1. Pinos 25, 26, 27 e 14 são do ADC2,
 * que não funciona com o WiFi ligado. Evitei os pinos já usados pelos
 * motores (19, 18, 5, 17, 16, 4), I2C (21, 22) e LED (2).
 */

#include <Arduino.h>
#include <WiFi.h>

// Define a quantidade de sensores que você vai usar
const uint8_t NUM_SENSORS = 8;
uint16_t sensorValues[NUM_SENSORS];

const uint8_t SENSOR_PINS[NUM_SENSORS] = {34, 35, 32, 33, 25, 26, 27, 14};

// Quantidade de leituras por canal para tirar a média e reduzir ruído
const uint8_t SAMPLES_PER_SENSOR = 8;

// true = gráfico de barras no terminal, false = formato do Serial Plotter
const bool BAR_CHART = true;
const uint8_t BAR_WIDTH = 50;
const uint16_t ADC_MAX = 4095;

void printPlotter() {
   for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.print("S");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.print(sensorValues[i]);
      Serial.print(i < NUM_SENSORS - 1 ? '\t' : '\n');
   }
}

void printBarChart() {
   // Monta o quadro inteiro antes de enviar para reduzir o piscar da tela
   String frame = "\033[H"; // cursor para o topo
   frame += "QTR-8A  (0 = claro, 4095 = escuro)\n\n";

   for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      uint8_t filled = (uint32_t)sensorValues[i] * BAR_WIDTH / ADC_MAX;

      char label[16];
      snprintf(label, sizeof(label), "S%u (GPIO%2u) ", i + 1, SENSOR_PINS[i]);
      frame += label;

      for (uint8_t b = 0; b < BAR_WIDTH; b++) {
         frame += b < filled ? "█" : "░";
      }

      char value[8];
      snprintf(value, sizeof(value), " %4u\n", sensorValues[i]);
      frame += value;
   }

   frame += "\033[J"; // apaga o que sobrar abaixo do quadro
   Serial.print(frame);
}

void testSensorIRSetup() {
   Serial.begin(115200);
   WiFi.mode(WIFI_OFF);

   // ADC de 12 bits (0-4095) com atenuação para faixa completa de 0-3.3V
   analogReadResolution(12);
   analogSetAttenuation(ADC_11db);

   for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      pinMode(SENSOR_PINS[i], INPUT);
   }

   Serial.println("QTR-8A Sensor IR Test Initialized");
   if (BAR_CHART) {
      Serial.print("\033[2J"); // limpa a tela
   }
}

void testSensorIRLoop() {
   // Leitura bruta: valores altos = superfície escura (pouca reflexão),
   // valores baixos = superfície clara (muita reflexão)
   for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      uint32_t sum = 0;
      for (uint8_t s = 0; s < SAMPLES_PER_SENSOR; s++) {
         sum += analogRead(SENSOR_PINS[i]);
      }
      sensorValues[i] = sum / SAMPLES_PER_SENSOR;
   }

   if (BAR_CHART) {
      printBarChart();
   } else {
      printPlotter();
   }

   delay(100);
}
