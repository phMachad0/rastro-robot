/**
 * @file test_ir.cpp
 *
 * @brief Teste do array QTR-8A através do IrArray.
 *
 * Sem fase de calibração: a classificação é por limiar fixo
 * (cfg::IR_BLACK_THRESHOLD), então o teste já entra lendo.
 *
 * O que observar no Teleplot:
 *   - `raw0`..`raw5`: as leituras brutas, com `threshold` plotado junto. Esta é
 *     a verificação mais importante do teste — com o limiar fixo em código, é
 *     preciso enxergar que ele cai numa folga clara entre os dois patamares,
 *     não em cima de um deles. Se algum canal encostar nos 4000 estando sobre o
 *     fundo, ou passar deles estando sobre a linha, o valor precisa mudar.
 *   - `position`: deve varrer de 0 a 5000 conforme a linha passa da esquerda
 *     para a direita, cruzando 2500 no centro.
 *   - `detectedCount`: quantos canais veem linha. Passando por cima da linha
 *     deve ficar em 1 ou 2. Piscando com o robô parado indica leitura oscilando
 *     em torno do limiar. Indo a 6, o array está sobre uma faixa transversal ou
 *     saiu da pista.
 *
 * Se `position` andar para o lado contrário do movimento real, a ordem de
 * pins::IR_CHANNELS não corresponde à ordem física dos sensores no chassi — é
 * o erro mais comum e inverteria o sinal da correção no controle.
 *
 * Ligação (ESP32 DevKit v1):
 *   VCC   -> 3V3 (NÃO usar 5V: o ADC do ESP32 suporta no máximo ~3.3 V)
 *   GND   -> GND
 *   LEDON -> 3V3 ou desconectado (emissores sempre ligados)
 *   canais-> pinos de pins::IR_CHANNELS
 */

#include "test/test_ir.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/robot_config.h"
#include "hal/ir_array.h"
#include "test/teleplot.h"

namespace {

IrArray ir;

constexpr uint32_t REFRESH_MS = 20;

}  // namespace

void testIrSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   ir.begin();

   Serial.println();
   Serial.printf("Teste do array IR: %u canais em ADC1\n", pins::IR_CHANNEL_COUNT);
   Serial.printf("Limiar fixo: leitura > %u e fundo escuro, <= %u e linha branca\n",
                 cfg::IR_BLACK_THRESHOLD, cfg::IR_BLACK_THRESHOLD);
   Serial.println("Series: raw0..raw5, threshold, position, error, detectedCount");
}

void testIrLoop()
{
   ir.sample();

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      char name[8];
      snprintf(name, sizeof(name), "raw%u", i);
      teleplot::send(name, (int32_t)ir.raw(i));
   }

   /* O limiar vai como série constante para poder ser lido no mesmo gráfico
      das leituras brutas — é o que permite conferir a folga de cada lado. */
   teleplot::send("threshold", (int32_t)cfg::IR_BLACK_THRESHOLD);

   teleplot::send("position", (int32_t)ir.position());
   teleplot::send("error", (int32_t)ir.error());
   teleplot::send("detectedCount", (int32_t)ir.detectedCount());
   teleplot::send("lineDetected", (int32_t)(ir.lineDetected() ? 1 : 0));

   delay(REFRESH_MS);
}
