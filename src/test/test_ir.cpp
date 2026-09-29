/**
 * @file test_ir.cpp
 *
 * @brief Teste do array QTR-8A através do IrArray.
 *
 * O teste tem duas fases. Na **calibração**, por cfg::IR_CALIBRATION_MS, é
 * preciso varrer o array sobre a linha de um extremo ao outro, para que cada
 * canal veja tanto o fundo escuro quanto a linha clara. Na fase de **leitura**,
 * a posição calculada é emitida continuamente.
 *
 * O que observar no Teleplot:
 *   - `S0`..`S5`: os pesos normalizados. O canal sobre a linha vai a ~1000.
 *   - `position`: deve varrer de 0 a 5000 conforme a linha passa da esquerda
 *     para a direita, cruzando 2500 no centro.
 *   - `lineDetected`: cai a zero quando o array sai da linha.
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

void runCalibration()
{
   Serial.println();
   Serial.printf("Calibracao: varra o array sobre a linha por %lu segundos.\n",
                 (unsigned long)(cfg::IR_CALIBRATION_MS / 1000));
   Serial.println("Cada canal precisa ver o fundo escuro E a linha clara.");

   ir.calibrationReset();

   const uint32_t deadline = millis() + cfg::IR_CALIBRATION_MS;

   while ((int32_t)(millis() - deadline) < 0) {
      ir.sample();
      ir.calibrationAccumulate();

      /* Durante a calibração ainda não há normalização, então o que se plota é
         o valor bruto — serve para ver, ao vivo, se todo canal está variando. */
      for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
         char name[8];
         snprintf(name, sizeof(name), "raw%u", i);
         teleplot::send(name, (int32_t)ir.raw(i));
      }

      delay(REFRESH_MS);
   }

   if (!ir.calibrationFinish()) {
      Serial.println("FALHA: algum canal nao viu contraste suficiente.");
      Serial.println("Conferir altura dos sensores, ligacao e se a varredura cobriu todo o array.");
   } else {
      Serial.println("Calibracao concluida.");
   }

   const IrCalibration& c = ir.calibration();
   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      Serial.printf("  S%u (GPIO%2u): min=%4u max=%4u amplitude=%4u\n",
                    i, pins::IR_CHANNELS[i], c.min[i], c.max[i],
                    (uint16_t)(c.max[i] - c.min[i]));
   }
}

}  // namespace

void testIrSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   ir.begin();

   Serial.println();
   Serial.printf("Teste do array IR: %u canais em ADC1\n", pins::IR_CHANNEL_COUNT);

   runCalibration();

   Serial.println("\nLeitura: series S0..S5 (peso), position (0..5000) e lineDetected.");
}

void testIrLoop()
{
   ir.sample();

   if (!ir.isCalibrated()) {
      /* Sem calibração válida não há posição para mostrar; cai para o bruto
         para que o teste ainda sirva de diagnóstico de ligação. */
      for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
         char name[8];
         snprintf(name, sizeof(name), "raw%u", i);
         teleplot::send(name, (int32_t)ir.raw(i));
      }

      delay(REFRESH_MS);
      return;
   }

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      char name[4];
      snprintf(name, sizeof(name), "S%u", i);
      teleplot::send(name, (int32_t)ir.weight(i));
   }

   teleplot::send("position", (int32_t)ir.position());
   teleplot::send("error", (int32_t)ir.error());
   teleplot::send("weightSum", (int32_t)ir.weightSum());
   teleplot::send("lineDetected", (int32_t)(ir.lineDetected() ? 1 : 0));

   delay(REFRESH_MS);
}
