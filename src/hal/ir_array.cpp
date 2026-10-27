/**
 * @file ir_array.cpp
 *
 * @brief Implementação do IrArray.
 */

#include "hal/ir_array.h"

#include <Arduino.h>

#include "config/robot_config.h"

void IrArray::begin()
{
   /* 12 bits (0 a 4095) com atenuação para a faixa completa de 0 a 3.3 V. */
   analogReadResolution(12);
   analogSetAttenuation(ADC_11db);

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      pinMode(pins::IR_CHANNELS[i], INPUT);
   }

   position_ = cfg::LINE_POSITION_CENTER;
}

void IrArray::sample()
{
   detected_count_ = 0;
   uint32_t position_sum = 0;

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      uint32_t sum = 0;

      for (uint8_t s = 0; s < cfg::IR_SAMPLES_PER_CHANNEL; s++) {
         sum += analogRead(pins::IR_CHANNELS[i]);
      }

      raw_[i] = (uint16_t)(sum / cfg::IR_SAMPLES_PER_CHANNEL);

      /* Leitura baixa é linha clara: a superfície branca reflete mais. */
      on_line_[i] = raw_[i] <= cfg::IR_BLACK_THRESHOLD;

      if (on_line_[i]) {
         detected_count_++;

         /* A orientação entra aqui, na conversão de índice de ligação para
            posição física. Os getters raw() e onLine() continuam na ordem de
            ligação, para que conferir um canal contra pins::IR_CHANNELS seja
            direto. */
         const uint8_t slot =
             cfg::IR_REVERSED ? (uint8_t)(pins::IR_CHANNEL_COUNT - 1 - i) : i;

         position_sum += (uint32_t)slot * cfg::IR_POSITION_UNIT;
      }
   }

   /* Com a linha perdida a média não tem significado, então a última posição
      válida é mantida: o controle continua virando para o lado em que a linha
      sumiu, que é a recuperação descrita na seção 5.5 da arquitetura. */
   if (detected_count_ > 0) {
      position_ = (uint16_t)(position_sum / detected_count_);
   }
}

int16_t IrArray::error() const
{
   return (int16_t)position_ - (int16_t)cfg::LINE_POSITION_CENTER;
}
