/**
 * @file ir_array.cpp
 *
 * @brief Implementação do IrArray.
 */

#include "hal/ir_array.h"

#include <Arduino.h>

#include "config/robot_config.h"

namespace {

constexpr uint16_t ADC_MAX = 4095;

uint16_t clampu16(int32_t v, uint16_t lo, uint16_t hi)
{
   if (v < (int32_t)lo) {
      return lo;
   }
   if (v > (int32_t)hi) {
      return hi;
   }
   return (uint16_t)v;
}

}  // namespace

void IrArray::begin()
{
   /* 12 bits (0 a 4095) com atenuação para a faixa completa de 0 a 3.3 V. */
   analogReadResolution(12);
   analogSetAttenuation(ADC_11db);

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      pinMode(pins::IR_CHANNELS[i], INPUT);
   }

   calibrationReset();

   position_ = cfg::LINE_POSITION_CENTER;
}

void IrArray::calibrationReset()
{
   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      /* Extremos invertidos de propósito: a primeira amostra substitui os dois. */
      calib_.min[i] = ADC_MAX;
      calib_.max[i] = 0;
   }

   calib_.valid = false;
}

void IrArray::calibrationAccumulate()
{
   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      if (raw_[i] < calib_.min[i]) {
         calib_.min[i] = raw_[i];
      }
      if (raw_[i] > calib_.max[i]) {
         calib_.max[i] = raw_[i];
      }
   }
}

bool IrArray::calibrationFinish()
{
   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      const int32_t span = (int32_t)calib_.max[i] - (int32_t)calib_.min[i];

      /* Um canal sem contraste normalizaria ruído para a escala inteira e
         envenenaria a posição com um peso aleatório. Rejeita a calibração
         toda, em vez de seguir com um canal mentindo. */
      if (span < (int32_t)cfg::IR_CALIBRATION_MIN_SPAN) {
         calib_.valid = false;
         return false;
      }
   }

   calib_.valid = true;
   return true;
}

void IrArray::setCalibration(const IrCalibration& calibration)
{
   calib_ = calibration;
}

void IrArray::sample()
{
   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      uint32_t sum = 0;

      for (uint8_t s = 0; s < cfg::IR_SAMPLES_PER_CHANNEL; s++) {
         sum += analogRead(pins::IR_CHANNELS[i]);
      }

      raw_[i] = (uint16_t)(sum / cfg::IR_SAMPLES_PER_CHANNEL);
   }

   if (!calib_.valid) {
      return;
   }

   weight_sum_ = 0;
   uint32_t weighted_position = 0;

   for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; i++) {
      const int32_t span = (int32_t)calib_.max[i] - (int32_t)calib_.min[i];
      const int32_t offset = (int32_t)raw_[i] - (int32_t)calib_.min[i];

      normalized_[i] = clampu16(offset * (int32_t)cfg::IR_NORMALIZED_MAX / span,
                                0, cfg::IR_NORMALIZED_MAX);

      /* Inversão: a linha é clara, então reflete mais e lê baixo no ADC.
         O peso é "quanto deste canal está sobre a linha". */
      weight_[i] = cfg::IR_NORMALIZED_MAX - normalized_[i];

      weight_sum_ += weight_[i];
      weighted_position += (uint32_t)weight_[i] * i * cfg::IR_NORMALIZED_MAX;
   }

   line_detected_ = weight_sum_ >= cfg::LINE_LOST_THRESHOLD;

   /* Com a linha perdida a média ponderada não tem significado, então a última
      posição válida é mantida: o controle continua virando para o lado em que
      a linha sumiu, que é a recuperação descrita na seção 5.5 da arquitetura. */
   if (line_detected_) {
      position_ = (uint16_t)(weighted_position / weight_sum_);
   }
}

int16_t IrArray::error() const
{
   return (int16_t)position_ - (int16_t)cfg::LINE_POSITION_CENTER;
}
