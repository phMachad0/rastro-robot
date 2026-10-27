/**
 * @file status_led.cpp
 *
 * @brief Implementação do StatusLed.
 */

#include "hal/status_led.h"

#include <Arduino.h>

#include "config/board_pins.h"

namespace {

/**
 * @brief Padrão de pisca de cada estado.
 *
 * `period_ms` é o ciclo completo e `pulses` quantos piscas cabem nele. Os
 * padrões são escolhidos para serem distinguíveis de relance, com o robô em
 * movimento: aceso contínuo só em RUNNING, e quanto mais rápido o pisca, mais
 * grave o estado.
 */
struct Pattern {
   uint16_t period_ms;
   uint8_t  pulses;
   bool     solid;
};

Pattern patternFor(RobotMode mode)
{
   switch (mode) {
      case RobotMode::BOOT:        return {200, 1, false};   /* pisca rápido */
      case RobotMode::CALIBRATING: return {400, 2, false};   /* dois piscas */
      case RobotMode::IDLE:        return {1600, 1, false};  /* pisca lento */
      case RobotMode::RUNNING:     return {0, 0, true};      /* aceso */
      case RobotMode::STOPPED:     return {1000, 3, false};  /* três piscas */
      case RobotMode::FAULT:       return {100, 1, false};   /* pisca muito rápido */
   }
   return {1000, 1, false};
}

}  // namespace

void StatusLed::begin()
{
   pinMode(pins::LED_STATUS, OUTPUT);
   digitalWrite(pins::LED_STATUS, LOW);
}

void StatusLed::setMode(RobotMode mode)
{
   mode_ = mode;
}

void StatusLed::update()
{
   const Pattern p = patternFor(mode_);

   if (p.solid) {
      digitalWrite(pins::LED_STATUS, HIGH);
      return;
   }

   /* Dentro do ciclo, cada pulso ocupa uma fatia igual, e o LED fica aceso na
      primeira metade de cada fatia. Derivar do relógio em vez de contar
      chamadas mantém o padrão igual qualquer que seja a taxa de update(). */
   const uint32_t phase = millis() % p.period_ms;
   const uint32_t slot = p.period_ms / (p.pulses * 2);

   const bool on = (slot > 0) && (phase / slot) % 2 == 0 &&
                   (phase / slot) < (uint32_t)(p.pulses * 2);

   digitalWrite(pins::LED_STATUS, on ? HIGH : LOW);
}
