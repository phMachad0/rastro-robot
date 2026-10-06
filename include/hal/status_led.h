/**
 * @file status_led.h
 *
 * @brief LED de status: indica o estado da máquina de estados por padrão de pisca.
 *
 * Existe porque a validação do RF03 e do RF04 acontece com o robô andando pela
 * pista, longe do cabo serial e antes de o dashboard existir. Sem um indicador
 * local não há como saber se o robô parou porque a política decidiu parar ou
 * porque entrou em falha.
 */

#ifndef HAL_STATUS_LED_H_
#define HAL_STATUS_LED_H_

#include <stdint.h>

#include "app/robot_state.h"

class StatusLed {
 public:
   void begin();

   /** @brief Define o padrão a partir do estado corrente. */
   void setMode(RobotMode mode);

   /**
    * @brief Avança o padrão. Deve ser chamada periodicamente.
    *
    * Não bloqueia: o padrão é calculado a partir do relógio, não com delays.
    */
   void update();

 private:
   RobotMode mode_ = RobotMode::BOOT;
};

#endif  // HAL_STATUS_LED_H_
