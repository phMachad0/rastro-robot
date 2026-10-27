/**
 * @file board_pins.h
 *
 * @brief Mapa de pinos do robô — fonte única de verdade.
 *
 * Nenhum outro arquivo deve conter número de GPIO literal. Se um pino mudar
 * na bancada, muda aqui e em lugar nenhum mais.
 *
 * Restrição fundamental: o ADC2 do ESP32 fica inutilizável enquanto o Wi-Fi
 * está ativo, e o Wi-Fi é requisito do projeto (RF05). Por isso todos os
 * canais do array IR estão em ADC1, o que limita o array a 6 sensores.
 *
 * Ver docs/ARQUITETURA.md, seção 2.
 */

#ifndef CONFIG_BOARD_PINS_H_
#define CONFIG_BOARD_PINS_H_

#include <stdint.h>

namespace pins {

/*****************************************
 * Motores — driver L298N
 *
 * Os dois motores de cada lado compartilham o mesmo canal do driver, então
 * o robô se comporta como um diferencial de duas rodas.
 *****************************************/

constexpr uint8_t MOTOR_L_PWM = 19;  /**< ENA — PWM do lado esquerdo. */
constexpr uint8_t MOTOR_L_IN1 = 18;  /**< Direção esquerda A. */

/**
 * @brief Direção esquerda B.
 *
 * @warning GPIO5 é pino de strapping e precisa estar em nível alto durante o
 *          boot. Se o L298N puxar esse pino para baixo no reset, o ESP32 não
 *          inicializa. Manter o driver desenergizado ou com pull-up no reset.
 */
constexpr uint8_t MOTOR_L_IN2 = 5;

constexpr uint8_t MOTOR_R_PWM = 4;   /**< ENB — PWM do lado direito. */
constexpr uint8_t MOTOR_R_IN3 = 17;  /**< Direção direita A. */
constexpr uint8_t MOTOR_R_IN4 = 16;  /**< Direção direita B. */

/*****************************************
 * Array de sensores IR — QTR-8A
 *****************************************/

/**
 * @brief Quantidade de canais do array efetivamente usados.
 *
 * O QTR-8A tem 8 canais, mas só 6 pinos de ADC1 estão disponíveis. Os dois
 * canais das pontas do módulo ficam desconectados.
 */
constexpr uint8_t IR_CHANNEL_COUNT = 6;

/**
 * @brief Pinos do array IR, em ordem física da esquerda para a direita.
 *
 * Todos em ADC1. Os quatro primeiros (36, 39, 34, 35) são input-only, o que
 * não é problema para leitura analógica.
 *
 * @warning A ordem aqui precisa corresponder à ordem física dos sensores no
 *          chassi. Trocar dois elementos inverte o sinal do erro de posição e
 *          faz o robô sair da linha em vez de corrigir.
 */
constexpr uint8_t IR_CHANNELS[IR_CHANNEL_COUNT] = {36, 39, 34, 35, 32, 33};

/*****************************************
 * IMU — MPU-6050 via I2C
 *****************************************/

constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

/**
 * @brief Saída de interrupção do MPU-6050 (data-ready).
 *
 * Ver docs/ARQUITETURA.md, seção 4.3.
 */
constexpr uint8_t IMU_INT = 23;

/*****************************************
 * Diversos
 *****************************************/

constexpr uint8_t LED_STATUS = 2;  /**< LED da placa. */

/**
 * @brief Botão de start/stop, com pull-up interno e interrupção de borda.
 *
 * Permite armar e desarmar o robô sem depender do Wi-Fi.
 */
constexpr uint8_t BTN_START = 13;

/*****************************************
 * Pinos livres para expansão
 *
 * 12, 14, 15, 25, 26, 27.
 *
 * Os quatro últimos são ADC2 e só servem como digitais enquanto o Wi-Fi
 * estiver ligado. GPIO12 e GPIO15 são pinos de strapping — usar com cuidado.
 *****************************************/

}  // namespace pins

#endif  // CONFIG_BOARD_PINS_H_
