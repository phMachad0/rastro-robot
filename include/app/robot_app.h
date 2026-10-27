/**
 * @file robot_app.h
 *
 * @brief Aplicação principal do robô.
 *
 * Ponto de entrada do ambiente `firmware`. A implementação de verdade — as
 * tarefas do FreeRTOS, a máquina de estados e a malha de controle — chega na
 * etapa 4 do roteiro.
 */

#ifndef APP_ROBOT_APP_H_
#define APP_ROBOT_APP_H_

void robotAppSetup();
void robotAppLoop();

#endif  // APP_ROBOT_APP_H_
