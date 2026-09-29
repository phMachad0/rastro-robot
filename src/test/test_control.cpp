/**
 * @file test_control.cpp
 *
 * @brief Marcador do teste de malha fechada. Implementação na etapa 3.
 *
 * Rodará a malha completa — posição da linha, PID, amortecimento por
 * giroscópio e classificador de regime — com os motores desabilitados,
 * imprimindo cada termo separadamente no Serial Plotter. Permite conferir o
 * sinal e a ordem de grandeza da ação de controle movendo o robô à mão sobre a
 * linha, sem risco de ele sair correndo pela bancada.
 *
 * Ver docs/ARQUITETURA.md, seções 5 e 11.
 */

#include "test/test_control.h"

#include <Arduino.h>

#include "config/robot_config.h"

void testControlSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);
   Serial.println();
   Serial.println("test_control: nao implementado. Chega na etapa 3 do roteiro.");
}

void testControlLoop()
{
   delay(1000);
}
