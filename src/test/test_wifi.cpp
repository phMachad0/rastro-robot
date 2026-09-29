/**
 * @file test_wifi.cpp
 *
 * @brief Marcador do teste de rede. Implementação na etapa 5.
 *
 * Exercitará conexão em modo STA, fallback para modo AP, mDNS e um WebSocket
 * emitindo telemetria sintética — tudo antes de haver dados reais para enviar,
 * de forma que problemas de rede não se confundam com problemas de controle.
 *
 * Ver docs/ARQUITETURA.md, seções 7 e 11.
 */

#include "test/test_wifi.h"

#include <Arduino.h>

#include "config/robot_config.h"

void testWifiSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);
   Serial.println();
   Serial.println("test_wifi: nao implementado. Chega na etapa 5 do roteiro.");
}

void testWifiLoop()
{
   delay(1000);
}
