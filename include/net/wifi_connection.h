/**
 * @file wifi_connection.h
 *
 * @brief Interface de inicialização, manutenção e consulta da conexão Wi-Fi.
 *
 * Inicialize a serial antes do setup e execute as chamadas na mesma tarefa.
 */

#pragma once

void wifiConnectionSetup();
void wifiConnectionLoop();
bool wifiConnectionConnected();
