/**
 * @file wifi_connection.cpp
 *
 * @brief Conexão Wi-Fi em modo STA com tentativas periódicas de reconexão.
 *
 * Usa as credenciais de secrets.h e registra mudanças de conexão na serial.
 * Compartilhado pelo firmware e pelos testes de rede.
 */

#include "config/features.h"

#if (FEATURE_TELEMETRY && \
     FEATURE_TELEMETRY_BACKEND == TELEMETRY_BACKEND_WIFI_MQTT) || \
    defined(APP_TARGET_TEST_WIFI) || defined(APP_TARGET_TEST_TELEMETRY)
#include "net/wifi_connection.h"
#include <Arduino.h>
#include <WiFi.h>
#include "config/robot_config.h"
#include "secrets.h"

namespace {
unsigned long lastAttemptMs = 0;
bool wasConnected = false;
}

void wifiConnectionSetup() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastAttemptMs = millis();
    wasConnected = false;
    Serial.println("[WiFi] Connecting...");
}

bool wifiConnectionConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void wifiConnectionLoop() {
    const bool connected = wifiConnectionConnected();
    if (connected != wasConnected) {
        if (connected) {
            Serial.print("[WiFi] CONNECTED - IP: ");
            Serial.println(WiFi.localIP());
        } else {
            Serial.println("[WiFi] DISCONNECTED");
            lastAttemptMs = millis();
        }
        wasConnected = connected;
    }
    if (!connected && millis() - lastAttemptMs >= cfg::WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println("[WiFi] Connection timeout; retrying. Check credentials / 2.4 GHz network.");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastAttemptMs = millis();
    }
}

#endif
