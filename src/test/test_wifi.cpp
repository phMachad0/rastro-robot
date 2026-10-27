/**
 * @file test_wifi.cpp
 *
 * @brief Testa conexão e reconexão Wi-Fi, com IP e RSSI na serial.
 *
 * Reporta sucesso ao conectar e continua monitorando quedas e recuperação.
 * Ambiente: `pio run -e test_wifi -t upload`.
 */

#include "test/test_wifi.h"
#include <Arduino.h>
#include <WiFi.h>
#include "config/robot_config.h"
#include "net/wifi_connection.h"

namespace {
unsigned long lastReportMs = 0;
bool passed = false;
}

void testWifiSetup() {
    Serial.begin(cfg::SERIAL_BAUD);
    Serial.println("\n=== Wi-Fi connection smoke test ===");
    wifiConnectionSetup();
    lastReportMs = millis();
    passed = false;
}

void testWifiLoop() {
    wifiConnectionLoop();
    if (!wifiConnectionConnected()) {
        passed = false;
    } else if (!passed) {
        Serial.println("[PASS] Wi-Fi connected. Disable/re-enable the AP to test recovery.");
        passed = true;
    }
    if (millis() - lastReportMs >= 5000) {
        lastReportMs = millis();
        Serial.printf("wifi_connected:%d\trssi:%d\n", wifiConnectionConnected(),
                      wifiConnectionConnected() ? WiFi.RSSI() : 0);
    }
    delay(1);
}
