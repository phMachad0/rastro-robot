/**
 * @file test_telemetry.cpp
 *
 * @brief Testa o envio MQTT de uma amostra sintética fixa, sem acionar motores.
 *
 * Reporta sucesso após dez envios locais; confira a recepção no dashboard.
 * Ambiente: `pio run -e test_telemetry -t upload`.
 */

#include "test/test_telemetry.h"
#include <Arduino.h>
#include "net/robot_telemetry.h"
#include "config/robot_config.h"

namespace {
unsigned long startedMs = 0;
unsigned long lastReportMs = 0;
bool passed = false;
}

void testTelemetrySetup() {
    Serial.begin(cfg::SERIAL_BAUD);
    Serial.println("\n=== MQTT test: SYNTHETIC data, no motor commands ===");
    RobotTelemetrySample sample;
    sample.sensorLeft = 1000;
    sample.sensorCenter = 2000;
    sample.sensorRight = 3000;
    sample.controlError = 2.0f;
    sample.motorLeft = 90;
    sample.motorRight = 50;
    sample.loopTimeUs = 100;
    sample.controlPeriodUs = 1000000UL / cfg::CONTROL_HZ;
    robotTelemetrySetup();
    robotTelemetrySetSample(sample);
    startedMs = lastReportMs = millis();
    passed = false;
}

void testTelemetryLoop() {
    robotTelemetryLoop();
    const unsigned long count = robotTelemetryPublishedCount();
    if (!passed && count >= 10) {
        Serial.println("[PASS] 10 publishes accepted locally; verify JSON at subscriber/dashboard.");
        passed = true;
    }
    if (millis() - lastReportMs >= 5000) {
        lastReportMs = millis();
        Serial.printf("published:%lu\tuptime_ms:%lu\n", count, millis());
        if (!passed && millis() - startedMs >= 30000) {
            Serial.println("[FAIL] Publish target not reached after 30 s; check Wi-Fi/broker. Retrying.");
        }
    }
    delay(1);
}
