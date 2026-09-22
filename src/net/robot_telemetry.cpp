/**
 * @file robot_telemetry.cpp
 *
 * @brief Publica a última amostra do robô em JSON via MQTT.
 *
 * Mantém as conexões e publica na frequência definida em robot_config.h.
 * Não gera dados simulados; no firmware, roda na tarefa de rede do núcleo 0.
 */

#include "config/features.h"

#if FEATURE_TELEMETRY || defined(APP_TARGET_TEST_TELEMETRY)
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "net/robot_telemetry.h"
#include "net/wifi_connection.h"
#include "config/robot_config.h"
#include "network_config.h"

namespace {
// Network configuration
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

constexpr unsigned long MQTT_RETRY_INTERVAL_MS = 3000;
unsigned long lastTelemetryMs = 0;
unsigned long lastMqttAttemptMs = 0;
unsigned long publishedCount = 0;
bool previousMqttConnected = false;
bool bufferReady = false;
bool sampleValid = false;
RobotTelemetrySample telemetry;

void updateMqttConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    if (mqttClient.connected()) {
        return;
    }

    unsigned long now = millis();

    if (now - lastMqttAttemptMs < MQTT_RETRY_INTERVAL_MS) {
        return;
    }

    lastMqttAttemptMs = now;

    String clientId = "esp32-robot-";
    clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

    Serial.println("[MQTT] Attempting connection...");

    if (!mqttClient.connect(clientId.c_str())) {
        Serial.print("[MQTT] Connection failed, state=");
        Serial.println(mqttClient.state());
    }
}

void updateConnectionDebug() {
    bool wifiConnected = WiFi.status() == WL_CONNECTED;
    bool mqttConnected = wifiConnected && mqttClient.connected();

    if (mqttConnected != previousMqttConnected) {
        if (mqttConnected) {
            Serial.println("[MQTT] CONNECTED");
        } else {
            Serial.println("[MQTT] DISCONNECTED");
        }
        previousMqttConnected = mqttConnected;
    }
}

void publishTelemetry() {
    const bool wifiConnected = wifiConnectionConnected();
    const bool mqttConnected = wifiConnected && mqttClient.connected();
    if (!mqttConnected || !bufferReady) return;

    JsonDocument doc;
    doc["uptime_ms"] = millis();
    doc["rssi"] = WiFi.RSSI();
    doc["wifi_connected"] = wifiConnected;
    doc["mqtt_connected"] = mqttConnected;
    doc["sample_valid"] = sampleValid;
    doc["sensor_left"] = telemetry.sensorLeft;
    doc["sensor_center"] = telemetry.sensorCenter;
    doc["sensor_right"] = telemetry.sensorRight;
    doc["control_error"] = telemetry.controlError;
    doc["motor_left"] = telemetry.motorLeft;
    doc["motor_right"] = telemetry.motorRight;
    doc["loop_time_us"] = telemetry.loopTimeUs;
    doc["control_period_us"] = telemetry.controlPeriodUs;

    char payload[416];
    if (doc.overflowed() || measureJson(doc) >= sizeof(payload)) {
        Serial.println("[MQTT] Telemetry JSON exceeds payload buffer");
        return;
    }
    size_t payloadSize = serializeJson(doc, payload, sizeof(payload));
    bool success = mqttClient.publish(MQTT_TOPIC_TELEMETRY, payload, payloadSize);

    if (success) {
        ++publishedCount;
    } else {
        Serial.println("[MQTT] Telemetry publish failed");
    }
}

} // namespace

void robotTelemetrySetup() {
    telemetry = RobotTelemetrySample{};
    publishedCount = 0;
    sampleValid = false;
    previousMqttConnected = false;
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setSocketTimeout(1);
    // Include MQTT fixed header, topic length, topic and the full JSON payload.
    bufferReady = mqttClient.setBufferSize(5 + 2 + sizeof(MQTT_TOPIC_TELEMETRY) + 416);
    if (!bufferReady) Serial.println("[MQTT] Buffer allocation failed");
    wifiConnectionSetup();
    lastMqttAttemptMs = millis() - MQTT_RETRY_INTERVAL_MS;
    lastTelemetryMs = millis();
}

void robotTelemetrySetSample(const RobotTelemetrySample& sample) {
    telemetry = sample;
    sampleValid = true;
}

unsigned long robotTelemetryPublishedCount() {
    return publishedCount;
}

void robotTelemetryLoop() {
    wifiConnectionLoop();
    if (!wifiConnectionConnected()) {
        // Drop the stale transport without trying to send an MQTT packet.
        wifiClient.stop();
    } else if (bufferReady) {
        updateMqttConnection();
        mqttClient.loop();
    }
    updateConnectionDebug();
    const unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMs >= cfg::TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = nowMs;
        publishTelemetry();
    }
}

#endif
