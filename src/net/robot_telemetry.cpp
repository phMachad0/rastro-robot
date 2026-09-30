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
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "net/robot_telemetry.h"
#include "net/wifi_connection.h"
#include "config/robot_config.h"
#include "network_config.h"

namespace {
// Configuração de rede
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

constexpr unsigned long MQTT_RETRY_INTERVAL_MS = 3000;
unsigned long lastTelemetryMs = 0;
unsigned long lastMqttAttemptMs = 0;
unsigned long publishedCount = 0;
bool previousMqttConnected = false;
bool bufferReady = false;
QueueHandle_t samples = nullptr;
constexpr size_t PAYLOAD_SIZE = 1024;

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

    RobotTelemetrySample telemetry;
    // Consome apenas a amostra mais recente, sem republicar dados já consumidos.
    if (!samples || xQueueReceive(samples, &telemetry, 0) != pdTRUE) return;
    JsonDocument doc;
    JsonArray raw = doc["ir"]["raw"].to<JsonArray>();
    JsonArray onLine = doc["ir"]["on_line"].to<JsonArray>();
    for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; ++i) {
        raw.add(telemetry.irRaw[i]);
        onLine.add(telemetry.irOnLine[i]);
    }
    doc["ir"]["line_detected"] = telemetry.lineDetected;
    if (telemetry.lineDetected) doc["ir"]["position"] = telemetry.linePosition;
    else doc["ir"]["position"] = nullptr;
    if (telemetry.imuValid) {
        doc["imu"]["accel_x_g"] = telemetry.accelX;
        doc["imu"]["accel_y_g"] = telemetry.accelY;
        doc["imu"]["accel_z_g"] = telemetry.accelZ;
        doc["imu"]["gyro_x_dps"] = telemetry.gyroX;
        doc["imu"]["gyro_y_dps"] = telemetry.gyroY;
        doc["imu"]["gyro_z_dps"] = telemetry.gyroZ;
        doc["imu"]["gyro_z_raw_dps"] = telemetry.gyroZRaw;
    } else {
        doc["imu"] = nullptr;
    }
    doc["motors"]["left_applied"] = telemetry.motorLeft;
    doc["motors"]["right_applied"] = telemetry.motorRight;
    doc["motors"]["armed"] = telemetry.motorsArmed;

    char payload[PAYLOAD_SIZE];
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

} // namespace anônimo

void robotTelemetrySetup() {
    samples = xQueueCreate(1, sizeof(RobotTelemetrySample));
    if (!samples) Serial.println("[Telemetry] Snapshot mailbox allocation failed");
    publishedCount = 0;
    previousMqttConnected = false;
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setSocketTimeout(1);
    // Reserva espaço para o cabeçalho MQTT, o tamanho do tópico, o tópico e o JSON completo.
    bufferReady = mqttClient.setBufferSize(5 + 2 + sizeof(MQTT_TOPIC_TELEMETRY) + PAYLOAD_SIZE);
    if (!bufferReady) Serial.println("[MQTT] Buffer allocation failed");
    wifiConnectionSetup();
    lastMqttAttemptMs = millis() - MQTT_RETRY_INTERVAL_MS;
    lastTelemetryMs = millis();
}

void robotTelemetrySetSample(const RobotTelemetrySample& sample) {
    if (samples) xQueueOverwrite(samples, &sample);
}

unsigned long robotTelemetryPublishedCount() {
    return publishedCount;
}

void robotTelemetryLoop() {
    wifiConnectionLoop();
    if (!wifiConnectionConnected()) {
        // Encerra a conexão inválida sem tentar enviar um pacote MQTT.
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
