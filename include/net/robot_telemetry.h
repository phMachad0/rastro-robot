/**
 * @file robot_telemetry.h
 *
 * @brief Amostra e interface do serviço de telemetria MQTT.
 *
 * As chamadas pertencem à mesma tarefa de rede; a conexão MQTT pode bloquear.
 * O contador de publicações indica envios locais, não confirmação de entrega.
 */

#pragma once

// Application snapshot. Motor values are percentages, not GPIO commands.
struct RobotTelemetrySample {
    int sensorLeft = 0;
    int sensorCenter = 0;
    int sensorRight = 0;
    float controlError = 0;
    int motorLeft = 0;
    int motorRight = 0;
    unsigned long loopTimeUs = 0;
    unsigned long controlPeriodUs = 0;
};

// Initialize Serial first. Call all functions from a single network task.
// MQTT connect may block: never call from a control task or ISR.
void robotTelemetrySetup();
void robotTelemetryLoop();
void robotTelemetrySetSample(const RobotTelemetrySample& sample);
unsigned long robotTelemetryPublishedCount();
