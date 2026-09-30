/**
 * @file robot_telemetry.h
 * @brief Amostra dos sensores e saídas dos motores para publicação MQTT.
 */
#pragma once
#include <stdint.h>
#include "config/board_pins.h"

struct RobotTelemetrySample {
    uint16_t irRaw[pins::IR_CHANNEL_COUNT] = {};
    bool irOnLine[pins::IR_CHANNEL_COUNT] = {};
    bool lineDetected = false;
    uint16_t linePosition = 0;
    bool imuValid = false;
    float accelX = 0, accelY = 0, accelZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0, gyroZRaw = 0;
    // Comandos aplicados normalizados, não velocidade medida nem ciclo de trabalho do PWM.
    float motorLeft = 0, motorRight = 0;
    bool motorsArmed = false;
};

// Inicializa uma única vez, antes das tarefas de aquisição e rede, com a serial pronta.
void robotTelemetrySetup();
// Chamar apenas na tarefa de rede: a conexão MQTT pode bloquear.
void robotTelemetryLoop();
// Fila da última amostra, segura entre tarefas; a aquisição nunca espera pela rede.
void robotTelemetrySetSample(const RobotTelemetrySample& sample);
// Consultar apenas na tarefa de rede.
unsigned long robotTelemetryPublishedCount();
