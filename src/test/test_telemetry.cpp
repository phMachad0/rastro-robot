/**
 * @file test_telemetry.cpp
 * @brief Safe standalone MQTT demo: alternating sample and no-sample periods.
 *
 * Uses synthetic telemetry only. It does not start the robot tasks or touch
 * sensor and motor GPIOs, so it can run on a bare ESP32 board.
 */
#include "test/test_telemetry.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "config/robot_config.h"
#include "net/robot_telemetry.h"

namespace {
constexpr uint32_t SAMPLE_WINDOW_MS = 5000;
constexpr uint32_t EMPTY_WINDOW_MS = 5000;
constexpr uint32_t DEMO_CYCLE_MS = SAMPLE_WINDOW_MS + EMPTY_WINDOW_MS;

void publishDemoSample(uint32_t nowMs)
{
    RobotTelemetrySample sample;
    sample.synthetic = true;
    sample.robotMode = "RUNNING";
    sample.robotFault = "-";
    sample.imuStatus = "READY";
    sample.lineAvailable = true;
    sample.imuAvailable = true;
    sample.controlAvailable = true;
    sample.controlHz = cfg::CONTROL_HZ;
    sample.lineSampleHz = cfg::IR_SAMPLE_HZ;
    sample.imuSampleHz = cfg::IMU_SAMPLE_HZ;
    const uint32_t samplePhaseMs = nowMs % DEMO_CYCLE_MS;
    const float t = nowMs / 1000.0f;

    // Move a synthetic line across the six sensors during each sample window.
    const uint16_t background = 4050;
    for (uint8_t i = 0; i < pins::IR_CHANNEL_COUNT; ++i) {
        sample.irRaw[i] = background;
        sample.irOnLine[i] = false;
    }
    const uint8_t lineChannel = (samplePhaseMs * pins::IR_CHANNEL_COUNT) / SAMPLE_WINDOW_MS;
    sample.irRaw[lineChannel] = 1300;
    sample.irOnLine[lineChannel] = true;
    sample.lineDetected = true;
    sample.linePosition = lineChannel * cfg::IR_POSITION_UNIT;

    // All IMU values are simulated in this standalone display demo.
    sample.imuValid = true;
    sample.accelX = 0.25f * sinf(t * 1.2f);
    sample.accelY = 0.18f * cosf(t * 0.9f);
    sample.accelZ = 0.98f + 0.05f * sinf(t * 0.7f);
    sample.gyroZ = 24.0f * sinf(t * 1.7f);
    sample.gyroZRaw = sample.gyroZ + 1.5f * cosf(t * 2.3f);

    // Simulated commands only; this standalone target never initializes a driver.
    const float steering = 0.22f * sinf(t * 0.8f);
    sample.motorLeft = 0.55f - steering;
    sample.motorRight = 0.55f + steering;
    sample.baseSpeed = 0.55f;
    sample.steeringAction = steering;
    sample.motorsArmed = true;
    robotTelemetrySetSample(sample);
}
}

void testTelemetrySetup()
{
    Serial.begin(cfg::SERIAL_BAUD);
    Serial.println("\n=== Standalone telemetry demo ===");
    Serial.println("Synthetic moving samples: 5 s on, then 5 s with no sample.");
    Serial.println("IR, IMU and motor values are synthetic.");
    Serial.println("No sensor or motor GPIO is initialized.");
    robotTelemetrySetup();
}

void testTelemetryLoop()
{
    static bool wasSampling = false;
    static bool first = true;
    const uint32_t nowMs = millis();
    const bool sampling = (nowMs % DEMO_CYCLE_MS) < SAMPLE_WINDOW_MS;

    if (sampling) publishDemoSample(nowMs);

    if (first || sampling != wasSampling) {
        Serial.println(sampling ? "[Demo] SAMPLE window" : "[Demo] NO SAMPLE window");
        wasSampling = sampling;
        first = false;
    }

    robotTelemetryLoop();
    vTaskDelay(pdMS_TO_TICKS(10));
}
