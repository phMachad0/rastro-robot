/**
 * @file test_telemetry.cpp
 * @brief Testa telemetria com sensores reais e motores desarmados.
 *
 * Verifique os sinais no dashboard; mantenha o robô imóvel durante o boot.
 * Ambiente: `pio run -e test_telemetry -t upload`.
 */
#include "test/test_telemetry.h"
#include "app/robot_app.h"

void testTelemetrySetup() { robotAppSetup(); }
void testTelemetryLoop() { robotAppLoop(); }
