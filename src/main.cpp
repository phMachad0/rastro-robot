#include <Arduino.h>
#include "TestLed.h"
#include "TestMotors.h"
#include "TestI2C.h"
#include "TestSensorIR.h"

void setup()
{
  // testLedSetup();
  // testMotorsSetup();
  // testI2CSetup();
  testSensorIRSetup();
}

void loop()
{
  // testLedLoop();
  // testMotorsLoop();
  // testI2CLoop();
  testSensorIRLoop();
}
