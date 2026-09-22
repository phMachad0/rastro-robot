#include <Arduino.h>
#include "TestLed.h"
#include "TestMotors.h"
#include "TestI2C.h"

void setup()
{
  // testLedSetup();
  // testMotorsSetup();
  testI2CSetup();
}

void loop()
{
  // testLedLoop();
  // testMotorsLoop();
  testI2CLoop();
}
