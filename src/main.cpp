/**
 * @file main.cpp
 *
 * @brief Ponto de entrada único.
 *
 * Toda a escolha de qual rotina rodar acontece em entry_points.h, a partir da
 * flag APP_TARGET_* que o ambiente do PlatformIO define. Este arquivo não deve
 * crescer.
 */

#include <Arduino.h>

#include "app/entry_points.h"

void setup()
{
   appSetup();
}

void loop()
{
   appLoop();
}
