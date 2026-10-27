/**
 * @file entry_points.h
 *
 * @brief Despachante de ponto de entrada resolvido em tempo de compilação.
 *
 * Cada ambiente do PlatformIO define exatamente um APP_TARGET_*, e este
 * cabeçalho amarra appSetup/appLoop à rotina correspondente. O main.cpp não
 * conhece nenhuma delas.
 *
 * Isso substitui o padrão de comentar e descomentar linhas no main.cpp: o
 * binário de teste não carrega o código do firmware completo, cada alvo pode
 * ter flags e dependências próprias, e o comando de build documenta sozinho
 * o que está sendo gravado na placa.
 *
 * Ver docs/ARQUITETURA.md, seção 8.
 */

#ifndef APP_ENTRY_POINTS_H_
#define APP_ENTRY_POINTS_H_

#if defined(APP_TARGET_MAIN)
  #include "app/robot_app.h"
  inline void appSetup() { robotAppSetup(); }
  inline void appLoop()  { robotAppLoop();  }

#elif defined(APP_TARGET_TEST_LED)
  #include "test/test_led.h"
  inline void appSetup() { testLedSetup(); }
  inline void appLoop()  { testLedLoop();  }

#elif defined(APP_TARGET_TEST_MOTORS)
  #include "test/test_motors.h"
  inline void appSetup() { testMotorsSetup(); }
  inline void appLoop()  { testMotorsLoop();  }

#elif defined(APP_TARGET_TEST_IR)
  #include "test/test_ir.h"
  inline void appSetup() { testIrSetup(); }
  inline void appLoop()  { testIrLoop();  }

#elif defined(APP_TARGET_TEST_IMU)
  #include "test/test_imu.h"
  inline void appSetup() { testImuSetup(); }
  inline void appLoop()  { testImuLoop();  }

#elif defined(APP_TARGET_TEST_WIFI)
  #include "test/test_wifi.h"
  inline void appSetup() { testWifiSetup(); }
  inline void appLoop()  { testWifiLoop();  }

#elif defined(APP_TARGET_TEST_TELEMETRY)
  #include "test/test_telemetry.h"
  inline void appSetup() { testTelemetrySetup(); }
  inline void appLoop()  { testTelemetryLoop();  }

#elif defined(APP_TARGET_TEST_CONTROL)
  #include "test/test_control.h"
  inline void appSetup() { testControlSetup(); }
  inline void appLoop()  { testControlLoop();  }

#else
  #error "Nenhum APP_TARGET_* definido. Compile por um ambiente do platformio.ini, por exemplo: pio run -e test_ir"
#endif

#endif  // APP_ENTRY_POINTS_H_
