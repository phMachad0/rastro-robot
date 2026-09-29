/**
 * @file test_imu.cpp
 *
 * @brief Teste do MPU-6050 através da classe Imu.
 *
 * Valida o caminho completo que o controle vai usar: interrupção de data-ready,
 * remoção de bias e filtro — não mais o polling cru da etapa 1.
 *
 * O que observar no Teleplot:
 *   - `rateHz`: deve ficar em torno de cfg::IMU_SAMPLE_HZ. Se cair para perto
 *     de zero, o pino de interrupção não está ligado ou a configuração do
 *     sensor não pegou — é o teste mais direto de que a ISR está chegando.
 *   - `gyroZ` e `gyroZraw`: com o robô imóvel devem ficar em zero. Se `gyroZraw`
 *     tem um patamar constante e `gyroZ` não, a remoção de bias está agindo.
 *   - Girando o robô à mão, `gyroZ` acompanha `gyroZraw` com um pequeno atraso:
 *     é o filtro IIR.
 *
 * @warning O robô precisa estar parado e apoiado durante a estimativa de bias,
 *          logo após o boot. Movê-lo nessa janela grava o movimento como zero.
 */

#include "test/test_imu.h"

#include <Arduino.h>

#include "config/board_pins.h"
#include "config/robot_config.h"
#include "hal/imu.h"
#include "test/teleplot.h"

namespace {

Imu imu;

bool imu_ok = false;

}  // namespace

void testImuSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   Serial.println();
   Serial.println("Inicializando MPU-6050...");

   if (!imu.begin()) {
      Serial.println("ERRO: MPU-6050 nao respondeu. Conferir SDA, SCL e alimentacao.");
      return;
   }

   Serial.printf("I2C: SDA=GPIO%u SCL=GPIO%u | data-ready: GPIO%u\n",
                 pins::I2C_SDA, pins::I2C_SCL, pins::IMU_INT);
   Serial.printf("Fundo de escala +-500 deg/s, DLPF ~44 Hz, taxa alvo %lu Hz\n",
                 (unsigned long)cfg::IMU_SAMPLE_HZ);

   Serial.printf("\nEstimando bias do giroscopio (%u amostras). NAO MOVER O ROBO.\n",
                 cfg::GYRO_BIAS_SAMPLES);

   if (!imu.calibrateGyroBias()) {
      Serial.println("ERRO: a IMU parou de entregar amostras durante a estimativa.");
      Serial.printf("Interrupcoes recebidas: %lu. Conferir a ligacao do pino INT.\n",
                    (unsigned long)imu.interruptCount());
      return;
   }

   Serial.printf("Bias de gyroZ: %.3f deg/s\n", imu.gyroBiasZ());
   Serial.println("Series do Teleplot: gyroZ, gyroZraw, accX/Y/Z, rateHz");

   imu_ok = true;
}

void testImuLoop()
{
   if (!imu_ok) {
      delay(1000);
      return;
   }

   if (!imu.update()) {
      Serial.println("AVISO: timeout esperando data-ready.");
      return;
   }

   teleplot::send("accX", imu.accelX(), "g");
   teleplot::send("accY", imu.accelY(), "g");
   teleplot::send("accZ", imu.accelZ(), "g");

   /* gyroZ é a taxa de guinada: o único canal que o controle usa. O cru vai
      junto para dar para ver o efeito do bias e do filtro lado a lado. */
   teleplot::send("gyroZ", imu.yawRate(), "deg/s");
   teleplot::send("gyroZraw", imu.yawRateRaw(), "deg/s");

   teleplot::send("rateHz", imu.measuredRateHz(), "Hz");
}
