/**
 * @file test_imu.cpp
 *
 * @brief Leitura de aceleração e taxa angular do MPU-6050 via I2C.
 *
 * O eixo que interessa ao controle é o Z do giroscópio: é a taxa de guinada
 * do chassi, usada para amortecer o zigue-zague em reta (RF06) e para
 * classificar o trecho da pista entre reta e curva.
 *
 * Fundo de escala em ±500 °/s, e não os ±250 de antes: em curva fechada com o
 * robô rápido, ±250 satura e o termo de amortecimento perde o sinal justo
 * quando ele é mais necessário.
 *
 * Visualização: Teleplot, acompanhando gyroZ enquanto o robô é girado à mão.
 * As séries do acelerômetro saem em g e as do giroscópio em deg/s, de forma que
 * o Teleplot não as misture no mesmo eixo.
 *
 * @note A interrupção de data-ready no GPIO de pins::IMU_INT, a remoção de
 *       bias e o filtro passa-baixa chegam na etapa 2, com a classe Imu. Aqui
 *       a leitura ainda é por polling.
 */

#include "test/test_imu.h"

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>

#include "config/board_pins.h"
#include "config/robot_config.h"
#include "test/teleplot.h"

namespace {

MPU6050 mpu;

/** @brief Fator de escala do acelerômetro em ±2 g, em LSB por g. */
constexpr float SCALE_ACCEL = 16384.0f;

/** @brief Fator de escala do giroscópio em ±500 °/s, em LSB por °/s. */
constexpr float SCALE_GYRO = 65.5f;

constexpr uint32_t I2C_FREQ_HZ = 400000;
constexpr uint32_t REFRESH_MS  = 50;

}  // namespace

void testImuSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   Wire.begin(pins::I2C_SDA, pins::I2C_SCL, I2C_FREQ_HZ);

   Serial.println();
   Serial.println("Inicializando MPU-6050...");
   mpu.initialize();

   if (!mpu.testConnection()) {
      Serial.println("ERRO: MPU-6050 nao respondeu. Conferir SDA, SCL e alimentacao.");
      return;
   }

   mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

   Serial.println("MPU-6050 pronto.");
   Serial.printf("I2C: SDA=GPIO%u SCL=GPIO%u | INT (etapa 2): GPIO%u\n",
                 pins::I2C_SDA, pins::I2C_SCL, pins::IMU_INT);
   Serial.println("Series do Teleplot: accX accY accZ (g), gyroX gyroY gyroZ (deg/s)");
}

void testImuLoop()
{
   int16_t ax, ay, az;
   int16_t gx, gy, gz;

   mpu.getAcceleration(&ax, &ay, &az);
   mpu.getRotation(&gx, &gy, &gz);

   teleplot::send("accX", ax / SCALE_ACCEL, "g");
   teleplot::send("accY", ay / SCALE_ACCEL, "g");
   teleplot::send("accZ", az / SCALE_ACCEL, "g");

   teleplot::send("gyroX", gx / SCALE_GYRO, "deg/s");
   teleplot::send("gyroY", gy / SCALE_GYRO, "deg/s");

   /* gyroZ é a taxa de guinada: o único canal que o controle usa. */
   teleplot::send("gyroZ", gz / SCALE_GYRO, "deg/s");

   delay(REFRESH_MS);
}
