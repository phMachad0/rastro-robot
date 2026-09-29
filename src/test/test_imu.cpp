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
 * Visualização: Serial Plotter, acompanhando gz enquanto o robô é girado à mão.
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
   Serial.println("Formato do Serial Plotter: accX accY accZ gyroX gyroY gyroZ");
}

void testImuLoop()
{
   int16_t ax, ay, az;
   int16_t gx, gy, gz;

   mpu.getAcceleration(&ax, &ay, &az);
   mpu.getRotation(&gx, &gy, &gz);

   /* Formato do Serial Plotter: rótulo:valor separado por tabulação, uma
      amostra por linha. gyroZ é o canal que importa para o controle. */
   Serial.printf("accX:%.2f\taccY:%.2f\taccZ:%.2f\t"
                 "gyroX:%.1f\tgyroY:%.1f\tgyroZ:%.1f\n",
                 ax / SCALE_ACCEL, ay / SCALE_ACCEL, az / SCALE_ACCEL,
                 gx / SCALE_GYRO, gy / SCALE_GYRO, gz / SCALE_GYRO);

   delay(REFRESH_MS);
}
