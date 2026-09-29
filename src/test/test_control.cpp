/**
 * @file test_control.cpp
 *
 * @brief Malha de controle completa, com os motores desabilitados.
 *
 * Roda `IrArray` → `LineFollower` → (PID + GyroDamper + TrackClassifier) na
 * mesma cadência do firmware e emite cada termo separadamente, sem acionar
 * nada. Dá para conferir sinal e ordem de grandeza da ação movendo o robô à mão
 * sobre a linha, sem risco de ele sair correndo pela bancada.
 *
 * O `MotorDriver` **não está compilado neste binário** (ver `build_src_filter`
 * do ambiente `test_control` no platformio.ini). Não é só que ele não é chamado:
 * o código não existe aqui, então não há caminho possível para energizar um
 * motor.
 *
 * Três verificações, em ordem de importância:
 *
 * 1. **Sinal do giroscópio.** Girando o robô à mão no sentido horário visto de
 *    cima, `yawRate` precisa sair **positivo**. Se sair negativo, inverter
 *    cfg::GYRO_Z_REVERSED. Com o sinal errado o amortecimento do RF06 deixa de
 *    ser realimentação negativa e passa a reforçar a rotação, levando a
 *    oscilação crescente — é a falha mais perigosa da etapa.
 *
 * 2. **Sinal da ação.** Deslocando o robô para a **esquerda** da linha (linha
 *    aparece à direita do array), `error` e `action` precisam sair **positivos**,
 *    e `left` precisa ficar maior que `right` — o robô viraria à direita, de
 *    volta para a linha. Sinais invertidos aqui fariam o robô fugir da linha.
 *
 * 3. **Troca de regime.** Com o robô parado, `blend` deve ir a 0 (reta) depois
 *    de alguns décimos de segundo. Girando continuamente, deve ir a 1 (curva),
 *    e os ganhos `kp` e `kg` devem caminhar junto, sem salto.
 *
 * Séries no Teleplot: `error`, `errorNorm`, `pTerm`, `iTerm`, `dTerm`,
 * `gyroTerm`, `action`, `left`, `right`, `yawRate`, `avgRate`, `blend`, `kp`,
 * `kg`, `baseSpeed`, `lineLost`, `satPid`, `satMix`.
 */

#include "test/test_control.h"

#include <Arduino.h>

#include "config/features.h"
#include "config/robot_config.h"
#include "control/line_follower.h"
#include "hal/imu.h"
#include "hal/ir_array.h"
#include "test/teleplot.h"

namespace {

IrArray      ir;
Imu          imu;
LineFollower follower;

bool imu_ok = false;

/** @brief Instante do ciclo anterior, para o dt real da malha. */
uint32_t last_cycle_us = 0;

}  // namespace

void testControlSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   Serial.println();
   Serial.println("=== Teste de malha fechada (motores fora do binario) ===");

   ir.begin();
   Serial.printf("IrArray: %u canais, limiar %u\n",
                 pins::IR_CHANNEL_COUNT, cfg::IR_BLACK_THRESHOLD);

   if (!imu.begin()) {
      Serial.println("ERRO: MPU-6050 nao respondeu. A malha roda sem amortecimento.");
   } else {
      Serial.printf("Estimando bias do giroscopio (%u amostras). NAO MOVER O ROBO.\n",
                    cfg::GYRO_BIAS_SAMPLES);

      if (imu.calibrateGyroBias()) {
         Serial.printf("Bias de gyroZ: %.3f deg/s\n", imu.gyroBiasZ());
         imu_ok = true;
      } else {
         Serial.println("ERRO: a IMU parou de entregar amostras durante a estimativa.");
      }
   }

   Serial.printf("Amortecimento por giroscopio: %s | ganhos adaptativos: %s\n",
                 FEATURE_GYRO_DAMPING ? "ligado" : "desligado",
                 FEATURE_ADAPTIVE_GAINS ? "ligados" : "desligados");
   Serial.printf("Inversao de gyroZ: %s\n", cfg::GYRO_Z_REVERSED ? "sim" : "nao");

   follower.reset();

   Serial.println("\nConferir, nesta ordem:");
   Serial.println(" 1. girando no sentido horario, yawRate sai POSITIVO");
   Serial.println(" 2. robo a esquerda da linha, error e action saem POSITIVOS e left > right");
   Serial.println(" 3. parado, blend vai a 0 (reta); girando, vai a 1 (curva)");

   last_cycle_us = micros();
}

void testControlLoop()
{
   /* Cadência da malha do firmware, para que o dt e o comportamento do termo
      derivativo sejam os mesmos que na pista. */
   delay(cfg::CONTROL_PERIOD_MS);

   const uint32_t now_us = micros();
   const float dt = (now_us - last_cycle_us) / 1000000.0f;
   last_cycle_us = now_us;

   ir.sample();

   /* Sem IMU válida a malha roda com taxa angular zero: o termo de amortecimento
      fica neutro e o classificador fica preso em reta, o que ainda permite
      conferir o caminho do PID. */
   float yaw_rate = 0.0f;

   if (imu_ok && imu.update()) {
      yaw_rate = imu.yawRate();
   }

   follower.update(ir.error(), ir.lineDetected(), yaw_rate, dt);

   /* Entrada do array */
   teleplot::send("position", (int32_t)ir.position());
   teleplot::send("error", (int32_t)ir.error());
   teleplot::send("detectedCount", (int32_t)ir.detectedCount());
   teleplot::send("lineLost", (int32_t)(follower.lineLost() ? 1 : 0));

   /* Termos do PID, separados — é o que permite ver qual deles domina */
   teleplot::send("errorNorm", follower.normalizedError());
   teleplot::send("pTerm", follower.pid().proportionalTerm());
   teleplot::send("iTerm", follower.pid().integralTerm());
   teleplot::send("dTerm", follower.pid().derivativeTerm());
   teleplot::send("integral", follower.pid().integral());
   teleplot::send("satPid", (int32_t)(follower.pid().saturated() ? 1 : 0));

   /* Amortecimento e regime */
   teleplot::send("yawRate", yaw_rate, "deg/s");
   teleplot::send("gyroTerm", follower.damper().lastTerm());
   teleplot::send("avgRate", follower.classifier().averageRate(), "deg/s");
   teleplot::send("blend", follower.classifier().blend());
   teleplot::send("kp", follower.classifier().gains().kp);
   teleplot::send("kg", follower.classifier().gains().kg);

   /* Saída da malha — o que iria para os motores */
   teleplot::send("action", follower.action());
   teleplot::send("baseSpeed", follower.baseSpeed());
   teleplot::send("left", follower.left());
   teleplot::send("right", follower.right());
   teleplot::send("satMix", (int32_t)(follower.mixSaturated() ? 1 : 0));
}
