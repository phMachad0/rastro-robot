/**
 * @file robot_app.cpp
 *
 * @brief Aplicação principal: dá partida nas tarefas e sai do caminho.
 *
 * Toda a lógica vive nas tarefas de `tasks.cpp`. Esta função existe para
 * configurar a serial, criar as tarefas e falhar de forma visível se a criação
 * não der certo.
 *
 * Ver docs/ARQUITETURA.md, seções 4 e 6.
 */

#include "app/robot_app.h"

#include <Arduino.h>

#include "app/tasks.h"
#include "config/board_pins.h"
#include "config/features.h"
#include "config/robot_config.h"
#include "control/stop_policy.h"

namespace {

bool g_started = false;

}  // namespace

void robotAppSetup()
{
   Serial.begin(cfg::SERIAL_BAUD);

   Serial.println();
   Serial.println("=== rastro-robot ===");
   Serial.printf("controle %lu Hz | array IR %lu Hz | IMU %lu Hz | supervisor %lu Hz\n",
                 (unsigned long)cfg::CONTROL_HZ,
                 (unsigned long)cfg::IR_SAMPLE_HZ,
                 (unsigned long)cfg::IMU_SAMPLE_HZ,
                 (unsigned long)cfg::SUPERVISOR_HZ);
   Serial.printf("politica de parada: %s | marcacao lateral: %s\n",
                 activeStopPolicy().name(),
                 FEATURE_LATERAL_MARKS ? "ligada" : "desligada");
   Serial.printf("amortecimento por giroscopio: %s | ganhos adaptativos: %s\n",
                 FEATURE_GYRO_DAMPING ? "ligado" : "desligado",
                 FEATURE_ADAPTIVE_GAINS ? "ligados" : "desligados");
   Serial.printf("botao de start/stop: GPIO%u\n", pins::BTN_START);

   g_started = tasksStart();

   if (!g_started) {
      Serial.println("ERRO FATAL: nao foi possivel criar tarefas ou primitivas.");
      return;
   }

   Serial.println("Tarefas em execucao. Pressione o botao para armar.");
}

void robotAppLoop()
{
   /* A tarefa do laço do Arduino não tem papel aqui. Ela existe no núcleo 1 com
      prioridade 1 e qualquer trabalho feito nela competiria com a malha de
      controle, então o certo é deixá-la dormindo.

      Com a partida falhada, serve de indicação: repete o erro em vez de deixar a
      placa silenciosa e aparentemente funcionando. */
   if (!g_started) {
      Serial.println("ERRO FATAL: tarefas nao iniciadas.");
      vTaskDelay(pdMS_TO_TICKS(2000));
      return;
   }

   vTaskDelay(pdMS_TO_TICKS(1000));
}
