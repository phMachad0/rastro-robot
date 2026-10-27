/**
 * @file supervisor.h
 *
 * @brief Máquina de estados do robô.
 *
 * Decide em que estado o robô está e, por consequência, se os motores podem ser
 * armados. Não toca em motor nem em sensor: publica o modo, e a TaskControl é
 * quem traduz isso em acionamento. A parada é feita na camada que aciona os
 * motores, não por convenção entre tarefas.
 *
 * Ver docs/ARQUITETURA.md, seção 6.
 */

#ifndef APP_SUPERVISOR_H_
#define APP_SUPERVISOR_H_

#include <stdint.h>

#include "app/robot_state.h"
#include "control/stop_policy.h"

class Supervisor {
 public:
   void begin();

   /**
    * @brief Avança a máquina de estados um ciclo.
    *
    * @param command     Comando recebido, ou Command::NONE.
    * @param imu_status  Progresso da inicialização da IMU, vindo da TaskImu.
    * @param line_lost_ms Tempo contínuo sem linha reportado pela malha.
    * @param now_ms      Relógio.
    */
   void update(Command command, ImuInitStatus imu_status, uint32_t line_lost_ms,
               uint32_t now_ms);

   /**
    * @brief Registra uma marcação lateral reconhecida — RF02.
    *
    * Concentrado aqui, e não na tarefa, para que a contagem e o aviso à política
    * aconteçam sempre juntos: separá-los permitiria uma política contar uma volta
    * que a telemetria não reporta, ou o contrário.
    */
   void notifyLap();

   const RobotState& state() const { return state_; }

   RobotMode mode() const { return state_.mode; }

   /** @brief true apenas em RUNNING. É o que autoriza armar os motores. */
   bool motorsAllowed() const { return state_.mode == RobotMode::RUNNING; }

   /** @brief true no ciclo em que o estado acabou de mudar. */
   bool justChanged() const { return just_changed_; }

   /** @brief Nome do estado, para telemetria. */
   static const char* modeName(RobotMode mode);

   static const char* faultName(FaultReason reason);

 private:
   void enter(RobotMode mode, uint32_t now_ms);

   RobotState state_ = {};
   uint32_t   mode_entered_ms_ = 0;
   bool       just_changed_ = false;
   StopPolicy* policy_ = nullptr;
};

#endif  // APP_SUPERVISOR_H_
