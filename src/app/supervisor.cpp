/**
 * @file supervisor.cpp
 *
 * @brief Implementação da máquina de estados.
 */

#include "app/supervisor.h"

#include "config/features.h"
#include "config/robot_config.h"

void Supervisor::begin()
{
   policy_ = &activeStopPolicy();
   policy_->reset();

   state_ = {};
   state_.mode = RobotMode::BOOT;
   state_.fault = FaultReason::NONE;
   state_.lap_count = 0;

   mode_entered_ms_ = 0;
   just_changed_ = true;
}

void Supervisor::notifyLap()
{
   state_.lap_count++;
   policy_->onLapEvent();
}

void Supervisor::enter(RobotMode mode, uint32_t now_ms)
{
   state_.mode = mode;
   mode_entered_ms_ = now_ms;
   state_.mode_elapsed_ms = 0;
   just_changed_ = true;
}

const char* Supervisor::modeName(RobotMode mode)
{
   switch (mode) {
      case RobotMode::BOOT:        return "BOOT";
      case RobotMode::CALIBRATING: return "CALIBRATING";
      case RobotMode::IDLE:        return "IDLE";
      case RobotMode::RUNNING:     return "RUNNING";
      case RobotMode::STOPPED:     return "STOPPED";
      case RobotMode::FAULT:       return "FAULT";
   }
   return "?";
}

const char* Supervisor::faultName(FaultReason reason)
{
   switch (reason) {
      case FaultReason::NONE:       return "-";
      case FaultReason::IMU_ABSENT: return "IMU_ABSENT";
      case FaultReason::IMU_SILENT: return "IMU_SILENT";
      case FaultReason::LINE_LOST:  return "LINE_LOST";
   }
   return "?";
}

void Supervisor::update(Command command, ImuInitStatus imu_status, uint32_t line_lost_ms,
                        uint32_t now_ms)
{
   just_changed_ = false;

   state_.mode_elapsed_ms = now_ms - mode_entered_ms_;
   state_.timestamp_ms = now_ms;

   /*****************************************
    * Comandos que valem em qualquer estado
    *
    * O reset é tratado antes da máquina porque é a única saída de FAULT, e o
    * stop antes porque desarmar precisa funcionar mesmo que a lógica de estado
    * tenha chegado a um ponto não previsto.
    *****************************************/
   if (command == Command::RESET) {
      state_.fault = FaultReason::NONE;
      policy_->reset();
      state_.lap_count = 0;
      state_.stop_count = 0;
      enter(RobotMode::IDLE, now_ms);
      return;
   }

   if (command == Command::STOP && state_.mode != RobotMode::FAULT) {
      enter(RobotMode::IDLE, now_ms);
      return;
   }

   switch (state_.mode) {
      case RobotMode::BOOT:
         /* O auto-teste é a própria inicialização da IMU, que roda na TaskImu
            porque é ela que detém o barramento I2C. O supervisor só observa o
            resultado — é o que evita dois núcleos conversando com o sensor. */
         if (imu_status == ImuInitStatus::FAILED) {
            state_.fault = FaultReason::IMU_ABSENT;
            enter(RobotMode::FAULT, now_ms);
         } else if (imu_status == ImuInitStatus::CALIBRATING) {
            enter(RobotMode::CALIBRATING, now_ms);
         } else if (imu_status == ImuInitStatus::READY) {
            enter(RobotMode::IDLE, now_ms);
         }
         break;

      case RobotMode::CALIBRATING:
         if (imu_status == ImuInitStatus::FAILED) {
            state_.fault = FaultReason::IMU_ABSENT;
            enter(RobotMode::FAULT, now_ms);
         } else if (imu_status == ImuInitStatus::READY) {
            enter(RobotMode::IDLE, now_ms);
         }
         break;

      case RobotMode::IDLE:
         if (command == Command::START || command == Command::TOGGLE_RUN) {
            policy_->reset();
            enter(RobotMode::RUNNING, now_ms);
            break;
         }

#if FEATURE_AUTOSTART
         /* Vale em toda entrada em IDLE, inclusive depois de um reset de falha:
            o robô volta a andar sozinho 3 s após sair de FAULT. É o que se quer
            enquanto não há comando nenhum disponível. */
         if (state_.mode_elapsed_ms >= cfg::AUTOSTART_DELAY_MS) {
            policy_->reset();
            enter(RobotMode::RUNNING, now_ms);
         }
#endif
         break;

      case RobotMode::RUNNING:
         if (command == Command::TOGGLE_RUN) {
            enter(RobotMode::IDLE, now_ms);
            break;
         }

         /* A falha por linha perdida tem precedência sobre a parada programada:
            se o robô saiu da pista, parar por tempo seria parar no lugar errado
            e depois retomar numa direção sem sentido. */
         if (line_lost_ms >= cfg::LINE_LOST_TIMEOUT_MS) {
            state_.fault = FaultReason::LINE_LOST;
            enter(RobotMode::FAULT, now_ms);
            break;
         }

         if (policy_->shouldStop(state_)) {
            enter(RobotMode::STOPPED, now_ms);
         }
         break;

      case RobotMode::STOPPED:
         if (command == Command::TOGGLE_RUN) {
            enter(RobotMode::IDLE, now_ms);
            break;
         }

         if (policy_->shouldResume(state_)) {
            state_.stop_count++;
            enter(RobotMode::RUNNING, now_ms);
         }
         break;

      case RobotMode::FAULT:
         /* Só sai por Command::RESET, tratado no início desta função. */
         break;
   }
}
