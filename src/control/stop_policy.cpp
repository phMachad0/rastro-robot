/**
 * @file stop_policy.cpp
 *
 * @brief Implementação das políticas de parada.
 */

#include "control/stop_policy.h"

#include "config/robot_config.h"

bool TimedPolicy::shouldStop(const RobotState& state)
{
   return state.mode_elapsed_ms >= cfg::RUN_DURATION_MS;
}

bool TimedPolicy::shouldResume(const RobotState& state)
{
   return state.mode_elapsed_ms >= cfg::STOP_DURATION_MS;
}

#if FEATURE_LATERAL_MARKS
void MarkCountPolicy::reset()
{
   laps_ = 0;
   laps_at_last_stop_ = 0;
}

void MarkCountPolicy::onLapEvent()
{
   laps_++;
}

bool MarkCountPolicy::shouldStop(const RobotState& state)
{
   (void)state;
   return (laps_ - laps_at_last_stop_) >= cfg::LAPS_TARGET;
}

bool MarkCountPolicy::shouldResume(const RobotState& state)
{
   /* A retomada por marcação não faz sentido com o robô parado — ele não cruza
      marcação nenhuma. Enquanto RF02 não estiver implementado de verdade, cai
      para o tempo de parada, para que o robô não fique travado para sempre. */
   return state.mode_elapsed_ms >= cfg::STOP_DURATION_MS;
}
#endif

StopPolicy& activeStopPolicy()
{
#if FEATURE_LATERAL_MARKS
   static MarkCountPolicy policy;
#else
   static TimedPolicy policy;
#endif
   return policy;
}
