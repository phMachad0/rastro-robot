/**
 * @file stop_policy.h
 *
 * @brief Política de parada e retomada — RF03 e RF04.
 *
 * O ciclo parar–retomar fica atrás de uma interface para que trocar o critério
 * não toque em nenhum outro módulo. É essa a razão da interface existir mesmo
 * com uma das duas implementações desligada: o supervisor é escrito uma vez, e
 * ligar a marcação lateral depois não o modifica.
 *
 * A seleção é em tempo de compilação, por FEATURE_LATERAL_MARKS.
 *
 * Ver docs/ARQUITETURA.md, seção 6.1.
 */

#ifndef CONTROL_STOP_POLICY_H_
#define CONTROL_STOP_POLICY_H_

#include "app/robot_state.h"
#include "config/features.h"

class StopPolicy {
 public:
   virtual ~StopPolicy() {}

   /** @brief RF03: true quando o robô deve interromper o deslocamento. */
   virtual bool shouldStop(const RobotState& state) = 0;

   /** @brief RF04: true quando o robô deve retomar o percurso. */
   virtual bool shouldResume(const RobotState& state) = 0;

   /** @brief Avisa que uma marcação foi reconhecida. Ignorado por TimedPolicy. */
   virtual void onLapEvent() {}

   /** @brief Zera o estado interno da política. */
   virtual void reset() {}

   /** @brief Nome curto, para telemetria. */
   virtual const char* name() const = 0;
};

/**
 * @brief Política por tempo — padrão, ativa com FEATURE_LATERAL_MARKS em 0.
 *
 * Para depois de cfg::RUN_DURATION_MS em RUNNING e retoma depois de
 * cfg::STOP_DURATION_MS em STOPPED. Não depende de nenhum sensor além do relógio,
 * o que a torna utilizável para validar RF03 e RF04 antes de existir detecção de
 * marcação.
 */
class TimedPolicy : public StopPolicy {
 public:
   bool shouldStop(const RobotState& state) override;
   bool shouldResume(const RobotState& state) override;
   const char* name() const override { return "timed"; }
};

#if FEATURE_LATERAL_MARKS
/**
 * @brief Política por contagem de marcações — RF02 ligado.
 *
 * Não é compilada com a flag em zero.
 */
class MarkCountPolicy : public StopPolicy {
 public:
   bool shouldStop(const RobotState& state) override;
   bool shouldResume(const RobotState& state) override;
   void onLapEvent() override;
   void reset() override;
   const char* name() const override { return "marks"; }

 private:
   uint8_t laps_at_last_stop_ = 0;
   uint8_t laps_ = 0;
};
#endif

/**
 * @brief Devolve a política ativa nesta compilação.
 *
 * Instância estática: não há alocação dinâmica no caminho de controle.
 */
StopPolicy& activeStopPolicy();

#endif  // CONTROL_STOP_POLICY_H_
