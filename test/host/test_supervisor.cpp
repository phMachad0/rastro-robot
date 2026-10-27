/**
 * @file test_supervisor.cpp
 *
 * @brief Verificacao da maquina de estados no host.
 *
 * Supervisor e StopPolicy nao dependem de nada do Arduino, e o relogio entra por
 * parametro em update() — o que torna RF03 e RF04 verificaveis de forma
 * deterministica, com tempo simulado, em vez de cronometrando o robo na pista.
 *
 * Roda com test/host/run.sh.
 *
 * Ver docs/ARQUITETURA.md, secoes 6 e 8.4.
 */

#include <cstdio>
#include <cstdint>
#include "app/supervisor.h"
#include "config/features.h"
#include "config/robot_config.h"

static int failures = 0;

static void check(bool ok, const char* name, const char* detail = "")
{
   printf("%s  %s %s\n", ok ? "[ ok ]" : "[FAIL]", name, detail);
   if (!ok) failures++;
}

// Relogio simulado, em passos do periodo do supervisor.
struct Clock {
   uint32_t ms = 0;
   void tick() { ms += cfg::SUPERVISOR_PERIOD_MS; }
};

// Leva o supervisor de BOOT ate IDLE.
static void bootToIdle(Supervisor& sup, Clock& clk)
{
   sup.update(Command::NONE, ImuInitStatus::CALIBRATING, 0, clk.ms);
   clk.tick();
   sup.update(Command::NONE, ImuInitStatus::READY, 0, clk.ms);
   clk.tick();
}

// Avanca o tempo rodando o supervisor, por `ms` milissegundos.
static void run(Supervisor& sup, Clock& clk, uint32_t ms, uint32_t line_lost_ms = 0)
{
   const uint32_t target = clk.ms + ms;
   while (clk.ms < target) {
      sup.update(Command::NONE, ImuInitStatus::READY, line_lost_ms, clk.ms);
      clk.tick();
   }
}

/**
 * Avanca ate o modo mudar e devolve quanto tempo o estado durou, em ms.
 *
 * Medir a duracao e mais forte do que checar "antes do prazo ainda esta parado":
 * esse tipo de checagem passa com qualquer prazo maior que a margem escolhida,
 * enquanto a duracao medida falha se o prazo estiver errado em qualquer direcao.
 *
 * Devolve UINT32_MAX se o modo nao mudou dentro do limite.
 */
static uint32_t timeInState(Supervisor& sup, Clock& clk, uint32_t limit_ms)
{
   const RobotMode start = sup.mode();
   const uint32_t entered = clk.ms;

   while (clk.ms - entered <= limit_ms) {
      sup.update(Command::NONE, ImuInitStatus::READY, 0, clk.ms);

      if (sup.mode() != start) {
         return clk.ms - entered;
      }

      clk.tick();
   }

   return UINT32_MAX;
}

// A transicao e avaliada a cada periodo do supervisor, entao o prazo so pode ser
// cumprido com granularidade de um periodo.
static bool withinOnePeriod(uint32_t measured, uint32_t expected)
{
   return measured >= expected && measured < expected + cfg::SUPERVISOR_PERIOD_MS;
}

int main()
{
   // ---- 1. BOOT -> CALIBRATING -> IDLE ----
   {
      Supervisor sup; Clock clk;
      sup.begin();
      check(sup.mode() == RobotMode::BOOT, "nasce em BOOT");
      check(!sup.motorsAllowed(), "BOOT nao autoriza motores");

      sup.update(Command::NONE, ImuInitStatus::CALIBRATING, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::CALIBRATING, "IMU calibrando -> CALIBRATING");
      check(!sup.motorsAllowed(), "CALIBRATING nao autoriza motores");

      sup.update(Command::NONE, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::IDLE, "IMU pronta -> IDLE");
      check(!sup.motorsAllowed(), "IDLE nao autoriza motores");
   }

   // ---- 2. BOOT -> FAULT sem IMU ----
   {
      Supervisor sup; Clock clk;
      sup.begin();
      sup.update(Command::NONE, ImuInitStatus::FAILED, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::FAULT, "IMU ausente -> FAULT");
      check(sup.state().fault == FaultReason::IMU_ABSENT, "motivo da falha registrado");
   }

   // ---- 3. IMU falha durante a calibracao ----
   {
      Supervisor sup; Clock clk;
      sup.begin();
      sup.update(Command::NONE, ImuInitStatus::CALIBRATING, 0, clk.ms); clk.tick();
      sup.update(Command::NONE, ImuInitStatus::FAILED, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::FAULT, "falha durante calibracao -> FAULT");
   }

   // ---- 4. Partida: IDLE -> RUNNING ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);

      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::RUNNING, "START -> RUNNING");
      check(sup.motorsAllowed(), "RUNNING e o UNICO estado que autoriza motores");
   }

   // ---- 5. RF03: para sozinho depois de RUN_DURATION_MS ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms);
      clk.tick();   // o estado RUNNING comeca a contar a partir daqui

      const uint32_t running_ms = timeInState(sup, clk, cfg::RUN_DURATION_MS * 2);
      check(sup.mode() == RobotMode::STOPPED, "RF03: RUNNING -> STOPPED");
      check(withinOnePeriod(running_ms, cfg::RUN_DURATION_MS - cfg::SUPERVISOR_PERIOD_MS),
            "RF03: duracao da corrida bate com RUN_DURATION_MS");
      check(!sup.motorsAllowed(), "RF03: STOPPED nao autoriza motores (fica parado)");
      printf("       correu %u ms (RUN_DURATION_MS = %u)\n",
             running_ms + cfg::SUPERVISOR_PERIOD_MS, cfg::RUN_DURATION_MS);
   }

   // ---- 6. RF04: retoma depois de STOP_DURATION_MS ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      timeInState(sup, clk, cfg::RUN_DURATION_MS * 2);
      check(sup.mode() == RobotMode::STOPPED, "preparado: em STOPPED");
      clk.tick();

      const uint32_t stopped_ms = timeInState(sup, clk, cfg::STOP_DURATION_MS * 3);
      check(sup.mode() == RobotMode::RUNNING, "RF04: STOPPED -> RUNNING");
      check(withinOnePeriod(stopped_ms, cfg::STOP_DURATION_MS - cfg::SUPERVISOR_PERIOD_MS),
            "RF04: duracao da parada bate com STOP_DURATION_MS");
      check(sup.state().stop_count == 1, "ciclo de parada contado");
      check(sup.state().lap_count == 0, "lap_count fica em zero com RF02 desligado");
      printf("       ficou parado %u ms (STOP_DURATION_MS = %u)\n",
             stopped_ms + cfg::SUPERVISOR_PERIOD_MS, cfg::STOP_DURATION_MS);
   }

   // ---- 7. Ciclo repetido: tres paradas seguidas, cada uma com o prazo certo ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();

      bool all_runs_ok = true;
      bool all_stops_ok = true;

      for (int i = 0; i < 3; i++) {
         const uint32_t ran = timeInState(sup, clk, cfg::RUN_DURATION_MS * 2);
         all_runs_ok &= (sup.mode() == RobotMode::STOPPED) &&
                        withinOnePeriod(ran, cfg::RUN_DURATION_MS - cfg::SUPERVISOR_PERIOD_MS);
         clk.tick();

         const uint32_t stopped = timeInState(sup, clk, cfg::STOP_DURATION_MS * 3);
         all_stops_ok &= (sup.mode() == RobotMode::RUNNING) &&
                         withinOnePeriod(stopped, cfg::STOP_DURATION_MS - cfg::SUPERVISOR_PERIOD_MS);
         clk.tick();
      }

      check(all_runs_ok, "tres corridas, todas com o prazo de RUN_DURATION_MS");
      check(all_stops_ok, "tres paradas, todas com o prazo de STOP_DURATION_MS");
      check(sup.state().stop_count == 3, "tres ciclos parar-retomar contados");
      check(sup.state().lap_count == 0, "lap_count segue em zero apos tres ciclos");
   }

   // ---- 8. Linha perdida -> FAULT ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();

      run(sup, clk, 100, cfg::LINE_LOST_TIMEOUT_MS - 1);
      check(sup.mode() == RobotMode::RUNNING, "abaixo do timeout, segue rodando");

      sup.update(Command::NONE, ImuInitStatus::READY, cfg::LINE_LOST_TIMEOUT_MS, clk.ms);
      check(sup.mode() == RobotMode::FAULT, "linha perdida alem do timeout -> FAULT");
      check(sup.state().fault == FaultReason::LINE_LOST, "motivo LINE_LOST registrado");
      check(!sup.motorsAllowed(), "FAULT nao autoriza motores");
   }

   // ---- 9. Linha perdida tem precedencia sobre a parada programada ----
   //
   // Se o robo saiu da pista, parar por tempo seria parar no lugar errado e
   // depois retomar numa direcao sem sentido.
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      run(sup, clk, cfg::RUN_DURATION_MS - 100);

      // prazo de parada E linha perdida satisfeitos no mesmo ciclo
      run(sup, clk, 200, cfg::LINE_LOST_TIMEOUT_MS);
      check(sup.mode() == RobotMode::FAULT, "FAULT vence a parada programada");
   }

   // ---- 10. FAULT so sai por RESET ----
   {
      Supervisor sup; Clock clk;
      sup.begin();
      sup.update(Command::NONE, ImuInitStatus::FAILED, 0, clk.ms); clk.tick();

      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::FAULT, "START ignorado em FAULT");

      sup.update(Command::TOGGLE_RUN, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::FAULT, "TOGGLE ignorado em FAULT");

      sup.update(Command::STOP, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::FAULT, "STOP nao tira de FAULT");

      sup.update(Command::RESET, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::IDLE, "RESET -> IDLE");
      check(sup.state().fault == FaultReason::NONE, "falha limpa no reset");
   }

   // ---- 11. Botao alterna RUNNING e IDLE ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);

      sup.update(Command::TOGGLE_RUN, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::RUNNING, "botao em IDLE -> RUNNING");

      sup.update(Command::TOGGLE_RUN, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::IDLE, "botao em RUNNING -> IDLE");
   }

   // ---- 12. STOP desarma a partir de RUNNING e de STOPPED ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      sup.update(Command::STOP, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::IDLE, "STOP em RUNNING -> IDLE");

      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      timeInState(sup, clk, cfg::RUN_DURATION_MS * 2);
      check(sup.mode() == RobotMode::STOPPED, "preparado: em STOPPED");
      sup.update(Command::STOP, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::IDLE, "STOP em STOPPED -> IDLE");
   }

   // ---- 13. Retomada zera o prazo de corrida ----
   //
   // Sem isso o segundo trecho de corrida terminaria imediatamente, porque o
   // tempo acumulado ainda estaria acima de RUN_DURATION_MS.
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      sup.update(Command::START, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      timeInState(sup, clk, cfg::RUN_DURATION_MS * 2);
      clk.tick();
      timeInState(sup, clk, cfg::STOP_DURATION_MS * 3);
      check(sup.mode() == RobotMode::RUNNING, "preparado: retomou");
      clk.tick();

      const uint32_t second_run = timeInState(sup, clk, cfg::RUN_DURATION_MS * 2);
      check(withinOnePeriod(second_run, cfg::RUN_DURATION_MS - cfg::SUPERVISOR_PERIOD_MS),
            "segundo trecho dura o prazo inteiro, nao termina na hora");
      /* stop_count conta ciclos CONCLUIDOS, e incrementa na retomada. Neste ponto
         o robo acabou de entrar na segunda parada, entao so o primeiro ciclo
         fechou — 1, e nao 2. */
      check(sup.mode() == RobotMode::STOPPED, "segundo trecho terminou em STOPPED");
      check(sup.state().stop_count == 1, "apenas o primeiro ciclo fechou ate aqui");
   }

#if FEATURE_AUTOSTART
   // ---- 14. Partida automatica: IDLE arranca sozinho ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      check(sup.mode() == RobotMode::IDLE, "preparado: em IDLE");

      const uint32_t idle_ms = timeInState(sup, clk, cfg::AUTOSTART_DELAY_MS * 3);
      check(sup.mode() == RobotMode::RUNNING, "IDLE arranca sozinho, sem comando");
      check(withinOnePeriod(idle_ms, cfg::AUTOSTART_DELAY_MS - cfg::SUPERVISOR_PERIOD_MS),
            "espera AUTOSTART_DELAY_MS antes de arrancar");
      printf("       esperou %u ms (AUTOSTART_DELAY_MS = %u)\n",
             idle_ms + cfg::SUPERVISOR_PERIOD_MS, cfg::AUTOSTART_DELAY_MS);
   }

   // ---- 14b. Nao arranca antes do prazo ----
   {
      Supervisor sup; Clock clk;
      sup.begin(); bootToIdle(sup, clk);
      run(sup, clk, cfg::AUTOSTART_DELAY_MS / 2);
      check(sup.mode() == RobotMode::IDLE, "metade do prazo: ainda em IDLE");
      check(!sup.motorsAllowed(), "nao autoriza motores antes do prazo");
   }

   // ---- 14c. FAULT nao arranca sozinho; so depois do reset ----
   //
   // Importante: a partida automatica vale para IDLE, nao para FAULT. Um robo que
   // saiu da pista nao pode voltar a andar sozinho.
   {
      Supervisor sup; Clock clk;
      sup.begin();
      sup.update(Command::NONE, ImuInitStatus::FAILED, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::FAULT, "preparado: em FAULT");

      run(sup, clk, cfg::AUTOSTART_DELAY_MS * 3);
      check(sup.mode() == RobotMode::FAULT, "FAULT nao arranca sozinho");

      sup.update(Command::RESET, ImuInitStatus::READY, 0, clk.ms); clk.tick();
      check(sup.mode() == RobotMode::IDLE, "reset leva a IDLE");
      run(sup, clk, cfg::AUTOSTART_DELAY_MS + cfg::SUPERVISOR_PERIOD_MS);
      check(sup.mode() == RobotMode::RUNNING, "depois do reset, arranca sozinho de novo");
   }
#endif

   // ---- 15. TimedPolicy isolada ----
   {
      StopPolicy& p = activeStopPolicy();
      RobotState s = {};

      s.mode_elapsed_ms = cfg::RUN_DURATION_MS - 1;
      check(!p.shouldStop(s), "shouldStop falso abaixo do prazo");
      s.mode_elapsed_ms = cfg::RUN_DURATION_MS;
      check(p.shouldStop(s), "shouldStop verdadeiro no prazo");

      s.mode_elapsed_ms = cfg::STOP_DURATION_MS - 1;
      check(!p.shouldResume(s), "shouldResume falso abaixo do prazo");
      s.mode_elapsed_ms = cfg::STOP_DURATION_MS;
      check(p.shouldResume(s), "shouldResume verdadeiro no prazo");
   }

   printf("\n%s (%d falhas)\n", failures ? "FALHOU" : "TODOS OS TESTES PASSARAM", failures);
   return failures ? 1 : 0;
}
