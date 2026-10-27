/**
 * @file test_control.cpp
 *
 * @brief Verificacao da camada de controle no host, sob sanitizadores.
 *
 * Nao faz parte do firmware e nao roda na placa. Existe porque control/ nao
 * depende de nada do Arduino: e matematica pura sobre floats, entao da para
 * conferir sinais, limites de array e comportamento do anti-windup em dois
 * segundos no PC, em vez de descobrir na pista.
 *
 * Roda com test/host/run.sh. O script liga AddressSanitizer,
 * UndefinedBehaviorSanitizer e -fsanitize=bounds-strict; este ultimo e o que
 * detecta estouro de array membro de classe, que foi exatamente o defeito da
 * versao original do PidAlgorithm.
 *
 * Ver docs/ARQUITETURA.md, secao 8.4.
 */
#include <cstdio>
#include <cmath>
#include "control/line_follower.h"
#include "control/pid_algorithm.h"
#include "control/track_classifier.h"

static int failures = 0;

static void check(bool ok, const char* name, const char* detail = "")
{
   printf("%s  %s %s\n", ok ? "[ ok ]" : "[FAIL]", name, detail);
   if (!ok) failures++;
}

static bool near(float a, float b, float tol = 1e-4f) { return fabsf(a - b) < tol; }

int main()
{
   const float dt = 1.0f / cfg::CONTROL_HZ;

   // ---- 1. Fila circular da derivada: 3x a janela, ASan pega estouro ----
   {
      PidAlgorithm pid;
      pid.setGains(1.0f, 0.0f, 1.0f);
      for (int i = 0; i < cfg::PID_DERIVATIVE_WINDOW * 3; i++) pid.update(0.5f, dt);
      check(near(pid.derivativeTerm(), 0.0f), "erro constante -> derivada zero");
   }

   // ---- 2. Derivada sobre rampa: janela cheia da taxa correta ----
   {
      PidAlgorithm pid;
      pid.setGains(0.0f, 0.0f, 1.0f);
      float e = 0.0f;
      const float slope = 2.0f;                  // erro normalizado por segundo
      for (int i = 0; i < 200; i++) { pid.update(e, dt); e += slope * dt; }
      check(near(pid.derivativeTerm(), slope, 0.05f), "rampa -> derivada = inclinacao",
            "esperado 2.0");
      printf("       derivada medida = %.4f\n", pid.derivativeTerm());
   }

   // ---- 3. Integral usa dt: 1 s de erro 1.0 acumula 1.0 ----
   {
      PidAlgorithm pid;
      pid.setGains(0.0f, 1.0f, 0.0f);
      pid.setOutputLimit(1000.0f);               // afasta a saturacao do teste
      for (int i = 0; i < (int)cfg::CONTROL_HZ; i++) pid.update(1.0f, dt);
      check(near(pid.integral(), 1.0f, 0.02f), "1 s de erro 1.0 -> integral 1.0");
      printf("       integral = %.4f\n", pid.integral());
   }

   // ---- 4. Teto do integrador ----
   {
      PidAlgorithm pid;
      pid.setGains(0.0f, 1.0f, 0.0f);
      pid.setOutputLimit(1000.0f);
      for (int i = 0; i < (int)cfg::CONTROL_HZ * 10; i++) pid.update(1.0f, dt);
      check(pid.integral() <= cfg::PID_INTEGRAL_LIMIT + 1e-3f, "integral respeita o teto");
   }

   // ---- 5. Anti-windup: integral carrega, satura, congela ----
   //
   // Ganhos escolhidos para que a saida NAO sature de saida: com Kp alto o
   // proporcional sozinho satura no primeiro ciclo, o integrador nunca carrega e
   // o teste passaria sem exercitar nada.
   {
      PidAlgorithm pid;
      pid.setGains(0.1f, 2.0f, 0.0f);
      pid.setOutputLimit(1.0f);

      pid.update(1.0f, dt);
      check(!pid.saturated(), "nao satura no primeiro ciclo");

      for (int i = 0; i < 200; i++) pid.update(1.0f, dt);     // carrega e satura
      check(pid.saturated(), "satura depois de carregar a integral");
      const float frozen = pid.integral();
      check(frozen > 0.1f, "integral carregou antes de congelar");

      for (int i = 0; i < 500; i++) pid.update(1.0f, dt);     // 2 s saturado
      check(near(pid.integral(), frozen), "integral congelada durante saturacao");
      printf("       integral %.6f -> %.6f (2 s saturado)\n", frozen, pid.integral());

      // sem anti-windup, 2 s de erro 1.0 teriam somado ~2.0
      check(pid.integral() < 1.0f, "integral longe do que o windup daria");
   }

   // ---- 6. Anti-windup libera quando o erro inverte ----
   {
      PidAlgorithm pid;
      pid.setGains(0.1f, 2.0f, 0.0f);
      pid.setOutputLimit(1.0f);
      for (int i = 0; i < 300; i++) pid.update(1.0f, dt);     // carrega e satura
      const float charged = pid.integral();

      for (int i = 0; i < 50; i++) pid.update(-1.0f, dt);
      check(pid.integral() < charged, "integral descarrega com erro invertido");
      printf("       integral %.6f -> %.6f apos inverter o erro\n", charged, pid.integral());
   }

   // ---- 6b. Saturado, mas erro no sentido oposto: integra normalmente ----
   //
   // O anti-windup so deve travar quando o erro empurra a saida ainda mais para
   // o limite. Travar em toda saturacao impediria o integrador de descarregar e
   // seria pior que nao ter anti-windup nenhum.
   {
      PidAlgorithm pid;
      pid.setGains(0.1f, 2.0f, 0.0f);
      pid.setOutputLimit(1.0f);
      for (int i = 0; i < 300; i++) pid.update(1.0f, dt);     // saturado no positivo
      check(pid.saturated(), "preparado: saida saturada");

      const float before = pid.integral();
      pid.update(-0.2f, dt);                                   // erro contrario
      check(pid.integral() < before, "saturado com erro oposto -> integra (descarrega)");
   }

   // ---- 7. reset() zera tudo ----
   {
      PidAlgorithm pid;
      pid.setGains(1.0f, 1.0f, 1.0f);
      for (int i = 0; i < 100; i++) pid.update(0.7f, dt);
      pid.reset();
      check(near(pid.integral(), 0.0f) && near(pid.output(), 0.0f), "reset zera o estado");
   }

   // ---- 8. Convencao de sinal: erro positivo -> vira a direita ----
   {
      LineFollower f;
      f.reset();
      f.update(+1000, true, 0.0f, dt);            // linha a direita do array
      check(f.normalizedError() > 0.0f, "erro cru positivo -> normalizado positivo");
      check(f.action() > 0.0f, "erro positivo -> acao positiva");
      check(f.left() > f.right(), "acao positiva -> esquerda mais rapida (vira a direita)");
      printf("       error=+1000 action=%.3f left=%.3f right=%.3f\n",
             f.action(), f.left(), f.right());
   }

   // ---- 9. Simetria ----
   {
      LineFollower a, b;
      a.reset(); b.reset();
      a.update(+1000, true, 0.0f, dt);
      b.update(-1000, true, 0.0f, dt);
      check(near(a.action(), -b.action()), "acao antissimetrica no erro");
      check(near(a.left(), b.right()) && near(a.right(), b.left()), "lados espelhados");
   }

   // ---- 10. Mixagem preserva a diferenca sob saturacao ----
   {
      LineFollower f;
      f.reset();
      for (int i = 0; i < 300; i++) f.update(+2500, true, 0.0f, dt);  // erro maximo
      const float diff = f.left() - f.right();
      check(f.left() <= cfg::MAX_SPEED + 1e-4f && f.right() >= -cfg::MAX_SPEED - 1e-4f,
            "lados dentro de MAX_SPEED");
      check(near(diff, 2.0f * f.action(), 0.05f) || f.mixSaturated(),
            "diferenca preservada, ou saturacao sinalizada");
      printf("       left=%.3f right=%.3f diff=%.3f action=%.3f satMix=%d\n",
             f.left(), f.right(), diff, f.action(), (int)f.mixSaturated());
   }

   // ---- 11. Linha perdida: acao congelada, base reduzida ----
   {
      LineFollower f;
      f.reset();
      for (int i = 0; i < 100; i++) f.update(+800, true, 0.0f, dt);
      const float last_action = f.action();
      const float base_ok = f.baseSpeed();

      f.update(+800, false, 0.0f, dt);
      check(f.lineLost(), "linha perdida sinalizada");
      check(near(f.action(), last_action), "acao diferencial congelada");

      const float expected_base = base_ok * cfg::LINE_LOST_SPEED_FACTOR;
      check(near(f.left() + f.right(), 2.0f * expected_base, 0.02f),
            "velocidade base reduzida na busca");

      // o integrador nao pode carregar com o erro repetido
      const float frozen = f.pid().integral();
      for (int i = 0; i < 500; i++) f.update(+800, false, 0.0f, dt);
      check(near(f.pid().integral(), frozen), "integral congelada sem linha");
      check(f.lineLostMs() >= cfg::LINE_LOST_TIMEOUT_MS, "contador de linha perdida corre");
   }

   // ---- 12. Classificador: histerese ----
   {
      TrackClassifier c;
      c.reset();
      for (int i = 0; i < 500; i++) c.update(0.0f, dt);
      check(c.regime() == TrackRegime::STRAIGHT, "parado -> regime de reta");
      check(near(c.blend(), 0.0f), "blend chega a 0 em reta");

      // entre os dois limiares: mantem o regime
      const float between = (cfg::GYRO_TH_LOW + cfg::GYRO_TH_HIGH) / 2.0f;
      for (int i = 0; i < 500; i++) c.update(between, dt);
      check(c.regime() == TrackRegime::STRAIGHT, "entre limiares -> mantem regime (histerese)");

      for (int i = 0; i < 500; i++) c.update(cfg::GYRO_TH_HIGH * 2.0f, dt);
      check(c.regime() == TrackRegime::CURVE, "acima do limiar alto -> curva");
      check(near(c.blend(), 1.0f), "blend chega a 1 em curva");
   }

   // ---- 13. Classificador: interpolacao nao salta ----
   {
      TrackClassifier c;
      c.reset();
      for (int i = 0; i < 500; i++) c.update(0.0f, dt);     // estabiliza em reta
      float max_step = 0.0f, prev = c.blend();
      for (int i = 0; i < 200; i++) {
         c.update(cfg::GYRO_TH_HIGH * 3.0f, dt);
         max_step = fmaxf(max_step, fabsf(c.blend() - prev));
         prev = c.blend();
      }
      const float allowed = dt / (cfg::REGIME_BLEND_MS / 1000.0f) + 1e-6f;
      check(max_step <= allowed, "blend respeita a taxa de interpolacao");
      printf("       passo maximo = %.6f (limite %.6f)\n", max_step, allowed);
   }

   // ---- 14. Ganhos interpolados ficam entre os dois conjuntos ----
   {
      TrackClassifier c;
      c.reset();
      // 15 ciclos de 4 ms sao 60 ms, metade de REGIME_BLEND_MS: pega o blend no
      // meio do caminho, que e onde a interpolacao de fato aparece.
      for (int i = 0; i < 15; i++) c.update(0.0f, dt);
      const ControlGains g = c.gains();
      const float lo = fminf(cfg::KP_STRAIGHT, cfg::KP_CURVE);
      const float hi = fmaxf(cfg::KP_STRAIGHT, cfg::KP_CURVE);
      check(g.kp >= lo - 1e-4f && g.kp <= hi + 1e-4f, "kp interpolado dentro da faixa");
      check(c.blend() > 0.01f && c.blend() < 0.99f, "amostrado no meio da transicao");
      check(g.kp > lo + 1e-4f && g.kp < hi - 1e-4f, "kp estritamente entre os dois conjuntos");
      printf("       blend=%.3f kp=%.4f kg=%.4f base=%.4f\n", c.blend(), g.kp, g.kg, g.base_speed);
   }

   // ---- 15. Amortecimento e realimentacao NEGATIVA ----
   {
      LineFollower a, b;
      a.reset(); b.reset();
      a.update(0, true, 0.0f, dt);          // sem rotacao
      b.update(0, true, +100.0f, dt);       // girando no sentido positivo
      check(b.action() < a.action(), "girando positivo -> acao reduzida (amortece)");
      printf("       action sem giro=%.4f, com +100 deg/s=%.4f\n", a.action(), b.action());
   }

   printf("\n%s (%d falhas)\n", failures ? "FALHOU" : "TODOS OS TESTES PASSARAM", failures);
   return failures ? 1 : 0;
}
