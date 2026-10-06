/**
 * @file motor_driver.h
 *
 * @brief Abstração dos dois lados de tração sobre o driver L298N.
 *
 * Os dois motores de cada lado compartilham um canal do driver, então o robô é
 * um diferencial de duas rodas e a interface expõe dois lados, não quatro
 * motores.
 *
 * Quem chama trabalha em **velocidade efetiva**, no intervalo [-1, 1]: -1 é ré
 * a toda velocidade, 0 é parado, +1 é frente a toda velocidade. "Frente" é a
 * frente do robô nos dois lados — o espelhamento mecânico do chassi é
 * compensado por cfg::MOTOR_L_REVERSED e cfg::MOTOR_R_REVERSED. A conversão
 * para duty de PWM, a compensação da zona morta, o trim entre os lados e o
 * mapeamento para os canais do driver ficam aqui dentro. A camada de controle não
 * sabe que zona morta existe, nem em qual canal do L298N cada lado está ligado.
 *
 * **Lado lógico × canal de hardware.** `setSpeed(left, right)` fala em lados
 * **físicos** do robô: "esquerda" é a esquerda de quem está atrás dele olhando
 * para a frente. Qual canal do driver atende cada lado é problema desta classe, e
 * depende de cfg::MOTOR_SIDES_SWAPPED. A tradução acontece no último passo, para
 * que saturação, trim e telemetria trabalhem todos em lados físicos.
 *
 * O driver **não filtra nem atrasa** o comando: o que a malha pede é o que sai,
 * a menos de saturação e trim. Moldar a dinâmica do robô é papel da camada de
 * controle, e qualquer suavização aplicada aqui entraria na malha fechada como
 * atraso de fase, custando margem de estabilidade.
 *
 * Ver docs/ARQUITETURA.md, seções 3.1 e 5.2.
 */

#ifndef HAL_MOTOR_DRIVER_H_
#define HAL_MOTOR_DRIVER_H_

#include <stdint.h>

/**
 * @brief Como os motores se comportam ao parar.
 */
enum class StopMode : uint8_t {
   /** Entradas de direção em nível baixo: o motor gira livre até frear sozinho. */
   COAST,
   /** Entradas de direção em nível alto: frenagem por curto-circuito do enrolamento. */
   BRAKE,
};

class MotorDriver {
 public:
   /**
    * @brief Configura os canais LEDC e os pinos de direção.
    *
    * Sai com os motores parados e o driver **desarmado**: setSpeed() não tem
    * efeito enquanto arm() não for chamado.
    */
   void begin();

   /**
    * @brief Define a velocidade de cada lado.
    *
    * Os valores são saturados em ±cfg::MAX_SPEED, recebem o trim e são
    * convertidos em duty. Chamar com o driver desarmado apenas força a saída
    * para zero.
    *
    * @param left  Velocidade do lado esquerdo, de -1 a 1.
    * @param right Velocidade do lado direito, de -1 a 1.
    */
   void setSpeed(float left, float right);

   /** @brief Para os dois lados imediatamente. */
   void stop(StopMode mode = StopMode::COAST);

   /**
    * @brief Habilita a saída. Sem isso, setSpeed() não move nada.
    *
    * A trava existe para que um reset, uma falha ou um estado não previsto da
    * máquina de estados deixem o robô parado por padrão, em vez de em
    * movimento. O supervisor arma explicitamente ao entrar em RUNNING.
    */
   void arm();

   /** @brief Desarma a saída e para os motores. */
   void disarm();

   bool isArmed() const { return armed_; }

   /**
    * @brief Velocidade efetivamente aplicada, após saturação e trim.
    *
    * É o que a telemetria informa como saída de cada lado, e o que permite
    * enxergar a saturação agindo quando o PID pede mais do que há disponível.
    */
   float appliedLeft() const { return applied_left_; }
   float appliedRight() const { return applied_right_; }

   /**
    * @brief Ajusta o trim entre os lados em tempo de execução.
    *
    * @param trim De -1 a 1. Positivo favorece o lado direito.
    */
   void setTrim(float trim);

   float trim() const { return trim_; }

 private:
   /** @brief Converte velocidade efetiva em duty de PWM, compensando a zona morta. */
   uint32_t speedToDuty(float speed) const;

   /**
    * @brief Aplica velocidade a um lado, já saturada e com trim.
    *
    * @param reversed true se o lado é montado espelhado e precisa ter o sentido
    *                 invertido para que velocidade positiva seja frente do robô.
    */
   void applySide(uint8_t channel, uint8_t in_a, uint8_t in_b, float speed,
                  bool reversed);

   bool  armed_ = false;
   float trim_ = 0.0f;

   /** @brief Última velocidade aplicada, para telemetria e testes. */
   float applied_left_ = 0.0f;
   float applied_right_ = 0.0f;
};

#endif  // HAL_MOTOR_DRIVER_H_
