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
 * a toda velocidade, 0 é parado, +1 é frente a toda velocidade. A conversão
 * para duty de PWM, a compensação da zona morta, o trim entre os lados e a
 * rampa de aceleração ficam todas aqui dentro. A camada de controle não sabe
 * que zona morta existe.
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
    * Os valores são saturados em ±cfg::MAX_SPEED, passam pela rampa de
    * aceleração e só então são convertidos em duty. Chamar com o driver
    * desarmado apenas força a saída para zero.
    *
    * @param left  Velocidade do lado esquerdo, de -1 a 1.
    * @param right Velocidade do lado direito, de -1 a 1.
    */
   void setSpeed(float left, float right);

   /**
    * @brief Para os dois lados imediatamente, sem passar pela rampa.
    *
    * A rampa é ignorada de propósito: parada é ação de segurança e não deve
    * esperar o tempo de descida. Também zera o estado interno da rampa, para
    * que a retomada comece do zero.
    */
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

   /** @brief Velocidade efetivamente aplicada no último ciclo, após a rampa. */
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

   /** @brief Aplica velocidade a um lado, já saturada e rampada. */
   void applySide(uint8_t channel, uint8_t in_a, uint8_t in_b, float speed);

   /** @brief Aproxima `current` de `target` respeitando cfg::MOTOR_SLEW_PER_S. */
   static float slew(float current, float target, float max_delta);

   bool  armed_ = false;
   float trim_ = 0.0f;

   /** @brief Última velocidade aplicada, entrada da rampa do ciclo seguinte. */
   float applied_left_ = 0.0f;
   float applied_right_ = 0.0f;

   /** @brief Instante da última chamada a setSpeed(), para o dt da rampa. */
   uint32_t last_update_us_ = 0;
};

#endif  // HAL_MOTOR_DRIVER_H_
