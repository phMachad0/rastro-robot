/**
 * @file pid_algorithm.h
 *
 * @brief Controlador PID de propósito geral, com passo de tempo explícito.
 *
 * Opera sobre o **erro**, não sobre um par setpoint/medida. A diferença é
 * deliberada: o setpoint deste projeto é o centro do array, e o `IrArray` já
 * entrega o erro pronto e com sinal definido. Manter um setpoint interno
 * duplicaria essa informação e criaria um segundo lugar onde a convenção de
 * sinal pode ser invertida por engano — que é a forma mais comum de fazer um
 * seguidor de linha corrigir para o lado errado.
 *
 * O erro esperado é **normalizado em [-1, 1]** (ver cfg::LINE_ERROR_SCALE), de
 * modo que os ganhos sejam adimensionais e comparáveis entre si.
 *
 * Ver docs/ARQUITETURA.md, seção 5.2.
 */

#ifndef CONTROL_PID_ALGORITHM_H_
#define CONTROL_PID_ALGORITHM_H_

#include <stdint.h>

#include "config/robot_config.h"

class PidAlgorithm {
 public:
   PidAlgorithm();

   /** @brief Define os ganhos. Pode ser chamado em tempo de execução. */
   void setGains(float kp, float ki, float kd);

   /**
    * @brief Limite simétrico da saída, usado também pelo anti-windup.
    *
    * O PID precisa conhecer o próprio limite para saber que saturou. A
    * alternativa — quem chama saturar e avisar depois — deixa a detecção um
    * ciclo atrasada, que é justamente o ciclo em que o integrador carrega.
    */
   void setOutputLimit(float limit);

   /** @brief Teto simétrico do integrador. */
   void setIntegralLimit(float limit);

   /**
    * @brief Calcula a ação de controle para um erro e um passo de tempo.
    *
    * @param error Erro normalizado, idealmente em [-1, 1].
    * @param dt    Tempo desde a chamada anterior, em segundos. Valores nulos ou
    *              negativos fazem a função devolver a ação anterior sem
    *              atualizar estado.
    *
    * @return Ação de controle, saturada em ±setOutputLimit().
    */
   float update(float error, float dt);

   /**
    * @brief Zera integrador, histórico da derivada e última saída.
    *
    * Obrigatório ao sair de um estado parado: sem isso o integral acumulado
    * antes da parada é aplicado de uma vez na retomada.
    */
   void reset();

   /**
    * @brief Congela o integrador sem zerá-lo.
    *
    * Usado quando a medida perdeu significado — linha fora do array — para que
    * o erro repetido não seja integrado como se fosse informação nova.
    */
   void holdIntegral(bool hold) { integral_held_ = hold; }

   /*****************************************
    * Introspecção — telemetria e sintonia
    *****************************************/

   /** @brief Contribuição proporcional da última atualização. */
   float proportionalTerm() const { return p_term_; }

   /** @brief Contribuição integral da última atualização. */
   float integralTerm() const { return i_term_; }

   /** @brief Contribuição derivativa da última atualização. */
   float derivativeTerm() const { return d_term_; }

   /** @brief Estado do integrador, antes de multiplicar por Ki. */
   float integral() const { return integral_; }

   /** @brief true se a última saída foi limitada por setOutputLimit(). */
   bool saturated() const { return saturated_; }

   float output() const { return output_; }

 private:
   float kp_ = 0.0f;
   float ki_ = 0.0f;
   float kd_ = 0.0f;

   float output_limit_ = cfg::PID_OUTPUT_LIMIT;
   float integral_limit_ = cfg::PID_INTEGRAL_LIMIT;

   float integral_ = 0.0f;
   bool  integral_held_ = false;

   /**
    * @brief Fila circular dos erros recentes, para a derivada de janela longa.
    *
    * O tamanho é exatamente cfg::PID_DERIVATIVE_WINDOW. A versão anterior desta
    * classe escrevia e lia uma posição além do fim do array em dois pontos
    * diferentes; o índice agora é sempre reduzido módulo o tamanho.
    */
   float error_history_[cfg::PID_DERIVATIVE_WINDOW] = {};

   /** @brief Próxima posição a escrever na fila circular. */
   uint8_t history_index_ = 0;

   /** @brief Quantas posições da fila já têm dado válido. */
   uint8_t history_filled_ = 0;

   float p_term_ = 0.0f;
   float i_term_ = 0.0f;
   float d_term_ = 0.0f;

   float output_ = 0.0f;
   bool  saturated_ = false;
};

#endif  // CONTROL_PID_ALGORITHM_H_
