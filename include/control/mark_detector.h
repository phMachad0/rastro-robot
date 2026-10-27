/**
 * @file mark_detector.h
 *
 * @brief Detecção de marcações laterais na pista — RF02.
 *
 * Com cfg / FEATURE_LATERAL_MARKS em 0, que é o padrão, esta classe é um stub:
 * `update()` não faz nada e `markDetected()` nunca devolve true. O código de
 * detecção não é compilado, nenhum pino é reservado e `MarkCountPolicy` não
 * existe no binário.
 *
 * Quando e se a funcionalidade entrar, o caminho é dedicar canais das pontas do
 * array (ou um sensor IR lateral extra num dos pinos livres) e implementar um
 * detector de borda com debounce espacial. A única mudança fora deste módulo é
 * ligar a flag.
 *
 * Ver docs/ARQUITETURA.md, seção 6.2.
 */

#ifndef CONTROL_MARK_DETECTOR_H_
#define CONTROL_MARK_DETECTOR_H_

#include <stdint.h>

#include "config/features.h"

class MarkDetector {
 public:
   void reset() { count_ = 0; }

   /**
    * @brief Processa uma amostra do array.
    *
    * @param detected_count Quantos canais do array veem linha. Uma faixa
    *                       transversal aciona todos de uma vez, que é o sinal
    *                       que a implementação futura vai procurar.
    * @param dt             Passo de tempo em segundos.
    *
    * @return true no ciclo em que uma marcação é reconhecida.
    */
   bool update(uint8_t detected_count, float dt)
   {
#if FEATURE_LATERAL_MARKS
#error "MarkDetector nao implementado. Ver etapa 7 do roteiro em docs/ARQUITETURA.md."
#else
      (void)detected_count;
      (void)dt;
      return false;
#endif
   }

   uint8_t count() const { return count_; }

 private:
   uint8_t count_ = 0;
};

#endif  // CONTROL_MARK_DETECTOR_H_
