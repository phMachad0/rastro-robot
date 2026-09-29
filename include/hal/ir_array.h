/**
 * @file ir_array.h
 *
 * @brief Array de sensores IR QTR-8A: amostragem e posição da linha.
 *
 * A pista é **linha branca sobre fundo escuro**, o inverso do caso mais comum.
 * Como o sensor mede reflexão, a linha aparece como leitura **baixa** no ADC.
 *
 * A classificação é por limiar fixo, sem calibração: leitura acima de
 * cfg::IR_BLACK_THRESHOLD é fundo, abaixo é linha. A faixa de operação do array
 * é conhecida e estável, então não há o que aprender em tempo de execução.
 *
 * Fluxo de uso:
 *
 *     ir.begin();
 *     while (rodando) { ir.sample(); usar(ir.position(), ir.lineDetected()); }
 *
 * Ver docs/ARQUITETURA.md, seção 5.1.
 */

#ifndef HAL_IR_ARRAY_H_
#define HAL_IR_ARRAY_H_

#include <stdint.h>

#include "config/board_pins.h"

class IrArray {
 public:
   /** @brief Configura o ADC e os pinos. */
   void begin();

   /**
    * @brief Lê os canais e recalcula a posição da linha.
    *
    * Cada canal é lido cfg::IR_SAMPLES_PER_CHANNEL vezes e promediado antes de
    * ser comparado com o limiar.
    */
   void sample();

   /** @brief Leitura bruta do ADC, de 0 a 4095. */
   uint16_t raw(uint8_t channel) const { return raw_[channel]; }

   /** @brief true se o canal está sobre a linha clara. */
   bool onLine(uint8_t channel) const { return on_line_[channel]; }

   /**
    * @brief Quantos canais estão sobre a linha.
    *
    * Vale monitorar além do zero: com todos os canais acionados ao mesmo tempo,
    * o array está sobre uma faixa transversal ou saiu da pista para uma área
    * clara. É também o gancho natural para o `MarkDetector` da etapa 7.
    */
   uint8_t detectedCount() const { return detected_count_; }

   /**
    * @brief Posição da linha, de 0 (extrema esquerda) a 5000 (extrema direita).
    *
    * O centro é cfg::LINE_POSITION_CENTER. Com a linha perdida, devolve a
    * última posição válida — é o que permite ao controle continuar virando
    * para o lado em que a linha sumiu, em vez de saltar para o centro.
    */
   uint16_t position() const { return position_; }

   /** @brief false quando nenhum canal está sobre a linha. */
   bool lineDetected() const { return detected_count_ > 0; }

   /**
    * @brief Erro de posição em relação ao centro, de -2500 a +2500.
    *
    * Positivo significa linha à direita do centro do array.
    */
   int16_t error() const;

 private:
   uint16_t raw_[pins::IR_CHANNEL_COUNT] = {};
   bool     on_line_[pins::IR_CHANNEL_COUNT] = {};

   uint8_t  detected_count_ = 0;
   uint16_t position_ = 0;
};

#endif  // HAL_IR_ARRAY_H_
