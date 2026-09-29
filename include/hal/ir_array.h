/**
 * @file ir_array.h
 *
 * @brief Array de sensores IR QTR-8A: amostragem, calibração e posição da linha.
 *
 * A pista é **linha branca sobre fundo escuro**, o inverso do caso mais comum.
 * Como o sensor mede reflexão, a linha aparece como leitura **baixa** no ADC, e
 * a inversão para peso ("quanto este canal vê de linha") acontece aqui dentro.
 *
 * Fluxo de uso:
 *
 *     ir.begin();
 *     ir.calibrationReset();
 *     while (varrendo) { ir.sample(); ir.calibrationAccumulate(); }
 *     ir.calibrationFinish();
 *     while (rodando)  { ir.sample(); usar(ir.position(), ir.lineDetected()); }
 *
 * Ver docs/ARQUITETURA.md, seção 5.1.
 */

#ifndef HAL_IR_ARRAY_H_
#define HAL_IR_ARRAY_H_

#include <stdint.h>

#include "config/board_pins.h"

/**
 * @brief Extremos observados por canal durante a calibração.
 *
 * Fica num struct separado, e não espalhado pela classe, porque a etapa 6
 * persiste exatamente isto em NVS — basta gravar e reler o bloco.
 */
struct IrCalibration {
   uint16_t min[pins::IR_CHANNEL_COUNT];
   uint16_t max[pins::IR_CHANNEL_COUNT];
   bool valid;
};

class IrArray {
 public:
   /** @brief Configura o ADC e os pinos. Não calibra. */
   void begin();

   /**
    * @brief Lê os canais e atualiza o estado interno.
    *
    * Cada canal é lido cfg::IR_SAMPLES_PER_CHANNEL vezes e promediado. Se já
    * houver calibração válida, também recalcula normalizados, pesos, posição e
    * o indicador de linha detectada.
    */
   void sample();

   /*****************************************
    * Calibração
    *****************************************/

   /** @brief Zera os extremos e invalida a calibração corrente. */
   void calibrationReset();

   /**
    * @brief Incorpora a última amostra aos extremos.
    *
    * Deve ser chamada depois de sample(), enquanto o array varre a linha.
    */
   void calibrationAccumulate();

   /**
    * @brief Fecha a calibração e verifica se ela é utilizável.
    *
    * @return false se algum canal não atingiu cfg::IR_CALIBRATION_MIN_SPAN de
    *         amplitude, o que indica sensor solto, mal posicionado ou varredura
    *         curta demais. Nesse caso a calibração continua inválida.
    */
   bool calibrationFinish();

   const IrCalibration& calibration() const { return calib_; }

   /** @brief Restaura uma calibração salva, por exemplo vinda da NVS. */
   void setCalibration(const IrCalibration& calibration);

   bool isCalibrated() const { return calib_.valid; }

   /*****************************************
    * Resultados
    *****************************************/

   /** @brief Leitura bruta do ADC, de 0 a 4095. */
   uint16_t raw(uint8_t channel) const { return raw_[channel]; }

   /** @brief Leitura normalizada pela calibração, de 0 a cfg::IR_NORMALIZED_MAX. */
   uint16_t normalized(uint8_t channel) const { return normalized_[channel]; }

   /**
    * @brief Quanto o canal vê de linha, de 0 a cfg::IR_NORMALIZED_MAX.
    *
    * É o normalizado invertido, já que a linha é clara.
    */
   uint16_t weight(uint8_t channel) const { return weight_[channel]; }

   /** @brief Soma dos pesos. Serve de medida de confiança da leitura. */
   uint32_t weightSum() const { return weight_sum_; }

   /**
    * @brief Posição da linha, de 0 (extrema esquerda) a 5000 (extrema direita).
    *
    * O centro é cfg::LINE_POSITION_CENTER. Com a linha perdida, devolve a
    * última posição válida — é o que permite ao controle continuar virando
    * para o lado em que a linha sumiu, em vez de saltar para o centro.
    */
   uint16_t position() const { return position_; }

   /** @brief false quando nenhum canal está sobre a linha. */
   bool lineDetected() const { return line_detected_; }

   /**
    * @brief Erro de posição em relação ao centro, de -2500 a +2500.
    *
    * Positivo significa linha à direita do centro do array.
    */
   int16_t error() const;

 private:
   IrCalibration calib_ = {};

   uint16_t raw_[pins::IR_CHANNEL_COUNT] = {};
   uint16_t normalized_[pins::IR_CHANNEL_COUNT] = {};
   uint16_t weight_[pins::IR_CHANNEL_COUNT] = {};

   uint32_t weight_sum_ = 0;
   uint16_t position_ = 0;
   bool     line_detected_ = false;
};

#endif  // HAL_IR_ARRAY_H_
