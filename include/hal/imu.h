/**
 * @file imu.h
 *
 * @brief MPU-6050: leitura por interrupção de data-ready, remoção de bias e filtro.
 *
 * A grandeza que importa ao controle é uma só — `yawRate()`, a taxa de guinada
 * do chassi em °/s. É ela que amortece o zigue-zague em reta (RF06) e que
 * classifica o trecho entre reta e curva.
 *
 * A amostragem é disparada pelo próprio sensor, não por polling: o MPU levanta
 * o pino de data-ready assim que a amostra existe, e a ISR apenas sinaliza. A
 * leitura I2C acontece fora do contexto de interrupção. Isso mantém o intervalo
 * entre amostras constante, o que importa porque um `dt` irregular distorce
 * tanto a integração do heading quanto o termo de amortecimento.
 *
 * Ver docs/ARQUITETURA.md, seções 4.3 e 5.3.
 */

#ifndef HAL_IMU_H_
#define HAL_IMU_H_

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Imu {
 public:
   /**
    * @brief Inicializa I2C, configura o sensor e liga a interrupção.
    *
    * Configura fundo de escala de ±500 °/s, DLPF em ~44 Hz e a taxa de
    * cfg::IMU_SAMPLE_HZ, e anexa a ISR ao pino pins::IMU_INT.
    *
    * @return false se o MPU-6050 não responder no barramento.
    */
   bool begin();

   /**
    * @brief Estima o bias do giroscópio com o robô imóvel.
    *
    * Promedia cfg::GYRO_BIAS_SAMPLES leituras. O MPU-6050 tem offset de fábrica
    * grande o bastante para o robô desenvolver uma tendência lateral constante
    * se ele não for removido.
    *
    * @warning O robô precisa estar parado e apoiado. Chamar em movimento grava
    *          o movimento como se fosse o zero do sensor.
    *
    * @return false se a IMU parar de entregar amostras no meio da estimativa.
    */
   bool calibrateGyroBias();

   /**
    * @brief Lê uma amostra e atualiza o estado filtrado.
    *
    * Espera a amostra por conta própria, consultando o contador da ISR. É a
    * forma usada pelas rotinas de teste, que rodam num laço único.
    *
    * @return false se não houve amostra nova dentro de cfg::IMU_TIMEOUT_MS.
    */
   bool update();

   /**
    * @brief Lê uma amostra imediatamente, sem esperar.
    *
    * Para quem já foi acordado pela notificação da ISR e portanto sabe que a
    * amostra existe — é o caminho da `TaskImu`. Chamar sem amostra nova devolve
    * a mesma leitura outra vez, o que não é erro, só desperdício de barramento.
    */
   void read();

   /**
    * @brief Bloqueia até a próxima amostra ficar pronta.
    *
    * @param timeout_ms Espera máxima.
    * @return false no estouro do tempo.
    */
   bool waitForSample(uint32_t timeout_ms);

   /** @brief true se há amostra não consumida. */
   bool dataReady() const;

   /**
    * @brief Registra a task a ser notificada pela ISR.
    *
    * Usado a partir da etapa 4, quando `TaskImu` passa a ser acordada por
    * `vTaskNotifyGiveFromISR`. Enquanto isso, a ISR só mantém o contador e as
    * rotinas de teste consultam dataReady().
    */
   void notifyTask(TaskHandle_t task);

   /*****************************************
    * Resultados
    *****************************************/

   /** @brief Taxa de guinada em °/s, sem bias e filtrada. É o canal do controle. */
   float yawRate() const { return yaw_rate_; }

   /** @brief Taxa de guinada em °/s, sem bias e sem o filtro do firmware. */
   float yawRateRaw() const { return yaw_rate_raw_; }

   float rollRate() const { return roll_rate_; }
   float pitchRate() const { return pitch_rate_; }

   /** @brief Aceleração em g. */
   float accelX() const { return accel_x_; }
   float accelY() const { return accel_y_; }
   float accelZ() const { return accel_z_; }

   /** @brief Bias estimado do eixo Z do giroscópio, em °/s. */
   float gyroBiasZ() const { return bias_z_; }

   /**
    * @brief Taxa efetiva de amostragem medida, em Hz.
    *
    * Comparar com cfg::IMU_SAMPLE_HZ é o teste mais direto de que a interrupção
    * está de fato chegando: se cair para perto de zero, o pino de INT não está
    * ligado ou a configuração do sensor não pegou.
    */
   float measuredRateHz() const { return measured_rate_hz_; }

   /** @brief Total de interrupções de data-ready desde begin(). */
   uint32_t interruptCount() const;

 private:

   float yaw_rate_ = 0.0f;
   float yaw_rate_raw_ = 0.0f;
   float roll_rate_ = 0.0f;
   float pitch_rate_ = 0.0f;

   float accel_x_ = 0.0f;
   float accel_y_ = 0.0f;
   float accel_z_ = 0.0f;

   float bias_x_ = 0.0f;
   float bias_y_ = 0.0f;
   float bias_z_ = 0.0f;

   bool filter_primed_ = false;

   /* Medição da taxa efetiva, atualizada a cada janela de 1 s. */
   uint32_t rate_window_start_ms_ = 0;
   uint32_t rate_window_count_ = 0;
   float    measured_rate_hz_ = 0.0f;
};

#endif  // HAL_IMU_H_
