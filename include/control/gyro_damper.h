/**
 * @file gyro_damper.h
 *
 * @brief Amortecimento da malha pela taxa de guinada do giroscópio — RF06.
 *
 * O array IR dá uma medida discreta e ruidosa da posição: com o limiar fixo, a
 * posição só assume onze valores. Um termo derivativo calculado sobre isso ou
 * precisa de Kd alto, que amplifica a quantização, ou de Kd baixo, que não
 * amortece nada.
 *
 * O giroscópio resolve porque mede diretamente a grandeza a amortecer — a
 * velocidade angular do chassi — de forma contínua e sem quantização. O termo
 * entra subtraindo:
 *
 *     u = Kp·e + Ki·∫e + Kd·de/dt − Kg·ωz
 *
 * Se o robô está girando, o termo subtrai da ação e freia a rotação. É
 * amortecimento de velocidade real, não derivada numérica de sinal quantizado.
 *
 * @warning O sinal de ωz depende da orientação do MPU no chassi. Invertido, a
 *          realimentação deixa de ser negativa e passa a **reforçar** a rotação,
 *          levando a oscilação crescente. Ver cfg::GYRO_Z_REVERSED.
 *
 * Ver docs/ARQUITETURA.md, seção 5.3.
 */

#ifndef CONTROL_GYRO_DAMPER_H_
#define CONTROL_GYRO_DAMPER_H_

class GyroDamper {
 public:
   void setGain(float kg) { kg_ = kg; }

   float gain() const { return kg_; }

   /**
    * @brief Calcula o termo de amortecimento a partir da taxa de guinada.
    *
    * @param yaw_rate_dps Taxa de guinada em °/s, já sem bias e filtrada.
    *
    * @return Termo a **somar** à ação de controle. Já vem com o sinal negativo
    *         e com a inversão de cfg::GYRO_Z_REVERSED aplicada, para que quem
    *         chama não tenha que lembrar de nenhuma das duas.
    */
   float term(float yaw_rate_dps);

   /** @brief Taxa de guinada normalizada da última chamada, em [-1, 1] típico. */
   float normalizedRate() const { return normalized_rate_; }

   /** @brief Termo calculado na última chamada. */
   float lastTerm() const { return last_term_; }

 private:
   float kg_ = 0.0f;
   float normalized_rate_ = 0.0f;
   float last_term_ = 0.0f;
};

#endif  // CONTROL_GYRO_DAMPER_H_
