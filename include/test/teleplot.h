/**
 * @file teleplot.h
 *
 * @brief Emissão de telemetria no formato da extensão Teleplot do VSCode.
 *
 * Teleplot lê linhas da serial e plota as que começam com `>`. As demais
 * aparecem no painel de log, então texto corrido e dados plotados convivem na
 * mesma porta sem atrapalhar um ao outro.
 *
 * Formato emitido aqui:
 *
 *     >nome:tempo_ms:valor
 *     >nome:tempo_ms:valor§unidade
 *
 * O timestamp é `millis()`. Sem ele o Teleplot usaria a hora de chegada do
 * dado no computador, que embute a latência da serial e do buffer do sistema
 * operacional — com ele, o eixo do tempo reflete o instante da amostra.
 *
 * Uso no VSCode: instalar a extensão Teleplot, abrir a paleta de comandos e
 * rodar "Teleplot: Connect to Serial Port", escolhendo a porta e 115200 baud.
 *
 * @warning O Teleplot abre a porta serial exclusivamente. Não é possível ter
 *          o `pio device monitor` rodando ao mesmo tempo — feche um antes de
 *          abrir o outro, senão a porta aparece como ocupada.
 *
 * Ver docs/ARQUITETURA.md, seção 8.3.
 */

#ifndef TEST_TELEPLOT_H_
#define TEST_TELEPLOT_H_

#include <Arduino.h>

namespace teleplot {

/** @brief Separador de unidade esperado pelo Teleplot (U+00A7 em UTF-8). */
constexpr char UNIT_SEPARATOR[] = "§";

/**
 * @brief Emite uma amostra inteira.
 *
 * @param name  Nome da série, como aparece na legenda do gráfico.
 * @param value Valor da amostra.
 */
inline void send(const char* name, int32_t value)
{
   Serial.printf(">%s:%lu:%ld\n", name, (unsigned long)millis(), (long)value);
}

/**
 * @brief Emite uma amostra real, com três casas decimais.
 */
inline void send(const char* name, float value)
{
   Serial.printf(">%s:%lu:%.3f\n", name, (unsigned long)millis(), value);
}

/**
 * @brief Emite uma amostra real acompanhada da unidade.
 *
 * A unidade aparece no eixo do gráfico, o que evita confundir séries de
 * grandezas diferentes plotadas lado a lado.
 *
 * @param unit Unidade, por exemplo "deg/s" ou "g".
 */
inline void send(const char* name, float value, const char* unit)
{
   Serial.printf(">%s:%lu:%.3f%s%s\n", name, (unsigned long)millis(), value,
                 UNIT_SEPARATOR, unit);
}

/**
 * @brief Emite uma amostra inteira acompanhada da unidade.
 */
inline void send(const char* name, int32_t value, const char* unit)
{
   Serial.printf(">%s:%lu:%ld%s%s\n", name, (unsigned long)millis(), (long)value,
                 UNIT_SEPARATOR, unit);
}

/**
 * @brief Imprime texto no painel de log do Teleplot.
 *
 * Existe para deixar explícito, em quem lê o teste, que a linha é log e não
 * série plotada — não faz nada além de um Serial.println.
 */
inline void log(const char* message)
{
   Serial.println(message);
}

}  // namespace teleplot

#endif  // TEST_TELEPLOT_H_
