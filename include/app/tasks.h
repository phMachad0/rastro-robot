/**
 * @file tasks.h
 *
 * @brief Criação das tarefas e das primitivas de sincronização.
 *
 * A divisão entre os núcleos é deliberada: **núcleo 1 roda controle e sensores e
 * nada mais**, **núcleo 0 roda supervisão, telemetria e rede**. Assim uma
 * reconexão de Wi-Fi ou um cliente lento não introduz jitter na malha.
 *
 * As tarefas sensoriais têm prioridade **acima** do controle: elas são curtas e
 * precisam entregar a amostra fresca antes de o controle acordar. Com o controle
 * em prioridade maior, ele leria dados de um ciclo atrasado.
 *
 * Ver docs/ARQUITETURA.md, seções 4, 4.1, 4.2 e 4.3.
 */

#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "app/robot_state.h"

/*****************************************
 * Bits do event group de modo
 *
 * A TaskControl é periódica por natureza — acorda por vTaskDelayUntil a cada
 * 4 ms de qualquer forma —, então lê os bits com xEventGroupGetBits em vez de
 * bloquear neles. O ganho do event group aqui é a leitura atômica do modo sem
 * fila e sem mutex.
 *****************************************/

constexpr EventBits_t EV_MODE_RUN   = (1 << 0);
constexpr EventBits_t EV_MODE_STOP  = (1 << 1);
constexpr EventBits_t EV_MODE_CALIB = (1 << 2);
constexpr EventBits_t EV_MODE_FAULT = (1 << 3);
constexpr EventBits_t EV_MODE_ALL =
    EV_MODE_RUN | EV_MODE_STOP | EV_MODE_CALIB | EV_MODE_FAULT;

/**
 * @brief Cria primitivas e tarefas, e liga as interrupções.
 *
 * @return false se alguma primitiva ou tarefa não pôde ser criada.
 */
bool tasksStart();

/*****************************************
 * Acesso às mailboxes
 *
 * Mailbox é fila de profundidade 1, escrita com xQueueOverwrite e lida com
 * xQueuePeek: o produtor nunca bloqueia e o consumidor sempre pega o valor mais
 * recente. Para sensor de alta taxa é o certo — ninguém quer processar uma
 * posição de linha de 40 ms atrás porque ela ficou enfileirada.
 *****************************************/

bool readLineState(LineState* out);
bool readImuState(ImuState* out);
bool readControlState(ControlState* out);
bool readRobotState(RobotState* out);

/**
 * @brief Enfileira um comando para o supervisor.
 *
 * Seguro a partir de qualquer tarefa. A versão para ISR é interna.
 */
bool postCommand(Command command);

#endif  // APP_TASKS_H_
