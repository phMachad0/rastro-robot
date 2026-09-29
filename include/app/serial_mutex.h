/**
 * @file serial_mutex.h
 *
 * @brief Exclusão mútua para acesso à porta serial.
 *
 * Só é necessário onde mais de uma tarefa imprime. No firmware completo isso
 * não acontece: todo log passa por TaskTelemetry, e nem TaskControl nem
 * nenhuma ISR podem tocar na serial. O mutex existe para as rotinas de teste
 * que criam mais de uma tarefa.
 *
 * @note A definição do handle vem junto com a camada de tarefas, na etapa 4.
 *
 * Ver docs/ARQUITETURA.md, seção 8.3.
 */

#ifndef APP_SERIAL_MUTEX_H_
#define APP_SERIAL_MUTEX_H_

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t serialMutex;

#endif  // APP_SERIAL_MUTEX_H_
