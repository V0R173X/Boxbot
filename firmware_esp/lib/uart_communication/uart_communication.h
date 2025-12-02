#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

/* Dependências */
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "utils.h" 

/* Definições (reutilizadas) */
#define FRAME_SOF 0xAA
#define FRAME_EOF 0xBB

// Configuração UART (UART0 é a porta USB/Console padrão)
#define UART_PORT_NUM      UART_NUM_0
#define BUF_SIZE           1024
#define RD_BUF_SIZE        1024

/* Declaração de Funções */
esp_err_t uart_communication_init();
esp_err_t uart_send_frame(char cmd, char id, uint16_t value);
void uart_read_task_loop();
void uart_send_task_loop(char task_id);

#endif // UART_COMMUNICATION_H