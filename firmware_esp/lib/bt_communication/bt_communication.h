#ifndef BT_COMMUNICATION_H
#define BT_COMMUNICATION_H

/* Incluindo Bibliotecas */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "utils.h"

/* Definições */
#define DEVICE_NAME "BoxBot"
#define FRAME_SOF 0xAA
#define FRAME_EOF 0xBB

/* Declaração de Funções */
esp_err_t bt_communication_init();
esp_err_t bt_communication_deinit();

#endif // BT_COMMUNICATION_H