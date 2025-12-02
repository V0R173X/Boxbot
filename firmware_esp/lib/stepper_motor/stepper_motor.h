#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <math.h> 
#include <stdatomic.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "utils.h"

#define STEPPER_MAX_SPEED   1000.0f // Passos por segundo

esp_err_t stepper_init();

void stepper_move(int steps);

void stepper_task_loop(void);

bool stepper_is_idle(void);

#endif // STEPPER_MOTOR_H