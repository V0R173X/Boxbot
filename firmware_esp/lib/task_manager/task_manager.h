#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "bt_communication.h"  
#include "uart_communication.h"
#include "utils.h"           

#include "potentiometer.h"
#include "ultrasonics.h"
#include "stepper_motor.h"
#include "pid.h"
#include "std_servo.h"

#define FREQ_COMMUNICATION 30
#define FREQ_CONTROL 50

esp_err_t init_tasks();

#endif // TASK_MANAGER_H