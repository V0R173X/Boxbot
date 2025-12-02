#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "utils.h"

/* Enum */
typedef enum{
    BASE = 0,
    ARM = 1
} servo_pos_t;


/* Macros */

// PWM Config 
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT   
#define LEDC_FREQUENCY          (5000)              // Hz

#define ARM_DEAD_ZONE_OFFSET 850
#define BASE_DEAD_ZONE_OFFSET 700

/* Macro functions */

#define MOTOR_INPUT_1(MOTOR) MOTOR == (BASE) ? INPUT_BASE_1 : INPUT_ARM_1
#define MOTOR_INPUT_2(MOTOR) MOTOR == (BASE) ? INPUT_BASE_2 : INPUT_ARM_2
#define MOTOR_CHANNEL(MOTOR) MOTOR == (BASE) ? LEDC_CHANNEL_BASE : LEDC_CHANNEL_ARM
#define MOTOR_DEADZONE(MOTOR) MOTOR == (BASE) ? BASE_DEAD_ZONE_OFFSET : ARM_DEAD_ZONE_OFFSET

/* Declarando Funções */
void init_hbridge();

esp_err_t update_motor(servo_pos_t motor, int u);

esp_err_t _set_forward(servo_pos_t motor);
esp_err_t _set_backward(servo_pos_t motor);

#endif // H_BRIDGE_H