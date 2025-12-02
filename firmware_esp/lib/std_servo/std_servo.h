#ifndef STD_SERVO_H
#define STD_SERVO_H

#include "driver/ledc.h"
#include <stdlib.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "utils.h"

typedef struct servo_handle_s* servo_handle_t;

typedef struct {
    int gpio_pin;           // GPIO ao qual o pino de sinal do servo está conectado
    int min_pulse_us;       // Largura de pulso em microssegundos para o ângulo de 0 graus
    int max_pulse_us;       // Largura de pulso em microssegundos para o ângulo máximo
    int max_angle;          // Ângulo máximo do servo

    // Recursos de hardware do ESP-IDF
    ledc_timer_t timer_num;     
    ledc_channel_t channel_num; 
} servo_config_t;

#define GRIPPER_TIMER          LEDC_TIMER_1
#define GRIPPER_CHANNEL        LEDC_CHANNEL_2

#define SERVO_LEDC_TIMER_RESOLUTION LEDC_TIMER_13_BIT
#define SERVO_LEDC_MAX_DUTY         ( (1 << SERVO_LEDC_TIMER_RESOLUTION) - 1 ) // 8191
#define SERVO_LEDC_FREQ_HZ          50 // Frequência padrão para servos
#define SERVO_LEDC_PERIOD_US        (1000000 / SERVO_LEDC_FREQ_HZ) // 20.000 us

servo_handle_t servo_init(const servo_config_t *config);
esp_err_t gripper_init();  
esp_err_t servo_set_angle(servo_handle_t handle, uint16_t angle);
esp_err_t gripper_set_angle(uint16_t angle);
esp_err_t servo_deinit(servo_handle_t handle);

#endif // STD_SERVO_H