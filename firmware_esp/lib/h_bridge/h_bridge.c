#include "h_bridge.h"

static void PWM_limit(int *PWM, servo_pos_t motor) {
  // Compensação de zona morta
  if (*PWM > 0) {
      // Se a saída for positiva, adicione o offset
      *PWM += MOTOR_DEADZONE(motor);
  } else if (*PWM < 0) {
      // Se a saída for negativa, subtraia o offset
      *PWM -= MOTOR_DEADZONE(motor);
  }

  // Clamp do valor
  if (*PWM > 4095) *PWM = 4095;
  else if (*PWM < -4095) *PWM = -4095;
}

void init_hbridge() {   
    // Inicializando GPIO
    gpio_set_direction(INPUT_BASE_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_BASE_2, GPIO_MODE_OUTPUT);
    

    gpio_set_direction(INPUT_ARM_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_ARM_2, GPIO_MODE_OUTPUT);

    gpio_set_direction(LEDC_OUTPUT_ARM, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDC_OUTPUT_BASE, GPIO_MODE_OUTPUT);

    gpio_set_direction(STBY, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY, HIGH);

    // Inicializando PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_base_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_BASE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_BASE,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
    };

    ledc_channel_config(&ledc_base_channel);

    ledc_channel_config_t ledc_arm_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_ARM,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_ARM,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };

    ledc_channel_config(&ledc_arm_channel);
}

esp_err_t update_motor(servo_pos_t motor, int u)
{
    u > 0 ? _set_forward(motor) : _set_backward(motor);
    
    u = u > 0 ? u : -u;  // Valor absoluto

    PWM_limit(&u, motor);

    ledc_set_duty(LEDC_MODE, MOTOR_CHANNEL(motor), u);
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNEL(motor));

    return ESP_OK;
}

esp_err_t _set_forward(servo_pos_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), LOW);
    gpio_set_level(MOTOR_INPUT_2(motor), HIGH);

    return ESP_OK;
}

esp_err_t _set_backward(servo_pos_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), HIGH);
    gpio_set_level(MOTOR_INPUT_2(motor), LOW);

    return ESP_OK;
}