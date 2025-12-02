#include "std_servo.h"

static const char *TAG = "GRIPPER";

typedef struct servo_handle_s {
    ledc_channel_t channel;     // Canal LEDC usado
    ledc_timer_t timer_num;     // Timer LEDC usado
    int min_pulse_us;       // Pulso para 0 graus
    int max_pulse_us;       // Pulso para ângulo máximo
    int max_angle;          // Ângulo máximo
    ledc_mode_t speed_mode;     // Modo de velocidade (High/Low)
} servo_handle_t_impl;

servo_handle_t gripper_servo = NULL;

static uint32_t angle_to_duty_cycle(servo_handle_t_impl* handle, uint16_t angle)
{
    double pulse_us = (double)handle->min_pulse_us +
                      ((double)angle / handle->max_angle) * (handle->max_pulse_us - handle->min_pulse_us);

    // Mapeia a largura de pulso (us) para o valor de duty (0-8191)
    uint32_t duty = (uint32_t)((pulse_us / SERVO_LEDC_PERIOD_US) * SERVO_LEDC_MAX_DUTY);
    
    return duty;
}

servo_handle_t servo_init(const servo_config_t *config)
{
    esp_err_t ret = ESP_OK;
    servo_handle_t_impl* handle = NULL;

    // Validações de entrada
    ESP_GOTO_ON_FALSE(config, ESP_ERR_INVALID_ARG, err, TAG, "Configuração é NULL");

    // Aloca memória para o handle
    handle = (servo_handle_t_impl*)malloc(sizeof(servo_handle_t_impl));
    ESP_GOTO_ON_FALSE(handle, ESP_ERR_NO_MEM, err, TAG, "Falha ao alocar memória para o handle");

    // Determina o modo de velocidade com base no timer
    ledc_mode_t speed_mode = (config->timer_num < LEDC_TIMER_2) ? LEDC_LOW_SPEED_MODE : LEDC_HIGH_SPEED_MODE;

    // Configura o Timer LEDC
    ledc_timer_config_t timer_cfg = {
        .speed_mode = speed_mode,
        .duty_resolution = SERVO_LEDC_TIMER_RESOLUTION,
        .timer_num = config->timer_num,
        .freq_hz = SERVO_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&timer_cfg);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Falha ao configurar timer LEDC: %s", esp_err_to_name(ret));

    // Configura o Canal LEDC
    ledc_channel_config_t channel_cfg = {
        .gpio_num = config->gpio_pin,
        .speed_mode = speed_mode,
        .channel = config->channel_num,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->timer_num,
        .duty = 0, // Inicia com duty 0
        .hpoint = 0
    };
    ret = ledc_channel_config(&channel_cfg);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Falha ao configurar canal LEDC: %s", esp_err_to_name(ret));

    // Preenche a struct do handle com os dados
    *handle = (servo_handle_t_impl){
        .channel = config->channel_num,
        .timer_num = config->timer_num,
        .min_pulse_us = config->min_pulse_us,
        .max_pulse_us = config->max_pulse_us,
        .max_angle = config->max_angle,
        .speed_mode = speed_mode
    };

    ESP_LOGI(TAG, "Servo no GPIO %d (Timer %d, Canal %d) inicializado.", config->gpio_pin, config->timer_num, config->channel_num);
    
    // Retorna o handle "opaco"
    return (servo_handle_t)handle;

// Ponto de limpeza em caso de erro
err:
    if (handle) {
        free(handle); // Libera memória se a alocação foi bem-sucedida mas algo falhou depois
    }
    ESP_LOGE(TAG, "Falha ao inicializar servo.");
    return NULL;
}

esp_err_t servo_set_angle(servo_handle_t handle, uint16_t angle)
{
    // Validação
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle do servo é NULL");
    
    // Converte o handle opaco de volta para o tipo interno
    servo_handle_t_impl* p_handle = (servo_handle_t_impl*)handle;

    // Satura o ângulo
    if (angle > p_handle->max_angle) {
        angle = p_handle->max_angle;
    }

    // Calcula o duty cycle correspondente
    uint32_t duty = angle_to_duty_cycle(p_handle, angle);

    // Define o duty no canal LEDC
    ESP_RETURN_ON_ERROR(
        ledc_set_duty(p_handle->speed_mode, p_handle->channel, duty),
        TAG, "Falha ao definir duty"
    );

    // Atualiza o canal para aplicar o novo duty
    ESP_RETURN_ON_ERROR(
        ledc_update_duty(p_handle->speed_mode, p_handle->channel),
        TAG, "Falha ao atualizar duty"
    );

    return ESP_OK;
}


esp_err_t servo_deinit(servo_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle do servo é NULL");

    servo_handle_t_impl* p_handle = (servo_handle_t_impl*)handle;

    // Para o canal LEDC (define o duty para 0 e para)
    esp_err_t ret = ledc_stop(p_handle->speed_mode, p_handle->channel, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Falha ao parar canal LEDC");

    ESP_LOGI(TAG, "Servo no canal %d desinicializado.", p_handle->channel);

    // Libera a memória da struct do handle
    free(p_handle);

    return ESP_OK;
}

// Funções específicas para o gripper usado
esp_err_t gripper_init()
{
    servo_config_t gripper_config = {
        .gpio_pin = GRIPPER_SERVO,
        .min_pulse_us = 500,   
        .max_pulse_us = 2500,  
        .max_angle = 180,       
        .timer_num = GRIPPER_TIMER,
        .channel_num = GRIPPER_CHANNEL
    };

    gripper_servo = servo_init(&gripper_config);
    ESP_RETURN_ON_FALSE(gripper_servo, ESP_ERR_INVALID_STATE, TAG, "Falha ao inicializar servo do gripper");

    return ESP_OK;
}

esp_err_t gripper_set_angle(uint16_t angle)
{
    ESP_RETURN_ON_FALSE(gripper_servo, ESP_ERR_INVALID_STATE, TAG, "Servo do gripper não inicializado");

    return servo_set_angle(gripper_servo, angle);
}