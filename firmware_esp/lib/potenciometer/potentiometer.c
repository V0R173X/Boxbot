#include "potentiometer.h"

static const char *TAG = "POT";

// Handle para a unidade ADC configurada
static adc_oneshot_unit_handle_t s_adc_unit_handle = NULL;
static bool s_adc_initialized = false;

esp_err_t potentiometer_init(pot_side_t side) {
    esp_err_t ret;
    if (!s_adc_initialized) {
        adc_oneshot_unit_init_cfg_t init_config = {
            // Assume que ambos ARM e BASE usam a mesma unidade
            .unit_id = POT_ADC_UNIT, 
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ret = adc_oneshot_new_unit(&init_config, &s_adc_unit_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao inicializar unidade ADC: %s", esp_err_to_name(ret));
            s_adc_unit_handle = NULL;
            return ret;
        }
        s_adc_initialized = true;
    }

    // Configura o canal ADC específico
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,     // Atenuação para faixa completa de tensão da ESP
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Resolução padrão de 12 bits
    };
    ret = adc_oneshot_config_channel(s_adc_unit_handle, POT_ADC_CHANNEL(side), &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar canal ADC %d: %s", POT_ADC_CHANNEL(side), esp_err_to_name(ret));
        // Se a configuração do canal falhar, podemos tentar desalocar a unidade
        adc_oneshot_del_unit(s_adc_unit_handle);
        s_adc_unit_handle = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "ADC inicializado com sucesso para GPIO %d (Canal %d)", (int)POT_PIN(side), (int)POT_ADC_CHANNEL(side));
    return ESP_OK;
}

void potentiometer_read(pot_side_t side) {
    if (s_adc_unit_handle == NULL) {
        ESP_LOGE(TAG, "Erro na leitura: ADC nao inicializado. Chame potentiometer_init() primeiro.");
        return;
    }

    int adc_raw_reading; // Variável para armazenar a leitura raw

    // Realiza a leitura do ADC
    esp_err_t ret = adc_oneshot_read(s_adc_unit_handle, POT_ADC_CHANNEL(side), &adc_raw_reading);

    if (ret == ESP_OK) {
        // Atualiza a variável global com o valor lido
        if(side == POT_ARM) {
            G_POT_ARM_VALUE = adc_raw_reading;
        } else{
            G_POT_BASE_VALUE = adc_raw_reading;
        }
        ESP_LOGD(TAG, "Leitura ADC Raw (GPIO %d): %d", (int)POT_PIN(side), POT_VALUE(side));
    } else {
        ESP_LOGE(TAG, "Falha na leitura do ADC: %s", esp_err_to_name(ret));
    }
}

void potentiometer_read_angle(pot_side_t side, int out_min, int out_max, bool inverse) {
    potentiometer_read(side);
    const int in_max = 4095;
    const int in_min = 0;

    int pwm;
    pwm = (inverse) ? (in_max - POT_VALUE(side)) : POT_VALUE(side);

    // Garante que min <= max
    if (out_min > out_max) {
        ESP_LOGW(TAG, "Erro no mapeamento: out_min (%d) > out_max (%d). Usando intervalo invertido.", out_min, out_max);
        // Troca os valores para continuar
        int temp = out_min;
        out_min = out_max;
        out_max = temp;
    }
    
    // Fórmula de mapeamento
    float mapped_value = (float)out_min + (float)(pwm - in_min) * (out_max - out_min) / (in_max - in_min) + 0.5f;

    // Garante que o valor final esteja estritamente dentro dos limites definidos
    if ((int)mapped_value < out_min) mapped_value = out_min;
    if ((int)mapped_value > out_max) mapped_value = out_max;

    // Atualiza a variável global mapeada
    if(side == POT_ARM) {
        G_POT_ARM_ANGLE = (int)mapped_value;
    } else {
        G_POT_BASE_ANGLE = (int)mapped_value;
    }
}