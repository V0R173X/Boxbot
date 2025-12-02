#include "stepper_motor.h"
static const char* TAG = "STEPPER";

// Pinos de controle (lidos dos defines em utils.h)
static int   GP_STEP;
static int   GP_DIR;
static float MAX_SPS;

// Variáveis atômicas para gerenciar os passos solicitados e restantes
static atomic_int s_target_steps = 0;
static atomic_int s_remaining_steps = 0;

static void move_blocking(atomic_int *remain_ptr, int gpio_step, int gpio_dir) {
    // Evita que novas chamadas a stepper_move interfiram neste bloco
    int remain = atomic_exchange(remain_ptr, 0);
    if (remain == 0) return; // Nada a fazer

    int dir   = (remain > 0) ? 0 : 1; // 1 para positivo, 0 para negativo
    int steps = abs(remain);

    // Calcula o período do pulso baseado na velocidade máxima
    float sps = MAX_SPS;
    if (sps < 1.0f) sps = 1.0f; // Garante velocidade mínima

    const uint32_t pulse_high_us = 3; // Duração mínima do pulso STEP HIGH (>= 1.9us para DRV8825)
    // Calcula o período total em microssegundos
    uint32_t period_us = (uint32_t)floorf(1000000.0f / sps);
    if (period_us <= pulse_high_us) {
        // Garante que haja tempo para o pulso LOW
        period_us = pulse_high_us + 1;
        ESP_LOGW(TAG, "Velocidade maxima limitada pelo tempo minimo do pulso. Periodo: %lu us", period_us);
    }

    uint32_t low_phase_us = period_us - pulse_high_us; // Tempo que o pino STEP fica LOW

    // Debug
    // ESP_LOGI(TAG, "Movendo %d passos, Direcao: %d, Periodo: %lu us (High: %lu, Low: %lu)",
    //          steps, dir, period_us, pulse_high_us, low_phase_us);

    // Define a direção
    gpio_set_level(gpio_dir, dir);

    // Loop para gerar os pulsos
    for (int i = 0; i < steps; ++i) {
        gpio_set_level(gpio_step, 1);
        esp_rom_delay_us(pulse_high_us); 
        gpio_set_level(gpio_step, 0);

        if (low_phase_us >= 2000) { // Se for maior que ~2ms, usa vTaskDelay
            uint32_t delay_ms = low_phase_us / 1000;
            uint32_t rem_us = low_phase_us % 1000;
            vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Pausa em milissegundos
            if (rem_us > 0) {
                esp_rom_delay_us(rem_us); // Delay preciso para o restante
            }
        } else if (low_phase_us > 0) {
            esp_rom_delay_us(low_phase_us); // Delay curto e preciso
        }

        // Cede controle para outras tarefas periodicamente para não travar o sistema
        if ((i & 0xFF) == 0xFF) { // A cada 256 passos
            taskYIELD();
        }
    }
    // ESP_LOGI(TAG, "Movimento de %d passos concluido.", steps);
}

esp_err_t stepper_init() {
    GP_STEP = STEP_PIN;
    GP_DIR  = DIR_PIN; 
    MAX_SPS = STEPPER_MAX_SPEED;
    int gpios[] = {GP_STEP, GP_DIR};
    for (int i = 0; i < 2; ++i){
        // Configura o pino como saída
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << gpios[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        esp_err_t err = gpio_config(&io_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao configurar GPIO %d: %s", gpios[i], esp_err_to_name(err));
            return err; // Retorna o erro se a configuração falhar
        }
        // Garante que os pinos comecem em nível baixo
        gpio_set_level(gpios[i], 0);
    }

    // Inicializa as variáveis atômicas
    atomic_store(&s_target_steps, 0);
    atomic_store(&s_remaining_steps, 0);

    ESP_LOGI(TAG, "Motor de passo inicializado. STEP=%d, DIR=%d, MAX_SPS=%.1f", GP_STEP, GP_DIR, MAX_SPS);
    return ESP_OK;
}

void stepper_move(int steps)
{
    if (steps == 0) return;
    // Adiciona atomicamente os passos solicitados às contagens
    atomic_fetch_add(&s_target_steps, steps);
    atomic_fetch_add(&s_remaining_steps, steps);
    ESP_LOGD(TAG, "Solicitado mover %d passos. Restantes agora: %d", steps, atomic_load(&s_remaining_steps));
}

void stepper_task_loop(void)
{
    // Verifica se há passos restantes de forma atômica
    if (atomic_load(&s_remaining_steps) != 0) {
        // Se houver, chama a função bloqueante para executá-los
        move_blocking(&s_remaining_steps, GP_STEP, GP_DIR);
    }
}

bool stepper_is_idle(void)
{
    // Verifica atomicamente se a contagem de passos restantes é zero
    return (atomic_load(&s_remaining_steps) == 0);
}