#include "task_manager.h"

static const char *COMM_TAG = "UART_TASK";
// static const char *COMM_TAG = "BT_TASK";
static const char *ACT_TAG = "ACT_TASK";

static void nvs_init(){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(COMM_TAG, "NVS inicializado.");
}

static void comm_receive_att(){
    switch (G_CMD.cmd) {
        case 'T': 
            switch (G_CMD.id){
                case 'B': G_BASE_TARGET_ANGLE = (float)G_CMD.value; break;
                case 'S': G_STEPPER_TARGET_DIST = ((float)G_CMD.value)/10; break;
                case 'A': G_ARM_TARGET_ANGLE = (float)G_CMD.value; break;
                case 'G': G_GRIPPER_ANGLE = (uint16_t)G_CMD.value; break;

                default: break;
            }
            break;

        case 'P':
            switch (G_CMD.id){
                case 'B': BASE_KP = ((float)G_CMD.value)/10; break;
                case 'S': STEPPER_KP = ((float)G_CMD.value)/10; break;
                case 'A': ARM_KP = ((float)G_CMD.value)/10; break;

                default: break;
            }
            break;

        case 'I':
            switch (G_CMD.id){
                case 'B': BASE_KI = ((float)G_CMD.value)/10; break;
                case 'S': STEPPER_KI = ((float)G_CMD.value)/10; break;
                case 'A': ARM_KI = ((float)G_CMD.value)/10; break;

                default: break;
            }
            break;

        case 'D':
            switch (G_CMD.id){
                case 'B': BASE_KD = ((float)G_CMD.value)/10; break;
                case 'S': STEPPER_KD = ((float)G_CMD.value)/10; break;
                case 'A': ARM_KD = ((float)G_CMD.value)/10; break;

                default: break;
            }
            break;

        default: break;
    }
}

void uart_task(void *pvParameters) {
    nvs_init();
    ESP_LOGI(COMM_TAG, "Inicializando comunicação serial (UART0/USB)");
    esp_err_t ret = uart_communication_init();
    if (ret != ESP_OK) {
        ESP_LOGE(COMM_TAG, "Falha catastrofica ao iniciar a UART. Deleta Task");
        vTaskDelete(NULL); 
        return;
    }
    ESP_LOGI(COMM_TAG, "Comunicação serial inicializada.");

    while(1){
        uart_read_task_loop();
        comm_receive_att();
        
        // Envia os dados em 3 partes para evitar congestionamento
        vTaskDelay(pdMS_TO_TICKS(FREQ_COMMUNICATION/3));
        uart_send_task_loop('B');
        vTaskDelay(pdMS_TO_TICKS(FREQ_COMMUNICATION/3));
        uart_send_task_loop('S');
        vTaskDelay(pdMS_TO_TICKS(FREQ_COMMUNICATION/3));
        uart_send_task_loop('A');
    }
}

void bt_task(void *pvParameters) {
    nvs_init();
    ESP_LOGI(COMM_TAG, "Inicializando Bluetooth...");
    
    // Inicializar o modulo de comunicacao Bluetooth
    esp_err_t ret = bt_communication_init();
    
    if (ret != ESP_OK) {
        ESP_LOGE(COMM_TAG, "Falha catastrofica ao iniciar o Bluetooth. Deleta Task");
        vTaskDelete(NULL); // Deleta a si mesma se a inicialização falhar
        return;
    }

    ESP_LOGI(COMM_TAG, "Bluetooth inicializado.");
    while(1){
        comm_receive_att();
        vTaskDelay(pdMS_TO_TICKS(FREQ_COMMUNICATION));
    }
}

void actuators_task(void *pvParameters) {
    ESP_LOGI(ACT_TAG, "Inicializando módulos de hardware...");
    init_hbridge();
    stepper_init();
    ultrassonic_init();
    gripper_init();

    potentiometer_init(POT_BASE);
    potentiometer_init(POT_ARM);

    ESP_LOGI(ACT_TAG, "Inicializando controladores PID...");
    init_pid(BASE_PID);
    init_pid(ARM_PID);
    init_pid(STEPPER_PID);

    // Variável para manter o tempo de execução periódico
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(FREQ_CONTROL);
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (G_NEW_GAIN) {
            pid_apply_new_parameters(BASE_PID);
            pid_apply_new_parameters(ARM_PID);
            pid_apply_new_parameters(STEPPER_PID);
            G_NEW_GAIN = false;
            G_GAIN_APPLIED = true;
        }

        pid_calculate(BASE_PID);
        pid_calculate(ARM_PID);

        update_motor(ARM, G_PWM_ARM);
        update_motor(BASE, G_PWM_BASE);

        gripper_set_angle(G_GRIPPER_ANGLE);

        if (stepper_is_idle()) {
            // ESP_LOGI(ACT_TAG, "Iniciando movimento do motor de passo: %d passos", G_STEPS);
            pid_calculate(STEPPER_PID);
            stepper_move(G_STEPS);
        }
        stepper_task_loop();

        // Debug
        // ESP_LOGI(ACT_TAG, "BASE: Ang=%d, PWM=%d | ARM: Ang=%d, PWM=%d | STEP: US=%.1f, Steps=%d", 
        //          G_POT_BASE_ANGLE, G_PWM_BASE, G_POT_ARM_ANGLE, G_PWM_ARM, G_US_CM, G_STEPS);
    }
}

esp_err_t init_tasks() {
    // Task 1 (core 0): comunicação
    // xTaskCreatePinnedToCore(bt_task, "bt_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(uart_task, "uart_task", 4096, NULL, 5, NULL, 0);

    // Task 2 (core 1): atuação
    xTaskCreatePinnedToCore(actuators_task, "actuators_task", 4096, NULL, 5, NULL, 1);

    return ESP_OK;
}