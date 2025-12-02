#include "bt_communication.h"

static const char *TAG = "BT";

// Variáveis estáticas para guardar o estado atual do parser e os dados do frame
static parser_state_t s_current_state = STATE_WAIT_SOF;
static uint8_t s_received_cmd;
static uint8_t s_received_value;

static void process_received_byte(uint8_t byte) {
    // Implementação de uma Máquina de Estados Finita (FSM) para obtenção do estado
    switch (s_current_state)
    {
    // Estado 1: Esperando pelo SOF
    case STATE_WAIT_SOF:
        if (byte == FRAME_SOF) {
            s_current_state = STATE_WAIT_CMD;
        }
        break;

    // Estado 2: Esperando pelo Comando
    case STATE_WAIT_CMD:
        s_received_cmd = byte;
        s_current_state = STATE_WAIT_VALUE; 
        break;

    // Estado 3: Esperando pelo Valor
    case STATE_WAIT_VALUE:
        s_received_value = byte; 
        s_current_state = STATE_WAIT_CHK; 
        break;

    // Estado 4: Esperando pelo Checksum
    case STATE_WAIT_CHK:
        {
            // Calcula o checksum esperado
            uint8_t calculated_chk = (s_received_cmd + s_received_value) & 0xFF;
            
            if (byte == calculated_chk) {
                s_current_state = STATE_WAIT_EOF;
            } else {
                // Falha no Checksum. Frame corrompido.
                ESP_LOGW(TAG, "Checksum falhou! Esperado: 0x%02X, Recebido: 0x%02X", calculated_chk, byte);
                s_current_state = STATE_WAIT_SOF;
            }
        }
        break;

    // Estado 5: Esperando pelo EOF
    case STATE_WAIT_EOF:
        if (byte == FRAME_EOF) {
            ESP_LOGI(TAG, "Frame válido recebido! CMD: %c (0x%02X), VALOR: %d", 
                    (char)s_received_cmd, s_received_cmd, s_received_value);

            // Monta a estrutura de comando
            G_CMD.id = (char)s_received_cmd;
            G_CMD.value = (int)s_received_value;

        } else {
            // Se não for o EOF, o frame está corrompido.
            ESP_LOGW(TAG, "EOF falhou! Esperado: 0x%02X, Recebido: 0x%02X", FRAME_EOF, byte);
        }
        
        // Reseta a máquina para esperar o próximo SOF.
        s_current_state = STATE_WAIT_SOF;
        break;
    
    default:
        // Caso de segurança: se o estado for inválido, reseta
        s_current_state = STATE_WAIT_SOF;
        break;
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP inicializado.");
        esp_bt_gap_set_device_name(DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
        break;

    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "Servidor SPP iniciado.");
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "Cliente conectado. Handle: %" PRIu32, param->srv_open.handle);
        // Ao conectar, reseta a máquina de estados para um estado limpo
        s_current_state = STATE_WAIT_SOF;
        break;

    case ESP_SPP_DATA_IND_EVT:
        // Alimenta a FSM byte a byte
        for (uint16_t i = 0; i < param->data_ind.len; i++) {
            process_received_byte(param->data_ind.data[i]);
        }
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "Conexão fechada. Handle: %" PRIu32, param->close.handle);
        // Reseta a máquina de estados
        s_current_state = STATE_WAIT_SOF;
        
        // Envia um comando especial 'D' (Disconnect) diretamente
        G_CMD.id = 'D'; 
        G_CMD.value = 0;
        break;

    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(TAG, "Servidor SPP parado.");
        break;

    case ESP_SPP_CONG_EVT:
        break;

    default:
        break;
    }
}

esp_err_t bt_communication_init() {
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o controlador BT: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao habilitar o controlador BT: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao habilitar bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao registrar callback SPP: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar SPP: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t bt_communication_deinit() {
    esp_spp_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    return ESP_OK;
}