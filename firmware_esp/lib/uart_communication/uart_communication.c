#include "uart_communication.h"

static const char *TAG = "UART";

// Variáveis estáticas para guardar o estado do parser
static parser_state_t s_current_state = STATE_WAIT_SOF;
static uint8_t s_received_cmd;
static uint8_t s_received_id;
static uint8_t s_data_index = 0;
static union {
    uint8_t bytes[2];
    uint16_t value;
} s_received_value;

// Parser de recebimento de bytes
static void process_received_byte(uint8_t byte) {
    switch (s_current_state)
    {
    case STATE_WAIT_SOF:
        if (byte == FRAME_SOF) {
            s_data_index = 0; // Reseta o índice para o próximo uso
            s_current_state = STATE_WAIT_CMD;
        }
        break;

    case STATE_WAIT_CMD:
        s_received_cmd = byte;
        s_current_state = STATE_WAIT_ID; 
        break;
    
    case STATE_WAIT_ID:
        s_received_id = byte;
        s_current_state = STATE_WAIT_VALUE; 
        break;

    case STATE_WAIT_VALUE:
        s_received_value.bytes[s_data_index] = byte;
        s_data_index++;

        // Verificação se já recebeu todos os bytes
        if (s_data_index >= 2) {
            s_current_state = STATE_WAIT_CHK;
        }
        break;

    case STATE_WAIT_CHK:
        {
            uint8_t calculated_chk = (s_received_cmd + s_received_id + s_received_value.bytes[0] + s_received_value.bytes[1]) & 0xFF;
            
            if (byte == calculated_chk) {
                s_current_state = STATE_WAIT_EOF;
            } else {
                ESP_LOGW(TAG, "Checksum falhou! Esperado: 0x%02X, Recebido: 0x%02X", calculated_chk, byte);
                s_current_state = STATE_WAIT_SOF;
            }
        }
        break;

    case STATE_WAIT_EOF:
        if (byte == FRAME_EOF) {
            // Frame válido recebido!
            ESP_LOGI(TAG, "Frame válido recebido! CMD: %c (0x%02X), ID: %c (0x%02X), VALOR: %d", 
                    (char)s_received_cmd, s_received_cmd, (char)s_received_id, s_received_id, s_received_value.value);
            
            // Monta a estrutura de comando
            G_CMD.cmd = (char)s_received_cmd;
            G_CMD.id = (char)s_received_id;
            G_CMD.value = (int)s_received_value.value;

            if(G_CMD.cmd == 'P' || G_CMD.cmd == 'I' || G_CMD.cmd == 'D'){
                G_NEW_GAIN = true; // Sinaliza que um novo ganho foi recebido
            }

        } else {
            ESP_LOGW(TAG, "EOF falhou! Esperado: 0x%02X, Recebido: 0x%02X", FRAME_EOF, byte);
        }
        
        // Reseta para o próximo frame.
        s_current_state = STATE_WAIT_SOF;
        break;
    
    default:
        s_current_state = STATE_WAIT_SOF;
        break;
    }
}

esp_err_t uart_communication_init() {
    // Configuração do UART
    const uart_config_t uart_config = {
        .baud_rate = 115200, // Velocidade padrão para comunicação USB/Console
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // Configura os parâmetros da UART
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    // Instala o driver da UART. 
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "Driver UART0 (USB) inicializado a 115200 bps.");
    return ESP_OK;
}

esp_err_t uart_send_frame(char cmd, char id, uint16_t value){
    // Prepara o buffer do frame
    uint8_t frame_buffer[7];

    union {
        uint16_t value;
        uint8_t bytes[2];
    } val_to_send;

    val_to_send.value = value;

    // Calcula o checksum
    uint8_t chk = ((uint8_t)cmd + (uint8_t)id + val_to_send.bytes[0] + val_to_send.bytes[1]) & 0xFF;

    // Monta o frame
    frame_buffer[0] = FRAME_SOF;
    frame_buffer[1] = (uint8_t)cmd;
    frame_buffer[2] = (uint8_t)id;
    frame_buffer[3] = val_to_send.bytes[0]; // Byte menos significativo
    frame_buffer[4] = val_to_send.bytes[1]; // Byte mais significativo
    frame_buffer[5] = chk;
    frame_buffer[6] = FRAME_EOF;  

    // Envia os 7 bytes de uma vez
    int bytes_sent = uart_write_bytes(UART_PORT_NUM, frame_buffer, 7);

    if (bytes_sent == 7) {
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Falha ao enviar frame, bytes_sent=%d (esperado 7)", bytes_sent);
        return ESP_FAIL;
    }
}

void uart_read_task_loop() {
    // Aloca o buffer de leitura estático para evitar Heap fragmentation
    static uint8_t data[RD_BUF_SIZE];

    int len = uart_read_bytes(UART_PORT_NUM, data, RD_BUF_SIZE, 0);

    if (len > 0) {
        // Processa cada byte que chegou
        for (int i = 0; i < len; i++) {
            process_received_byte(data[i]);
        }
    }
}

void uart_send_task_loop(char task_id){
    // Envia os valores atuais dos feedbacks via comunicação
    uint16_t b_return = (uint16_t)G_POT_BASE_ANGLE;
    uint16_t s_return = (uint16_t)((G_US_DIST - G_US_CM + 0.5f)*10); // Distancia em cm arredondada
    uint16_t a_return = (uint16_t)G_POT_ARM_ANGLE;

    if(task_id == 'B'){
        uart_send_frame('R', 'B', b_return);
    }else if(task_id == 'S'){
        uart_send_frame('R', 'S', s_return);
    }else if(task_id == 'A'){
        uart_send_frame('R', 'A', a_return);
    }

    // Valida recebimento de ganhos novos
    if(G_GAIN_APPLIED){
        union{
            uint16_t ret;
            uint8_t bytes[2];
        } u_ok;
        u_ok.bytes[0] = (uint8_t)'O';
        u_ok.bytes[1] = (uint8_t)'K';

        uart_send_frame('R', 'V', u_ok.ret);
        ESP_LOGI(TAG, "Novos ganhos aplicados");
        G_GAIN_APPLIED = false;
    }
}