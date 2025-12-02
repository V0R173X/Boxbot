#include "pid.h"

static pid_ctrl_block_handle_t PID_BASE = NULL;
static pid_ctrl_block_handle_t PID_ARM = NULL;
static pid_ctrl_block_handle_t PID_STEPPER = NULL;

// Filtro digital para leitura dos potenciômetros
static float filtered_pot_angle_arm = 0.0f;
static float filtered_pot_angle_base = 0.0f;
static float filtered_stepper = 0.0f;

const char *TAG_PID = "PID";

esp_err_t init_pid(motor_pid_t motor) {

  pid_ctrl_config_t config_pid; 
  pid_ctrl_block_handle_t pid_block; 
  

  pid_ctrl_parameter_t values_pid = {
    .kd = PID_SIDE_KD(motor),
    .kp = PID_SIDE_KP(motor),
    .ki = PID_SIDE_KI(motor),
    .min_integral = MIN_INTEGRAL(motor),
    .max_integral = MAX_INTEGRAL(motor),
    .min_output = MIN_OUTPUT(motor),
    .max_output = MAX_OUTPUT(motor),
    .cal_type = PID_CAL_TYPE_POSITIONAL,
  };

  config_pid.init_param = values_pid;

  esp_err_t ret = pid_new_control_block(&config_pid, &pid_block);  
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_PID, "Erro ao inicializar PID: %s", esp_err_to_name(ret));
    return ret;
  }
  ret = pid_update_parameters(pid_block, &values_pid);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_PID, "Erro ao atualizar parâmetros do PID: %s", esp_err_to_name(ret));
    return ret;
  }

  // Atribuição do handle ao motor correspondente
  if (motor == BASE_PID) {
    PID_BASE = pid_block;
  } 
  else if (motor == ARM_PID) {
    PID_ARM = pid_block;
  }
  else if (motor == STEPPER_PID) {
    PID_STEPPER = pid_block;
  }

  return ESP_OK;
}

esp_err_t pid_calculate(motor_pid_t motor) {
  // Estabelecendo o bloco
  pid_ctrl_block_handle_t pid_block;
  pid_block = (motor == ARM_PID) ? PID_ARM : (motor == BASE_PID) ? PID_BASE : PID_STEPPER;
  if(pid_block == NULL) {
    ESP_LOGE(TAG_PID, "Erro na leitura: PID não inicializado. Chame init_pid() primeiro.");
    return ESP_ERR_INVALID_STATE;
  }

  // Calculando e estabelecendo o feedback
  float feedback;
  float raw;
  float alpha;
  float* filtered;

  if(motor == STEPPER_PID) {
    ultrassonic_sample_once();
    raw = G_US_DIST - G_US_CM;
    alpha = 0.3f;
    filtered = &filtered_stepper;
  } else {
    pot_side_t side = (motor == ARM_PID) ? POT_ARM : POT_BASE;
    bool inverse = (motor == ARM_PID) ? true : false;
    int limit_max = (motor == ARM_PID) ? 215 : 60;

    potentiometer_read_angle(side, 0, limit_max, inverse);

    raw = POT_ANGLE(side);
    alpha = 0.4f;
    filtered = (side == POT_ARM) ? &filtered_pot_angle_arm : &filtered_pot_angle_base;
  }

  // Filtro EMA
  *filtered = (alpha * raw) + ((1.0f - alpha) * (*filtered));
  feedback = *filtered;
  
  // Atribuindo Target
  float target;
  target = (motor == STEPPER_PID) ? G_STEPPER_TARGET_DIST : (motor == ARM_PID) ? G_ARM_TARGET_ANGLE : G_BASE_TARGET_ANGLE;

  // Calculando Erro
  float error = target - feedback;

  // Aplicando ação de controle
  float action = 0;
  float threshold = 0.0f;

  if (motor == STEPPER_PID) {
      threshold = 0.2f; 
  } else {
      threshold = 1.0f;
  }

  if (fabsf(error) <= threshold){
    action = 0.0f;
  } else {
    esp_err_t ret = pid_compute(pid_block, error, &action);
    if(ret != ESP_OK) {
      ESP_LOGE(TAG_PID, "Error computing PID");
      return ret;
    }
  }

  if(motor == STEPPER_PID) {
    const float period_sec = ((float)PERIOD / 1000.0f);
    float steps_float = action * period_sec;
    G_STEPS = (int)roundf(steps_float);
  } else {
    action += 0.5f;
    if(motor == ARM_PID) {
      G_PWM_ARM = (int)action;
    } else if(motor == BASE_PID){
      G_PWM_BASE = (int)action;
    }
  }

  return ESP_OK;
}

esp_err_t pid_apply_new_parameters(motor_pid_t motor) {
    pid_ctrl_block_handle_t pid_block = NULL;

    // Obter o handle do bloco PID correto
    if (motor == ARM_PID) pid_block = PID_ARM;
    else if (motor == BASE_PID) pid_block = PID_BASE;
    else if (motor == STEPPER_PID) pid_block = PID_STEPPER;

    if (pid_block == NULL) {
        return ESP_ERR_INVALID_STATE; // Bloco não inicializado
    }

    // Criar a estrutura de parâmetros com os novos valores globais
    pid_ctrl_parameter_t new_values = {
        .kd = PID_SIDE_KD(motor), 
        .kp = PID_SIDE_KP(motor),
        .ki = PID_SIDE_KI(motor),
        // Mantém limites fixos
        .min_integral = MIN_INTEGRAL(motor),
        .max_integral = MAX_INTEGRAL(motor),
        .min_output = MIN_OUTPUT(motor),
        .max_output = MAX_OUTPUT(motor),
        .cal_type = PID_CAL_TYPE_POSITIONAL,
    };

    pid_update_parameters(pid_block, &new_values);

    return ESP_OK;
}