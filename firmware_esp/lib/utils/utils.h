#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include "esp_system.h"
#include "driver/gpio.h"

/* GPIO */

// Ultrassonicos
#define US_TRIG_PIN GPIO_NUM_16
#define US_ECHO_PIN GPIO_NUM_17

// Potenciometro
#define POT_ADC_UNIT    ADC_UNIT_2
#define POT_ARM_ADC_CHANNEL ADC_CHANNEL_7
#define POT_BASE_ADC_CHANNEL ADC_CHANNEL_9

#define POT_ARM_PIN  GPIO_NUM_27
#define POT_BASE_PIN  GPIO_NUM_26

// Servo Motor
#define STBY                GPIO_NUM_22 // Unused

#define INPUT_BASE_1        GPIO_NUM_32
#define INPUT_BASE_2        GPIO_NUM_33
#define LEDC_OUTPUT_BASE    GPIO_NUM_25 // Enable PWM B
#define LEDC_CHANNEL_BASE   LEDC_CHANNEL_1

#define INPUT_ARM_1         GPIO_NUM_18
#define INPUT_ARM_2         GPIO_NUM_19
#define LEDC_OUTPUT_ARM     GPIO_NUM_23 // Enable PWM A 
#define LEDC_CHANNEL_ARM    LEDC_CHANNEL_0

#define GRIPPER_SERVO       GPIO_NUM_13

// Motor de passo
#define STEP_PIN        GPIO_NUM_12
#define DIR_PIN         GPIO_NUM_14
#define ENABLE_PIN      GPIO_NUM_21 // Unused


#define LOW     0
#define HIGH    1

/* Estrutura do Frame de comunicação */
typedef enum {
    STATE_WAIT_SOF,     // Start of Frame
    STATE_WAIT_CMD,     // Comando ('T', 'P', 'I', 'D', 'R')
    STATE_WAIT_ID,      // Identificador de junta ('B', 'A', 'S', 'G')
    STATE_WAIT_VALUE,   // Valor
    STATE_WAIT_CHK,     // Checksum
    STATE_WAIT_EOF      // End of Frame
} parser_state_t;

/* Estrutura da mensagem útil */
typedef struct {
    char cmd;
    char id;
    int value;
} command_t;

/* Variáveis Globais */
// Comando Comunicação
extern command_t G_CMD;
extern bool G_NEW_GAIN;
extern bool G_GAIN_APPLIED;

// Ultrassonics sensor values
extern float   G_US_CM;
extern uint64_t G_US_TS_MS;
extern const float G_US_DIST;

// Potenciometros
extern int G_POT_ARM_VALUE;
extern int G_POT_ARM_ANGLE;
extern int G_POT_BASE_VALUE;
extern int G_POT_BASE_ANGLE;

// Ação de Controle
extern int G_PWM_ARM;
extern int G_PWM_BASE;
extern int G_STEPS;
extern uint16_t G_GRIPPER_ANGLE;

// PID
extern float BASE_KP; 
extern float BASE_KI;
extern float BASE_KD; 
extern float ARM_KP; 
extern float ARM_KI; 
extern float ARM_KD; 
extern float STEPPER_KP; 
extern float STEPPER_KI;
extern float STEPPER_KD; 

// Targets
extern float G_BASE_TARGET_ANGLE;  
extern float G_ARM_TARGET_ANGLE;  
extern float G_STEPPER_TARGET_DIST;

#endif // UTILS_H