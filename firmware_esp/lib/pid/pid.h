#ifndef PID_H
#define PID_H

/* Dependencies */
#include "pid_ctrl.h"
#include "h_bridge.h"
#include "math.h"

#include "ultrasonics.h"
#include "potentiometer.h"
#include "utils.h"

/* Enum */
typedef enum{
    BASE_PID = 0,
    ARM_PID = 1,
    STEPPER_PID = 2
} motor_pid_t;

/* Variáveis de Controle */
// Servo
#define SERVO_MAX_OUTPUT 4095 // Saída PID mapeada para 12 bits
#define SERVO_MIN_OUTPUT -4095
#define SERVO_MAX_INTEGRAL 1000 // Limite para evitar windup integral
#define SERVO_MIN_INTEGRAL -1000

// Stepper
#define STEPPER_MAX_OUTPUT 1000 // Limita quantos passos o PID pode comandar por ciclo
#define STEPPER_MIN_OUTPUT -1000
#define STEPPER_MAX_INTEGRAL 100 // Limite pequeno ou zero
#define STEPPER_MIN_INTEGRAL -100

#define PERIOD 50

/* Macros para parâmetros PID */
#define PID_SIDE_KP(MOTOR) ((MOTOR) == BASE_PID ? BASE_KP : (MOTOR) == ARM_PID ? ARM_KP : STEPPER_KP)
#define PID_SIDE_KI(MOTOR) ((MOTOR) == BASE_PID ? BASE_KI : (MOTOR) == ARM_PID ? ARM_KI : STEPPER_KI)
#define PID_SIDE_KD(MOTOR) ((MOTOR) == BASE_PID ? BASE_KD : (MOTOR) == ARM_PID ? ARM_KD : STEPPER_KD)

#define MAX_OUTPUT(MOTOR) ((MOTOR) == STEPPER_PID ? STEPPER_MAX_OUTPUT : SERVO_MAX_OUTPUT)
#define MIN_OUTPUT(MOTOR) ((MOTOR) == STEPPER_PID ? STEPPER_MIN_OUTPUT : SERVO_MIN_OUTPUT)

#define MAX_INTEGRAL(MOTOR) ((MOTOR) == STEPPER_PID ? STEPPER_MAX_INTEGRAL : SERVO_MAX_INTEGRAL)
#define MIN_INTEGRAL(MOTOR) ((MOTOR) == STEPPER_PID ? STEPPER_MIN_INTEGRAL : SERVO_MIN_INTEGRAL)

esp_err_t init_pid(motor_pid_t motor);

esp_err_t pid_calculate(motor_pid_t motor);

esp_err_t pid_apply_new_parameters(motor_pid_t motor);

#endif // PID_H
