#include "utils.h"

command_t G_CMD = { .cmd = '0', .id = '0', .value = 0 };
bool G_NEW_GAIN = false;
bool G_GAIN_APPLIED = false;

float   G_US_CM  = 0.0f;
uint64_t G_US_TS_MS = 0ULL;
const float G_US_DIST = 11.0f;

int G_POT_ARM_VALUE = 0;
int G_POT_ARM_ANGLE = 0;
int G_POT_BASE_VALUE = 0;
int G_POT_BASE_ANGLE = 0;

int G_PWM_ARM = 0;
int G_PWM_BASE = 0;
int G_STEPS = 0;
uint16_t G_GRIPPER_ANGLE = 0;

float BASE_KP = 85.0; 
float BASE_KI = 2.0;
float BASE_KD = 10.0; 
float ARM_KP = 55.0; 
float ARM_KI = 2.0; 
float ARM_KD = 7.5; 
float STEPPER_KP = 700.0; 
float STEPPER_KI = 1.0;
float STEPPER_KD = 75.0; 

float G_BASE_TARGET_ANGLE = 0;  
float G_ARM_TARGET_ANGLE = 0;  
float G_STEPPER_TARGET_DIST = 0;