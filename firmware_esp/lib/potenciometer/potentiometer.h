#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#include "utils.h"

/* Enum */
typedef enum{
    POT_BASE = 0,
    POT_ARM = 1
} pot_side_t;

/* Macros */
#define POT_ADC_CHANNEL(SIDE)   ((SIDE) == POT_ARM ? POT_ARM_ADC_CHANNEL : POT_BASE_ADC_CHANNEL)
#define POT_PIN(SIDE)   ((SIDE) == POT_ARM ? POT_ARM_PIN : POT_BASE_PIN)

#define POT_VALUE(SIDE)   ((SIDE) == POT_ARM ? G_POT_ARM_VALUE : G_POT_BASE_VALUE)
#define POT_ANGLE(SIDE)   ((SIDE) == POT_ARM ? G_POT_ARM_ANGLE : G_POT_BASE_ANGLE)


esp_err_t potentiometer_init(pot_side_t side);
void potentiometer_read(pot_side_t side);
void potentiometer_read_angle(pot_side_t side, int ang_min, int ang_max, bool inverse);

#endif // POTENTIOMETER_H