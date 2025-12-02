#ifndef ULTRASONICS_H
#define ULTRASONICS_H

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ultrasonic.h"  
#include "utils.h"

#define US_MAX_DISTANCE_M 2.0f

esp_err_t ultrassonic_init(void);

void ultrassonic_sample_once(void);

#endif // ULTRASONICS_H
