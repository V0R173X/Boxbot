#include "ultrasonics.h"

static const char *TAG = "ULTRASONIC";

// Handle do sensor ultrassonico
static ultrasonic_sensor_t s_sensor;
static bool s_ready = false;

esp_err_t ultrassonic_init(void)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = US_TRIG_PIN,
        .echo_pin    = US_ECHO_PIN
    };
    ultrasonic_init(&sensor);

    s_sensor = sensor;

    G_US_CM = 0.0f;
    G_US_TS_MS = 0ULL;

    s_ready = true;
    return ESP_OK;
}

void ultrassonic_sample_once(void)
{
    if (!s_ready) {
        ESP_LOGI(TAG, "Ultrassônico não inicializado. Chame ultrassonic_init() primeiro.");
        return;
    }

    float dist_m = NAN;
    esp_err_t res = ultrasonic_measure(&s_sensor, US_MAX_DISTANCE_M, &dist_m);
    if (res == ESP_OK) {
        float dist_cm = dist_m * 100.0f;
        G_US_CM = dist_cm;
        
        // Debug
        // ESP_LOGI(TAG, "Distância medida: %.2f cm", dist_cm);
    }

    vTaskDelay(pdMS_TO_TICKS(15));
    G_US_TS_MS = (uint64_t)(esp_timer_get_time() / 1000ULL);

    return;
}
