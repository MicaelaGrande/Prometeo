#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hcsr04.h"

static const char *TAG = "main";

#define NUM_SENSORS 3

void app_main(void)
{
    /* Un config por sensor: TRIG y ECHO propios */
    hcsr04_config_t cfg[NUM_SENSORS] = {
        { .trig_gpio = GPIO_NUM_18, .echo_gpio = GPIO_NUM_19 },  /* sensor 0 */
        { .trig_gpio = GPIO_NUM_21, .echo_gpio = GPIO_NUM_22 },  /* sensor 1 */
        { .trig_gpio = GPIO_NUM_25, .echo_gpio = GPIO_NUM_26 },  /* sensor 2 */
    };
    hcsr04_handle_t sensors[NUM_SENSORS] = { NULL };

    for (int i = 0; i < NUM_SENSORS; i++) {
        ESP_ERROR_CHECK(hcsr04_new(&cfg[i], &sensors[i]));
    }

    while (1) {
        /* De a uno, para que los sensores no se interfieran */
        for (int i = 0; i < NUM_SENSORS; i++) {
            float cm = 0;
            esp_err_t err = hcsr04_measure_cm(sensors[i], &cm);

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "S%d: %.1f cm", i, cm);
            } else if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "S%d: sin eco, revisar cableado", i);
            } else if (err == ESP_ERR_INVALID_RESPONSE) {
                ESP_LOGW(TAG, "S%d: fuera de rango (2 a 400 cm)", i);
            } else {
                ESP_LOGE(TAG, "S%d: %s", i, esp_err_to_name(err));
            }
        }
    }
}