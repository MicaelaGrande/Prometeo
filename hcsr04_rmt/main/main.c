#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hcsr04.h"

static const char *TAG = "main";

void app_main(void)
{
    /* TRIG = GPIO18, ECHO = GPIO19 (ECHO con divisor resistivo a 3,3 V) */
    hcsr04_config_t config = HCSR04_CONFIG_DEFAULT();
    hcsr04_handle_t sensor = NULL;
    ESP_ERROR_CHECK(hcsr04_new(&config, &sensor));

    while (1) {
        float cm = 0;
        esp_err_t err = hcsr04_measure_cm(sensor, &cm);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Distancia: %.1f cm", cm);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Sin eco: revisar cableado y alimentacion del sensor");
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGW(TAG, "Fuera de rango (2 a 400 cm)");
        } else {
            ESP_LOGE(TAG, "Error: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(100));  /* el driver ya garantiza >= 60 ms entre disparos */
    }
}
