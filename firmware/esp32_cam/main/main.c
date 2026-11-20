

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_manager.h"
#include "wifi_manager.h"
#include "stream_server.h"

static const char *TAG = "PROMETEO";

// LED rojo chico de la ESP32-CAM AI-Thinker: GPIO33, prende con nivel BAJO.
// Indicador de "el firmware esta corriendo": un destello corto cada tanto, apagado el resto
// del tiempo (consumo minimo). No depende de que la camara ni el Wi-Fi hayan arrancado.
#define LED_GPIO        GPIO_NUM_33
#define LED_PULSO_MS    30      // duracion del destello
#define LED_PERIODO_MS  2000    // un destello cada 2 s

static void led_vida_task(void *arg)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);   // apagado

    while (1) {
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(LED_PULSO_MS));
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_PERIODO_MS - LED_PULSO_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando Prometeo ESP32-CAM");

    // El LED late desde el primer momento: si destella, el firmware esta corriendo
    xTaskCreate(led_vida_task, "led_vida", 2048, NULL, 1, NULL);

    if (camera_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando camara");
        return;
    }

    if (wifi_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error creando el hotspot Wi-Fi");
        return;
    }

    if (stream_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando streaming");
        return;
    }

    ESP_LOGI(TAG, "Prometeo listo para transmitir");
}
