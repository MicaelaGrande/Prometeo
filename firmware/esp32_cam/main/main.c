

#include "esp_log.h"
#include "camera_manager.h"
#include "wifi_manager.h"
#include "stream_server.h"

static const char *TAG = "PROMETEO";

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando Prometeo ESP32-CAM");

    if (camera_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando camara");
        return;
    }

    if (wifi_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error conectando Wi-Fi");
        return;
    }

    if (stream_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando streaming");
        return;
    }

    ESP_LOGI(TAG, "Prometeo listo para transmitir");
}
