
#include "wifi_manager.h"

#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif_ip_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "WIFI_MANAGER";

static EventGroupHandle_t wifi_events;

#define WIFI_CONNECTED_BIT BIT0

// Callback que recibe eventos de Wi-Fi
static void wifi_event_handler(
    void *arg,
    esp_event_base_t base,
    int32_t id,
    void *data)
{
    if (base == WIFI_EVENT &&
        id == WIFI_EVENT_STA_START) {

        esp_wifi_connect();
    }

    else if (base == WIFI_EVENT &&
             id == WIFI_EVENT_STA_DISCONNECTED) {

        xEventGroupClearBits(
            wifi_events, WIFI_CONNECTED_BIT);

        ESP_LOGW(TAG, "Wi-Fi desconectado. Reconectando...");
        esp_wifi_connect();
    }

    else if (base == IP_EVENT &&
             id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event = data;

        ESP_LOGI(TAG, "Conectado. IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));

        xEventGroupSetBits(
            wifi_events, WIFI_CONNECTED_BIT);
    }
}

// Inicializacion de Wi-Fi
esp_err_t wifi_manager_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        return err;
    }

    wifi_events = xEventGroupCreate();

    if (wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_event_handler,
        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_PROMETEO_WIFI_SSID,
            .password = CONFIG_PROMETEO_WIFI_PASSWORD,
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_set_config(
        WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        wifi_events,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(15000));

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "No se obtuvo IP en 15 segundos");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}
