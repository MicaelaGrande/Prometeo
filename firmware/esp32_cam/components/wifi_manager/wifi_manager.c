#include "wifi_manager.h"

#include <string.h>

#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif_ip_addr.h"

static const char *TAG = "WIFI_MANAGER";

// Callback que recibe eventos de Wi-Fi (solo para avisar por log quien se conecta)
static void wifi_event_handler(
    void *arg,
    esp_event_base_t base,
    int32_t id,
    void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "Cliente conectado: " MACSTR " (aid=%d)",
                 MAC2STR(e->mac), e->aid);
    }
    else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGW(TAG, "Cliente desconectado: " MACSTR " (aid=%d)",
                 MAC2STR(e->mac), e->aid);
    }
}

// Inicializacion de Wi-Fi: la camara crea su propio hotspot (modo AP).
// Los clientes reciben IP por DHCP; la camara queda en 192.168.4.1.
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

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .channel = CONFIG_PROMETEO_WIFI_CHANNEL,
            .max_connection = CONFIG_PROMETEO_WIFI_MAX_CLIENTES,
            .authmode = WIFI_AUTH_WPA2_PSK,
        }
    };

    strlcpy((char *)wifi_config.ap.ssid, CONFIG_PROMETEO_WIFI_SSID,
            sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen((char *)wifi_config.ap.ssid);
    strlcpy((char *)wifi_config.ap.password, CONFIG_PROMETEO_WIFI_PASSWORD,
            sizeof(wifi_config.ap.password));

    if (strlen(CONFIG_PROMETEO_WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    } else if (strlen(CONFIG_PROMETEO_WIFI_PASSWORD) < 8) {
        ESP_LOGE(TAG, "La contrasena del hotspot debe tener al menos 8 caracteres");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip;
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap && esp_netif_get_ip_info(ap, &ip) == ESP_OK) {
        ESP_LOGI(TAG, "Hotspot '%s' activo (canal %d). IP de la camara: " IPSTR,
                 CONFIG_PROMETEO_WIFI_SSID, CONFIG_PROMETEO_WIFI_CHANNEL,
                 IP2STR(&ip.ip));
    }

    return ESP_OK;
}
