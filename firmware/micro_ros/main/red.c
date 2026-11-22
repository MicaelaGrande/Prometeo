/*
 * Red del cuerpo: WiFi (cliente del hotspot de la camara) y busqueda del agente micro-ROS.
 * Va aparte de main.c para que este quede solo con micro-ROS (topics y callbacks).
 */
#include "red.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "sdkconfig.h"

#include <rmw_microros/rmw_microros.h>

static const char *TAG = "red";

// =========================================================
// WIFI: se une al hotspot de la camara y reintenta SIEMPRE
// =========================================================
// (La interfaz de red de micro-ROS se rinde despues de unos pocos intentos y no vuelve a
// probar: si la camara tardaba en levantar el hotspot, el cuerpo se quedaba sin WiFi.)
static void wifi_evento(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    static uint32_t fallos = 0;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (fallos++ % 20 == 0) {   // no llenar el log: solo avisa cada tanto
            ESP_LOGW(TAG, "Sin hotspot '%s': reintentando (intentos: %u)",
                     CONFIG_ESP_WIFI_SSID, (unsigned)fallos);
        }
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = data;
        fallos = 0;
        ESP_LOGI(TAG, "WiFi conectado. IP: " IPSTR, IP2STR(&ev->ip_info.ip));
    }
}

// Arranca el WiFi y vuelve enseguida (no espera a conectarse).
esp_err_t red_wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_evento, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_evento, NULL));

    wifi_config_t wifi_config = { 0 };
    strlcpy((char *)wifi_config.sta.ssid, CONFIG_ESP_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, CONFIG_ESP_WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Sin ahorro de energia: si no, la radio duerme ~300 ms entre escuchas,
    // los /cmd_vel llegan amontonados, salta el failsafe y se "pierde" el agente
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    return ESP_OK;
}

#ifdef CONFIG_PROMETEO_AGENTE_AUTO
// Prueba el puerto del agente en las IP .host_ini ... .host_fin de la red /24 de la ESP32.
// Si alguna responde, la deja en ip_out y devuelve true.
static bool probar_hosts(rmw_init_options_t *rmw_options, char *ip_out, size_t ip_len,
                         uint32_t red, uint32_t propia, uint32_t gateway,
                         uint32_t host_ini, uint32_t host_fin, int espera_ms)
{
    for (uint32_t host = host_ini; host <= host_fin && host < 255; host++) {
        uint32_t ip = red | host;
        if (ip == propia || ip == gateway) {   // ni yo mismo ni la camara
            continue;
        }
        char candidata[16];
        snprintf(candidata, sizeof(candidata), "%u.%u.%u.%u",
                 (unsigned)(ip >> 24), (unsigned)((ip >> 16) & 255),
                 (unsigned)((ip >> 8) & 255), (unsigned)(ip & 255));
        rmw_uros_options_set_udp_address(candidata, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
        if (rmw_uros_ping_agent_options(espera_ms, 1, rmw_options) == RMW_RET_OK) {
            strlcpy(ip_out, candidata, ip_len);
            return true;
        }
    }
    return false;
}

// Busca el agente en la red. El DHCP de la camara reparte las IP en orden desde .2 (tiene
// pocos clientes), asi que el agente casi siempre esta en las primeras: se prueban esas
// con una espera larga (la compu puede tardar en contestar el ARP si ahorra energia).
// La barrida completa de la red (.2 a .254) es lenta, por eso solo se hace cada tanto.
bool red_buscar_agente(rmw_init_options_t *rmw_options, char *ip_out, size_t ip_len)
{
    static unsigned ciclo = 0;

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info;
    if (netif == NULL || esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == 0) {
        return false;
    }

    uint32_t propia = lwip_ntohl(info.ip.addr);
    uint32_t red = propia & 0xFFFFFF00u;
    uint32_t gateway = lwip_ntohl(info.gw.addr);

    uint32_t rapidos_fin = 1 + CONFIG_PROMETEO_AGENTE_HOSTS_RAPIDOS;   // .2 ... .(1+N)
    if (probar_hosts(rmw_options, ip_out, ip_len, red, propia, gateway, 2, rapidos_fin, 3 * CONFIG_PROMETEO_AGENTE_PING_MS)) {
        return true;
    }

    if (++ciclo % CONFIG_PROMETEO_AGENTE_BARRIDO_CADA == 0) {
        ESP_LOGW(TAG, "No esta en las primeras IP: barrida completa de la red...");
        return probar_hosts(rmw_options, ip_out, ip_len, red, propia, gateway, rapidos_fin + 1, 254, CONFIG_PROMETEO_AGENTE_PING_MS);
    }
    return false;
}
#endif
