
#include "stream_server.h"
#include "camera_manager.h"

#include <stdio.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_system.h"

static const char *TAG = "STREAM_SERVER";
static httpd_handle_t server = NULL;          // /stream (puerto 80)
static httpd_handle_t status_server = NULL;   // /status (puerto 81)

#define STREAM_TYPE \
    "multipart/x-mixed-replace;boundary=frame"

#define STREAM_BOUNDARY "\r\n--frame\r\n"

const char *stream_server_razon_reinicio(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "DESCONOCIDO";
    }
}

// Estado de la placa en JSON: sirve para revisar la camara sin cable serie.
//   curl http://192.168.4.1:81/status
static esp_err_t status_handler(httpd_req_t *req)
{
    wifi_sta_list_t clientes = {0};
    esp_wifi_ap_get_sta_list(&clientes);

    char json[320];
    snprintf(json, sizeof(json),
             "{\"uptime_s\":%lld,\"ultimo_reinicio\":\"%s\",\"camara\":\"%s\","
             "\"camara_inicios_fallidos\":%u,\"camara_fallos_seguidos\":%u,"
             "\"clientes_wifi\":%d,\"heap_libre\":%u}\n",
             (long long)(esp_timer_get_time() / 1000000),
             stream_server_razon_reinicio(),
             camera_manager_lista() ? "ok" : "error",
             camera_manager_inicios_fallidos(),
             camera_manager_fallos_seguidos(),
             clientes.num,
             (unsigned)esp_get_free_heap_size());

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    if (!camera_manager_lista()) {
        // La camara todavia no arranco o se esta reiniciando: el cliente reintenta
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "camara no disponible\n");
    }

    esp_err_t res = httpd_resp_set_type(req, STREAM_TYPE);

    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    ESP_LOGI(TAG, "Cliente conectado al streaming");

    while (1) {
        camera_fb_t *frame = camera_manager_capture();

        if (frame == NULL) {
            ESP_LOGE(TAG, "No se pudo capturar frame");
            res = ESP_FAIL;
            break;
        }

        if (frame->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Frame no es JPEG");
            camera_manager_release(frame);
            res = ESP_FAIL;
            break;
        }

        char header[96];

        int len = snprintf(
            header,
            sizeof(header),
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n",
            (unsigned)frame->len
        );

        if (len < 0 || len >= sizeof(header)) {
            camera_manager_release(frame);
            res = ESP_FAIL;
            break;
        }

        res = httpd_resp_send_chunk(
            req,
            STREAM_BOUNDARY,
            sizeof(STREAM_BOUNDARY) - 1
        );

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(
                req, header, len
            );
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(
                req,
                (const char *)frame->buf,
                frame->len
            );
        }

        camera_manager_release(frame);

        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Cliente desconectado");
            break;
        }
    }

    return res;
}

esp_err_t stream_server_start(void)
{
    if (server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.send_wait_timeout = 5;

    esp_err_t err = httpd_start(&server, &config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando servidor HTTP");
        return err;
    }

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };

    err = httpd_register_uri_handler(server, &stream_uri);

    if (err != ESP_OK) {
        httpd_stop(server);
        server = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Streaming disponible en /stream");

    // /status va en un segundo servidor (puerto 81): el de /stream atiende un pedido a la
    // vez y mientras alguien mira el video no contestaria.
    httpd_config_t cfg_status = HTTPD_DEFAULT_CONFIG();
    cfg_status.server_port = 81;
    cfg_status.ctrl_port = cfg_status.ctrl_port + 1;
    cfg_status.stack_size = 4096;

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&status_server, &cfg_status) == ESP_OK &&
        httpd_register_uri_handler(status_server, &status_uri) == ESP_OK) {
        ESP_LOGI(TAG, "Estado disponible en el puerto 81: /status");
    } else {
        ESP_LOGW(TAG, "No se pudo iniciar /status (el video sigue funcionando)");
    }

    return ESP_OK;
}
