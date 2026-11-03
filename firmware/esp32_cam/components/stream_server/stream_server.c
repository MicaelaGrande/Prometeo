
#include "stream_server.h"
#include "camera_manager.h"

#include <stdio.h>
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "STREAM_SERVER";
static httpd_handle_t server = NULL;

#define STREAM_TYPE \
    "multipart/x-mixed-replace;boundary=frame"

#define STREAM_BOUNDARY "\r\n--frame\r\n"

static esp_err_t stream_handler(httpd_req_t *req)
{
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

    return ESP_OK;
}
