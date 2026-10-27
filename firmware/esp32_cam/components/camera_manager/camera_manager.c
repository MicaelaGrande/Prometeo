
#include "camera_manager.h"
#include "esp_log.h"

static const char *TAG = "CAMERA_MANAGER";

esp_err_t camera_manager_init(void)
{
    camera_config_t config = {
        // Pines ESP32-CAM AI-Thinker
        .pin_pwdn = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sccb_sda = 26,
        .pin_sccb_scl = 27,

        .pin_d0 = 5,
        .pin_d1 = 18,
        .pin_d2 = 19,
        .pin_d3 = 21,
        .pin_d4 = 36,
        .pin_d5 = 39,
        .pin_d6 = 34,
        .pin_d7 = 35,

        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,

        // Configuracion de captura
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "Camara inicializada correctamente");
    return ESP_OK;
}

camera_fb_t *camera_manager_capture(void)
{
    camera_fb_t *frame = esp_camera_fb_get();

    if (frame == NULL) {
        ESP_LOGE(TAG, "Error al capturar imagen");
    }

    return frame;
}

void camera_manager_release(camera_fb_t *frame)
{
    if (frame != NULL) {
        esp_camera_fb_return(frame);
    }
}
