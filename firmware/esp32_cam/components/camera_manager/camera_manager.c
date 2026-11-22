
#include "camera_manager.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "CAMERA_MANAGER";

#define VALIDACION_CUADROS 8   // cuadros que se piden al arrancar hasta ver un JPEG completo

static volatile bool     s_lista = false;
static volatile unsigned s_fallos_seguidos = 0;
static volatile unsigned s_inicios_fallidos = 0;
static volatile int64_t  s_ultima_captura_us = 0;

static bool jpeg_completo(const camera_fb_t *frame)
{
    return frame != NULL &&
           frame->format == PIXFORMAT_JPEG &&
           frame->len >= 4 &&
           frame->buf[0] == 0xFF && frame->buf[1] == 0xD8 &&
           frame->buf[frame->len - 2] == 0xFF && frame->buf[frame->len - 1] == 0xD9;
}

static esp_err_t camera_iniciar(void)
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
        .xclk_freq_hz = 10000000,
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

    // Obtener acceso al sensor OV3660
    sensor_t *sensor = esp_camera_sensor_get();

    if (sensor == NULL) {
        ESP_LOGE(TAG, "No se pudo obtener el sensor");
        esp_camera_deinit();
        return ESP_FAIL;
    }

    // Mantener la orientacion ya validada
    int resultado_vflip = sensor->set_vflip(sensor, 1);
    int resultado_hmirror = sensor->set_hmirror(sensor, 0);

    if (resultado_vflip != 0 || resultado_hmirror != 0) {
        ESP_LOGW(TAG, "No se pudo aplicar completamente la orientacion");
    } else {
        ESP_LOGI(TAG, "Orientacion de camara configurada");
    }

    // Prueba de brillo: +1
    int resultado_brillo = sensor->set_brightness(sensor, 1);

    if (resultado_brillo == 0) {
        ESP_LOGI(TAG, "Brillo configurado en +1");
    } else {
        ESP_LOGW(TAG, "No se pudo configurar el brillo");
    }
    // Prueba de contraste: +1
    int resultado_contraste = sensor->set_contrast(sensor, 1);

    if (resultado_contraste == 0) {
        ESP_LOGI(TAG, "Contraste configurado en +1");
    } else {
        ESP_LOGW(TAG, "No se pudo configurar el contraste");
    }
    // Prueba: balance de blancos automatico
    int resultado_awb = sensor->set_whitebal(sensor, 1);

    if (resultado_awb == 0) {
        ESP_LOGI(TAG, "Balance de blancos automatico activado");
    } else {
        ESP_LOGW(TAG, "No se pudo configurar el balance de blancos");
    }

    ESP_LOGI(TAG, "Camara inicializada correctamente");
    return ESP_OK;
    
}

esp_err_t camera_manager_init(void)
{
    if (s_lista) {
        return ESP_OK;
    }

    esp_err_t err = camera_iniciar();

    if (err == ESP_OK) {
        // Con poca tension el sensor a veces inicia pero entrega cuadros rotos:
        // se exige ver un JPEG completo antes de dar la camara por buena.
        bool ok = false;

        for (int i = 0; i < VALIDACION_CUADROS && !ok; i++) {
            camera_fb_t *frame = esp_camera_fb_get();
            if (frame != NULL) {
                ok = jpeg_completo(frame);
                esp_camera_fb_return(frame);
            }
        }

        if (!ok) {
            ESP_LOGE(TAG, "La camara no entrega un JPEG completo en %d cuadros", VALIDACION_CUADROS);
            esp_camera_deinit();
            err = ESP_FAIL;
        }
    }

    if (err != ESP_OK) {
        s_inicios_fallidos++;
        return err;
    }

    s_fallos_seguidos = 0;
    s_ultima_captura_us = esp_timer_get_time();
    s_lista = true;
    ESP_LOGI(TAG, "Camara validada: entrega JPEG completos");
    return ESP_OK;
}

void camera_manager_deinit(void)
{
    s_lista = false;
    esp_camera_deinit();
}

bool camera_manager_lista(void)
{
    return s_lista;
}

camera_fb_t *camera_manager_capture(void)
{
    if (!s_lista) {
        return NULL;
    }

    camera_fb_t *frame = esp_camera_fb_get();
    s_ultima_captura_us = esp_timer_get_time();

    if (frame == NULL) {
        s_fallos_seguidos++;
        ESP_LOGE(TAG, "Error al capturar imagen (fallos seguidos: %u)", s_fallos_seguidos);
        return NULL;
    }

    s_fallos_seguidos = 0;

    // Diagnostico de integridad JPEG
    if (frame->format == PIXFORMAT_JPEG && !jpeg_completo(frame)) {
        ESP_LOGW(TAG, "JPEG posiblemente incompleto: len=%u", (unsigned)frame->len);
    }

    return frame;
}

void camera_manager_release(camera_fb_t *frame)
{
    if (frame != NULL) {
        esp_camera_fb_return(frame);
    }
}

unsigned camera_manager_fallos_seguidos(void)
{
    return s_fallos_seguidos;
}

bool camera_manager_probar(void)
{
    for (int i = 0; i < 3; i++) {
        camera_fb_t *frame = camera_manager_capture();
        if (frame == NULL) {
            return false;   // camera_manager_capture ya conto la falla
        }
        bool ok = jpeg_completo(frame);
        camera_manager_release(frame);
        if (ok) {
            return true;
        }
    }
    s_fallos_seguidos++;   // salieron cuadros, pero ninguno completo
    return false;
}

int64_t camera_manager_inactiva_ms(void)
{
    return (esp_timer_get_time() - s_ultima_captura_us) / 1000;
}

unsigned camera_manager_inicios_fallidos(void)
{
    return s_inicios_fallidos;
}
