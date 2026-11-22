#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_manager.h"
#include "wifi_manager.h"
#include "stream_server.h"

static const char *TAG = "PROMETEO";

// ---------------------------------------------------------------------------
// LED rojo chico de la ESP32-CAM AI-Thinker: GPIO33, prende con nivel BAJO.
// Indicador de estado, apagado casi todo el tiempo (consumo minimo). Cada 2 s repite
// una tanda de destellos cortos; la cantidad dice que pasa:
//   1 destello  = todo bien (hotspot arriba y camara entregando video)
//   2 destellos = hotspot arriba, pero la camara no responde (se reintenta sola)
//   3 destellos = el hotspot no esta arriba
// Sin destellos = el firmware no esta corriendo (sin alimentacion o reiniciando).
// ---------------------------------------------------------------------------
#define LED_GPIO        GPIO_NUM_33
#define LED_PULSO_MS    30      // duracion de cada destello
#define LED_PAUSA_MS    170     // pausa entre destellos de una misma tanda
#define LED_PERIODO_MS  2000    // cada cuanto se repite la tanda

static volatile bool s_hotspot_ok = false;
static volatile bool s_camara_ok = false;

static int destellos_segun_estado(void)
{
    if (!s_hotspot_ok) {
        return 3;
    }
    return s_camara_ok ? 1 : 2;
}

static void led_estado_task(void *arg)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);   // apagado

    while (1) {
        int n = destellos_segun_estado();

        for (int i = 0; i < n; i++) {
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(LED_PULSO_MS));
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(LED_PAUSA_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(LED_PERIODO_MS - n * (LED_PULSO_MS + LED_PAUSA_MS)));
    }
}

// ---------------------------------------------------------------------------
// Vigilante de la camara. La camara arranca en una tarea aparte para que el hotspot no
// dependa de ella: si el sensor no responde (por ejemplo con poca tension) se reintenta
// con espera creciente, y si despues se rompe se la reinicia sola.
// ---------------------------------------------------------------------------
#define CAMARA_ESPERA_MIN_MS    1000
#define CAMARA_ESPERA_MAX_MS    10000
#define CAMARA_FALLOS_MAX       3       // capturas fallidas seguidas antes de reiniciarla
#define CAMARA_PRUEBA_CADA_MS   10000   // si nadie mira el video, se la prueba cada tanto

static void camara_task(void *arg)
{
    uint32_t espera_ms = CAMARA_ESPERA_MIN_MS;

    while (1) {
        if (!camera_manager_lista()) {
            if (camera_manager_init() == ESP_OK) {
                s_camara_ok = true;
                espera_ms = CAMARA_ESPERA_MIN_MS;
                ESP_LOGI(TAG, "Camara lista");
            } else {
                s_camara_ok = false;
                ESP_LOGE(TAG, "La camara no responde (intentos fallidos: %u). Reintento en %u ms",
                         camera_manager_inicios_fallidos(), (unsigned)espera_ms);
                vTaskDelay(pdMS_TO_TICKS(espera_ms));
                espera_ms = (espera_ms * 2 > CAMARA_ESPERA_MAX_MS) ? CAMARA_ESPERA_MAX_MS : espera_ms * 2;
                continue;
            }
        }
        else if (camera_manager_inactiva_ms() >= CAMARA_PRUEBA_CADA_MS) {
            camera_manager_probar();
        }

        if (camera_manager_fallos_seguidos() >= CAMARA_FALLOS_MAX) {
            ESP_LOGE(TAG, "La camara dejo de entregar video: se reinicia");
            camera_manager_deinit();
            s_camara_ok = false;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Si algo esencial falla al arrancar no tiene sentido quedarse colgado: se reinicia la placa.
static void reiniciar_en(const char *motivo, int segundos)
{
    ESP_LOGE(TAG, "%s: reiniciando en %d s", motivo, segundos);
    vTaskDelay(pdMS_TO_TICKS(segundos * 1000));
    esp_restart();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando Prometeo ESP32-CAM (ultimo reinicio: %s)",
             stream_server_razon_reinicio());

    if (esp_reset_reason() == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "El ultimo reinicio fue por BAJA TENSION: revisar la alimentacion de la placa");
    }

    xTaskCreate(led_estado_task, "led_estado", 2048, NULL, 1, NULL);

    // 1. Hotspot primero: no depende de la camara
    esp_err_t err = wifi_manager_init();

    if (err == ESP_ERR_INVALID_ARG) {
        // Contrasena invalida en menuconfig: reiniciar no lo arregla (el LED queda en 3)
        ESP_LOGE(TAG, "Configuracion del hotspot invalida: corregirla en menuconfig");
        return;
    } else if (err != ESP_OK) {
        reiniciar_en("Error creando el hotspot Wi-Fi", 3);
    }
    s_hotspot_ok = true;

    // 2. Servidor de video y de estado (la camara puede no estar lista todavia: /stream
    //    contesta 503 hasta que lo este)
    if (stream_server_start() != ESP_OK) {
        reiniciar_en("Error iniciando el servidor de video", 3);
    }

    // 3. La camara arranca (y se vigila) en su propia tarea
    xTaskCreate(camara_task, "camara", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "Hotspot y servidor listos; la camara arranca en segundo plano");
}
