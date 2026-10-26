#include "hcsr04.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "hcsr04";

/* ---- Parametros del RMT ---- */
#define RMT_RESOLUTION_HZ (1 * 1000 * 1000) /* 1 MHz -> 1 tick = 1 us */
#define TRIGGER_TICKS 10                    /* pulso de TRIG: 10 ticks = 10 us (datasheet: minimo 10 us) */
#define RX_BUFFER_SYMBOLS 64                /* minimo de memoria por canal en el ESP32 clasico */
#define ECHO_FILTER_NS 1250                 /* ignora pulsos mas cortos que esto (ruido). Limite del hardware ~3,1 us */
#define ECHO_IDLE_MAX_NS (25 * 1000 * 1000) /* 25 ms: si la senal no cambia en este tiempo, se da por terminada */

/* ---- Parametros del sensor (datasheet) ---- */
#define HCSR04_MIN_CYCLE_US 60000 /* >= 60 ms entre disparos */
#define HCSR04_US_PER_CM 58.0f    /* distancia(cm) = tiempo_eco(us) / 58 */
#define ECHO_MIN_US 116           /* 2 cm   * 58 */
#define ECHO_MAX_US 23200         /* 400 cm * 58 */

struct hcsr04_t
{
    rmt_channel_handle_t tx_chan;
    rmt_channel_handle_t rx_chan;
    rmt_encoder_handle_t copy_encoder;
    QueueHandle_t rx_queue;
    rmt_receive_config_t rx_config;
    rmt_symbol_word_t rx_buffer[RX_BUFFER_SYMBOLS];
};

/* Se ejecuta (en interrupcion) cuando el RX termino de recibir el eco.
 * Solo reenvia el resultado a una cola; el procesamiento se hace en la tarea. */
static bool IRAM_ATTR on_recv_done(rmt_channel_handle_t channel,
                                   const rmt_rx_done_event_data_t *edata,
                                   void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    xQueueSendFromISR(queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

esp_err_t hcsr04_new(const hcsr04_config_t *config, hcsr04_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config && out_handle, ESP_ERR_INVALID_ARG, TAG, "argumento nulo");

    esp_err_t ret = ESP_OK;
    struct hcsr04_t *h = calloc(1, sizeof(struct hcsr04_t));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "sin memoria");

    h->rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    ESP_GOTO_ON_FALSE(h->rx_queue, ESP_ERR_NO_MEM, err, TAG, "no se pudo crear la cola");

    /* ---- Canal TX -> TRIG ---- */
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,     // selecciona el reloj, que utilizan varios canales
        .gpio_num = config->trig_gpio,      // a que pin se conecta en la esp32
        .mem_block_symbols = 64,            // el minimo de memoria si dma no esta activado
        .resolution_hz = RMT_RESOLUTION_HZ, // resolucion de contador de icks interno
        .trans_queue_depth = 1,             // cuantas transacciones pueden esperar en la cola
        .intr_priority = 0,                  // configura la prioridad
        .flags.allow_pd = false};           // el periferico no se apaga en light sleep solo importa si usas ese modo
    ESP_GOTO_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &h->tx_chan), err, TAG, "fallo canal TX");

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_cfg, &h->copy_encoder), err, TAG, "fallo encoder");

    /* ---- Canal RX <- ECHO (mismo clk_src que el TX) ---- */
    rmt_rx_channel_config_t rx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->echo_gpio,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .flags.invert_in = false, // do not invert input signal
        .flags.with_dma = false,
    };
    ESP_GOTO_ON_ERROR(rmt_new_rx_channel(&rx_cfg, &h->rx_chan), err, TAG, "fallo canal RX");

    rmt_rx_event_callbacks_t cbs = {.on_recv_done = on_recv_done};
    ESP_GOTO_ON_ERROR(rmt_rx_register_event_callbacks(h->rx_chan, &cbs, h->rx_queue),
                      err, TAG, "fallo callback RX");

    h->rx_config.signal_range_min_ns = ECHO_FILTER_NS;
    h->rx_config.signal_range_max_ns = ECHO_IDLE_MAX_NS;

    ESP_GOTO_ON_ERROR(rmt_enable(h->tx_chan), err, TAG, "fallo enable TX");
    ESP_GOTO_ON_ERROR(rmt_enable(h->rx_chan), err, TAG, "fallo enable RX");

    *out_handle = h;
    return ESP_OK;

err:
    hcsr04_del(h);
    return ret;
}

/* Ultimo disparo de CUALQUIER sensor: los 60 ms se respetan entre turnos de
 * todos los sensores, para que los rebotes de un pulso no lleguen al siguiente. */
static int64_t s_last_trigger_us = INT64_MIN / 2;

/* Arma el receptor y manda el pulso de TRIG. No espera el eco. */
static esp_err_t start(hcsr04_handle_t h)
{
    /* Armar el receptor ANTES de disparar (el eco llega pocos cientos de us despues) */
    xQueueReset(h->rx_queue);
    ESP_RETURN_ON_ERROR(rmt_receive(h->rx_chan, h->rx_buffer, sizeof(h->rx_buffer), &h->rx_config),
                        TAG, "fallo rmt_receive");

    /* Pulso de trigger: 10 us en alto y luego bajo */
    static const rmt_symbol_word_t trigger_symbol = {
        .level0 = 1,
        .duration0 = TRIGGER_TICKS,
        .level1 = 0,
        .duration1 = 1,
    };
    const rmt_transmit_config_t tx_cfg = {.loop_count = 0};
    ESP_RETURN_ON_ERROR(rmt_transmit(h->tx_chan, h->copy_encoder, &trigger_symbol,
                                     sizeof(trigger_symbol), &tx_cfg),
                        TAG, "fallo rmt_transmit");
    return ESP_OK;
}

/* Espera el eco de un sensor ya disparado y lo convierte a cm. */
static esp_err_t finish(hcsr04_handle_t h, float *distance_cm)
{
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(h->tx_chan, 100), TAG, "TX no termino");

    /* Esperar el resultado del RX (eco hasta ~23 ms + 25 ms de silencio de cierre) */
    rmt_rx_done_event_data_t evt;
    if (xQueueReceive(h->rx_queue, &evt, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        /* No hubo ningun flanco: cancelar la recepcion pendiente reiniciando el canal */
        rmt_disable(h->rx_chan);
        rmt_enable(h->rx_chan);
        return ESP_ERR_TIMEOUT;
    }

    /* Buscar el tramo en ALTO: su duracion (en ticks = us) es el ancho del eco */
    uint32_t echo_us = 0;
    for (size_t i = 0; i < evt.num_symbols; i++)
    {
        const rmt_symbol_word_t *s = &evt.received_symbols[i];
        if (s->level0 == 1 && s->duration0 > 0)
        {
            echo_us = s->duration0;
            break;
        }
        if (s->level1 == 1 && s->duration1 > 0)
        {
            echo_us = s->duration1;
            break;
        }
    }

    if (echo_us < ECHO_MIN_US || echo_us > ECHO_MAX_US)
    {
        ESP_LOGD(TAG, "eco fuera de rango: %lu us", (unsigned long)echo_us);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *distance_cm = (float)echo_us / HCSR04_US_PER_CM;
    return ESP_OK;
}

esp_err_t hcsr04_measure_group_cm(const hcsr04_handle_t *hs, size_t n,
                                  float *distance_cm, esp_err_t *errs)
{
    ESP_RETURN_ON_FALSE(hs && n > 0 && distance_cm && errs, ESP_ERR_INVALID_ARG, TAG, "argumento nulo");

    /* 1) Respetar >= 60 ms desde el ultimo disparo de cualquier sensor */
    int64_t elapsed = esp_timer_get_time() - s_last_trigger_us;
    if (elapsed < HCSR04_MIN_CYCLE_US)
    {
        uint32_t wait_ms = (uint32_t)((HCSR04_MIN_CYCLE_US - elapsed + 999) / 1000);
        vTaskDelay(pdMS_TO_TICKS(wait_ms) + 1);
    }

    /* 2) Disparar todos los sensores del grupo casi a la vez (unos pocos us de diferencia) */
    s_last_trigger_us = esp_timer_get_time();
    for (size_t i = 0; i < n; i++)
    {
        errs[i] = hs[i] ? start(hs[i]) : ESP_ERR_INVALID_ARG;
    }

    /* 3) Recoger el eco de cada uno */
    for (size_t i = 0; i < n; i++)
    {
        if (errs[i] == ESP_OK)
        {
            errs[i] = finish(hs[i], &distance_cm[i]);
        }
    }
    return ESP_OK;
}

esp_err_t hcsr04_measure_cm(hcsr04_handle_t h, float *distance_cm)
{
    ESP_RETURN_ON_FALSE(h && distance_cm, ESP_ERR_INVALID_ARG, TAG, "argumento nulo");

    esp_err_t err;
    hcsr04_measure_group_cm(&h, 1, distance_cm, &err);
    return err;
}

esp_err_t hcsr04_del(hcsr04_handle_t h)
{
    if (!h)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (h->rx_chan)
    {
        rmt_disable(h->rx_chan);
        rmt_del_channel(h->rx_chan);
    }
    if (h->tx_chan)
    {
        rmt_disable(h->tx_chan);
        rmt_del_channel(h->tx_chan);
    }
    if (h->copy_encoder)
    {
        rmt_del_encoder(h->copy_encoder);
    }
    if (h->rx_queue)
    {
        vQueueDelete(h->rx_queue);
    }
    free(h);
    return ESP_OK;
}
