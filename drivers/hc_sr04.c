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
#define RMT_RESOLUTION_HZ   (1 * 1000 * 1000)  /* esto define la resolucion de rmt por lo tanto el valor de los ticks 1 MHz -> 1 tick = 1 us */
#define TRIGGER_TICKS       10                 /* pulso de TRIG: 10 ticks = 10 us (datasheet: minimo 10 us) */
#define RX_BUFFER_SYMBOLS   64                 /* minimo de memoria por canal en el ESP32 clasico */
#define ECHO_FILTER_NS      1250               /* ignora pulsos mas cortos que esto (ruido). Limite del hardware ~3,1 us */
#define ECHO_IDLE_MAX_NS    (25 * 1000 * 1000) /* 25 ms: si la senal no cambia en este tiempo, se da por terminada */

/* ---- Parametros del sensor (datasheet) ---- */
#define HCSR04_MIN_CYCLE_US 60000              /* >= 60 ms entre disparos */
#define HCSR04_US_PER_CM    58.0f              /* distancia(cm) = tiempo_eco(us) / 58 */
#define ECHO_MIN_US         116                /* 2 cm   * 58 */
#define ECHO_MAX_US         23200              /* 400 cm * 58 */

struct hcsr04_t {
    rmt_channel_handle_t tx_chan;
    rmt_channel_handle_t rx_chan;
    rmt_encoder_handle_t copy_encoder;
    QueueHandle_t        rx_queue;
    rmt_receive_config_t rx_config;
    rmt_symbol_word_t    rx_buffer[RX_BUFFER_SYMBOLS];
    int64_t              last_trigger_us;
};