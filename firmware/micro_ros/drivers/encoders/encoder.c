#include "encoder.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "encoder";

#define FILTRO_NS   10000   // pulsos mas cortos que esto son ruido (maximo del hardware ~12,7 us a 80 MHz)

// Pin de cada rueda, en el mismo orden que encoder_id_t
static const gpio_num_t PINES[ENC_CANTIDAD] = {
    [ENC_DEL_IZQ]  = CONFIG_PROMETEO_ENCODER_DEL_IZQ_PIN,
    [ENC_DEL_DER]  = CONFIG_PROMETEO_ENCODER_DEL_DER_PIN,
    [ENC_TRAS_IZQ] = CONFIG_PROMETEO_ENCODER_TRAS_IZQ_PIN,
    [ENC_TRAS_DER] = CONFIG_PROMETEO_ENCODER_TRAS_DER_PIN,
};

static pcnt_unit_handle_t unidad[ENC_CANTIDAD];
static uint32_t cuenta_ref[ENC_CANTIDAD];   // cuenta al principio de la ventana actual
static int64_t  t_ref_us[ENC_CANTIDAD];     // instante del principio de la ventana de cada rueda
static float    rpm[ENC_CANTIDAD];          // ultimo valor medido, SIN filtrar
static float    rpm_f[ENC_CANTIDAD];        // rpm filtradas (pasa-bajos de primer orden)
static int64_t  t_f_us[ENC_CANTIDAD];       // instante de la ultima actualizacion del filtro

void encoders_init(void)
{
    // PCNT no configura las resistencias: entrada con pull-up.
    uint64_t mascara = 0;
    for (int e = 0; e < ENC_CANTIDAD; e++) mascara |= 1ULL << PINES[e];
    gpio_config_t cfg = {
        .pin_bit_mask = mascara,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    for (int e = 0; e < ENC_CANTIDAD; e++) {
        // accum_count: el driver suma solo los desbordes del contador de 16 bits, asi la
        // cuenta crece sin parar y se puede leer sin borrarla (sin carreras)
        pcnt_unit_config_t u = {
            .high_limit = 32767,
            .low_limit  = -32768,
            .flags.accum_count = 1,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&u, &unidad[e]));

        pcnt_glitch_filter_config_t filtro = { .max_glitch_ns = FILTRO_NS };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unidad[e], &filtro));

        pcnt_chan_config_t c = {
            .edge_gpio_num  = PINES[e],
            .level_gpio_num = -1,          // sin senal de nivel: un solo canal
        };
        pcnt_channel_handle_t canal;
        ESP_ERROR_CHECK(pcnt_new_channel(unidad[e], &c, &canal));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(canal, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                            ENC_FLANCOS == 2 ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
                                             : PCNT_CHANNEL_EDGE_ACTION_HOLD));

        ESP_ERROR_CHECK(pcnt_unit_enable(unidad[e]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(unidad[e]));
        ESP_ERROR_CHECK(pcnt_unit_start(unidad[e]));
        cuenta_ref[e] = 0;
        t_ref_us[e] = esp_timer_get_time();
        rpm_f[e] = 0.0f;
        t_f_us[e] = t_ref_us[e];
    }

    ESP_LOGI(TAG, "Encoders PCNT listos (DEL_IZQ=%d, DEL_DER=%d, TRAS_IZQ=%d, TRAS_DER=%d, %d cuentas/vuelta)",
             PINES[ENC_DEL_IZQ], PINES[ENC_DEL_DER], PINES[ENC_TRAS_IZQ], PINES[ENC_TRAS_DER],
             ENC_CUENTAS_VUELTA);
}

// Pasa-bajos de primer orden sobre rpm[e]. Las ventanas se cierran a intervalos irregulares, asi que
// el coeficiente depende del tiempo transcurrido: a = dt / (tau + dt). Con tau = 0 no filtra.
static void filtrar(int e, int64_t ahora)
{
    int64_t dt = ahora - t_f_us[e];
    t_f_us[e] = ahora;
    if (ENC_FILTRO_TAU_MS <= 0 || dt <= 0) {
        rpm_f[e] = rpm[e];
        return;
    }
    const float tau_us = (float)ENC_FILTRO_TAU_MS * 1000.0f;
    const float a = (float)dt / (tau_us + (float)dt);
    rpm_f[e] += a * (rpm[e] - rpm_f[e]);
}

// Cierra, si corresponde, la ventana de medicion de UNA rueda (lee un solo registro)
static void actualizar(int e)
{
    int64_t ahora = esp_timer_get_time();
    int64_t dt = ahora - t_ref_us[e];
    if (dt < (int64_t)ENC_VENTANA_MIN_MS * 1000) return;     // pedido muy seguido: vale el ultimo

    int cuenta;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unidad[e], &cuenta));
    uint32_t delta = (uint32_t)cuenta - cuenta_ref[e];
    const float por_cuenta = 60.0e6f / ((float)ENC_CUENTAS_VUELTA * (float)dt);   // rpm por cada cuenta en dt

    if (delta > 0) {                         // hubo cuentas: medida normal
        rpm[e] = (float)delta * por_cuenta;
        cuenta_ref[e] = (uint32_t)cuenta;
        t_ref_us[e] = ahora;
        filtrar(e, ahora);
    } else if (dt >= (int64_t)ENC_TIMEOUT_MS * 1000) {
        rpm[e] = 0.0f;                       // parada: tambien el filtrado, sin cola lenta
        rpm_f[e] = 0.0f;
        t_f_us[e] = ahora;
        t_ref_us[e] = ahora;
    } else if (por_cuenta < rpm[e]) {
        // Menos de 1 cuenta en dt: no se sabe cuanto, pero es menos que esto. La ventana sigue
        // abierta (crece), asi que la cota baja sola y la medida a baja velocidad sale precisa.
        rpm[e] = por_cuenta;
        filtrar(e, ahora);
    }
}

float encoder_get_rpm_raw(encoder_id_t e)
{
    if (e >= ENC_CANTIDAD) return 0.0f;
    actualizar(e);
    return rpm[e];
}

float encoder_get_rpm(encoder_id_t e)
{
    if (e >= ENC_CANTIDAD) return 0.0f;
    actualizar(e);
    return rpm_f[e];
}
