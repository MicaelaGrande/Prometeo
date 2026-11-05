#include "encoder.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

static const char *TAG = "encoder";

#define PI_F            3.14159265f
#define ANTIRREBOTE_US  2000   // ignora pulsos a menos de 2 ms del anterior (ruido)

// Pin de cada rueda, en el mismo orden que encoder_id_t
static const gpio_num_t PINES[ENC_CANTIDAD] = {
    [ENC_DEL_IZQ]  = CONFIG_PROMETEO_ENCODER_DEL_IZQ_PIN,
    [ENC_DEL_DER]  = CONFIG_PROMETEO_ENCODER_DEL_DER_PIN,
    [ENC_TRAS_IZQ] = CONFIG_PROMETEO_ENCODER_TRAS_IZQ_PIN,
    [ENC_TRAS_DER] = CONFIG_PROMETEO_ENCODER_TRAS_DER_PIN,
};

// ===== 1. CONTADORES =====
// Una posición por rueda que va sumando pulsos
static volatile uint32_t cuenta[ENC_CANTIDAD];
static volatile int64_t  ultimo_us[ENC_CANTIDAD];
// Tiempo entre los dos últimos pulsos válidos (0 = todavía no hay medición)
static volatile int64_t  periodo_us[ENC_CANTIDAD];
// Pulsos desde que arrancó (nunca se pone en 0): para ver si una rueda se movió
static volatile uint32_t total[ENC_CANTIDAD];

// Últimos valores calculados
static volatile float rpm[ENC_CANTIDAD];
static volatile float vel[ENC_CANTIDAD];

// "Candado" para leer y resetear los contadores sin que una ISR se meta en el medio
static portMUX_TYPE candado = portMUX_INITIALIZER_UNLOCKED;

// ===== 2. INTERRUPCIÓN: se ejecuta en cada pulso =====
// Cada vez que un sensor da un pulso, la ESP32 frena lo que está haciendo,
// suma 1 y vuelve. Es instantáneo y no se pierde ningún pulso, aunque el resto
// del programa esté ocupado. La misma función sirve para las 4 ruedas: el
// argumento dice cuál fue.
static void IRAM_ATTR isr_encoder(void *arg)
{
    int e = (int)(intptr_t)arg;
    int64_t ahora = esp_timer_get_time();
    taskENTER_CRITICAL_ISR(&candado);
    int64_t hueco = ahora - ultimo_us[e];
    if (hueco > ANTIRREBOTE_US) {   // filtro de ruido
        cuenta[e]++;
        total[e]++;
        // Si la rueda venía parada, el hueco no es un período real: se descarta
        periodo_us[e] = (hueco > TIMEOUT_PERIODO_US) ? 0 : hueco;
        ultimo_us[e] = ahora;
    }
    taskEXIT_CRITICAL_ISR(&candado);
}

// ===== 3. TIMER: cada PERIOD_MS calcula la velocidad =====
// Lee cuántos pulsos se contaron y vuelve los contadores a 0
static void timer_cb(void *arg)
{
    uint32_t pulsos[ENC_CANTIDAD];

    // Leer y poner en 0 los contadores (protegido)
    taskENTER_CRITICAL(&candado);
    for (int e = 0; e < ENC_CANTIDAD; e++) {
        pulsos[e] = cuenta[e];
        cuenta[e] = 0;
    }
    taskEXIT_CRITICAL(&candado);

    float periodo_s = PERIOD_MS / 1000.0f;

    for (int e = 0; e < ENC_CANTIDAD; e++) {
        float vueltas = (float)pulsos[e] / PPR;
        rpm[e] = vueltas * (60.0f / periodo_s);
        vel[e] = vueltas * PI_F * DIAMETRO_RUEDA_M / periodo_s;
    }
}

// ===== INICIALIZACIÓN =====
void encoders_init(void)
{
    // Pines como entrada, interrupción en flanco de subida.
    // La plaquita del sensor ya trae pull-up; el interno es un refuerzo
    // (se activa recién acá, después del arranque, así no molesta al GPIO2 de strapping).
    uint64_t mascara = 0;
    for (int e = 0; e < ENC_CANTIDAD; e++) {
        mascara |= (1ULL << PINES[e]);
    }
    gpio_config_t cfg = {
        .pin_bit_mask = mascara,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    // El servicio de interrupciones se instala UNA sola vez en todo el programa.
    // Si otro driver (ej. el ultrasónico) ya lo instaló, no es error.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    for (int e = 0; e < ENC_CANTIDAD; e++) {
        ESP_ERROR_CHECK(gpio_isr_handler_add(PINES[e], isr_encoder, (void *)(intptr_t)e));
    }

    // Timer periódico
    esp_timer_handle_t timer;
    const esp_timer_create_args_t args = {
        .callback = timer_cb,
        .name     = "encoder_vel",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, PERIOD_MS * 1000ULL));

    ESP_LOGI(TAG, "Encoders listos (DEL_IZQ=%d, DEL_DER=%d, TRAS_IZQ=%d, TRAS_DER=%d, PPR=%d, periodo=%d ms)",
             PINES[ENC_DEL_IZQ], PINES[ENC_DEL_DER], PINES[ENC_TRAS_IZQ], PINES[ENC_TRAS_DER],
             PPR, PERIOD_MS);
}

// ===== 4. FUNCIONES DEL MENÚ =====
// Devuelven el último valor calculado
float encoder_get_rpm(encoder_id_t e) { return (e < ENC_CANTIDAD) ? rpm[e] : 0.0f; }
float encoder_get_vel(encoder_id_t e) { return (e < ENC_CANTIDAD) ? vel[e] : 0.0f; }

float encoder_get_rpm_izq(void) { return (rpm[ENC_DEL_IZQ] + rpm[ENC_TRAS_IZQ]) / 2.0f; }
float encoder_get_rpm_der(void) { return (rpm[ENC_DEL_DER] + rpm[ENC_TRAS_DER]) / 2.0f; }
float encoder_get_vel_izq(void) { return (vel[ENC_DEL_IZQ] + vel[ENC_TRAS_IZQ]) / 2.0f; }
float encoder_get_vel_der(void) { return (vel[ENC_DEL_DER] + vel[ENC_TRAS_DER]) / 2.0f; }

// Velocidad por tiempo entre pulsos: sirve a baja velocidad, donde contar pulsos
// en 100 ms no alcanza (a 5 rpm hay 1 pulso cada 0,6 s).
float encoder_get_rpm_lenta(encoder_id_t e)
{
    if (e >= ENC_CANTIDAD) return 0.0f;

    taskENTER_CRITICAL(&candado);
    int64_t periodo = periodo_us[e];
    int64_t ultimo  = ultimo_us[e];
    taskEXIT_CRITICAL(&candado);

    int64_t desde_ultimo = esp_timer_get_time() - ultimo;
    if (periodo <= 0 || desde_ultimo > TIMEOUT_PERIODO_US) return 0.0f;

    // Si ya pasó más tiempo que el último período, la rueda está frenando:
    // se usa ese tiempo para que la lectura baje sola en vez de quedar congelada
    int64_t t = (desde_ultimo > periodo) ? desde_ultimo : periodo;
    return 60.0e6f / ((float)PPR * (float)t);
}

uint32_t encoder_get_pulsos(encoder_id_t e) { return (e < ENC_CANTIDAD) ? total[e] : 0; }
int      encoder_get_pin(encoder_id_t e)    { return (e < ENC_CANTIDAD) ? (int)PINES[e] : -1; }
