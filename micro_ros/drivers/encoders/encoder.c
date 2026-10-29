#include "encoder.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "encoder";

#define PI_F            3.14159265f
#define ANTIRREBOTE_US  2000   // ignora pulsos a menos de 2 ms del anterior (ruido)

// ===== 1. CONTADORES =====
//Una variable por rueda que va sumando pulsos
static volatile uint32_t cuenta_der = 0;
static volatile uint32_t cuenta_izq = 0;
static volatile int64_t  ultimo_der_us = 0;
static volatile int64_t  ultimo_izq_us = 0;

// Últimos valores calculados
static volatile float rpm_der = 0, rpm_izq = 0;
static volatile float vel_der = 0, vel_izq = 0;

// "Candado" para leer y resetear los contadores sin que una ISR se meta en el medio
static portMUX_TYPE candado = portMUX_INITIALIZER_UNLOCKED;

// ===== 2. INTERRUPCIONES: se ejecutan en cada pulso =====
//Cada vez que el sensor da un pulso, la ESP32 frena lo que está haciendo, 
// suma 1 y vuelve. Es instantáneo y no se pierde ningún pulso, aunque el resto del programa esté ocupado.
static void IRAM_ATTR isr_der(void *arg)
{
    int64_t ahora = esp_timer_get_time();
    if (ahora - ultimo_der_us > ANTIRREBOTE_US) {   // filtro de ruido
        cuenta_der++;
        ultimo_der_us = ahora;
    }
}

static void IRAM_ATTR isr_izq(void *arg)
{
    int64_t ahora = esp_timer_get_time();
    if (ahora - ultimo_izq_us > ANTIRREBOTE_US) {
        cuenta_izq++;
        ultimo_izq_us = ahora;
    }
}

// ===== 3. TIMER: cada PERIOD_MS calcula la velocidad =====
//Lee cuántos pulsos se contaron y vuelve el contador a 0
static void timer_cb(void *arg)
{
    // Leer y poner en 0 los contadores (protegido)
    taskENTER_CRITICAL(&candado);
    uint32_t pd = cuenta_der; cuenta_der = 0;
    uint32_t pi = cuenta_izq; cuenta_izq = 0;
    taskEXIT_CRITICAL(&candado);

    float periodo_s = PERIOD_MS / 1000.0f;

    float vueltas_der = (float)pd / PPR;
    float vueltas_izq = (float)pi / PPR;

    rpm_der = vueltas_der * (60.0f / periodo_s);
    rpm_izq = vueltas_izq * (60.0f / periodo_s);

    vel_der = vueltas_der * PI_F * DIAMETRO_RUEDA_M / periodo_s;
    vel_izq = vueltas_izq * PI_F * DIAMETRO_RUEDA_M / periodo_s;
}

// ===== INICIALIZACIÓN =====
//Devuelven el último valor calculado
void encoders_init(void)
{
    // Pines como entrada, interrupción en flanco de subida
    // (34 y 35 no tienen pull-up interno: lo trae la plaquita del sensor)
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_ENC_DER) | (1ULL << PIN_ENC_IZQ),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
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

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENC_DER, isr_der, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENC_IZQ, isr_izq, NULL));

    // Timer periódico
    esp_timer_handle_t timer;
    const esp_timer_create_args_t args = {
        .callback = timer_cb,
        .name     = "encoder_vel",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, PERIOD_MS * 1000ULL));

    ESP_LOGI(TAG, "Encoders listos (DER=%d, IZQ=%d, PPR=%d, periodo=%d ms)",
             PIN_ENC_DER, PIN_ENC_IZQ, PPR, PERIOD_MS);
}

// ===== 4. FUNCIONES DEL MENÚ =====
float encoder_get_rpm_der(void) { return rpm_der; }
float encoder_get_rpm_izq(void) { return rpm_izq; }
float encoder_get_vel_der(void) { return vel_der; }
float encoder_get_vel_izq(void) { return vel_izq; }