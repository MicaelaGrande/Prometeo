#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "motores.h"

static const char *TAG = "motores";

#define PWM_FREC_HZ 1000
#define PWM_MAX     255   // 8 bits

// ===== LÍMITE DE VELOCIDAD =====
#define V_MOTOR       6.0f   // tensión máxima del motor TT
#define V_BATERIA     6.0f   // CAMBIAR si cambian la batería
#define VELOCIDAD_MAX (V_MOTOR / V_BATERIA)

// ===== ZONA MUERTA =====
// PWM mínimo (0..1) para que la rueda empiece a girar. AJUSTAR probando.
#define ZONA_MUERTA   0.40f

// ===== RAMPA =====
#define RAMPA_PASO        0.05f  // cuánto cambia la velocidad en cada ciclo
#define RAMPA_PERIODO_MS  20     // cada cuánto se actualiza (0 -> 1 en ~0,4 s)

// ===== FAILSAFE =====
#define TIMEOUT_MS        300    // sin órdenes por más de esto -> freno

// ===== MOTORES INVERTIDOS =====
//                                           A  B  C  D
static const int invertido[CANT_MOTORES] = { 0, 0, 0, 0 };

// Qué motores van de cada lado (AJUSTAR según el chasis)
static const motor_id_t lado_izq[] = { MOTOR_A, MOTOR_C };
static const motor_id_t lado_der[] = { MOTOR_B, MOTOR_D };

// {pin IA, pin IB} de cada motor
static const int pines[CANT_MOTORES][2] = {
    {MOTOR_A1, MOTOR_A2},
    {MOTOR_B1, MOTOR_B2},
    {MOTOR_C1, MOTOR_C2},
    {MOTOR_D1, MOTOR_D2},
};

static volatile float objetivo[CANT_MOTORES] = {0};
static volatile float actual[CANT_MOTORES]   = {0};

static volatile int64_t ultima_orden_us = 0;
static volatile bool en_falla = true;   // arranca frenado hasta la primera orden

static void escribir(int canal, int valor)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)canal, valor);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)canal);
}

static void aplicar(motor_id_t m, float v)
{
    if (invertido[m]) v = -v;

    float mag = (v < 0) ? -v : v;   // tamaño de la velocidad, sin signo

    // Zona muerta: si no es 0, llevarlo al rango ZONA_MUERTA..1
    if (mag > 0.001f) {
        mag = ZONA_MUERTA + mag * (1.0f - ZONA_MUERTA);
    } else {
        mag = 0;
    }

    int duty = (int)(mag * VELOCIDAD_MAX * PWM_MAX);

    if (v > 0 && duty > 0) {        // adelante: IA = PWM, IB = 0
        escribir(2 * m, duty);
        escribir(2 * m + 1, 0);
    } else if (v < 0 && duty > 0) { // atrás: IA = 0, IB = PWM
        escribir(2 * m, 0);
        escribir(2 * m + 1, duty);
    } else {                        // freno: IA = 0, IB = 0
        escribir(2 * m, 0);
        escribir(2 * m + 1, 0);
    }
}

static void rampa_task(void *arg)
{
    TickType_t t = xTaskGetTickCount();
    while (1) {
        // ---- FAILSAFE ----
        int64_t ahora = esp_timer_get_time();
        if (!en_falla && (ahora - ultima_orden_us) > (int64_t)TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "Sin ordenes por %d ms -> FRENO DE EMERGENCIA", TIMEOUT_MS);
            motores_frenar();
            en_falla = true;
        }

        // ---- RAMPA ----
        if (!en_falla) {
            for (int m = 0; m < CANT_MOTORES; m++) {
                float d = objetivo[m] - actual[m];
                if (d >  RAMPA_PASO) d =  RAMPA_PASO;
                if (d < -RAMPA_PASO) d = -RAMPA_PASO;
                actual[m] += d;
                if (actual[m] > -0.001f && actual[m] < 0.001f) actual[m] = 0;
                aplicar((motor_id_t)m, actual[m]);
            }
        }
        vTaskDelayUntil(&t, pdMS_TO_TICKS(RAMPA_PERIODO_MS));
    }
}

void motores_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREC_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    for (int i = 0; i < CANT_MOTORES * 2; i++) {
        ledc_channel_config_t canal = {
            .gpio_num = pines[i / 2][i % 2],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&canal);
    }
    motores_frenar();
    en_falla = true;

    xTaskCreate(rampa_task, "rampa", 3072, NULL, 10, NULL);
}

void motor_set(motor_id_t m, float v)
{
    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    objetivo[m] = v;
}

void motores_set(float izq, float der)
{
    for (int i = 0; i < 2; i++) {
        motor_set(lado_izq[i], izq);
        motor_set(lado_der[i], der);
    }
    ultima_orden_us = esp_timer_get_time();   // "llegó una orden fresca"
    en_falla = false;                         // vuelve a arrancar desde 0 con rampa
}

void motores_frenar(void)
{
    for (int m = 0; m < CANT_MOTORES; m++) {
        objetivo[m] = 0;
        actual[m] = 0;
        aplicar((motor_id_t)m, 0);
    }
}

bool motores_en_falla(void) { return en_falla; }
