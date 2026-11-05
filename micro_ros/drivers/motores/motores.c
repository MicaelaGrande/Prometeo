#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <stdint.h>
#include "motores.h"
#include "encoder.h"
#include "sdkconfig.h"

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

// ===== SEGURIDAD =====
#define TICK_MS            RAMPA_PERIODO_MS
#define STALL_GRACIA_MS    300                       // tras arrancar no se vigila el atasco
#define STALL_DUTY_MIN     (CONFIG_PROMETEO_MOTORES_STALL_DUTY_MIN_PCT / 100.0f)
#define STALL_VENTANA_MS   CONFIG_PROMETEO_MOTORES_STALL_WINDOW_MS
#define STALL_MIN_PPS      ((float)CONFIG_PROMETEO_MOTORES_STALL_MIN_PPS)
#ifdef CONFIG_PROMETEO_MOTORES_EFFORT_LIMIT_PCT
#define EFFORT_LIMITE      (CONFIG_PROMETEO_MOTORES_EFFORT_LIMIT_PCT / 100.0f)
#else
#define EFFORT_LIMITE      2.0f                      // vigilancia desactivada
#endif
#define EFFORT_TAU_MS      (CONFIG_PROMETEO_MOTORES_EFFORT_TAU_S * 1000.0f)
#define EFFORT_ENFRIADO    0.5f                      // motores_liberar_fallas exige esfuerzo <= 50 % del limite

// Encoder de cada motor (mismo orden que en menuconfig: A = delantero izq, B = trasero izq, ...)
static const encoder_id_t ENC_DE[CANT_MOTORES] = {
    ENC_DEL_IZQ, ENC_TRAS_IZQ, ENC_DEL_DER, ENC_TRAS_DER,
};

// ===== MOTORES INVERTIDOS =====
//                                           A  B  C  D
static const int invertido[CANT_MOTORES] = { 0, 0, 0, 0 };

// Qué motores van de cada lado (AJUSTAR según el chasis)
// A = delantero izq, B = trasero izq, C = delantero der, D = trasero der (igual que en menuconfig)
static const motor_id_t lado_izq[] = { MOTOR_A, MOTOR_B };
static const motor_id_t lado_der[] = { MOTOR_C, MOTOR_D };

// {pin IA, pin IB} de cada motor
static const int pines[CANT_MOTORES][2] = {
    {MOTOR_A1, MOTOR_A2},
    {MOTOR_B1, MOTOR_B2},
    {MOTOR_C1, MOTOR_C2},
    {MOTOR_D1, MOTOR_D2},
};

static volatile float objetivo[CANT_MOTORES] = {0};
static volatile float actual[CANT_MOTORES]   = {0};

// Último sentido aplicado a cada motor (+1 adelante, -1 atrás). Los encoders son de un
// canal y no saben para qué lado gira la rueda: lo toman de acá. Con el motor en 0 se
// mantiene el último (la rueda sigue girando por inercia).
static volatile int sentido[CANT_MOTORES] = {1, 1, 1, 1};

// 32 bits: en el ESP32 un int64 se lee en dos pasos y la tarea de rampa podía leer
// un valor a medio escribir (falso failsafe). Se compara con resta sin signo (vale hasta 71 min).
static volatile uint32_t ultima_orden_us = 0;
static volatile bool en_falla = true;   // arranca frenado hasta la primera orden

static void escribir(int canal, int valor)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)canal, valor);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)canal);
}

// Devuelve el duty que quedo aplicado (0..1, sin signo)
static float aplicar(motor_id_t m, float v)
{
    // Sentido para el signo de los encoders (antes de 'invertido': es para dónde va el robot)
    if (v >  0.001f) sentido[m] =  1;
    if (v < -0.001f) sentido[m] = -1;

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
        duty = 0;
    }
    return (float)duty / PWM_MAX;
}

// ===== SEGURIDAD: estado =====
// fallas_seg y esfuerzo los escribe la tarea de rampa y los leen/limpian otras tareas: van con candado
static portMUX_TYPE mux_seg = portMUX_INITIALIZER_UNLOCKED;
static uint32_t fallas_seg;                     // bits MOTORES_FALLA_* (latcheados)
static float    esfuerzo[2];                    // promedio de duty^2 por puente (0 = izq, 1 = der)
// solo la tarea de rampa:
static uint32_t pulsos_prev[CANT_MOTORES];
static uint32_t stall_acum_ms[CANT_MOTORES];    // tiempo de la ventana de medicion
static uint32_t stall_pulsos[CANT_MOTORES];     // pulsos contados en la ventana
static uint32_t gracia_ms[CANT_MOTORES];

// Motores que hay que mantener frenados por una falla latcheada (bit m)
static uint32_t motores_bloqueados(void)
{
    taskENTER_CRITICAL(&mux_seg);
    uint32_t f = fallas_seg;
    taskEXIT_CRITICAL(&mux_seg);
    if (!f) return 0;
#ifdef CONFIG_PROMETEO_MOTORES_STOP_ALL
    return (1u << CANT_MOTORES) - 1;
#else
    uint32_t b = 0;
    for (int m = 0; m < CANT_MOTORES; m++)
        if ((f & MOTORES_FALLA_STALL(m)) || (f & MOTORES_FALLA_SOBRE(m / 2))) b |= 1u << m;
    return b;
#endif
}

static void frenar_motor(int m)
{
    objetivo[m] = 0;
    actual[m] = 0;
    aplicar((motor_id_t)m, 0);
}

// Mira encoders y esfuerzo con el duty que se acaba de aplicar. Latchea fallas y frena.
static void vigilar_seguridad(const float duty[CANT_MOTORES])
{
    uint32_t nuevas = 0;
    float P[2] = { 0.0f, 0.0f };

    for (int m = 0; m < CANT_MOTORES; m++) {
        uint32_t pul = encoder_get_pulsos(ENC_DE[m]);
        uint32_t dp = pul - pulsos_prev[m];
        pulsos_prev[m] = pul;

        P[m / 2] += duty[m] * duty[m];

#ifdef CONFIG_PROMETEO_MOTORES_STALL_ENABLE
        if (duty[m] < STALL_DUTY_MIN) {
            stall_acum_ms[m] = 0;
            stall_pulsos[m] = 0;
            gracia_ms[m] = STALL_GRACIA_MS;
        } else if (gracia_ms[m] > 0) {
            gracia_ms[m] = (gracia_ms[m] > TICK_MS) ? gracia_ms[m] - TICK_MS : 0;
            stall_acum_ms[m] = 0;
            stall_pulsos[m] = 0;
        } else {
            stall_acum_ms[m] += TICK_MS;
            stall_pulsos[m] += dp;
            if (stall_acum_ms[m] >= STALL_VENTANA_MS) {
                float pps = (float)stall_pulsos[m] * 1000.0f / (float)stall_acum_ms[m];
                if (stall_pulsos[m] == 0 || pps < STALL_MIN_PPS) {
                    nuevas |= MOTORES_FALLA_STALL(m);
                    ESP_LOGE(TAG, "motor %c sin velocidad (%.1f pulsos/s < %.1f): FALLA", 'A' + m,
                             (double)pps, (double)STALL_MIN_PPS);
                } else {
                    stall_acum_ms[m] = 0;
                    stall_pulsos[m] = 0;
                }
            }
        }
#else
        (void)dp;
#endif
    }

#ifdef CONFIG_PROMETEO_MOTORES_EFFORT_ENABLE
    const float a = (float)TICK_MS / EFFORT_TAU_MS;
    taskENTER_CRITICAL(&mux_seg);
    for (int b = 0; b < 2; b++) {
        esfuerzo[b] += (P[b] - esfuerzo[b]) * a;
        if (esfuerzo[b] > EFFORT_LIMITE && !(fallas_seg & MOTORES_FALLA_SOBRE(b))) nuevas |= MOTORES_FALLA_SOBRE(b);
    }
    taskEXIT_CRITICAL(&mux_seg);
    for (int b = 0; b < 2; b++)
        if (nuevas & MOTORES_FALLA_SOBRE(b))
            ESP_LOGE(TAG, "sobreesfuerzo en el puente %s (%.2f > %.2f): FALLA",
                     b ? "derecho" : "izquierdo", (double)esfuerzo[b], (double)EFFORT_LIMITE);
#else
    taskENTER_CRITICAL(&mux_seg);
    for (int b = 0; b < 2; b++) esfuerzo[b] += (P[b] - esfuerzo[b]) * ((float)TICK_MS / EFFORT_TAU_MS);
    taskEXIT_CRITICAL(&mux_seg);
#endif

    if (nuevas) {
        taskENTER_CRITICAL(&mux_seg);
        fallas_seg |= nuevas;
        taskEXIT_CRITICAL(&mux_seg);
        uint32_t b = motores_bloqueados();
        for (int m = 0; m < CANT_MOTORES; m++)
            if (b & (1u << m)) frenar_motor(m);
#ifdef CONFIG_PROMETEO_MOTORES_STOP_ALL
        ESP_LOGE(TAG, "TODOS los motores frenados. motores_liberar_fallas() para soltar");
#else
        ESP_LOGE(TAG, "motores afectados frenados (mascara 0x%x). motores_liberar_fallas() para soltar", (unsigned)b);
#endif
    }
}

static void rampa_task(void *arg)
{
    TickType_t t = xTaskGetTickCount();
    while (1) {
        // ---- FAILSAFE ----
        uint32_t ahora = (uint32_t)esp_timer_get_time();
        if (!en_falla && (uint32_t)(ahora - ultima_orden_us) > (uint32_t)TIMEOUT_MS * 1000u) {
            ESP_LOGW(TAG, "Sin ordenes por %d ms -> FRENO DE EMERGENCIA", TIMEOUT_MS);
            motores_frenar();
            en_falla = true;
        }

        // ---- RAMPA ----
        float duty_ef[CANT_MOTORES] = { 0 };           // duty realmente aplicado (para la seguridad)
        if (!en_falla) {
            uint32_t bloq = motores_bloqueados();
            for (int m = 0; m < CANT_MOTORES; m++) {
                if (bloq & (1u << m)) {                // falla latcheada: sigue frenado aunque haya ordenes
                    frenar_motor(m);
                    continue;
                }
                float d = objetivo[m] - actual[m];
                if (d >  RAMPA_PASO) d =  RAMPA_PASO;
                if (d < -RAMPA_PASO) d = -RAMPA_PASO;
                actual[m] += d;
                if (actual[m] > -0.001f && actual[m] < 0.001f) actual[m] = 0;
                duty_ef[m] = aplicar((motor_id_t)m, actual[m]);
            }
        }
        vigilar_seguridad(duty_ef);                    // tambien parado: el esfuerzo se enfria
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
    if (isnan(v)) v = 0.0f;   // un NaN en la rampa no se recupera nunca
    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    objetivo[m] = v;
}

void motores_alimentar(void)
{
    ultima_orden_us = (uint32_t)esp_timer_get_time();
    en_falla = false;
}

void motores_set(float izq, float der)
{
    for (int i = 0; i < 2; i++) {
        motor_set(lado_izq[i], izq);
        motor_set(lado_der[i], der);
    }
    ultima_orden_us = (uint32_t)esp_timer_get_time();   // "llegó una orden fresca"
    en_falla = false;                         // vuelve a arrancar desde 0 con rampa
}

// ===== CINEMÁTICA DIFERENCIAL (4 ruedas, skid-steer) =====
// Las ruedas de cada lado giran juntas. Al girar patinan, por eso el ancho
// efectivo es mayor que el real: se corrige con PROMETEO_TURN_FACTOR_PCT.
#define VEL_MAX_MPS   (CONFIG_PROMETEO_VEL_MAX_MM_S / 1000.0f)
#define ANCHO_EFECT_M (CONFIG_PROMETEO_WHEEL_SEPARATION_MM / 1000.0f * CONFIG_PROMETEO_TURN_FACTOR_PCT / 100.0f)

void motores_set_twist(float v_mps, float w_rads)
{
    // Velocidad de cada lado en m/s
    float v_izq = v_mps - w_rads * ANCHO_EFECT_M / 2.0f;
    float v_der = v_mps + w_rads * ANCHO_EFECT_M / 2.0f;

    // A duty -1..1
    float izq = v_izq / VEL_MAX_MPS;
    float der = v_der / VEL_MAX_MPS;

    // Si un lado se pasa de 1, escalar los dos: se conserva el radio de giro
    float mayor = fmaxf(fabsf(izq), fabsf(der));
    if (mayor > 1.0f) {
        izq /= mayor;
        der /= mayor;
    }
    motores_set(izq, der);
}

void motores_frenar(void)
{
    for (int m = 0; m < CANT_MOTORES; m++) {
        objetivo[m] = 0;
        actual[m] = 0;
        aplicar((motor_id_t)m, 0);
    }
}

// Velocidad estimada a partir del duty que se está aplicando (no hay encoders):
// replica la zona muerta de aplicar() y usa la misma cinemática que motores_set_twist().
static float velocidad_motor_mps(motor_id_t m)
{
    float a = actual[m];
    float mag = (a < 0) ? -a : a;
    if (mag <= 0.001f) return 0.0f;
    float v = (ZONA_MUERTA + mag * (1.0f - ZONA_MUERTA)) * VEL_MAX_MPS;
    // 'invertido' ya está compensado en aplicar(): la rueda gira físicamente según el signo de 'actual'
    if (a < 0) v = -v;
    return v;
}

void motores_get_velocidad(float *v_mps, float *w_rads)
{
    float vi = 0, vd = 0;
    int n = sizeof(lado_izq) / sizeof(lado_izq[0]);
    for (int i = 0; i < n; i++) {
        vi += velocidad_motor_mps(lado_izq[i]);
        vd += velocidad_motor_mps(lado_der[i]);
    }
    vi /= n;
    vd /= n;
    *v_mps  = (vi + vd) / 2.0f;
    *w_rads = (vd - vi) / ANCHO_EFECT_M;
}

bool motores_en_falla(void) { return en_falla; }

uint32_t motores_get_fallas(void)
{
    taskENTER_CRITICAL(&mux_seg);
    uint32_t f = fallas_seg;
    taskEXIT_CRITICAL(&mux_seg);
    return f;
}

bool motores_liberar_fallas(void)
{
    bool todo = true;
    taskENTER_CRITICAL(&mux_seg);
    uint32_t f = fallas_seg & ~MOTORES_FALLA_STALL_TODOS;       // los atascos se sueltan siempre
    for (int b = 0; b < 2; b++) {
        if (!(f & MOTORES_FALLA_SOBRE(b))) continue;
        if (esfuerzo[b] <= EFFORT_LIMITE * EFFORT_ENFRIADO) f &= ~MOTORES_FALLA_SOBRE(b);
        else todo = false;                                      // todavia caliente
    }
    fallas_seg = f;
    taskEXIT_CRITICAL(&mux_seg);
    return todo;
}

float motores_get_esfuerzo(int lado)
{
    if (lado < 0 || lado > 1) return 0.0f;
    taskENTER_CRITICAL(&mux_seg);
    float e = esfuerzo[lado];
    taskEXIT_CRITICAL(&mux_seg);
    return e;
}

int motor_get_sentido(motor_id_t m) { return (m < CANT_MOTORES) ? sentido[m] : 1; }
