#include "l9110s.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "encoder.h"
#include "sdkconfig.h"

static const char *TAG = "l9110s";

// ===== PWM =====
#define PWM_FREC_HZ      1000
#define PWM_RESOLUCION   LEDC_TIMER_8_BIT
#define PWM_MAX          255
#define PWM_TIMER        LEDC_TIMER_1            // motores.c usa el 0
#define PWM_MODO         LEDC_LOW_SPEED_MODE

// ===== TIEMPOS =====
#define TICK_MS          20
#define TIMEOUT_US       300000u                 // sin ordenes por mas de esto = frena
#define STALL_GRACIA_MS  300                     // tras arrancar no se vigila el atasco

// ===== SEGURIDAD (valores iniciales, menuconfig "Prometeo - Seguridad L9110S") =====
#define STALL_MIN_PPS    ((float)CONFIG_PROMETEO_L9110S_STALL_MIN_PPS)
#ifdef CONFIG_PROMETEO_L9110S_STOP_ALL
#define STOP_ALL_DEF     true
#else
#define STOP_ALL_DEF     false
#endif
#ifdef CONFIG_PROMETEO_L9110S_EFFORT_ENABLE
#define EFFORT_ENABLE_DEF true
#else
#define EFFORT_ENABLE_DEF false
#endif
#define EFFORT_LIMIT_DEF (CONFIG_PROMETEO_L9110S_EFFORT_LIMIT_PCT / 100.0f)
#define EFFORT_TAU_DEF   ((float)CONFIG_PROMETEO_L9110S_EFFORT_TAU_S)
#define EFFORT_COOL_FRAC 0.5f                    // l9110s_clear_overload exige esfuerzo <= 50 % del limite

// ===== PINES: {IA, IB} por motor =====
static const int pines[L9110S_N][2] = {
    { CONFIG_PROMETEO_MOTOR_A1_PIN, CONFIG_PROMETEO_MOTOR_A2_PIN },   // FL
    { CONFIG_PROMETEO_MOTOR_B1_PIN, CONFIG_PROMETEO_MOTOR_B2_PIN },   // RL
    { CONFIG_PROMETEO_MOTOR_C1_PIN, CONFIG_PROMETEO_MOTOR_C2_PIN },   // FR
    { CONFIG_PROMETEO_MOTOR_D1_PIN, CONFIG_PROMETEO_MOTOR_D2_PIN },   // RR
};

static const encoder_id_t enc_de[L9110S_N] = {
    ENC_DEL_IZQ, ENC_TRAS_IZQ, ENC_DEL_DER, ENC_TRAS_DER,
};

// Valores iniciales de cada motor (cambiar aca o con l9110s_cfg_set)
static const l9110s_cfg_t cfg_def[L9110S_N] = {
    //  inv    zona  dmax  rampa  trim  dead  stall_ms  stall_duty  min_pps
    { false, 0.40f, 1.0f, 0.05f, 1.0f, 150,  1500,     0.40f,      STALL_MIN_PPS },   // FL
    { false, 0.40f, 1.0f, 0.05f, 1.0f, 150,  1500,     0.40f,      STALL_MIN_PPS },   // RL
    { false, 0.40f, 1.0f, 0.05f, 1.0f, 150,  1500,     0.40f,      STALL_MIN_PPS },   // FR
    { false, 0.40f, 1.0f, 0.05f, 1.0f, 150,  1500,     0.40f,      STALL_MIN_PPS },   // RR
};

typedef struct {
    l9110s_cfg_t cfg;
    float    objetivo;        // pedido (-1..1)
    float    actual;          // tras rampa (-1..1)
    float    aplicado;        // duty firmado escrito al hardware
    int      dead_restante;   // ticks de deadtime que faltan
    bool     invirtiendo;     // bajando a 0 para cambiar de sentido
    uint32_t stall_acum_ms;   // tiempo acumulado de la ventana de medicion de velocidad
    uint32_t stall_pulsos;    // pulsos contados en esa ventana
    uint32_t gracia_ms;
    uint32_t pulsos_prev;
} motor_state_t;

static motor_state_t   st[L9110S_N];
static SemaphoreHandle_t mtx;
static uint32_t        fallas;            // bits STALL y OVERLOAD (latcheados)
static l9110s_safety_t seg = { STOP_ALL_DEF, EFFORT_ENABLE_DEF, EFFORT_LIMIT_DEF, EFFORT_TAU_DEF };
static float           esfuerzo[2];       // esfuerzo medio por puente (0=izq, 1=der)
static bool            falla_timeout = true;
static uint32_t        ultima_orden_us;
static bool            listo;
static bool            iniciando;

// ---------- utilidades ----------
static float lim(float v)
{
    if (!isfinite(v)) return 0.0f;
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

static int sgn(float v) { return (v > 0.0f) - (v < 0.0f); }

static inline uint32_t ahora_us(void) { return (uint32_t)esp_timer_get_time(); }

// Unico lugar que escribe el PWM. f = duty firmado -1..1 (+ = IA, - = IB)
static void escribir(int m, float f)
{
    int duty = (int)lroundf(fabsf(f) * PWM_MAX);
    if (duty > PWM_MAX) duty = PWM_MAX;
    int a = (f > 0.0f) ? duty : 0;
    int b = (f < 0.0f) ? duty : 0;
    ledc_channel_t ca = (ledc_channel_t)(2 * m);
    ledc_channel_t cb = (ledc_channel_t)(2 * m + 1);
    // primero se apaga el lado que se va, despues se prende el otro
    if (a == 0) { ledc_set_duty(PWM_MODO, ca, 0); ledc_update_duty(PWM_MODO, ca); }
    if (b == 0) { ledc_set_duty(PWM_MODO, cb, 0); ledc_update_duty(PWM_MODO, cb); }
    if (a > 0)  { ledc_set_duty(PWM_MODO, ca, a); ledc_update_duty(PWM_MODO, ca); }
    if (b > 0)  { ledc_set_duty(PWM_MODO, cb, b); ledc_update_duty(PWM_MODO, cb); }
}

// trim -> zona muerta -> tope -> inversion
static float dar_forma(const l9110s_cfg_t *c, float a)
{
    float v = a * c->trim;
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    float mag = fabsf(v);
    if (mag < 0.001f) return 0.0f;
    mag = c->zona_muerta + mag * (1.0f - c->zona_muerta);
    if (mag > c->duty_max) mag = c->duty_max;
    float out = copysignf(mag, v);
    return c->invertido ? -out : out;
}

// Frena un motor (mutex tomado)
static void frenar_motor(int m)
{
    motor_state_t *s = &st[m];
    s->objetivo = 0.0f;
    s->actual = 0.0f;
    s->aplicado = 0.0f;
    s->dead_restante = 0;
    s->invirtiendo = false;
    s->stall_acum_ms = 0;
    s->stall_pulsos = 0;
    s->gracia_ms = STALL_GRACIA_MS;
    escribir(m, 0.0f);
}

static bool cfg_valida(const l9110s_cfg_t *c)
{
    if (!c) return false;
    if (!isfinite(c->zona_muerta) || c->zona_muerta < 0.0f || c->zona_muerta > 0.9f) return false;
    if (!isfinite(c->duty_max) || c->duty_max <= c->zona_muerta || c->duty_max > 1.0f) return false;
    if (!isfinite(c->rampa_paso) || c->rampa_paso <= 0.0f || c->rampa_paso > 1.0f) return false;
    if (!isfinite(c->trim) || c->trim < 0.5f || c->trim > 1.5f) return false;
    if (c->deadtime_ms > 2000) return false;
    if (c->stall_ms != 0 && (c->stall_ms < 100 || c->stall_ms > 10000)) return false;
    if (!isfinite(c->stall_duty_min) || c->stall_duty_min < 0.0f || c->stall_duty_min > 1.0f) return false;
    if (!isfinite(c->stall_min_pps) || c->stall_min_pps < 0.0f || c->stall_min_pps > 1000.0f) return false;
    return true;
}

static bool seg_valida(const l9110s_safety_t *s)
{
    if (!s) return false;
    if (!isfinite(s->effort_limit) || s->effort_limit <= 0.0f || s->effort_limit > 2.0f) return false;
    if (!isfinite(s->effort_tau_s) || s->effort_tau_s < 1.0f || s->effort_tau_s > 600.0f) return false;
    return true;
}

// Promedia el esfuerzo de cada puente (P = suma de duty^2 de sus 2 motores) y latchea el sobreesfuerzo.
// Devuelve los bits OVERLOAD recien latcheados. Mutex tomado.
static uint32_t actualizar_esfuerzo(const float P[2])
{
    uint32_t nuevas = 0;
    float a = (float)TICK_MS / (seg.effort_tau_s * 1000.0f);
    for (int b = 0; b < 2; b++) {
        esfuerzo[b] += (P[b] - esfuerzo[b]) * a;
        if (seg.effort_enable && esfuerzo[b] > seg.effort_limit &&
            !(fallas & L9110S_FAULT_OVERLOAD(b))) {
            fallas |= L9110S_FAULT_OVERLOAD(b);
            nuevas |= L9110S_FAULT_OVERLOAD(b);
        }
    }
    return nuevas;
}

// ---------- tarea de control ----------
#define FALLAS_LATCH (L9110S_FAULT_ANY_STALL | L9110S_FAULT_ANY_OVERLOAD)

static void tick(void)
{
    uint32_t nuevas = 0;                              // fallas latcheadas en este tick
    float    P[2] = { 0.0f, 0.0f };                   // suma de duty^2 por puente

    xSemaphoreTake(mtx, portMAX_DELAY);

    // 1) watchdog de ordenes
    if (falla_timeout) {
        actualizar_esfuerzo(P);                       // todo frenado: el esfuerzo se enfria
        xSemaphoreGive(mtx);
        return;                                       // ya frenado
    }
    if ((uint32_t)(ahora_us() - ultima_orden_us) > TIMEOUT_US) {
        for (int m = 0; m < L9110S_N; m++) frenar_motor(m);
        falla_timeout = true;
        xSemaphoreGive(mtx);
        ESP_LOGW(TAG, "timeout de ordenes: frenado");
        return;
    }

    // con stop_all, una falla latcheada mantiene los 4 motores frenados
    bool bloqueo = seg.stop_all && (fallas & FALLAS_LATCH);

    for (int m = 0; m < L9110S_N; m++) {
        motor_state_t *s = &st[m];
        const l9110s_cfg_t *c = &s->cfg;

        uint32_t pul = encoder_get_pulsos(enc_de[m]);
        uint32_t dp = pul - s->pulsos_prev;
        s->pulsos_prev = pul;

        // 2) bloqueo global o motor atascado: queda frenado hasta que se liberen las fallas
        if (bloqueo || (fallas & L9110S_FAULT_STALL(m))) {
            frenar_motor(m);
            continue;
        }

        // 3) deadtime al invertir
        float tgt = s->objetivo;
        int ot = sgn(tgt), oa = sgn(s->actual);
        if (s->dead_restante > 0) {
            s->dead_restante--;
            tgt = 0.0f;
        } else if (ot && oa && ot != oa) {
            tgt = 0.0f;
            s->invirtiendo = true;
        } else if (ot == oa && oa != 0) {
            s->invirtiendo = false;                   // volvio a pedir el mismo sentido
        }

        // 4) rampa
        float d = tgt - s->actual;
        if (fabsf(d) <= c->rampa_paso) s->actual = tgt;
        else s->actual += copysignf(c->rampa_paso, d);

        if (s->invirtiendo && s->actual == 0.0f) {
            s->invirtiendo = false;
            s->dead_restante = (int)((c->deadtime_ms + TICK_MS - 1) / TICK_MS);
        }

        // 5) forma y salida
        float duty = dar_forma(c, s->actual);
        s->aplicado = duty;
        escribir(m, duty);
        P[m / 2] += duty * duty;                      // aporte al esfuerzo del puente

        // 6) atasco: con duty alto el encoder tiene que dar al menos stall_min_pps
        //    (y siempre algun pulso) en cada ventana de stall_ms
        float mag = fabsf(duty);
        if (c->stall_ms == 0 || mag < c->stall_duty_min) {
            s->stall_acum_ms = 0;
            s->stall_pulsos = 0;
            s->gracia_ms = STALL_GRACIA_MS;
        } else if (s->gracia_ms > 0) {
            s->gracia_ms = (s->gracia_ms > TICK_MS) ? s->gracia_ms - TICK_MS : 0;
            s->stall_acum_ms = 0;
            s->stall_pulsos = 0;
        } else {
            s->stall_acum_ms += TICK_MS;
            s->stall_pulsos += dp;
            if (s->stall_acum_ms >= c->stall_ms) {
                float pps = (float)s->stall_pulsos * 1000.0f / (float)s->stall_acum_ms;
                bool lento = (s->stall_pulsos == 0) || (pps < c->stall_min_pps);
                if (lento) {
                    fallas |= L9110S_FAULT_STALL(m);
                    nuevas |= L9110S_FAULT_STALL(m);
                    frenar_motor(m);
                } else {
                    s->stall_acum_ms = 0;
                    s->stall_pulsos = 0;
                }
            }
        }
    }

    // 7) esfuerzo de cada puente (modelo termico, sin sensor de corriente)
    float esf[2];
    nuevas |= actualizar_esfuerzo(P);
    esf[0] = esfuerzo[0]; esf[1] = esfuerzo[1];

    // 8) reaccion: con stop_all se frenan los 4 motores; si no, solo los afectados
    if (nuevas) {
        for (int m = 0; m < L9110S_N; m++) {
            bool afectado = (nuevas & L9110S_FAULT_STALL(m)) ||
                            (nuevas & L9110S_FAULT_OVERLOAD(m / 2));
            if (seg.stop_all || afectado) frenar_motor(m);
        }
    }

    xSemaphoreGive(mtx);

    for (int m = 0; m < L9110S_N; m++) {
        if (nuevas & L9110S_FAULT_STALL(m))
            ESP_LOGE(TAG, "motor %d atascado (menos de %.1f pulsos/s del encoder): %s. "
                          "Revisar encoders_init() y la rueda; l9110s_clear_fault() para liberar",
                     m, (double)st[m].cfg.stall_min_pps, seg.stop_all ? "TODOS los motores frenados" : "motor frenado");
    }
    for (int b = 0; b < 2; b++) {
        if (nuevas & L9110S_FAULT_OVERLOAD(b))
            ESP_LOGE(TAG, "sobreesfuerzo en el puente %s (%.2f > %.2f): %s. "
                          "Esperar que se enfrie y l9110s_clear_overload() para liberar",
                     b ? "derecho" : "izquierdo", (double)esf[b], (double)seg.effort_limit,
                     seg.stop_all ? "TODOS los motores frenados" : "puente frenado");
    }
}

static void control_task(void *arg)
{
    TickType_t t0 = xTaskGetTickCount();
    for (;;) {
        tick();
        vTaskDelayUntil(&t0, pdMS_TO_TICKS(TICK_MS));
    }
}

// ---------- API ----------
esp_err_t l9110s_init(void)
{
    if (listo || iniciando) return ESP_ERR_INVALID_STATE;
    iniciando = true;

    // pines: solo salidas validas y sin repetir
    for (int m = 0; m < L9110S_N; m++) {
        for (int p = 0; p < 2; p++) {
            int pin = pines[m][p];
            if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
                ESP_LOGE(TAG, "motor %d pin %d no sirve como salida", m, pin);
                iniciando = false;
                return ESP_ERR_INVALID_ARG;
            }
            for (int m2 = 0; m2 < L9110S_N; m2++)
                for (int p2 = 0; p2 < 2; p2++)
                    if ((m2 != m || p2 != p) && pines[m2][p2] == pin && (m2 < m || (m2 == m && p2 < p))) {
                        ESP_LOGE(TAG, "pin %d repetido (motor %d y motor %d)", pin, m2, m);
                        iniciando = false;
                        return ESP_ERR_INVALID_ARG;
                    }
        }
    }

    mtx = xSemaphoreCreateMutex();
    if (!mtx) { iniciando = false; return ESP_ERR_NO_MEM; }

    ledc_timer_config_t tcfg = {
        .speed_mode = PWM_MODO,
        .duty_resolution = PWM_RESOLUCION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREC_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t e = ledc_timer_config(&tcfg);
    if (e != ESP_OK) { ESP_LOGE(TAG, "timer LEDC: %s", esp_err_to_name(e)); iniciando = false; return e; }

    for (int m = 0; m < L9110S_N; m++) {
        for (int p = 0; p < 2; p++) {
            ledc_channel_config_t ch = {
                .gpio_num = pines[m][p],
                .speed_mode = PWM_MODO,
                .channel = (ledc_channel_t)(2 * m + p),
                .timer_sel = PWM_TIMER,
                .duty = 0,
                .hpoint = 0,
            };
            e = ledc_channel_config(&ch);
            if (e != ESP_OK) { ESP_LOGE(TAG, "canal LEDC %d: %s", 2 * m + p, esp_err_to_name(e)); iniciando = false; return e; }
        }
        memset(&st[m], 0, sizeof(st[m]));
        st[m].cfg = cfg_def[m];
        st[m].gracia_ms = STALL_GRACIA_MS;
        st[m].pulsos_prev = encoder_get_pulsos(enc_de[m]);
    }

    fallas = 0;
    esfuerzo[0] = esfuerzo[1] = 0.0f;
    falla_timeout = true;                              // frenado hasta la primera orden
    listo = true;

    if (xTaskCreate(control_task, "l9110s", 4096, NULL, 10, NULL) != pdPASS) {
        listo = false;
        iniciando = false;
        return ESP_ERR_NO_MEM;
    }
    iniciando = false;
    return ESP_OK;
}

esp_err_t l9110s_cfg_set(l9110s_motor_t m, const l9110s_cfg_t *cfg)
{
    if (!listo) return ESP_ERR_INVALID_STATE;
    if ((unsigned)m >= L9110S_N || !cfg_valida(cfg)) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(mtx, portMAX_DELAY);
    st[m].cfg = *cfg;
    xSemaphoreGive(mtx);
    return ESP_OK;
}

void l9110s_cfg_get(l9110s_motor_t m, l9110s_cfg_t *cfg)
{
    if (!listo || !cfg || (unsigned)m >= L9110S_N) return;
    xSemaphoreTake(mtx, portMAX_DELAY);
    *cfg = st[m].cfg;
    xSemaphoreGive(mtx);
}

void l9110s_motor_set(l9110s_motor_t m, float v)
{
    if (!listo || (unsigned)m >= L9110S_N) return;
    xSemaphoreTake(mtx, portMAX_DELAY);
    st[m].objetivo = lim(v);
    xSemaphoreGive(mtx);
}

void l9110s_side_set(l9110s_side_t lado, float v)
{
    if (!listo) return;
    float x = lim(v);
    xSemaphoreTake(mtx, portMAX_DELAY);
    if (lado == L9110S_LEFT) { st[L9110S_FL].objetivo = x; st[L9110S_RL].objetivo = x; }
    else                     { st[L9110S_FR].objetivo = x; st[L9110S_RR].objetivo = x; }
    xSemaphoreGive(mtx);
}

void l9110s_set(float izq, float der)
{
    if (!listo) return;
    float i = lim(izq), d = lim(der);
    xSemaphoreTake(mtx, portMAX_DELAY);
    st[L9110S_FL].objetivo = i; st[L9110S_RL].objetivo = i;
    st[L9110S_FR].objetivo = d; st[L9110S_RR].objetivo = d;
    ultima_orden_us = ahora_us();
    falla_timeout = false;
    xSemaphoreGive(mtx);
}

void l9110s_feed(void)
{
    if (!listo) return;
    xSemaphoreTake(mtx, portMAX_DELAY);
    ultima_orden_us = ahora_us();
    falla_timeout = false;
    xSemaphoreGive(mtx);
}

void l9110s_brake_all(void)
{
    if (!listo) return;
    xSemaphoreTake(mtx, portMAX_DELAY);
    for (int m = 0; m < L9110S_N; m++) frenar_motor(m);
    xSemaphoreGive(mtx);
}

uint32_t l9110s_fault_mask(void)
{
    if (!listo) return 0;
    xSemaphoreTake(mtx, portMAX_DELAY);
    uint32_t f = fallas | (falla_timeout ? L9110S_FAULT_TIMEOUT : 0);
    xSemaphoreGive(mtx);
    return f;
}

bool l9110s_clear_fault(l9110s_motor_t m)
{
    if (!listo || (unsigned)m >= L9110S_N) return false;
    xSemaphoreTake(mtx, portMAX_DELAY);
    bool hubo = (fallas & L9110S_FAULT_STALL(m)) != 0;
    fallas &= ~L9110S_FAULT_STALL(m);
    if (hubo) frenar_motor(m);                         // arranca de cero, pide orden nueva
    xSemaphoreGive(mtx);
    return hubo;
}

esp_err_t l9110s_safety_set(const l9110s_safety_t *s)
{
    if (!listo) return ESP_ERR_INVALID_STATE;
    if (!seg_valida(s)) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(mtx, portMAX_DELAY);
    seg = *s;
    xSemaphoreGive(mtx);
    return ESP_OK;
}

void l9110s_safety_get(l9110s_safety_t *s)
{
    if (!listo || !s) return;
    xSemaphoreTake(mtx, portMAX_DELAY);
    *s = seg;
    xSemaphoreGive(mtx);
}

bool l9110s_clear_overload(l9110s_side_t lado)
{
    if (!listo || (unsigned)lado > L9110S_RIGHT) return false;
    xSemaphoreTake(mtx, portMAX_DELAY);
    bool hubo = (fallas & L9110S_FAULT_OVERLOAD(lado)) != 0;
    bool frio = esfuerzo[lado] <= seg.effort_limit * EFFORT_COOL_FRAC;
    bool ok = hubo && frio;
    if (ok) {
        fallas &= ~L9110S_FAULT_OVERLOAD(lado);
        for (int m = 2 * (int)lado; m < 2 * (int)lado + 2; m++) frenar_motor(m);   // arranca de cero, pide orden nueva
    }
    xSemaphoreGive(mtx);
    return ok;
}

float l9110s_get_effort(l9110s_side_t lado)
{
    if (!listo || (unsigned)lado > L9110S_RIGHT) return 0.0f;
    xSemaphoreTake(mtx, portMAX_DELAY);
    float e = esfuerzo[lado];
    xSemaphoreGive(mtx);
    return e;
}

float l9110s_get_duty(l9110s_motor_t m)
{
    if (!listo || (unsigned)m >= L9110S_N) return 0.0f;
    xSemaphoreTake(mtx, portMAX_DELAY);
    float d = st[m].aplicado;
    xSemaphoreGive(mtx);
    return d;
}
