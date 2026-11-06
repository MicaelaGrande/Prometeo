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

#define VEL_MAX_MPS   (CONFIG_PROMETEO_VEL_MAX_MM_S / 1000.0f)

// ===== LAZO DE VELOCIDAD POR RUEDA (encoders) =====
// Cada motor tiene su propia velocidad pedida (m/s) y su propio encoder. El lazo mide la
// rueda y suma o resta duty a ESE motor hasta que gire a la velocidad pedida: asi las 4
// ruedas giran igual aunque los motores, la carga o la friccion sean distintos.
// Solo con la rampa ya terminada. El duty pedido (a ciegas, con VEL_MAX) es el punto de
// partida; el refuerzo puede ser positivo (rueda lenta) o negativo (rueda rapida).
// La velocidad se mide sobre una ventana larga: a 100 ms un solo pulso son ~0,1 m/s, mas que
// la velocidad de un giro en el lugar. Con motor_set()/motores_set() a mano el lazo queda apagado.
#ifdef CONFIG_PROMETEO_MOTORES_SPEED_LOOP
#define LAZO_VENTANA_MS   CONFIG_PROMETEO_MOTORES_SPEED_WINDOW_MS
#define LAZO_KP           (CONFIG_PROMETEO_MOTORES_SPEED_KP_X100 / 100.0f)   // duty por (error / VEL_MAX)
#define LAZO_KI           (CONFIG_PROMETEO_MOTORES_SPEED_KI_X100 / 100.0f)   // duty por segundo por (error / VEL_MAX)
#define LAZO_KD           (CONFIG_PROMETEO_MOTORES_SPEED_KD_X100 / 100.0f)   // duty*segundo por (error / VEL_MAX)
#define LAZO_SUBE_MAX     (CONFIG_PROMETEO_MOTORES_SPEED_BOOST_MAX_PCT / 100.0f)
#define LAZO_BAJA_MAX     (CONFIG_PROMETEO_MOTORES_SPEED_REDUCE_MAX_PCT / 100.0f)
#define LAZO_TOL_RAMPA    0.03f    // la rampa "llego" si falta menos que esto
#define LAZO_RUEDA_M      (3.14159265f * DIAMETRO_RUEDA_M)

static volatile float vel_obj[CANT_MOTORES] = { -1.0f, -1.0f, -1.0f, -1.0f };   // |m/s| pedido; < 0 = lazo apagado
static float    refuerzo[CANT_MOTORES];     // termino integral: duty extra (+) o de menos (-), en la direccion del objetivo
static float    pd_duty[CANT_MOTORES];      // terminos P + D de la ultima ventana (se mantienen hasta la siguiente)
static float    err_prev[CANT_MOTORES];     // error de la ventana anterior (para D)
static bool     primera_ventana[CANT_MOTORES] = { true, true, true, true };
static int      lazo_signo[CANT_MOTORES];   // signo del objetivo con que se midio
static uint32_t lazo_acum_ms[CANT_MOTORES];
static uint32_t lazo_pulsos[CANT_MOTORES];
static uint32_t lazo_prev[CANT_MOTORES];

static inline float limitar_refuerzo(float x)
{
    return x > LAZO_SUBE_MAX ? LAZO_SUBE_MAX : (x < -LAZO_BAJA_MAX ? -LAZO_BAJA_MAX : x);
}

static inline void lazo_reiniciar_ventana(int m)
{
    lazo_acum_ms[m] = 0;
    lazo_pulsos[m] = 0;
}

// Duty que realmente se manda a un motor: el pedido mas su refuerzo
static float meta_motor(motor_id_t m)
{
    float o = objetivo[m];
    if (vel_obj[m] < 0.0f || (o > -0.001f && o < 0.001f)) return o;
    float corr = limitar_refuerzo(refuerzo[m] + pd_duty[m]);    // PID completo
    float mag = ((o > 0) ? o : -o) + corr;
    if (mag < 0.01f) mag = 0.01f;          // nunca cambia de sentido (la zona muerta lo sube al minimo)
    if (mag > 1.0f)  mag = 1.0f;
    return (o > 0) ? mag : -mag;
}

static void lazo_velocidad(void)
{
    for (int m = 0; m < CANT_MOTORES; m++) {
        uint32_t pul = encoder_get_pulsos(ENC_DE[m]);
        uint32_t dp = pul - lazo_prev[m];
        lazo_prev[m] = pul;

        float o = objetivo[m];
        int signo = (o > 0.001f) ? 1 : (o < -0.001f ? -1 : 0);
        if (en_falla || vel_obj[m] <= 0.0f || signo == 0 || signo != lazo_signo[m]) {
            refuerzo[m] = 0.0f;            // lazo apagado, rueda parada o cambio de sentido: de cero
            pd_duty[m] = 0.0f;
            err_prev[m] = 0.0f;
            primera_ventana[m] = true;
            lazo_signo[m] = signo;
            lazo_reiniciar_ventana(m);
            continue;
        }

        float falta = meta_motor((motor_id_t)m) - actual[m];
        if (falta > LAZO_TOL_RAMPA || falta < -LAZO_TOL_RAMPA) {   // todavia acelerando: no se mide
            lazo_reiniciar_ventana(m);
            continue;
        }

        lazo_acum_ms[m] += TICK_MS;
        lazo_pulsos[m] += dp;
        if (lazo_acum_ms[m] >= LAZO_VENTANA_MS) {
            float seg = lazo_acum_ms[m] / 1000.0f;
            float medida = (float)lazo_pulsos[m] / PPR * LAZO_RUEDA_M / seg;   // m/s de esta rueda
            float error = (vel_obj[m] - medida) / VEL_MAX_MPS;
            // PID discreto, una vez por ventana. I con tope (anti-windup); P y D se mantienen
            // hasta la proxima medicion. En la primera ventana no hay error previo: D = 0.
            refuerzo[m] = limitar_refuerzo(refuerzo[m] + LAZO_KI * error * seg);
            float deriv = primera_ventana[m] ? 0.0f : (error - err_prev[m]) / seg;
            primera_ventana[m] = false;
            err_prev[m] = error;
            pd_duty[m] = LAZO_KP * error + LAZO_KD * deriv;
            ESP_LOGD(TAG, "lazo %c: pedida %.3f medida %.3f m/s I %+.2f PD %+.2f", 'A' + m,
                     (double)vel_obj[m], (double)medida, (double)refuerzo[m], (double)pd_duty[m]);
            lazo_reiniciar_ventana(m);
        }
    }
}
#endif

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
#ifdef CONFIG_PROMETEO_MOTORES_SPEED_LOOP
            lazo_velocidad();
#endif
            uint32_t bloq = motores_bloqueados();
            for (int m = 0; m < CANT_MOTORES; m++) {
                if (bloq & (1u << m)) {                // falla latcheada: sigue frenado aunque haya ordenes
                    frenar_motor(m);
                    continue;
                }
#ifdef CONFIG_PROMETEO_MOTORES_SPEED_LOOP
                float d = meta_motor((motor_id_t)m) - actual[m];
#else
                float d = objetivo[m] - actual[m];
#endif
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
#ifdef CONFIG_PROMETEO_MOTORES_SPEED_LOOP
    vel_obj[m] = -1.0f;   // orden a mano: sin lazo (motores_set_ruedas lo vuelve a prender)
#endif
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
#define ANCHO_EFECT_M (CONFIG_PROMETEO_WHEEL_SEPARATION_MM / 1000.0f * CONFIG_PROMETEO_TURN_FACTOR_PCT / 100.0f)

// Velocidad de cada rueda (m/s, con signo) para un movimiento del robot.
// Siempre las 4 ruedas; las de un lado piden lo mismo. Al girar (w != 0) el giro se refuerza
// PROMETEO_TURN_BOOST_PCT % (mas duty, mas giro) para vencer el patinaje.
// Si un lado se pasa de VEL_MAX se escala todo: se conserva el radio de giro.
#define GIRO_MIN_RADS 0.01f   // por debajo de esto se considera recto
#define GIRO_DUTY_MIN (CONFIG_PROMETEO_TURN_MIN_DUTY_PCT / 100.0f)
void motores_twist_a_ruedas(float v_mps, float w_rads, float v_rueda_mps[CANT_MOTORES])
{
    bool girando = fabsf(w_rads) > GIRO_MIN_RADS;
    if (girando) w_rads *= 1.0f + CONFIG_PROMETEO_TURN_BOOST_PCT / 100.0f;

    float v_izq = v_mps - w_rads * ANCHO_EFECT_M / 2.0f;
    float v_der = v_mps + w_rads * ANCHO_EFECT_M / 2.0f;

    // Al girar EN EL LUGAR las 4 ruedas patinan: con poco duty no se mueven. Se sube el lado
    // mas rapido hasta PROMETEO_TURN_MIN_DUTY_PCT (y el otro en proporcion: se conserva el
    // radio de giro). Solo si los lados van en sentido contrario: si el robot avanza y gira
    // un poco (por ejemplo una correccion de centrado) no se toca, o saldria disparado.
    float mayor = fmaxf(fabsf(v_izq), fabsf(v_der)) / VEL_MAX_MPS;
    bool en_el_lugar = girando && (v_izq * v_der <= 0.0f);
    if (en_el_lugar && mayor > 0.0f && mayor < GIRO_DUTY_MIN) {
        v_izq *= GIRO_DUTY_MIN / mayor;
        v_der *= GIRO_DUTY_MIN / mayor;
        mayor = GIRO_DUTY_MIN;
    }
    if (mayor > 1.0f) {
        v_izq /= mayor;
        v_der /= mayor;
    }
    for (int i = 0; i < 2; i++) {
        v_rueda_mps[lado_izq[i]] = v_izq;
        v_rueda_mps[lado_der[i]] = v_der;
    }
}

// Pone la velocidad de CADA rueda por separado y alimenta el failsafe. El duty inicial sale de
// VEL_MAX; el lazo de encoders (si esta prendido) lo corrige rueda por rueda.
void motores_set_ruedas(const float v_rueda_mps[CANT_MOTORES])
{
    for (int m = 0; m < CANT_MOTORES; m++) {
        float v = v_rueda_mps[m];
        if (isnan(v)) v = 0.0f;
        float duty = v / VEL_MAX_MPS;
        motor_set((motor_id_t)m, duty);
#ifdef CONFIG_PROMETEO_MOTORES_SPEED_LOOP
        vel_obj[m] = fabsf(objetivo[m]) * VEL_MAX_MPS;   // ya con el limite de -1..1
#endif
    }
    ultima_orden_us = (uint32_t)esp_timer_get_time();
    en_falla = false;
}

void motores_set_twist(float v_mps, float w_rads)
{
    float v[CANT_MOTORES];
    motores_twist_a_ruedas(v_mps, w_rads, v);
    motores_set_ruedas(v);
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
