/*
 * FIRMWARE DE TEST DEL DRIVER DE MOTORES (reemplaza a main.c cuando PROMETEO_BUILD_MOTOR_TEST=y).
 *
 * Prueba drivers/motores (LEDC + rampa) por micro-ROS: no hace falta puerto serie.
 * El driver solo avanza la rampa UN paso por cada llamada a motores_single_set_ramp(), asi que
 * este firmware llama cada RAMP_PERIOD_MS (20 ms) con el objetivo vigente de cada motor, que es
 * lo que tiene que hacer cualquier usuario real del driver.
 *
 * micro-ROS (namespace prometeo, nodo motor_test):
 *   publica      /prometeo/motores_test/estado      std_msgs/String  (JSON, cada 250 ms)
 *   publica      /prometeo/motores_test/rpm         std_msgs/Float32MultiArray (best effort, 50 ms)
 *                [rpm_med A..D (filtradas, con signo del sentido), duty A..D, modo, alarma, rpm_raw A..D]
 *   publica      /prometeo/motores_test/resultados  std_msgs/String  (JSON, un test cada 200 ms, en ciclo)
 *   se suscribe  /prometeo/motores_test/cmd         std_msgs/String  (comandos de texto)
 *
 * Comandos (X = A|B|C|D|all, duty = -1..1):
 *   set X duty [ms]       manual: motor X (o "all") a ese duty durante ms (def. 2000, máx. 10000)
 *   setall a b c d [ms]   manual: cada motor con SU duty
 *   pausa | parar | stop  frena todo (motores_lock) y queda en pausa
 *   unlock | liberar      motores_unlock() sin mover nada (para ver el estado de falla a mano)
 *   ext on                modo EXT: el lazo de control corre en la PC (nodo motor_test_pi.py)
 *   ext a b c d           (solo en modo EXT) duty de cada motor. Sin duties nuevos en EXT_MS se frena.
 *   ext off               sale del modo EXT y frena
 *   selftest              corre la secuencia de pruebas del driver (levantar las ruedas del piso)
 *   selftest abort        la aborta (también "pausa")
 *
 * Resultados del selftest: topic /resultados (un mensaje por test) y contadores en /estado
 * (t_ok, t_fallo, t_total, fase). resultado: pendiente | ok | fallo | medida (solo mide, sin criterio).
 *   T01-T11  pruebas con criterio (leen el duty REAL de cada pin con motores_get_pwm)
 *   M01-M03  mediciones con los encoders (zona muerta, rpm vs duty, signo)
 *
 * Motores (igual que menuconfig): A delantero izq, B trasero izq, C delantero der, D trasero der
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

#include "motores.h"
#include "encoder.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>

#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5
#define TEST_TASK_STACK      6144
#define TEST_TASK_PRIO       6
#define TICK_MS              RAMP_PERIOD_MS      // el driver rampea un paso por llamada
#define PING_PERIODO_MS      1000
#define ENTIDAD_TIMEOUT_MS   5000    // espera máxima del agente al crear cada entidad (WiFi)
#define ESTADO_PERIODO_MS    250
#define RPM_PERIODO_MS       50      // topic rapido de rpm para el lazo en la PC
#define RES_PERIODO_MS       200     // un test por mensaje, en ciclo
#define EXT_MS               300     // modo EXT: sin duties nuevos en este tiempo -> frena
#define RPM_MSG_LEN          14      // [0..3] rpm filtradas, [4..7] duty, [8] modo, [9] alarma, [10..13] rpm crudas
#define MANUAL_MS_DEF        2000
#define MANUAL_MS_MAX        10000
#define CMD_LEN              64
#define JSON_LEN             1024
#define RES_LEN              320

#define RPM_ARRANQUE         3.0f    // por encima de esto la rueda se considera girando
#define JITTER_MAX_US        15000   // T11: desvio maximo aceptado del periodo del lazo (el tick de FreeRTOS es 10 ms)

static const char *TAG = "test_motores";

static const char NOMBRES[CANT_MOTORES] = { 'A', 'B', 'C', 'D' };
static const char *DESCRIPCION[CANT_MOTORES] = {
    "delantero izquierdo", "trasero izquierdo", "delantero derecho", "trasero derecho",
};
// Encoder de la rueda que mueve cada motor
static const encoder_id_t ENCODER_DE[CANT_MOTORES] = {
    [MOTOR_A] = ENC_DEL_IZQ, [MOTOR_B] = ENC_TRAS_IZQ,
    [MOTOR_C] = ENC_DEL_DER, [MOTOR_D] = ENC_TRAS_DER,
};

// =========================================================
// ESTADO COMPARTIDO (lo escribe la tarea de test, lo lee micro-ROS)
// =========================================================
// Los valores numericos de modo coinciden con motor_test_relay.py / motor_test_pi.py (EXT = 4)
typedef enum { MODO_PAUSA = 0, MODO_SELFTEST = 1, MODO_MANUAL = 2, MODO_EXT = 4 } modo_t;
static const char *MODO_TXT[] = { "pausa", "selftest", "manual", "rpm", "ext" };

typedef struct {
    modo_t   modo;
    bool     falla;                     // motores_get_falla()
    float    pedido[CANT_MOTORES];      // objetivo que se le pasa al driver
    float    duty[CANT_MOTORES];        // motores_get_duty(): lo que aplica el driver (-1..1)
    uint32_t pwm[CANT_MOTORES][2];      // duty real de los pines IA, IB (motores_get_pwm)
    int      sentido[CANT_MOTORES];     // motores_get_sentido()
    float    rpm_med[CANT_MOTORES];     // rpm del encoder FILTRADAS (encoder_get_rpm) con el signo del sentido
    float    rpm_raw[CANT_MOTORES];     // rpm del encoder sin filtrar, con el signo del sentido
    int      ramp_ms;                   // ultima subida 0 -> objetivo medida (T05)
    char     fase[40];                  // que esta haciendo el selftest
} estado_t;

static estado_t estado;
static portMUX_TYPE estado_mux = portMUX_INITIALIZER_UNLOCKED;

// Resultados del selftest
typedef enum { R_PEND, R_OK, R_FALLO, R_MEDIDA } res_t;
static const char *R_TXT[] = { "pendiente", "ok", "fallo", "medida" };
enum { T01, T02, T03, T04, T05, T06, T07, T08, T09, T10, T11, M01, M02, M03, N_TESTS };
typedef struct { const char *id; const char *nombre; res_t r; char det[96]; } test_t;
static test_t tests[N_TESTS] = {
    [T01] = { "T01", "bloqueado: pines en 0 y rechaza ordenes" },
    [T02] = { "T02", "unlock: sin falla, pines en 0" },
    [T03] = { "T03", "pines IA/IB segun el signo, en cada motor" },
    [T04] = { "T04", "nunca IA e IB a la vez, nunca sobre el maximo" },
    [T05] = { "T05", "rampa de subida monotona, pasos <= ACCEL_STEP" },
    [T06] = { "T06", "rampa de bajada monotona, pasos <= DECEL_STEP" },
    [T07] = { "T07", "inversion: un tick en 0, sin saltar de signo" },
    [T08] = { "T08", "get_sentido y get_duty (ultimo sentido al frenar)" },
    [T09] = { "T09", "lock en plena rampa: pines en 0 al instante" },
    [T10] = { "T10", "duty fuera de rango y NaN" },
    [T11] = { "T11", "4 motores a la vez y periodo del lazo" },
    [M01] = { "M01", "zona muerta: menor duty que arranca la rueda" },
    [M02] = { "M02", "rpm vs duty 30/50/70/100 %" },
    [M03] = { "M03", "signo del encoder al frenar (inercia)" },
};

// =========================================================
// COMANDOS (callback de micro-ROS -> cola -> tarea de test)
// =========================================================
typedef enum {
    C_SET, C_SETALL, C_PAUSA, C_UNLOCK, C_EXT_ON, C_EXT, C_SELFTEST, C_ABORT
} cmd_tipo_t;

typedef struct {
    cmd_tipo_t tipo;
    int        motor;     // 0..3, -1 = todos
    float      valor;     // duty (set)
    float      duties[CANT_MOTORES];   // setall / ext
    int        ms;        // duración (set / setall)
} cmd_t;

static QueueHandle_t cola_cmd;

static int motor_de(const char *t)
{
    if (strcmp(t, "all") == 0 || strcmp(t, "todos") == 0) return -1;
    if (strlen(t) == 1) {
        char c = (char)toupper((unsigned char)t[0]);
        if (c >= 'A' && c <= 'D') return c - 'A';
    }
    return -2;   // inválido
}

static float limitar1(float d) { return d > 1.0f ? 1.0f : d < -1.0f ? -1.0f : d; }
static int limitar_ms(int ms) { return ms < 100 ? 100 : ms > MANUAL_MS_MAX ? MANUAL_MS_MAX : ms; }

static bool parsear(const char *txt, cmd_t *c)
{
    char buf[CMD_LEN];
    strncpy(buf, txt, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    for (char *p = buf; *p; p++) *p = (char)tolower((unsigned char)*p);

    char w[3][16] = {{0}};
    int n = sscanf(buf, "%15s %15s %15s", w[0], w[1], w[2]);
    if (n < 1) return false;
    *c = (cmd_t){ .motor = -1, .ms = MANUAL_MS_DEF };

    if (!strcmp(w[0], "pausa") || !strcmp(w[0], "parar") || !strcmp(w[0], "stop")) {
        c->tipo = C_PAUSA; return true;
    }
    if (!strcmp(w[0], "unlock") || !strcmp(w[0], "liberar")) { c->tipo = C_UNLOCK; return true; }
    if (!strcmp(w[0], "selftest")) {
        c->tipo = (n >= 2 && !strcmp(w[1], "abort")) ? C_ABORT : C_SELFTEST;
        return true;
    }
    if (!strcmp(w[0], "setall")) {
        float d[CANT_MOTORES];
        int ms = MANUAL_MS_DEF;
        int k = sscanf(buf, "%*s %f %f %f %f %d", &d[0], &d[1], &d[2], &d[3], &ms);
        if (k < CANT_MOTORES) return false;
        for (int i = 0; i < CANT_MOTORES; i++) {
            if (!isfinite(d[i])) return false;
            c->duties[i] = limitar1(d[i]);
        }
        c->tipo = C_SETALL; c->ms = limitar_ms(ms);
        return true;
    }
    if (!strcmp(w[0], "ext") && n >= 2) {
        if (!strcmp(w[1], "on"))  { c->tipo = C_EXT_ON; return true; }
        if (!strcmp(w[1], "off")) { c->tipo = C_PAUSA;  return true; }
        float d[CANT_MOTORES];
        if (sscanf(buf, "%*s %f %f %f %f", &d[0], &d[1], &d[2], &d[3]) < CANT_MOTORES) return false;
        for (int i = 0; i < CANT_MOTORES; i++) {
            if (!isfinite(d[i])) return false;
            c->duties[i] = limitar1(d[i]);
        }
        c->tipo = C_EXT;
        return true;
    }
    if (!strcmp(w[0], "set") && n >= 3) {
        int ms = MANUAL_MS_DEF;
        sscanf(buf, "%*s %*s %*s %d", &ms);   // "set X duty [ms]": el ms es el 4º token
        c->motor = motor_de(w[1]);
        if (c->motor == -2) return false;
        char *fin;
        float d = strtof(w[2], &fin);
        if (fin == w[2] || !isfinite(d)) return false;
        c->tipo = C_SET; c->valor = limitar1(d); c->ms = limitar_ms(ms);
        return true;
    }
    return false;
}

// =========================================================
// TAREA DE TEST: dueña de los motores
// =========================================================
// Objetivo vigente de cada motor: se vuelve a pasar al driver en CADA tick (la rampa avanza un
// paso por llamada). Solo lo toca la tarea de test.
static float tgt[CANT_MOTORES];
static int   ms_manual = 0, ms_ext = 0;
static bool  en_selftest = false, abortar = false, pedir_selftest = false;

// Ultima muestra (solo la lee la tarea de test)
static uint32_t s_pwm[CANT_MOTORES][2];
static float    s_rpm[CANT_MOTORES];          // rpm del encoder SIN filtrar, siempre >= 0 (los tests no pueden tolerar el atraso)
static int      viol_ambos = 0;               // ticks con IA e IB a la vez en algun motor (T04)
static uint32_t MOTORES_PWM_MAX_visto = 0;            // mayor duty de pin visto (T04)
static int      jitter_us = 0;                // mayor desvio del periodo del lazo
static int64_t  t_prev_us = 0;
static TickType_t t_ultimo;

static void set_modo(modo_t m)
{
    taskENTER_CRITICAL(&estado_mux);
    estado.modo = m;
    taskEXIT_CRITICAL(&estado_mux);
}

static void set_fase(const char *txt)
{
    taskENTER_CRITICAL(&estado_mux);
    strncpy(estado.fase, txt, sizeof(estado.fase) - 1);
    estado.fase[sizeof(estado.fase) - 1] = 0;
    taskEXIT_CRITICAL(&estado_mux);
}

static void todos(float v) { for (int m = 0; m < CANT_MOTORES; m++) tgt[m] = v; }

// Lee driver + encoders y lo deja en el estado publicado
static void muestrear(void)
{
    float duty[CANT_MOTORES], rpm_signo[CANT_MOTORES], raw_signo[CANT_MOTORES], pedido[CANT_MOTORES];
    int sent[CANT_MOTORES];
    for (int m = 0; m < CANT_MOTORES; m++) {
        motores_get_pwm((motor_id_t)m, &s_pwm[m][0], &s_pwm[m][1]);
        if (s_pwm[m][0] && s_pwm[m][1]) viol_ambos++;
        if (s_pwm[m][0] > MOTORES_PWM_MAX_visto) MOTORES_PWM_MAX_visto = s_pwm[m][0];
        if (s_pwm[m][1] > MOTORES_PWM_MAX_visto) MOTORES_PWM_MAX_visto = s_pwm[m][1];
        duty[m] = motores_get_duty((motor_id_t)m);
        sent[m] = motores_get_sentido((motor_id_t)m);
        s_rpm[m] = encoder_get_rpm_raw(ENCODER_DE[m]);
        raw_signo[m] = s_rpm[m] * (float)sent[m];
        rpm_signo[m] = encoder_get_rpm(ENCODER_DE[m]) * (float)sent[m];
        pedido[m] = (tgt[m] == tgt[m]) ? tgt[m] : 0.0f;   // NaN no es JSON valido
    }
    bool falla = motores_get_falla();
    taskENTER_CRITICAL(&estado_mux);
    estado.falla = falla;
    memcpy(estado.duty, duty, sizeof(duty));
    memcpy(estado.pedido, pedido, sizeof(pedido));
    memcpy(estado.sentido, sent, sizeof(sent));
    memcpy(estado.rpm_med, rpm_signo, sizeof(rpm_signo));
    memcpy(estado.rpm_raw, raw_signo, sizeof(raw_signo));
    for (int m = 0; m < CANT_MOTORES; m++) { estado.pwm[m][0] = s_pwm[m][0]; estado.pwm[m][1] = s_pwm[m][1]; }
    taskEXIT_CRITICAL(&estado_mux);
}

// Un tick del lazo: objetivo -> driver, muestra, espera el proximo periodo
static void tick(void)
{
    for (int m = 0; m < CANT_MOTORES; m++) motores_ramp_step((motor_id_t)m, tgt[m]);
    muestrear();
    vTaskDelayUntil(&t_ultimo, pdMS_TO_TICKS(TICK_MS));
    int64_t ahora = esp_timer_get_time();
    if (t_prev_us) {
        int64_t e = ahora - t_prev_us - (int64_t)TICK_MS * 1000;
        if (e < 0) e = -e;
        if (e > jitter_us) jitter_us = (int)e;
    }
    t_prev_us = ahora;
}

static void pausar(void)
{
    todos(0);
    motores_lock(true);
    ms_manual = ms_ext = 0;
    set_modo(MODO_PAUSA);
}

static void atender_cmds(void)
{
    cmd_t c;
    while (xQueueReceive(cola_cmd, &c, 0) == pdTRUE) {
        if (en_selftest) {   // durante el selftest solo se acepta abortar
            if (c.tipo == C_PAUSA || c.tipo == C_ABORT) abortar = true;
            continue;
        }
        switch (c.tipo) {
        case C_SET:
            if (motores_get_falla()) motores_unlock();
            for (int m = 0; m < CANT_MOTORES; m++) tgt[m] = (c.motor == -1 || c.motor == m) ? c.valor : 0.0f;
            ms_manual = c.ms; ms_ext = 0;
            set_modo(MODO_MANUAL);
            break;
        case C_SETALL:
            if (motores_get_falla()) motores_unlock();
            memcpy(tgt, c.duties, sizeof(tgt));
            ms_manual = c.ms; ms_ext = 0;
            set_modo(MODO_MANUAL);
            break;
        case C_PAUSA:
            pausar();
            break;
        case C_UNLOCK:
            motores_unlock();
            break;
        case C_EXT_ON:
            if (motores_get_falla()) motores_unlock();
            todos(0);
            ms_ext = EXT_MS; ms_manual = 0;
            set_modo(MODO_EXT);
            break;
        case C_EXT: {   // solo vale dentro del modo EXT (un "pausa" del usuario no se pisa)
            taskENTER_CRITICAL(&estado_mux);
            bool ext = (estado.modo == MODO_EXT);
            taskEXIT_CRITICAL(&estado_mux);
            if (ext) { memcpy(tgt, c.duties, sizeof(tgt)); ms_ext = EXT_MS; }
            break;
        }
        case C_SELFTEST:
            pedir_selftest = true;
            break;
        case C_ABORT:
            break;
        }
    }
}

// ---------------------------------------------------------
// SELFTEST
// ---------------------------------------------------------
static void poner(int i, res_t r, const char *fmt, ...)
{
    char b[sizeof(tests[0].det)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    taskENTER_CRITICAL(&estado_mux);
    tests[i].r = r;
    memcpy(tests[i].det, b, sizeof(b));
    taskEXIT_CRITICAL(&estado_mux);
}

static bool paso(void) { atender_cmds(); tick(); return !abortar; }

static bool esperar(int ms)
{
    for (int n = (ms + TICK_MS - 1) / TICK_MS; n > 0; n--) {
        if (!paso()) return false;
    }
    return true;
}

static int  firmado(int m) { return (int)s_pwm[m][0] - (int)s_pwm[m][1]; }   // duty de pin con signo
static bool pines_cero(void)
{
    for (int m = 0; m < CANT_MOTORES; m++) if (s_pwm[m][0] || s_pwm[m][1]) return false;
    return true;
}
static int pwm_de(float v) { return (int)(v * (float)MOTORES_PWM_MAX); }   // igual que motores_single_set_ramp (trim = 1)

#define TRAZA_MAX 90
static int16_t traza[TRAZA_MAX];

// Corre ticks hasta que el motor m llegue a 'objetivo' (duty de pin con signo) o se acaben los ticks.
// Deja el duty de cada tick en traza[]; devuelve cuantos ticks corrieron.
static int correr_hasta(int m, int objetivo, int max_ticks)
{
    int n = 0;
    while (n < max_ticks && n < TRAZA_MAX) {
        if (!paso()) { traza[n++] = (int16_t)firmado(m); break; }
        traza[n++] = (int16_t)firmado(m);
        if (firmado(m) == objetivo) break;
    }
    return n;
}

static void selftest(void)
{
    char txt[40];
    en_selftest = true; abortar = false;
    taskENTER_CRITICAL(&estado_mux);
    for (int i = 0; i < N_TESTS; i++) { tests[i].r = R_PEND; tests[i].det[0] = 0; }
    estado.ramp_ms = 0;
    taskEXIT_CRITICAL(&estado_mux);
    set_modo(MODO_SELFTEST);
    viol_ambos = 0; MOTORES_PWM_MAX_visto = 0;
    ms_manual = ms_ext = 0;
    todos(0);
    motores_lock(true);
    ESP_LOGW(TAG, "SELFTEST: levantar las ruedas del piso");

    // ---- T01 bloqueado ----
    set_fase("T01 bloqueado");
    if (!esperar(100)) goto fin;
    {
        bool cero0 = pines_cero();
        bool falla = motores_get_falla();
        tgt[0] = 0.5f;
        if (!esperar(200)) goto fin;
        bool rechaza = pines_cero();
        tgt[0] = 0;
        poner(T01, (cero0 && falla && rechaza) ? R_OK : R_FALLO,
              "pines0=%d falla=%d rechaza=%d", cero0, falla, rechaza);
    }

    // ---- T02 unlock ----
    set_fase("T02 unlock");
    motores_unlock();
    if (!esperar(100)) goto fin;
    {
        bool falla = motores_get_falla();
        bool cero = pines_cero();
        poner(T02, (!falla && cero) ? R_OK : R_FALLO, "falla=%d pines0=%d", falla, cero);
    }

    // ---- T03 pines por motor ----
    {
        char det[sizeof(tests[0].det)];
        int len = 0;
        bool ok = true;
        det[0] = 0;
        for (int m = 0; m < CANT_MOTORES; m++) {
            snprintf(txt, sizeof(txt), "T03 pines motor %c", NOMBRES[m]);
            set_fase(txt);
            tgt[m] = 0.5f;
            if (!esperar(500)) goto fin;
            bool fwd = s_pwm[m][0] > 0 && s_pwm[m][1] == 0;
            bool otros = true;
            float otra_rpm = 0;
            for (int k = 0; k < CANT_MOTORES; k++) {
                if (k == m) continue;
                if (s_pwm[k][0] || s_pwm[k][1]) otros = false;
                if (s_rpm[k] > otra_rpm) otra_rpm = s_rpm[k];
            }
            int rpm_propia = (int)(s_rpm[m] + 0.5f);
            int rpm_otra = (int)(otra_rpm + 0.5f);
            tgt[m] = 0;
            if (!esperar(400)) goto fin;
            tgt[m] = -0.5f;
            if (!esperar(500)) goto fin;
            bool rev = s_pwm[m][1] > 0 && s_pwm[m][0] == 0;
            tgt[m] = 0;
            if (!esperar(400)) goto fin;
            bool bien = fwd && rev && otros;
            ok = ok && bien;
            // rpm propia / mayor rpm de otra rueda: si la otra es mayor, cables de encoder cruzados
            if (len < (int)sizeof(det))
                len += snprintf(det + len, sizeof(det) - len, "%c:%s r=%d/%d ", NOMBRES[m],
                                bien ? "ok" : (!fwd ? "fwd?" : !rev ? "rev?" : "otros?"), rpm_propia, rpm_otra);
        }
        poner(T03, ok ? R_OK : R_FALLO, "%s", det);
    }

    // ---- T05 rampa de subida (motor A) ----
    set_fase("T05 rampa de subida A");
    {
        int obj = pwm_de(0.5f);
        tgt[0] = 0.5f;
        int n = correr_hasta(0, obj, TRAZA_MAX);
        if (abortar) goto fin;
        bool llego = n > 0 && traza[n - 1] == obj;
        bool mono = true, pasos = true;
        int prev = 0;
        for (int i = 0; i < n; i++) {
            int d = traza[i] - prev;
            if (d < 0) mono = false;
            int lim = (prev == 0 ? MIN_DUTY : 0) + ACCEL_STEP;
            if (d > lim) pasos = false;
            prev = traza[i];
        }
        taskENTER_CRITICAL(&estado_mux); estado.ramp_ms = n * TICK_MS; taskEXIT_CRITICAL(&estado_mux);
        poner(T05, (llego && mono && pasos) ? R_OK : R_FALLO,
              "llego=%d mono=%d pasos=%d %d ticks %d ms obj=%d", llego, mono, pasos, n, n * TICK_MS, obj);
    }

    // ---- T06 rampa de bajada ----
    set_fase("T06 rampa de bajada A");
    if (!esperar(200)) goto fin;
    {
        tgt[0] = 0;
        int n = correr_hasta(0, 0, TRAZA_MAX);
        if (abortar) goto fin;
        bool llego = n > 0 && traza[n - 1] == 0;
        bool mono = true, pasos = true, zona = true;
        int prev = pwm_de(0.5f);
        for (int i = 0; i < n; i++) {
            int d = prev - traza[i];
            if (d < 0) mono = false;
            if (d > DECEL_STEP) pasos = false;
            if (traza[i] > 0 && traza[i] < MIN_DUTY) zona = false;
            prev = traza[i];
        }
        poner(T06, (llego && mono && pasos && zona) ? R_OK : R_FALLO,
              "llego=%d mono=%d pasos=%d zona=%d %d ticks", llego, mono, pasos, zona, n);
    }

    // ---- T07 inversion ----
    set_fase("T07 inversion A");
    {
        int obj = pwm_de(0.5f);
        tgt[0] = 0.5f;
        correr_hasta(0, obj, TRAZA_MAX);
        if (abortar || !esperar(200)) goto fin;
        tgt[0] = -0.5f;
        int n = correr_hasta(0, -obj, TRAZA_MAX);
        if (abortar) goto fin;
        bool llego = n > 0 && traza[n - 1] == -obj;
        int ceros = 0;
        bool salto = false;
        int prev = obj;
        for (int i = 0; i < n; i++) {
            if (traza[i] == 0) ceros++;
            if ((prev > 0 && traza[i] < 0) || (prev < 0 && traza[i] > 0)) salto = true;
            prev = traza[i];
        }
        poner(T07, (llego && ceros == 1 && !salto) ? R_OK : R_FALLO,
              "llego=%d ceros=%d salto=%d %d ticks", llego, ceros, salto, n);
    }

    // ---- T08 sentido y duty (motor A queda en -0.5) ----
    set_fase("T08 sentido y duty");
    {
        float d = motores_get_duty(MOTOR_A);
        bool duty_ok = fabsf(d + 0.5f) < 0.02f;                 // ~ -0.5, con decimales
        bool s_rev = motores_get_sentido(MOTOR_A) == -1;
        tgt[0] = 0;
        if (!esperar(500)) goto fin;
        bool s_mantiene = motores_get_sentido(MOTOR_A) == -1 && pines_cero();   // frenado: sigue -1
        tgt[0] = 0.5f;
        if (!esperar(500)) goto fin;
        bool s_fwd = motores_get_sentido(MOTOR_A) == 1;
        tgt[0] = 0;
        if (!esperar(500)) goto fin;
        bool s_mantiene2 = motores_get_sentido(MOTOR_A) == 1;
        bool ok = duty_ok && s_rev && s_mantiene && s_fwd && s_mantiene2;
        poner(T08, ok ? R_OK : R_FALLO, "duty=%d/100 rev=%d mant=%d fwd=%d mant=%d",
              (int)(d * 100.0f), s_rev, s_mantiene, s_fwd, s_mantiene2);
    }

    // ---- T09 lock en plena rampa ----
    set_fase("T09 lock en rampa");
    {
        tgt[0] = 0.8f;
        if (!esperar(100)) goto fin;
        int antes = firmado(0);
        motores_lock(true);
        uint32_t ia, ib;
        motores_get_pwm(MOTOR_A, &ia, &ib);
        bool inmediato = ia == 0 && ib == 0 && motores_get_falla() && motores_get_duty(MOTOR_A) == 0.0f;
        if (!esperar(100)) goto fin;      // el objetivo sigue en 0.8: el driver bloqueado lo tiene que ignorar
        bool ignora = pines_cero();
        tgt[0] = 0;
        motores_unlock();
        poner(T09, (antes > 0 && inmediato && ignora) ? R_OK : R_FALLO,
              "antes=%d inmediato=%d ignora=%d", antes, inmediato, ignora);
    }

    // ---- T10 fuera de rango y NaN (motor B) ----
    set_fase("T10 rango y NaN");
    {
        tgt[1] = 1.5f;
        if (!esperar(800)) goto fin;
        bool max_pos = s_pwm[1][0] == (uint32_t)MOTORES_PWM_MAX && s_pwm[1][1] == 0;
        tgt[1] = -1.5f;
        if (!esperar(1200)) goto fin;
        bool max_neg = s_pwm[1][1] == (uint32_t)MOTORES_PWM_MAX && s_pwm[1][0] == 0;
        tgt[1] = NAN;
        if (!esperar(600)) goto fin;
        bool nan_cero = s_pwm[1][0] == 0 && s_pwm[1][1] == 0;
        tgt[1] = 0;
        poner(T10, (max_pos && max_neg && nan_cero) ? R_OK : R_FALLO,
              "+1.5=%d -1.5=%d nan=%d max=%d", max_pos, max_neg, nan_cero, MOTORES_PWM_MAX);
    }

    // ---- T11 los 4 a la vez + periodo del lazo ----
    set_fase("T11 cuatro motores");
    {
        jitter_us = 0;
        todos(0.5f);
        if (!esperar(800)) goto fin;
        bool todos_fwd = true;
        for (int m = 0; m < CANT_MOTORES; m++) {
            if (!(s_pwm[m][0] == (uint32_t)pwm_de(0.5f) && s_pwm[m][1] == 0)) todos_fwd = false;
        }
        int jit = jitter_us;
        todos(0);
        if (!esperar(500)) goto fin;
        poner(T11, (todos_fwd && jit < JITTER_MAX_US) ? R_OK : R_FALLO,
              "todos=%d jitter=%d us (max %d)", todos_fwd, jit, JITTER_MAX_US);
    }

    // ---- M01 zona muerta ----
    {
        char det[sizeof(tests[0].det)];
        int len = 0;
        det[0] = 0;
        for (int m = 0; m < CANT_MOTORES; m++) {
            snprintf(txt, sizeof(txt), "M01 zona muerta %c", NOMBRES[m]);
            set_fase(txt);
            int umbral = 0;
            for (int paso_pct = 5; paso_pct <= 40; paso_pct += 5) {
                tgt[m] = paso_pct / 100.0f;
                if (!esperar(1000)) goto fin;
                if (s_rpm[m] > RPM_ARRANQUE) { umbral = paso_pct; break; }
            }
            tgt[m] = 0;
            if (!esperar(600)) goto fin;
            if (len < (int)sizeof(det)) {
                if (umbral) len += snprintf(det + len, sizeof(det) - len, "%c=%d%% ", NOMBRES[m], umbral);
                else        len += snprintf(det + len, sizeof(det) - len, "%c=>40%% ", NOMBRES[m]);
            }
        }
        poner(M01, R_MEDIDA, "%s", det);
    }

    // ---- M02 rpm vs duty ----
    {
        static const int NIVELES[4] = { 30, 50, 70, 100 };
        char det[sizeof(tests[0].det)];
        int len = 0;
        det[0] = 0;
        for (int m = 0; m < CANT_MOTORES; m++) {
            snprintf(txt, sizeof(txt), "M02 rpm vs duty %c", NOMBRES[m]);
            set_fase(txt);
            int r[4];
            bool mono = true;
            for (int k = 0; k < 4; k++) {
                tgt[m] = NIVELES[k] / 100.0f;
                if (!esperar(1500)) goto fin;
                r[k] = (int)(s_rpm[m] + 0.5f);
                if (k > 0 && (float)r[k] < 0.9f * (float)r[k - 1]) mono = false;
            }
            tgt[m] = 0;
            if (!esperar(700)) goto fin;
            if (len < (int)sizeof(det))
                len += snprintf(det + len, sizeof(det) - len, "%c%s:%d/%d/%d/%d ", NOMBRES[m],
                                mono ? "" : "!", r[0], r[1], r[2], r[3]);
        }
        poner(M02, R_MEDIDA, "%s", det);
    }

    // ---- M03 signo del encoder al frenar (motor A) ----
    set_fase("M03 signo al frenar A");
    {
        tgt[0] = 0.5f;
        if (!esperar(1200)) goto fin;
        int rpm_f = (int)(s_rpm[0] + 0.5f);
        tgt[0] = 0;
        if (!esperar(140)) goto fin;
        int inercia_f = (int)(s_rpm[0] + 0.5f);
        int sent_f = motores_get_sentido(MOTOR_A);
        if (!esperar(800)) goto fin;
        tgt[0] = -0.5f;
        if (!esperar(1200)) goto fin;
        int rpm_r = (int)(s_rpm[0] + 0.5f);
        tgt[0] = 0;
        if (!esperar(140)) goto fin;
        int inercia_r = (int)(s_rpm[0] + 0.5f);
        int sent_r = motores_get_sentido(MOTOR_A);
        poner(M03, R_MEDIDA, "fwd %d->%d s=%d | rev %d->%d s=%d", rpm_f, inercia_f, sent_f, rpm_r, inercia_r, sent_r);
    }

    // ---- T04 se evalua al final: cubre todas las fases ----
    poner(T04, (viol_ambos == 0 && MOTORES_PWM_MAX_visto <= (uint32_t)MOTORES_PWM_MAX) ? R_OK : R_FALLO,
          "ticks con IA e IB a la vez=%d, duty max=%u/%d", viol_ambos, (unsigned)MOTORES_PWM_MAX_visto, MOTORES_PWM_MAX);
    set_fase("terminado");
    goto cierre;

fin:
    set_fase("abortado");
    ESP_LOGW(TAG, "SELFTEST abortado");
cierre:
    todos(0);
    motores_lock(true);
    en_selftest = false; abortar = false;
    set_modo(MODO_PAUSA);
}

static void test_task(void *arg)
{
    (void)arg;
    int ms_arranque = 0;
#ifdef CONFIG_PROMETEO_TEST_AUTOSTART
    ms_arranque = CONFIG_PROMETEO_TEST_AUTOSTART_DELAY_S * 1000;
    ESP_LOGW(TAG, "Selftest automático en %d s: levantar las ruedas", CONFIG_PROMETEO_TEST_AUTOSTART_DELAY_S);
#endif
    t_ultimo = xTaskGetTickCount();
    set_fase("en espera");
    while (1) {
        atender_cmds();

        if (ms_arranque > 0) {
            ms_arranque -= TICK_MS;
            if (ms_arranque <= 0) pedir_selftest = true;
        }

        // Los comandos mantenidos vencen solos: el driver no tiene failsafe propio
        taskENTER_CRITICAL(&estado_mux);
        modo_t modo = estado.modo;
        taskEXIT_CRITICAL(&estado_mux);
        if (modo == MODO_MANUAL && (ms_manual -= TICK_MS) <= 0) pausar();
        if (modo == MODO_EXT && (ms_ext -= TICK_MS) <= 0) {
            ESP_LOGW(TAG, "EXT: sin duties de la PC en %d ms -> pausa", EXT_MS);
            pausar();
        }

        tick();

        if (pedir_selftest) {
            pedir_selftest = false;
            selftest();
            t_ultimo = xTaskGetTickCount();
            t_prev_us = 0;
        }
    }
}

// =========================================================
// micro-ROS
// =========================================================
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t *rmw_options;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t pub_estado, pub_rpm, pub_res;
static rcl_subscription_t sub_cmd;
static rcl_timer_t timer, timer_rpm, timer_res;
static rclc_executor_t executor;
static std_msgs__msg__String msg_estado, msg_cmd, msg_res;
static std_msgs__msg__Float32MultiArray msg_rpm;
static char buf_json[JSON_LEN];
static char buf_cmd[CMD_LEN];
static char buf_res[RES_LEN];
static float buf_rpm[RPM_MSG_LEN];

#define RCRET(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { \
    ESP_LOGE(TAG, "Fallo en linea %d: rc=%d", __LINE__, (int)temp_rc); return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { \
    ESP_LOGW(TAG, "Fallo en linea %d: %d. Continuando.", __LINE__, (int)temp_rc); } }

static void cmd_callback(const void *msgin)
{
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msgin;
    char txt[CMD_LEN];
    size_t n = m->data.size < sizeof(txt) - 1 ? m->data.size : sizeof(txt) - 1;
    memcpy(txt, m->data.data, n);
    txt[n] = 0;

    cmd_t c;
    if (!parsear(txt, &c)) {
        ESP_LOGW(TAG, "Comando no válido: '%s'", txt);
        return;
    }
    if (xQueueSend(cola_cmd, &c, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Cola de comandos llena: '%s' descartado", txt);
    }
}

static void timer_rpm_callback(rcl_timer_t *t, int64_t last_call_time)
{
    (void)last_call_time;
    if (t == NULL) return;

    estado_t e;
    taskENTER_CRITICAL(&estado_mux);
    e = estado;
    taskEXIT_CRITICAL(&estado_mux);

    for (int m = 0; m < CANT_MOTORES; m++) {
        buf_rpm[m] = e.rpm_med[m];
        buf_rpm[CANT_MOTORES + m] = e.duty[m];
    }
    buf_rpm[8] = (float)e.modo;
    buf_rpm[9] = 0.0f;                  // sin alarma: el driver nuevo no tiene proteccion propia
    for (int m = 0; m < CANT_MOTORES; m++) buf_rpm[10 + m] = e.rpm_raw[m];
    msg_rpm.data.size = RPM_MSG_LEN;
    RCSOFTCHECK(rcl_publish(&pub_rpm, &msg_rpm, NULL));
}

static void timer_callback(rcl_timer_t *t, int64_t last_call_time)
{
    (void)last_call_time;
    if (t == NULL) return;

    estado_t e;
    int t_ok = 0, t_fallo = 0;
    taskENTER_CRITICAL(&estado_mux);
    e = estado;
    for (int i = 0; i < N_TESTS; i++) {
        if (tests[i].r == R_OK) t_ok++;
        if (tests[i].r == R_FALLO) t_fallo++;
    }
    taskEXIT_CRITICAL(&estado_mux);

    // Las claves "viejas" (duties, rpm_med, ...) se mantienen para el panel web y el relay de PlotJuggler.
    // duties = lo que aplica el driver; el resto son campos nuevos del driver nuevo.
    int n = snprintf(buf_json, sizeof(buf_json),
        "{\"modo\":\"%s\",\"motor\":null,\"accion\":\"detenido\",\"duty\":0.00,"
        "\"duties\":[%.3f,%.3f,%.3f,%.3f],"
        "\"vmax\":{\"A\":1.00,\"B\":1.00,\"C\":1.00,\"D\":1.00},\"ciclo\":0,"
        "\"resultados\":{\"A\":\"pendiente\",\"B\":\"pendiente\",\"C\":\"pendiente\",\"D\":\"pendiente\"},"
        "\"rpm_obj\":[0,0,0,0],\"rpm_med\":[%.1f,%.1f,%.1f,%.1f],\"rpm_raw\":[%.1f,%.1f,%.1f,%.1f],\"pi\":{\"kp\":0,\"ki\":0},"
        "\"proteccion\":false,\"alarma\":null,\"failsafe\":%s,"
        "\"falla\":%s,\"pedido\":[%.2f,%.2f,%.2f,%.2f],\"sentido\":[%d,%d,%d,%d],"
        "\"pwm\":[[%u,%u],[%u,%u],[%u,%u],[%u,%u]],\"ramp_ms\":%d,\"fase\":\"%s\","
        "\"t_ok\":%d,\"t_fallo\":%d,\"t_total\":%d}",
        MODO_TXT[e.modo],
        e.duty[0], e.duty[1], e.duty[2], e.duty[3],
        e.rpm_med[0], e.rpm_med[1], e.rpm_med[2], e.rpm_med[3],
        e.rpm_raw[0], e.rpm_raw[1], e.rpm_raw[2], e.rpm_raw[3],
        e.falla ? "true" : "false", e.falla ? "true" : "false",
        e.pedido[0], e.pedido[1], e.pedido[2], e.pedido[3],
        e.sentido[0], e.sentido[1], e.sentido[2], e.sentido[3],
        (unsigned)e.pwm[0][0], (unsigned)e.pwm[0][1], (unsigned)e.pwm[1][0], (unsigned)e.pwm[1][1],
        (unsigned)e.pwm[2][0], (unsigned)e.pwm[2][1], (unsigned)e.pwm[3][0], (unsigned)e.pwm[3][1],
        e.ramp_ms, e.fase, t_ok, t_fallo, N_TESTS);
    if (n < 0 || n >= (int)sizeof(buf_json)) return;

    msg_estado.data.data = buf_json;
    msg_estado.data.size = (size_t)n;
    msg_estado.data.capacity = sizeof(buf_json);
    RCSOFTCHECK(rcl_publish(&pub_estado, &msg_estado, NULL));
}

// Un test por mensaje (en ciclo): mensajes chicos, entran en el MTU de micro-ROS
static void timer_res_callback(rcl_timer_t *t, int64_t last_call_time)
{
    (void)last_call_time;
    if (t == NULL) return;
    static int idx = 0;

    test_t tt;
    taskENTER_CRITICAL(&estado_mux);
    tt = tests[idx];
    taskEXIT_CRITICAL(&estado_mux);

    int n = snprintf(buf_res, sizeof(buf_res), "{\"i\":%d,\"id\":\"%s\",\"n\":\"%s\",\"r\":\"%s\",\"d\":\"%s\"}",
                     idx, tt.id, tt.nombre, R_TXT[tt.r], tt.det);
    idx = (idx + 1) % N_TESTS;
    if (n < 0 || n >= (int)sizeof(buf_res)) return;

    msg_res.data.data = buf_res;
    msg_res.data.size = (size_t)n;
    msg_res.data.capacity = sizeof(buf_res);
    RCSOFTCHECK(rcl_publish(&pub_res, &msg_res, NULL));
}

static bool crear_entidades(void)
{
    RCRET(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Por WiFi cada entidad espera la respuesta del agente: el default (1000 ms) no alcanza
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_creation_session_timeout(rmw_context, ENTIDAD_TIMEOUT_MS);

    RCRET(rclc_node_init_default(&node, "motor_test", "prometeo", &support));
    RCRET(rclc_publisher_init_default(&pub_estado, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motores_test/estado"));
    RCRET(rclc_publisher_init_default(&pub_res, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motores_test/resultados"));
    RCRET(rclc_publisher_init_best_effort(&pub_rpm, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motores_test/rpm"));
    RCRET(rclc_subscription_init_default(&sub_cmd, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motores_test/cmd"));
    RCRET(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ESTADO_PERIODO_MS), timer_callback));
    RCRET(rclc_timer_init_default(&timer_rpm, &support, RCL_MS_TO_NS(RPM_PERIODO_MS), timer_rpm_callback));
    RCRET(rclc_timer_init_default(&timer_res, &support, RCL_MS_TO_NS(RES_PERIODO_MS), timer_res_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCRET(rclc_executor_init(&executor, &support.context, 4, &allocator));   // 3 timers + 1 suscripción
    RCRET(rclc_executor_add_timer(&executor, &timer));
    RCRET(rclc_executor_add_timer(&executor, &timer_rpm));
    RCRET(rclc_executor_add_timer(&executor, &timer_res));
    RCRET(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
    return true;
}

static void destruir_entidades(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void) rcl_subscription_fini(&sub_cmd, &node);
    (void) rcl_publisher_fini(&pub_estado, &node);
    (void) rcl_publisher_fini(&pub_res, &node);
    (void) rcl_publisher_fini(&pub_rpm, &node);
    (void) rcl_timer_fini(&timer);
    (void) rcl_timer_fini(&timer_rpm);
    (void) rcl_timer_fini(&timer_res);
    (void) rclc_executor_fini(&executor);
    (void) rcl_node_fini(&node);
    (void) rclc_support_fini(&support);
}

typedef enum { ESPERANDO_AGENTE, AGENTE_DISPONIBLE, CONECTADO, DESCONECTADO } estado_uros_t;

static void micro_ros_task(void *arg)
{
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 33);   // mismo ROS_DOMAIN_ID que el agente y la PC
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

    // Buffers fijos: no se pide memoria dinámica para los mensajes
    msg_cmd.data.data = buf_cmd;
    msg_cmd.data.size = 0;
    msg_cmd.data.capacity = sizeof(buf_cmd);
    // Float32MultiArray sin layout: dim vacío, solo data (buffer fijo)
    msg_rpm.layout.dim.data = NULL;
    msg_rpm.layout.dim.size = 0;
    msg_rpm.layout.dim.capacity = 0;
    msg_rpm.layout.data_offset = 0;
    msg_rpm.data.data = buf_rpm;
    msg_rpm.data.size = 0;
    msg_rpm.data.capacity = RPM_MSG_LEN;
    msg_estado.data.data = buf_json;
    msg_estado.data.size = 0;
    msg_estado.data.capacity = sizeof(buf_json);
    msg_res.data.data = buf_res;
    msg_res.data.size = 0;
    msg_res.data.capacity = sizeof(buf_res);

    estado_uros_t st = ESPERANDO_AGENTE;
    int ms_desde_ping = 0;

    while (1) {
        switch (st) {
        case ESPERANDO_AGENTE:
            if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK) {
                ESP_LOGI(TAG, "Agente encontrado");
                st = AGENTE_DISPONIBLE;
            } else {
                ESP_LOGW(TAG, "Buscando agente en %s:%s ...",
                         CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case AGENTE_DISPONIBLE:
            if (crear_entidades()) {
                ESP_LOGI(TAG, "CONECTADO. Publicando /motores_test/{estado,rpm,resultados}, escuchando /motores_test/cmd");
                st = CONECTADO;
                ms_desde_ping = 0;
            } else {
                destruir_entidades();
                st = ESPERANDO_AGENTE;
            }
            break;

        case CONECTADO:
            if (ms_desde_ping >= PING_PERIODO_MS) {
                ms_desde_ping = 0;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    st = DESCONECTADO;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            vTaskDelay(pdMS_TO_TICKS(10));
            ms_desde_ping += 20;
            break;

        case DESCONECTADO: {
            ESP_LOGE(TAG, "Agente PERDIDO -> motores en pausa y reconexion");
            cmd_t pausa = { .tipo = C_PAUSA };
            xQueueSend(cola_cmd, &pausa, 0);     // sin comandos no se sigue probando (aborta el selftest)
            destruir_entidades();
            st = ESPERANDO_AGENTE;
            break;
        }
        }
    }
}

void app_main(void)
{
    ESP_LOGW(TAG, "=== FIRMWARE DE TEST DEL DRIVER DE MOTORES === (levantar las ruedas del piso)");

    cola_cmd = xQueueCreate(8, sizeof(cmd_t));
    configASSERT(cola_cmd != NULL);

    motores_init();     // arranca bloqueado (falla = true)
    encoders_init();    // PCNT cuenta por hardware desde ya
    for (int m = 0; m < CANT_MOTORES; m++) {
        ESP_LOGI(TAG, "Motor %c = %s", NOMBRES[m], DESCRIPCION[m]);
    }

    xTaskCreate(test_task, "test_task", TEST_TASK_STACK, NULL, TEST_TASK_PRIO, NULL);

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   // sin ahorro de energía: menos latencia

    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
}
