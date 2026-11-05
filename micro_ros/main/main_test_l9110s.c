/*
 * FIRMWARE DE TEST DEL DRIVER L9110S (reemplaza a main.c cuando PROMETEO_BUILD_L9110S_TEST=y).
 *
 * Corre una bateria de tests sobre drivers/l9110s y reporta TODO por micro-ROS.
 * LEVANTAR LAS RUEDAS DEL PISO antes de correr: los motores giran.
 *
 * micro-ROS (namespace prometeo, nodo l9110s_test):
 *   publica      /prometeo/l9110s_test/estado  std_msgs/String  JSON cada 250 ms
 *   publica      /prometeo/l9110s_test/log     std_msgs/String  una linea por evento ([PASS]/[FAIL]/[INFO])
 *   se suscribe  /prometeo/l9110s_test/cmd     std_msgs/String  comandos de texto
 *
 * Comandos (motores: A=FL delantero izq, B=RL trasero izq, C=FR delantero der, D=RR trasero der):
 *   run | run all        corre boot-check ya hecho + dirs, side, timeout, deadtime, cfg, nan
 *   run <test>           corre un test: dirs side timeout deadtime cfg nan
 *   run stall <A|B|C|D>  MANUAL: bloquear con la mano la rueda del motor X antes de que arranque
 *   stop                 aborta el test en curso y frena todo
 *   clear <A|B|C|D|all>  libera atasco latcheado
 *   list                 lista los tests en el log
 *
 * Tests:
 *   boot      (al arrancar) frenado, duty 0 y falla de timeout activa antes de la primera orden
 *   dirs      cada motor adelante y atras: signo del duty + pulsos del encoder
 *   side      cada lado por separado: se mueven sus 2 motores y los del otro lado no
 *   timeout   dejar de alimentar: todo frena en <= 450 ms y marca L9110S_FAULT_TIMEOUT
 *   deadtime  invertir el sentido: nunca salta de + a - sin pasar por 0, y la pausa >= deadtime_ms
 *   cfg       cfg_set rechaza valores invalidos; invertido, duty_max y trim se aplican
 *   nan       un NaN como orden frena el motor
 *   stall     rueda bloqueada: falla STALL latcheada, motor frenado, clear_fault lo libera
 */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

#include "l9110s.h"
#include "encoder.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>

#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5
#define TEST_TASK_STACK      6144
#define TEST_TASK_PRIO       6
#define PING_PERIODO_MS      1000
#define ENTIDAD_TIMEOUT_MS   5000
#define ESTADO_PERIODO_MS    250
#define CMD_LEN              64
#define JSON_LEN             768
#define LOG_LEN              160
#define PASO_MS              10          // granularidad de esperas/muestreo
#define MIN_PULSOS           3           // pulsos minimos para dar por girada una rueda
#define TIMEOUT_MAX_MS       450         // 300 ms de watchdog + margen de tick

static const char *TAG = "test_l9110s";

#define DUTY_TEST (CONFIG_PROMETEO_L9110S_TEST_DUTY_PCT / 100.0f)

static const char NOMBRES[L9110S_N] = { 'A', 'B', 'C', 'D' };
static const char *DESCRIPCION[L9110S_N] = {
    "delantero izquierdo", "trasero izquierdo", "delantero derecho", "trasero derecho",
};
static const encoder_id_t enc_de[L9110S_N] = {
    ENC_DEL_IZQ, ENC_TRAS_IZQ, ENC_DEL_DER, ENC_TRAS_DER,
};

// =========================================================
// TESTS Y ESTADO COMPARTIDO
// =========================================================
typedef enum { T_BOOT, T_DIRS, T_SIDE, T_TIMEOUT, T_DEAD, T_CFG, T_NAN, T_STALL, T_N } test_id_t;
static const char *T_NOMBRE[T_N] = { "boot", "dirs", "side", "timeout", "deadtime", "cfg", "nan", "stall" };

typedef enum { R_PEND, R_CORRIENDO, R_PASS, R_FAIL, R_PARCIAL, R_ABORT } res_t;
static const char *R_TXT[] = { "pendiente", "corriendo", "pass", "fail", "parcial", "abortado" };

typedef struct {
    res_t  res[T_N];
    int    actual;           // test en curso (-1 = ninguno)
    int    corridas;         // cantidad de "run" terminados
    bool   conectado;
} estado_t;

static estado_t estado = { .actual = -1 };
static portMUX_TYPE estado_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool abortar;

static void set_res(int t, res_t r)
{
    taskENTER_CRITICAL(&estado_mux);
    estado.res[t] = r;
    taskEXIT_CRITICAL(&estado_mux);
}

// =========================================================
// LOG POR micro-ROS (cola -> micro_ros_task lo publica)
// =========================================================
typedef struct { char s[LOG_LEN]; } linea_t;
static QueueHandle_t cola_log;

static void replog(const char *nivel, const char *fmt, ...)
{
    linea_t l;
    int n = snprintf(l.s, sizeof(l.s), "[%s] ", nivel);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(l.s + n, sizeof(l.s) - n, fmt, ap);
    va_end(ap);
    ESP_LOGI(TAG, "%s", l.s);
    xQueueSend(cola_log, &l, 0);          // si está llena se descarta (el serie igual lo tiene)
}
#define INFO(...) replog("INFO", __VA_ARGS__)

// =========================================================
// COMANDOS
// =========================================================
typedef enum { C_RUN, C_STOP, C_CLEAR, C_LIST } cmd_tipo_t;
typedef struct { cmd_tipo_t tipo; int test; int motor; } cmd_t;   // test -1 = all; motor -1 = todos/ninguno
static QueueHandle_t cola_cmd;

static int motor_de(const char *t)
{
    if (!strcmp(t, "all") || !strcmp(t, "todos")) return -1;
    if (strlen(t) == 1 && t[0] >= 'a' && t[0] <= 'd') return t[0] - 'a';
    return -2;
}

static bool parsear(const char *txt, cmd_t *c)
{
    char buf[CMD_LEN];
    strncpy(buf, txt, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    for (char *p = buf; *p; p++) *p = (char)tolower((unsigned char)*p);

    char w[3][16] = {{0}};
    int n = sscanf(buf, "%15s %15s %15s", w[0], w[1], w[2]);
    if (n < 1) return false;
    *c = (cmd_t){ .test = -1, .motor = -1 };

    if (!strcmp(w[0], "stop") || !strcmp(w[0], "pausa") || !strcmp(w[0], "parar")) { c->tipo = C_STOP; return true; }
    if (!strcmp(w[0], "list")) { c->tipo = C_LIST; return true; }
    if (!strcmp(w[0], "clear") && n >= 2) {
        c->motor = motor_de(w[1]);
        if (c->motor == -2) return false;
        c->tipo = C_CLEAR; return true;
    }
    if (!strcmp(w[0], "run")) {
        c->tipo = C_RUN;
        if (n == 1 || !strcmp(w[1], "all")) return true;
        for (int t = T_DIRS; t < T_N; t++) {
            if (!strcmp(w[1], T_NOMBRE[t])) {
                c->test = t;
                if (t == T_STALL) {
                    if (n < 3) return false;
                    c->motor = motor_de(w[2]);
                    return c->motor >= 0;
                }
                return true;
            }
        }
    }
    return false;
}

// =========================================================
// UTILIDADES DE TEST
// =========================================================
static uint32_t pulsos(int m) { return encoder_get_pulsos(enc_de[m]); }

static int signo(float v) { return (v > 0.001f) - (v < -0.001f); }

static int64_t ms_ahora(void) { return esp_timer_get_time() / 1000; }

// Espera ms; con alimentar=true mantiene vivo el watchdog. false = abortado.
static bool esperar(int ms, bool alimentar)
{
    for (int t = 0; t < ms; t += PASO_MS) {
        if (abortar) return false;
        if (alimentar) l9110s_feed();
        vTaskDelay(pdMS_TO_TICKS(PASO_MS));
    }
    return !abortar;
}
#define ESPERA(ms, al) do { if (!esperar((ms), (al))) return R_ABORT; } while (0)

static void limpiar(void)
{
    for (int m = 0; m < L9110S_N; m++) l9110s_clear_fault((l9110s_motor_t)m);
    l9110s_set(0, 0);
}

static res_t resultado(const char *test, bool ok, const char *fmt, ...)
{
    char det[LOG_LEN - 40];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(det, sizeof(det), fmt, ap);
    va_end(ap);
    replog(ok ? "PASS" : "FAIL", "%s: %s", test, det);
    return ok ? R_PASS : R_FAIL;
}

// =========================================================
// TESTS
// =========================================================
static res_t test_boot(void)
{
    uint32_t mask = l9110s_fault_mask();
    bool ceros = true;
    for (int m = 0; m < L9110S_N; m++) if (l9110s_get_duty((l9110s_motor_t)m) != 0.0f) ceros = false;
    return resultado("boot", ceros && (mask & L9110S_FAULT_TIMEOUT) && !(mask & L9110S_FAULT_ANY_STALL),
                     "fallas=0x%03x duties_en_cero=%d (esperado: TIMEOUT activo, sin STALL)", (unsigned)mask, ceros);
}

static res_t test_dirs(void)
{
    limpiar();
    bool ok = true;
    for (int m = 0; m < L9110S_N; m++) {
        l9110s_cfg_t cfg;
        l9110s_cfg_get((l9110s_motor_t)m, &cfg);
        for (int dir = +1; dir >= -1; dir -= 2) {
            uint32_t p0 = pulsos(m);
            l9110s_motor_set((l9110s_motor_t)m, dir * DUTY_TEST);
            ESPERA(1500, true);
            float d = l9110s_get_duty((l9110s_motor_t)m);
            uint32_t dp = pulsos(m) - p0;
            uint32_t f = l9110s_fault_mask();
            int esperado = cfg.invertido ? -dir : dir;
            bool bien = signo(d) == esperado && dp >= MIN_PULSOS && !(f & L9110S_FAULT_STALL(m));
            replog(bien ? "PASS" : "FAIL", "dirs %c (%s) %s: duty=%.2f pulsos=%u%s", NOMBRES[m], DESCRIPCION[m],
                 dir > 0 ? "adelante" : "atras", d, (unsigned)dp,
                 (f & L9110S_FAULT_STALL(m)) ? " ATASCADO" : "");
            ok = ok && bien;
            l9110s_motor_set((l9110s_motor_t)m, 0);
            ESPERA(500, true);
            l9110s_clear_fault((l9110s_motor_t)m);
        }
    }
    return resultado("dirs", ok, ok ? "los 4 motores giran en ambos sentidos" : "algun motor no giro (ver lineas anteriores)");
}

static res_t test_side(void)
{
    limpiar();
    bool ok = true;
    for (int lado = 0; lado < 2; lado++) {
        int a = lado == 0 ? L9110S_FL : L9110S_FR, b = lado == 0 ? L9110S_RL : L9110S_RR;   // activos
        int c = lado == 0 ? L9110S_FR : L9110S_FL, e = lado == 0 ? L9110S_RR : L9110S_RL;   // quietos
        uint32_t pa = pulsos(a), pb = pulsos(b), pc = pulsos(c), pe = pulsos(e);
        l9110s_side_set(lado == 0 ? L9110S_LEFT : L9110S_RIGHT, DUTY_TEST);
        ESPERA(1500, true);
        float da = l9110s_get_duty((l9110s_motor_t)a), db = l9110s_get_duty((l9110s_motor_t)b);
        float dc = l9110s_get_duty((l9110s_motor_t)c), de = l9110s_get_duty((l9110s_motor_t)e);
        uint32_t dpa = pulsos(a) - pa, dpb = pulsos(b) - pb, dpc = pulsos(c) - pc, dpe = pulsos(e) - pe;
        bool bien = signo(da) > 0 && signo(db) > 0 && dc == 0.0f && de == 0.0f &&
                    dpa >= MIN_PULSOS && dpb >= MIN_PULSOS && dpc <= 1 && dpe <= 1;
        replog(bien ? "PASS" : "FAIL", "side %s: duties=%.2f/%.2f otro_lado=%.2f/%.2f pulsos=%u/%u otro_lado=%u/%u",
             lado == 0 ? "izq" : "der", da, db, dc, de,
             (unsigned)dpa, (unsigned)dpb, (unsigned)dpc, (unsigned)dpe);
        ok = ok && bien;
        l9110s_set(0, 0);
        ESPERA(500, true);
        limpiar();
    }
    return resultado("side", ok, ok ? "cada lado maneja solo sus 2 motores" : "ver lineas anteriores");
}

static res_t test_timeout(void)
{
    limpiar();
    l9110s_set(DUTY_TEST, DUTY_TEST);
    ESPERA(800, true);                                    // llegar a régimen
    bool andando = signo(l9110s_get_duty(L9110S_FL)) > 0;
    INFO("timeout: dejo de alimentar el watchdog ahora");
    int64_t t0 = ms_ahora();
    int64_t dt = -1;
    while (ms_ahora() - t0 < 1000) {
        if (abortar) return R_ABORT;
        bool todos0 = true;
        for (int m = 0; m < L9110S_N; m++) if (l9110s_get_duty((l9110s_motor_t)m) != 0.0f) todos0 = false;
        if (todos0) { dt = ms_ahora() - t0; break; }
        vTaskDelay(1);
    }
    uint32_t mask = l9110s_fault_mask();
    bool ok = andando && dt >= 0 && dt <= TIMEOUT_MAX_MS && (mask & L9110S_FAULT_TIMEOUT);
    res_t r = resultado("timeout", ok, "andaba=%d frenó en %lld ms (max %d) fallas=0x%03x", andando,
                        (long long)dt, TIMEOUT_MAX_MS, (unsigned)mask);
    limpiar();
    // una orden nueva tiene que sacar al driver de la falla
    l9110s_set(0, 0);
    bool libre = !(l9110s_fault_mask() & L9110S_FAULT_TIMEOUT);
    if (!libre) { replog("FAIL", "timeout: una orden nueva no limpió la falla"); return R_FAIL; }
    return r;
}

static res_t test_deadtime(void)
{
    limpiar();
    l9110s_cfg_t cfg;
    l9110s_cfg_get(L9110S_FL, &cfg);
    l9110s_set(DUTY_TEST, DUTY_TEST);
    ESPERA(1000, true);
    if (signo(l9110s_get_duty(L9110S_FL)) <= 0) return resultado("deadtime", false, "no arrancó hacia adelante");

    INFO("deadtime: invierto el sentido (deadtime_ms=%u)", (unsigned)cfg.deadtime_ms);
    l9110s_set(-DUTY_TEST, -DUTY_TEST);
    int64_t t_cero = -1, gap = -1;
    bool salto_directo = false, llego_atras = false;
    int ult = +1;
    for (int64_t t0 = ms_ahora(); ms_ahora() - t0 < 2500 && !llego_atras; ) {
        if (abortar) return R_ABORT;
        l9110s_feed();
        int s = signo(l9110s_get_duty(L9110S_FL));
        if (s == 0 && t_cero < 0) t_cero = ms_ahora();
        if (s != 0 && s != ult) {
            if (t_cero < 0) salto_directo = true;
            else gap = ms_ahora() - t_cero;
            llego_atras = true;
        }
        if (s != 0) ult = s;
        vTaskDelay(pdMS_TO_TICKS(PASO_MS));
    }
    // 25 ms de tolerancia: tick de control (20 ms) + muestreo
    bool ok = llego_atras && !salto_directo && gap >= (int64_t)cfg.deadtime_ms - 25;
    res_t r = resultado("deadtime", ok, "revirtió=%d salto_directo=%d pausa_en_cero=%lld ms (min %lld)",
                        llego_atras, salto_directo, (long long)gap, (long long)cfg.deadtime_ms - 25);
    limpiar();
    return r;
}

static res_t test_cfg(void)
{
    limpiar();
    l9110s_cfg_t orig;
    l9110s_cfg_get(L9110S_FL, &orig);
    bool ok = true;

    // 1) valores invalidos rechazados
    struct { const char *que; l9110s_cfg_t c; } malos[8];
    int nm = 0;
    #define MALO(txt, campo, val) do { malos[nm].que = txt; malos[nm].c = orig; malos[nm].c.campo = (val); nm++; } while (0)
    MALO("trim=2.0", trim, 2.0f);
    MALO("trim=NaN", trim, NAN);
    MALO("zona_muerta=0.95", zona_muerta, 0.95f);
    MALO("duty_max<=zona", duty_max, orig.zona_muerta);
    MALO("rampa_paso=0", rampa_paso, 0.0f);
    MALO("deadtime=5000", deadtime_ms, 5000);
    MALO("stall_ms=50", stall_ms, 50);
    MALO("stall_duty=1.5", stall_duty_min, 1.5f);
    for (int i = 0; i < nm; i++) {
        bool rech = l9110s_cfg_set(L9110S_FL, &malos[i].c) == ESP_ERR_INVALID_ARG;
        if (!rech) replog("FAIL", "cfg: aceptó %s", malos[i].que);
        ok = ok && rech;
    }
    if (l9110s_cfg_set((l9110s_motor_t)9, &orig) != ESP_ERR_INVALID_ARG) { replog("FAIL", "cfg: aceptó motor 9"); ok = false; }
    {   // sin cambios tras los rechazos
        l9110s_cfg_t now;
        l9110s_cfg_get(L9110S_FL, &now);
        if (memcmp(&now, &orig, sizeof(now)) != 0) { replog("FAIL", "cfg: un rechazo modificó la config"); ok = false; }
    }

    // 2) cada motor tiene su propia config: cambiar FL no toca RL
    {
        l9110s_cfg_t a = orig, rl_antes, rl_desp;
        l9110s_cfg_get(L9110S_RL, &rl_antes);
        a.trim = 0.9f;
        if (l9110s_cfg_set(L9110S_FL, &a) != ESP_OK) { replog("FAIL", "cfg: no aceptó trim=0.9"); ok = false; }
        l9110s_cfg_t g;
        l9110s_cfg_get(L9110S_FL, &g);
        l9110s_cfg_get(L9110S_RL, &rl_desp);
        bool indep = fabsf(g.trim - 0.9f) < 1e-6f && memcmp(&rl_antes, &rl_desp, sizeof(rl_antes)) == 0;
        if (!indep) replog("FAIL", "cfg: la config no es independiente por motor");
        ok = ok && indep;
    }

    // 3) efectos en el duty real (sin vigilar atasco para no depender del encoder)
    #define PROBAR(txt, mod, orden, cond) do { \
        l9110s_cfg_t c = orig; c.stall_ms = 0; mod; \
        if (l9110s_cfg_set(L9110S_FL, &c) != ESP_OK) { replog("FAIL", "cfg: %s no aceptada", txt); ok = false; break; } \
        l9110s_motor_set(L9110S_FL, (orden)); \
        l9110s_feed(); \
        ESPERA(700, true); \
        float d = l9110s_get_duty(L9110S_FL); \
        bool b = (cond); \
        replog(b ? "PASS" : "FAIL", "cfg %s: duty=%.3f", txt, d); \
        ok = ok && b; \
        l9110s_motor_set(L9110S_FL, 0); ESPERA(700, true); \
    } while (0)

    PROBAR("invertido", c.invertido = true, 0.5f, d < -0.01f);
    PROBAR("duty_max=0.6", (c.zona_muerta = 0.3f, c.duty_max = 0.6f), 1.0f, fabsf(d - 0.6f) < 0.02f);
    PROBAR("trim=0.5 zona=0", (c.zona_muerta = 0.0f, c.trim = 0.5f, c.duty_max = 1.0f), 0.8f, fabsf(d - 0.4f) < 0.02f);

    l9110s_cfg_set(L9110S_FL, &orig);                     // restaurar
    l9110s_cfg_t fin;
    l9110s_cfg_get(L9110S_FL, &fin);
    if (memcmp(&fin, &orig, sizeof(fin)) != 0) { replog("FAIL", "cfg: no se pudo restaurar la config de FL"); ok = false; }
    res_t r = resultado("cfg", ok, ok ? "validacion, independencia y efectos OK" : "ver lineas anteriores");
    limpiar();
    return r;
}

static res_t test_nan(void)
{
    limpiar();
    l9110s_motor_set(L9110S_FL, DUTY_TEST);
    ESPERA(600, true);
    bool andaba = signo(l9110s_get_duty(L9110S_FL)) > 0;
    l9110s_motor_set(L9110S_FL, NAN);
    ESPERA(700, true);
    float d = l9110s_get_duty(L9110S_FL);
    float inf_d;
    l9110s_motor_set(L9110S_FL, INFINITY);                // inf no es finito -> 0 tambien
    ESPERA(700, true);
    inf_d = l9110s_get_duty(L9110S_FL);
    res_t r = resultado("nan", andaba && d == 0.0f && inf_d == 0.0f, "andaba=%d duty_tras_NaN=%.2f duty_tras_inf=%.2f", andaba, d, inf_d);
    limpiar();
    return r;
}

static res_t test_stall(int m)
{
    limpiar();
    INFO("stall %c: BLOQUEAR la rueda %c (%s) con la mano. Arranca en 5 s ('stop' para abortar)",
         NOMBRES[m], NOMBRES[m], DESCRIPCION[m]);
    ESPERA(5000, false);
    int64_t t0 = ms_ahora(), dt = -1;
    uint32_t p0 = pulsos(m);
    l9110s_motor_set((l9110s_motor_t)m, DUTY_TEST);
    while (ms_ahora() - t0 < 6000) {
        if (abortar) return R_ABORT;
        l9110s_feed();
        if (l9110s_fault_mask() & L9110S_FAULT_STALL(m)) { dt = ms_ahora() - t0; break; }
        vTaskDelay(pdMS_TO_TICKS(PASO_MS));
    }
    uint32_t dp = pulsos(m) - p0;
    bool latch = dt >= 0;
    bool frenado = l9110s_get_duty((l9110s_motor_t)m) == 0.0f;
    ESPERA(300, true);
    bool sigue = (l9110s_fault_mask() & L9110S_FAULT_STALL(m)) && l9110s_get_duty((l9110s_motor_t)m) == 0.0f;
    bool libera = l9110s_clear_fault((l9110s_motor_t)m) && !(l9110s_fault_mask() & L9110S_FAULT_STALL(m));
    if (latch && dp >= 10) INFO("stall %c: la rueda giró (%u pulsos): no estaba bien bloqueada", NOMBRES[m], (unsigned)dp);
    if (!latch) INFO("stall %c: no hubo atasco: ¿rueda libre o encoder sin conectar?", NOMBRES[m]);
    bool ok = latch && frenado && sigue && libera;
    char nom[16];
    snprintf(nom, sizeof(nom), "stall %c", NOMBRES[m]);
    res_t r = resultado(nom, ok, "latch=%d en %lld ms freno=%d sigue_frenado=%d clear_ok=%d pulsos=%u",
                        latch, (long long)dt, frenado, sigue, libera, (unsigned)dp);
    limpiar();
    INFO("stall %c: soltar la rueda", NOMBRES[m]);
    return r;
}

// =========================================================
// TAREA DE TEST
// =========================================================
static void correr(int t, int motor)
{
    abortar = false;
    taskENTER_CRITICAL(&estado_mux);
    estado.actual = t;
    estado.res[t] = R_CORRIENDO;
    taskEXIT_CRITICAL(&estado_mux);
    INFO("=== test %s ===", T_NOMBRE[t]);

    res_t r;
    switch (t) {
    case T_DIRS:    r = test_dirs();     break;
    case T_SIDE:    r = test_side();     break;
    case T_TIMEOUT: r = test_timeout();  break;
    case T_DEAD:    r = test_deadtime(); break;
    case T_CFG:     r = test_cfg();      break;
    case T_NAN:     r = test_nan();      break;
    case T_STALL:   r = test_stall(motor); break;
    default:        r = R_FAIL;          break;
    }

    if (t == T_STALL) {                    // pass solo con los 4 motores probados
        static uint32_t ok_mask;
        if (r == R_PASS) ok_mask |= 1u << motor; else ok_mask &= ~(1u << motor);
        if (r != R_ABORT) r = ok_mask == 0xF ? R_PASS : ok_mask ? R_PARCIAL : R_FAIL;
        else r = ok_mask ? R_PARCIAL : R_ABORT;
    }
    if (r == R_ABORT) replog("INFO", "test %s abortado", T_NOMBRE[t]);
    l9110s_brake_all();
    taskENTER_CRITICAL(&estado_mux);
    estado.res[t] = r;
    estado.actual = -1;
    taskEXIT_CRITICAL(&estado_mux);
}

static void resumen(void)
{
    int pass = 0, fail = 0;
    char s[LOG_LEN];
    int n = snprintf(s, sizeof(s), "RESUMEN:");
    taskENTER_CRITICAL(&estado_mux);
    for (int t = 0; t < T_N; t++) {
        res_t r = estado.res[t];
        if (r == R_PASS) pass++; else if (r == R_FAIL) fail++;
        if (n < (int)sizeof(s) - 24) n += snprintf(s + n, sizeof(s) - n, " %s=%s", T_NOMBRE[t], R_TXT[r]);
    }
    estado.corridas++;
    taskEXIT_CRITICAL(&estado_mux);
    replog(fail ? "FAIL" : "INFO", "%s (pass=%d fail=%d)", s, pass, fail);
}

static void test_task(void *arg)
{
    // El boot-check va primero: el driver todavía no recibió ninguna orden
    set_res(T_BOOT, test_boot());

    int autostart_ms = CONFIG_PROMETEO_L9110S_TEST_AUTOSTART_DELAY_S * 1000;
    for (;;) {
        cmd_t c;
        if (xQueueReceive(cola_cmd, &c, pdMS_TO_TICKS(100)) != pdTRUE) {
            bool con;
            taskENTER_CRITICAL(&estado_mux); con = estado.conectado; taskEXIT_CRITICAL(&estado_mux);
            if (autostart_ms > 0 && con) {
                autostart_ms -= 100;
                if (autostart_ms <= 0) { c = (cmd_t){ .tipo = C_RUN, .test = -1, .motor = -1 }; }
                else continue;
            } else continue;
        }
        switch (c.tipo) {
        case C_STOP:
            abortar = true; l9110s_brake_all(); INFO("stop: todo frenado");
            break;
        case C_LIST:
            INFO("tests: dirs side timeout deadtime cfg nan | manual: run stall <A|B|C|D>");
            break;
        case C_CLEAR:
            for (int m = 0; m < L9110S_N; m++)
                if (c.motor == -1 || c.motor == m) {
                    bool h = l9110s_clear_fault((l9110s_motor_t)m);
                    INFO("clear %c: %s", NOMBRES[m], h ? "falla liberada" : "no había falla");
                }
            break;
        case C_RUN:
            if (c.test >= 0) correr(c.test, c.motor);
            else for (int t = T_DIRS; t < T_STALL && !abortar; t++) correr(t, -1);
            resumen();
            break;
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
static rcl_publisher_t pub_estado, pub_log;
static rcl_subscription_t sub_cmd;
static rcl_timer_t timer;
static rclc_executor_t executor;
static std_msgs__msg__String msg_estado, msg_log, msg_cmd;
static char buf_json[JSON_LEN];
static char buf_log[LOG_LEN];
static char buf_cmd[CMD_LEN];

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
        replog("INFO", "comando no válido: '%.60s' (run [all|dirs|side|timeout|deadtime|cfg|nan|stall X], stop, clear X, list)", txt);
        return;
    }
    if (c.tipo == C_STOP) { abortar = true; l9110s_brake_all(); }   // inmediato, sin esperar a la cola
    if (xQueueSend(cola_cmd, &c, 0) != pdTRUE) ESP_LOGW(TAG, "Cola de comandos llena: '%s' descartado", txt);
}

static void timer_callback(rcl_timer_t *t, int64_t last_call_time)
{
    (void)last_call_time;
    if (t == NULL) return;

    estado_t e;
    taskENTER_CRITICAL(&estado_mux);
    e = estado;
    taskEXIT_CRITICAL(&estado_mux);

    int n = snprintf(buf_json, sizeof(buf_json), "{\"test\":%s%s%s,\"duties\":[",
                     e.actual >= 0 ? "\"" : "", e.actual >= 0 ? T_NOMBRE[e.actual] : "null", e.actual >= 0 ? "\"" : "");
    for (int m = 0; m < L9110S_N; m++)
        n += snprintf(buf_json + n, sizeof(buf_json) - n, "%s%.2f", m ? "," : "", l9110s_get_duty((l9110s_motor_t)m));
    n += snprintf(buf_json + n, sizeof(buf_json) - n, "],\"pulsos\":[");
    for (int m = 0; m < L9110S_N; m++)
        n += snprintf(buf_json + n, sizeof(buf_json) - n, "%s%u", m ? "," : "", (unsigned)pulsos(m));
    uint32_t mask = l9110s_fault_mask();
    n += snprintf(buf_json + n, sizeof(buf_json) - n,
                  "],\"fallas\":{\"timeout\":%s,\"stall\":[%s,%s,%s,%s]},\"resultados\":{",
                  (mask & L9110S_FAULT_TIMEOUT) ? "true" : "false",
                  (mask & L9110S_FAULT_STALL(0)) ? "true" : "false", (mask & L9110S_FAULT_STALL(1)) ? "true" : "false",
                  (mask & L9110S_FAULT_STALL(2)) ? "true" : "false", (mask & L9110S_FAULT_STALL(3)) ? "true" : "false");
    for (int i = 0; i < T_N; i++)
        n += snprintf(buf_json + n, sizeof(buf_json) - n, "%s\"%s\":\"%s\"", i ? "," : "", T_NOMBRE[i], R_TXT[e.res[i]]);
    n += snprintf(buf_json + n, sizeof(buf_json) - n, "},\"corridas\":%d}", e.corridas);
    if (n < 0 || n >= (int)sizeof(buf_json)) return;

    msg_estado.data.data = buf_json;
    msg_estado.data.size = (size_t)n;
    msg_estado.data.capacity = sizeof(buf_json);
    RCSOFTCHECK(rcl_publish(&pub_estado, &msg_estado, NULL));
}

static bool crear_entidades(void)
{
    RCRET(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_creation_session_timeout(rmw_context, ENTIDAD_TIMEOUT_MS);

    RCRET(rclc_node_init_default(&node, "l9110s_test", "prometeo", &support));
    RCRET(rclc_publisher_init_default(&pub_estado, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "l9110s_test/estado"));
    RCRET(rclc_publisher_init_default(&pub_log, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "l9110s_test/log"));
    RCRET(rclc_subscription_init_default(&sub_cmd, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "l9110s_test/cmd"));
    RCRET(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ESTADO_PERIODO_MS), timer_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCRET(rclc_executor_init(&executor, &support.context, 2, &allocator));   // timer + 1 suscripción
    RCRET(rclc_executor_add_timer(&executor, &timer));
    RCRET(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
    return true;
}

static void destruir_entidades(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    (void) rcl_subscription_fini(&sub_cmd, &node);
    (void) rcl_publisher_fini(&pub_log, &node);
    (void) rcl_publisher_fini(&pub_estado, &node);
    (void) rcl_timer_fini(&timer);
    (void) rclc_executor_fini(&executor);
    (void) rcl_node_fini(&node);
    (void) rclc_support_fini(&support);
}

static void publicar_logs(void)
{
    linea_t l;
    for (int i = 0; i < 4 && xQueueReceive(cola_log, &l, 0) == pdTRUE; i++) {
        strncpy(buf_log, l.s, sizeof(buf_log) - 1);
        buf_log[sizeof(buf_log) - 1] = 0;
        msg_log.data.data = buf_log;
        msg_log.data.size = strlen(buf_log);
        msg_log.data.capacity = sizeof(buf_log);
        RCSOFTCHECK(rcl_publish(&pub_log, &msg_log, NULL));
    }
}

typedef enum { ESPERANDO_AGENTE, AGENTE_DISPONIBLE, CONECTADO, DESCONECTADO } estado_uros_t;

static void micro_ros_task(void *arg)
{
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

    msg_cmd.data.data = buf_cmd;
    msg_cmd.data.size = 0;
    msg_cmd.data.capacity = sizeof(buf_cmd);
    msg_estado.data.data = buf_json;
    msg_estado.data.size = 0;
    msg_estado.data.capacity = sizeof(buf_json);
    msg_log.data.data = buf_log;
    msg_log.data.size = 0;
    msg_log.data.capacity = sizeof(buf_log);

    estado_uros_t st = ESPERANDO_AGENTE;
    int ms_desde_ping = 0;

    while (1) {
        switch (st) {
        case ESPERANDO_AGENTE:
            if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK) {
                ESP_LOGI(TAG, "Agente encontrado");
                st = AGENTE_DISPONIBLE;
            } else {
                ESP_LOGW(TAG, "Buscando agente en %s:%s ...", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case AGENTE_DISPONIBLE:
            if (crear_entidades()) {
                ESP_LOGI(TAG, "CONECTADO. Publicando /l9110s_test/estado y /l9110s_test/log, escuchando /l9110s_test/cmd");
                st = CONECTADO;
                ms_desde_ping = 0;
                taskENTER_CRITICAL(&estado_mux); estado.conectado = true; taskEXIT_CRITICAL(&estado_mux);
                INFO("conectado. 'list' para ver los tests, 'run' para correrlos (rueda levantadas!)");
                // el resultado del boot-check se hizo antes de conectar: lo repito aca
                taskENTER_CRITICAL(&estado_mux); res_t rb = estado.res[T_BOOT]; taskEXIT_CRITICAL(&estado_mux);
                INFO("boot: %s", R_TXT[rb]);
            } else {
                destruir_entidades();
                st = ESPERANDO_AGENTE;
            }
            break;

        case CONECTADO:
            if (ms_desde_ping >= PING_PERIODO_MS) {
                ms_desde_ping = 0;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) { st = DESCONECTADO; break; }
            }
            publicar_logs();
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            vTaskDelay(pdMS_TO_TICKS(10));
            ms_desde_ping += 20;
            break;

        case DESCONECTADO: {
            ESP_LOGE(TAG, "Agente PERDIDO -> test abortado, motores frenados");
            abortar = true;
            l9110s_brake_all();
            taskENTER_CRITICAL(&estado_mux); estado.conectado = false; taskEXIT_CRITICAL(&estado_mux);
            destruir_entidades();
            st = ESPERANDO_AGENTE;
            break;
        }
        }
    }
}

void app_main(void)
{
    ESP_LOGW(TAG, "=== FIRMWARE DE TEST DEL DRIVER L9110S === (levantar las ruedas del piso)");

    cola_cmd = xQueueCreate(8, sizeof(cmd_t));
    cola_log = xQueueCreate(32, sizeof(linea_t));
    configASSERT(cola_cmd != NULL && cola_log != NULL);
    for (int t = 0; t < T_N; t++) estado.res[t] = R_PEND;

    encoders_init();                       // antes del driver: la deteccion de atasco lee los encoders
    esp_err_t e = l9110s_init();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "l9110s_init fallo: %s (pines repetidos o invalidos en menuconfig?)", esp_err_to_name(e));
        return;                            // sin driver no hay nada que probar
    }
    for (int m = 0; m < L9110S_N; m++) ESP_LOGI(TAG, "Motor %c = %s", NOMBRES[m], DESCRIPCION[m]);

    xTaskCreate(test_task, "test_task", TEST_TASK_STACK, NULL, TEST_TASK_PRIO, NULL);

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
}
