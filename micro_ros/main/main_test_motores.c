/*
 * FIRMWARE DE TEST DE MOTORES (reemplaza a main.c cuando PROMETEO_BUILD_MOTOR_TEST=y).
 *
 * Ciclo automático: cada motor A, B, C, D se prueba adelante y atrás, con rampa de
 * potencia hasta PROMETEO_TEST_DUTY_MAX_PCT. Al terminar los 4, el ciclo se pausa.
 *
 * ATENCIÓN: no hay realimentación eléctrica (ni encoders ni corriente). "probado"
 * significa que el motor FUE ACCIONADO; quien mira el robot confirma si giró y marca
 * el resultado con los comandos "ok X" / "fallo X".
 *
 * micro-ROS (namespace prometeo, nodo motor_test):
 *   publica      /prometeo/motores_test/estado  std_msgs/String  (JSON, cada 250 ms)
 *   se suscribe  /prometeo/motores_test/cmd     std_msgs/String  (comandos de texto)
 *
 * Comandos (X = A|B|C|D, duty = -1..1):
 *   auto               arranca/continúa el ciclo automático (desde el motor actual)
 *   reset              borra resultados y arranca el ciclo desde el motor A
 *   pausa | parar      frena todo y queda en pausa
 *   siguiente          salta al siguiente motor (en auto)
 *   setall a b c d [ms]  manual: los 4 motores a la vez, cada uno con SU duty (A B C D, -1..1)
 *   set X duty [ms]    manual: motor X (o "all") a ese duty durante ms (def. 2000, máx. 10000)
 *   vmax v             potencia máxima (0.1..1.0) del ciclo automático, de los 4 motores
 *   vmax X v           idem, solo del motor X (cada motor tiene la suya)
 *   ok X | fallo X     marca el resultado del motor X
 *
 * Motores (igual que menuconfig):
 *   A delantero izq, B trasero izq, C delantero der, D trasero der
 */
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
#include "esp_wifi.h"
#include "sdkconfig.h"

#include "motores.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>

#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5
#define TEST_TASK_STACK      4096
#define TEST_TASK_PRIO       6
#define TICK_MS              50
#define PING_PERIODO_MS      1000
#define ENTIDAD_TIMEOUT_MS   5000    // espera máxima del agente al crear cada entidad (WiFi)
#define ESTADO_PERIODO_MS    250
#define MANUAL_MS_DEF        2000
#define MANUAL_MS_MAX        10000
#define RAMPA_BAJA_MS        400
#define CMD_LEN              64
#define JSON_LEN             384

static const char *TAG = "test_motores";

static const char NOMBRES[CANT_MOTORES]   = { 'A', 'B', 'C', 'D' };
static const char *DESCRIPCION[CANT_MOTORES] = {
    "delantero izquierdo", "trasero izquierdo", "delantero derecho", "trasero derecho",
};

// =========================================================
// ESTADO COMPARTIDO (lo escribe la tarea de test, lo lee micro-ROS)
// =========================================================
typedef enum { MODO_PAUSA, MODO_AUTO, MODO_MANUAL } modo_t;
typedef enum { R_PENDIENTE, R_PROBANDO, R_PROBADO, R_OK, R_FALLO } resultado_t;
static const char *R_TXT[]    = { "pendiente", "probando", "probado", "ok", "fallo" };
static const char *MODO_TXT[] = { "pausa", "auto", "manual" };

typedef struct {
    modo_t      modo;
    int         motor;          // motor en prueba (-1 = ninguno / varios)
    int         dir;            // +1 adelante, -1 atrás, 0 detenido
    float       duty;           // potencia del motor en prueba (-1..1)
    float       duties[CANT_MOTORES]; // potencia que se está aplicando a cada motor
    float       vmax[CANT_MOTORES]; // potencia máxima del ciclo automático, de cada motor
    int         ciclo;          // ciclos completados
    resultado_t resultado[CANT_MOTORES];
} estado_t;

static estado_t estado;
static portMUX_TYPE estado_mux = portMUX_INITIALIZER_UNLOCKED;

// =========================================================
// COMANDOS (callback de micro-ROS -> cola -> tarea de test)
// =========================================================
typedef enum {
    C_AUTO, C_RESET, C_PAUSA, C_SIGUIENTE, C_SET, C_SETALL, C_VMAX, C_OK, C_FALLO
} cmd_tipo_t;

typedef struct {
    cmd_tipo_t tipo;
    int        motor;     // 0..3, -1 = todos
    float      valor;     // duty (set) o vmax
    float      duties[CANT_MOTORES]; // duty de cada motor (setall)
    int        ms;        // duración (set)
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

    if (!strcmp(w[0], "auto"))      { c->tipo = C_AUTO;      return true; }
    if (!strcmp(w[0], "reset"))     { c->tipo = C_RESET;     return true; }
    if (!strcmp(w[0], "pausa") || !strcmp(w[0], "parar") || !strcmp(w[0], "stop")) {
        c->tipo = C_PAUSA; return true;
    }
    if (!strcmp(w[0], "siguiente")) { c->tipo = C_SIGUIENTE; return true; }

    if (!strcmp(w[0], "vmax") && n >= 2) {
        // "vmax v" (todos) o "vmax X v" (un motor)
        const char *num = w[1];
        if (n >= 3) {
            c->motor = motor_de(w[1]);
            if (c->motor == -2) return false;
            num = w[2];
        }
        char *fin;
        float v = strtof(num, &fin);
        if (fin == num || !(v >= 0.1f && v <= 1.0f)) return false;
        c->tipo = C_VMAX; c->valor = v; return true;
    }
    if ((!strcmp(w[0], "ok") || !strcmp(w[0], "fallo")) && n >= 2) {
        c->motor = motor_de(w[1]);
        if (c->motor < 0) return false;
        c->tipo = !strcmp(w[0], "ok") ? C_OK : C_FALLO;
        return true;
    }
    if (!strcmp(w[0], "setall")) {
        float d[CANT_MOTORES];
        int ms = MANUAL_MS_DEF;
        int k = sscanf(buf, "%*s %f %f %f %f %d", &d[0], &d[1], &d[2], &d[3], &ms);
        if (k < CANT_MOTORES) return false;
        for (int i = 0; i < CANT_MOTORES; i++) {
            if (!isfinite(d[i])) return false;
            c->duties[i] = d[i] > 1.0f ? 1.0f : d[i] < -1.0f ? -1.0f : d[i];
        }
        if (ms < 100) ms = 100;
        if (ms > MANUAL_MS_MAX) ms = MANUAL_MS_MAX;
        c->tipo = C_SETALL; c->ms = ms;
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
        if (d > 1.0f) d = 1.0f;
        if (d < -1.0f) d = -1.0f;
        if (ms < 100) ms = 100;
        if (ms > MANUAL_MS_MAX) ms = MANUAL_MS_MAX;
        c->tipo = C_SET; c->valor = d; c->ms = ms;
        return true;
    }
    return false;
}

// =========================================================
// TAREA DE TEST: dueña de los motores
// =========================================================
#define N_PASOS (CANT_MOTORES * 2)   // cada motor: adelante y atrás

typedef enum { F_SUBE, F_MANTIENE, F_BAJA, F_PAUSA } fase_t;

// Único lugar donde se manda duty a los motores: también lo deja en el estado publicado
static void aplicar_duties(const float d[CANT_MOTORES])
{
    for (int m = 0; m < CANT_MOTORES; m++) {
        motor_set((motor_id_t)m, d[m]);
    }
    motores_alimentar();
    taskENTER_CRITICAL(&estado_mux);
    memcpy(estado.duties, d, sizeof(estado.duties));
    taskEXIT_CRITICAL(&estado_mux);
}

// motor 0..3 = solo ese; -1 = los 4 al mismo duty (los demás, 0)
static void aplicar_duty(int motor, float duty)
{
    float d[CANT_MOTORES];
    for (int m = 0; m < CANT_MOTORES; m++) {
        d[m] = (motor == -1 || m == motor) ? duty : 0.0f;
    }
    aplicar_duties(d);
}

static void publicar_estado(modo_t modo, int motor, int dir, float duty)
{
    taskENTER_CRITICAL(&estado_mux);
    estado.modo = modo; estado.motor = motor; estado.dir = dir; estado.duty = duty;
    if (modo == MODO_PAUSA) memset(estado.duties, 0, sizeof(estado.duties));   // pausa = frenados
    taskEXIT_CRITICAL(&estado_mux);
}

static void marcar(int motor, resultado_t r)
{
    taskENTER_CRITICAL(&estado_mux);
    estado.resultado[motor] = r;
    taskEXIT_CRITICAL(&estado_mux);
}

static resultado_t leer_resultado(int motor)
{
    taskENTER_CRITICAL(&estado_mux);
    resultado_t r = estado.resultado[motor];
    taskEXIT_CRITICAL(&estado_mux);
    return r;
}

static void test_task(void *arg)
{
    modo_t modo = MODO_PAUSA;
    int paso = 0;                 // 0..N_PASOS-1
    fase_t fase = F_SUBE;
    int t_fase = 0;               // ms en la fase actual
    int ms_manual = 0;            // ms que le quedan al comando manual
    float vmax[CANT_MOTORES];
    for (int m = 0; m < CANT_MOTORES; m++) vmax[m] = CONFIG_PROMETEO_TEST_DUTY_MAX_PCT / 100.0f;
    int ms_arranque = 0;

    taskENTER_CRITICAL(&estado_mux);
    memcpy(estado.vmax, vmax, sizeof(vmax));
    taskEXIT_CRITICAL(&estado_mux);

#ifdef CONFIG_PROMETEO_TEST_AUTOSTART
    ms_arranque = CONFIG_PROMETEO_TEST_AUTOSTART_DELAY_S * 1000;
    ESP_LOGW(TAG, "Arranque automático en %d s: levantar las ruedas", CONFIG_PROMETEO_TEST_AUTOSTART_DELAY_S);
#endif

    TickType_t t = xTaskGetTickCount();
    while (1) {
        // ---- comandos ----
        cmd_t c;
        while (xQueueReceive(cola_cmd, &c, 0) == pdTRUE) {
            switch (c.tipo) {
            case C_AUTO:
                if (paso >= N_PASOS) paso = 0;
                modo = MODO_AUTO; fase = F_SUBE; t_fase = 0;
                break;
            case C_RESET:
                for (int m = 0; m < CANT_MOTORES; m++) marcar(m, R_PENDIENTE);
                paso = 0; modo = MODO_AUTO; fase = F_SUBE; t_fase = 0;
                break;
            case C_PAUSA:
                modo = MODO_PAUSA; motores_frenar(); publicar_estado(MODO_PAUSA, -1, 0, 0);
                break;
            case C_SIGUIENTE:
                if (modo == MODO_AUTO) {
                    int m = paso / 2;
                    if (leer_resultado(m) == R_PROBANDO) marcar(m, R_PENDIENTE);
                    paso = (m + 1) * 2; fase = F_SUBE; t_fase = 0; aplicar_duty(-1, 0);
                    if (paso >= N_PASOS) {   // era el último motor: ciclo terminado
                        modo = MODO_PAUSA; motores_frenar(); publicar_estado(MODO_PAUSA, -1, 0, 0);
                        taskENTER_CRITICAL(&estado_mux); estado.ciclo++; taskEXIT_CRITICAL(&estado_mux);
                    }
                }
                break;
            case C_SET:
                modo = MODO_MANUAL; ms_manual = c.ms;
                aplicar_duty(c.motor, c.valor);
                publicar_estado(MODO_MANUAL, c.motor, c.valor > 0 ? 1 : c.valor < 0 ? -1 : 0, c.valor);
                break;
            case C_SETALL:
                modo = MODO_MANUAL; ms_manual = c.ms;
                aplicar_duties(c.duties);
                publicar_estado(MODO_MANUAL, -1, 0, 0);
                break;
            case C_VMAX:
                for (int m = 0; m < CANT_MOTORES; m++) {
                    if (c.motor == -1 || c.motor == m) vmax[m] = c.valor;
                }
                taskENTER_CRITICAL(&estado_mux); memcpy(estado.vmax, vmax, sizeof(vmax)); taskEXIT_CRITICAL(&estado_mux);
                break;
            case C_OK:    marcar(c.motor, R_OK);    break;
            case C_FALLO: marcar(c.motor, R_FALLO); break;
            }
        }

        // ---- arranque automático diferido ----
        if (ms_arranque > 0) {
            ms_arranque -= TICK_MS;
            if (ms_arranque <= 0 && modo == MODO_PAUSA) {
                modo = MODO_AUTO; paso = 0; fase = F_SUBE; t_fase = 0;
            }
        }

        // ---- manual: mantiene el duty hasta que se acabe el tiempo ----
        if (modo == MODO_MANUAL) {
            ms_manual -= TICK_MS;
            if (ms_manual <= 0) {
                modo = MODO_PAUSA; motores_frenar(); publicar_estado(MODO_PAUSA, -1, 0, 0);
            } else {
                motores_alimentar();   // sin esto el failsafe frena a los 300 ms
            }
        }

        // ---- ciclo automático ----
        if (modo == MODO_AUTO) {
            int motor = paso / 2;
            int dir = (paso % 2 == 0) ? +1 : -1;
            float duty = 0.0f;
            t_fase += TICK_MS;

            if (fase == F_SUBE) {
                if (leer_resultado(motor) == R_PENDIENTE) marcar(motor, R_PROBANDO);
                duty = dir * vmax[motor] * fminf((float)t_fase / CONFIG_PROMETEO_TEST_RAMP_MS, 1.0f);
                if (t_fase >= CONFIG_PROMETEO_TEST_RAMP_MS) { fase = F_MANTIENE; t_fase = 0; }
            } else if (fase == F_MANTIENE) {
                duty = dir * vmax[motor];
                if (t_fase >= CONFIG_PROMETEO_TEST_HOLD_MS) { fase = F_BAJA; t_fase = 0; }
            } else if (fase == F_BAJA) {
                duty = dir * vmax[motor] * fmaxf(1.0f - (float)t_fase / RAMPA_BAJA_MS, 0.0f);
                if (t_fase >= RAMPA_BAJA_MS) { fase = F_PAUSA; t_fase = 0; }
            } else {   // F_PAUSA
                if (t_fase >= CONFIG_PROMETEO_TEST_PAUSE_MS) {
                    // terminó adelante y atrás de este motor
                    if (dir == -1 && leer_resultado(motor) == R_PROBANDO) marcar(motor, R_PROBADO);
                    paso++; fase = F_SUBE; t_fase = 0;
                    if (paso >= N_PASOS) {   // ciclo completo
                        modo = MODO_PAUSA; motores_frenar();
                        taskENTER_CRITICAL(&estado_mux); estado.ciclo++; taskEXIT_CRITICAL(&estado_mux);
                        ESP_LOGI(TAG, "Ciclo completo. 'auto' para repetir, 'ok X'/'fallo X' para marcar");
                    }
                }
            }

            if (modo == MODO_AUTO) {
                aplicar_duty(motor, duty);
                publicar_estado(MODO_AUTO, motor, fase == F_PAUSA ? 0 : dir, duty);
            } else {
                publicar_estado(MODO_PAUSA, -1, 0, 0);
            }
        }

        vTaskDelayUntil(&t, pdMS_TO_TICKS(TICK_MS));
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
static rcl_publisher_t pub_estado;
static rcl_subscription_t sub_cmd;
static rcl_timer_t timer;
static rclc_executor_t executor;
static std_msgs__msg__String msg_estado;
static std_msgs__msg__String msg_cmd;
static char buf_json[JSON_LEN];
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
        return;
    }
    if (xQueueSend(cola_cmd, &c, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Cola de comandos llena: '%s' descartado", txt);
    }
}

static void timer_callback(rcl_timer_t *t, int64_t last_call_time)
{
    (void)last_call_time;
    if (t == NULL) return;

    estado_t e;
    taskENTER_CRITICAL(&estado_mux);
    e = estado;
    taskEXIT_CRITICAL(&estado_mux);

    const char *accion = e.dir > 0 ? "adelante" : e.dir < 0 ? "atras" : "detenido";
    char motor[8];
    if (e.motor >= 0 && e.motor < CANT_MOTORES) snprintf(motor, sizeof(motor), "\"%c\"", NOMBRES[e.motor]);
    else                                         snprintf(motor, sizeof(motor), "null");

    int n = snprintf(buf_json, sizeof(buf_json),
        "{\"modo\":\"%s\",\"motor\":%s,\"accion\":\"%s\",\"duty\":%.2f,\"duties\":[%.2f,%.2f,%.2f,%.2f],\"vmax\":{\"A\":%.2f,\"B\":%.2f,\"C\":%.2f,\"D\":%.2f},\"ciclo\":%d,"
        "\"resultados\":{\"A\":\"%s\",\"B\":\"%s\",\"C\":\"%s\",\"D\":\"%s\"},\"failsafe\":%s}",
        MODO_TXT[e.modo], motor, accion, e.duty,
        e.duties[0], e.duties[1], e.duties[2], e.duties[3],
        e.vmax[0], e.vmax[1], e.vmax[2], e.vmax[3], e.ciclo,
        R_TXT[e.resultado[0]], R_TXT[e.resultado[1]], R_TXT[e.resultado[2]], R_TXT[e.resultado[3]],
        motores_en_falla() ? "true" : "false");
    if (n < 0 || n >= (int)sizeof(buf_json)) return;

    msg_estado.data.data = buf_json;
    msg_estado.data.size = (size_t)n;
    msg_estado.data.capacity = sizeof(buf_json);
    RCSOFTCHECK(rcl_publish(&pub_estado, &msg_estado, NULL));
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
    RCRET(rclc_subscription_init_default(&sub_cmd, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motores_test/cmd"));
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
    (void) rcl_publisher_fini(&pub_estado, &node);
    (void) rcl_timer_fini(&timer);
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
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

    // Buffers fijos: no se pide memoria dinámica para los mensajes
    msg_cmd.data.data = buf_cmd;
    msg_cmd.data.size = 0;
    msg_cmd.data.capacity = sizeof(buf_cmd);
    msg_estado.data.data = buf_json;
    msg_estado.data.size = 0;
    msg_estado.data.capacity = sizeof(buf_json);

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
                ESP_LOGI(TAG, "CONECTADO. Publicando /motores_test/estado, escuchando /motores_test/cmd");
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
            xQueueSend(cola_cmd, &pausa, 0);     // sin comandos no se sigue probando
            destruir_entidades();
            st = ESPERANDO_AGENTE;
            break;
        }
        }
    }
}

void app_main(void)
{
    ESP_LOGW(TAG, "=== FIRMWARE DE TEST DE MOTORES === (levantar las ruedas del piso)");

    cola_cmd = xQueueCreate(8, sizeof(cmd_t));
    configASSERT(cola_cmd != NULL);
    for (int m = 0; m < CANT_MOTORES; m++) estado.vmax[m] = CONFIG_PROMETEO_TEST_DUTY_MAX_PCT / 100.0f;
    estado.motor = -1;

    motores_init();     // arranca frenado
    for (int m = 0; m < CANT_MOTORES; m++) {
        ESP_LOGI(TAG, "Motor %c = %s", NOMBRES[m], DESCRIPCION[m]);
    }

    xTaskCreate(test_task, "test_task", TEST_TASK_STACK, NULL, TEST_TASK_PRIO, NULL);

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   // sin ahorro de energía: menos latencia

    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
}
