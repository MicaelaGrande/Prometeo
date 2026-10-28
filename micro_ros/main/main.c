#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"

// Nuestras librerías de motores y sensores
#include "motores.h"
#include "sensores.h"
#include "mg90s.h"

// Librerías base de micro-ROS
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// Stack y prioridad de las tareas
#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5

#define PING_PERIODO_MS 1000   // cada cuánto se chequea que el agente siga vivo
#define DIST_PERIODO_MS 200    // cada cuánto se publica /distancias (5 Hz)

// ===== SERVOS =====
// Todos los servos se mueven a la misma velocidad. AJUSTAR probando.
#define SERVO_VEL_GPS 60.0f    // grados por segundo

// Qué servo físico es cada articulación (AJUSTAR según el cableado)
#define SERVO_CAMARA  SERVO_1  // giro izq/der de la cámara
#define SERVO_BASE    SERVO_2  // giro izq/der del brazo
#define SERVO_HOMBRO  SERVO_3
#define SERVO_CODO    SERVO_4
#define SERVO_GARRA   SERVO_5

// Orden de los valores en /brazo/cmd: [base, hombro, codo, garra]
#define BRAZO_JUNTAS 4
static const servo_id_t brazo[BRAZO_JUNTAS] = { SERVO_BASE, SERVO_HOMBRO, SERVO_CODO, SERVO_GARRA };

static const char *TAG = "microROS_motores";

static const char *volatile estado_actual = "ESPERANDO ORDENES";

// =========================================================
// OBJETOS DE MICRO-ROS
// =========================================================
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t *rmw_options;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_publisher_t pub_dist;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_camara;
static rcl_subscription_t sub_brazo;
static rcl_timer_t timer;
static rcl_timer_t timer_dist;
static rclc_executor_t executor;
static std_msgs__msg__String msg_estado;
static std_msgs__msg__Float32MultiArray msg_dist;
static float buf_dist[NUM_SENSORS];
static geometry_msgs__msg__Twist msg_cmd_vel;
static std_msgs__msg__Float32 msg_camara;
static std_msgs__msg__Float32MultiArray msg_brazo;
static float buf_brazo[BRAZO_JUNTAS];

#define RCRET(fn) { if ((fn) != RCL_RET_OK) { \
    ESP_LOGE(TAG, "Fallo en linea %d", __LINE__); return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { \
    ESP_LOGW(TAG, "Fallo en linea %d: %d. Continuando.", __LINE__, (int)temp_rc); } }

// =========================================================
// LLEGA UNA ORDEN POR /cmd_vel
// =========================================================
static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

    float avance = (float)m->linear.x;    // + adelante, - atrás
    float giro   = (float)m->angular.z;   // + izquierda, - derecha

    // Cinemática diferencial (versión simple, valores -1..1)
    float izq = avance - giro;
    float der = avance + giro;

    motores_set(izq, der);   // la librería limita a -1..1 y alimenta el failsafe
    estado_actual = "MOVIENDO";
}

// =========================================================
// LLEGA UNA ORDEN POR /camara/cmd: ángulo de la cámara (grados)
// =========================================================
static void camara_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;

    if (!isnan(m->data)) {
        servo_mover(SERVO_CAMARA, m->data, SERVO_VEL_GPS);   // la librería limita al rango
    }
}

// =========================================================
// LLEGA UNA ORDEN POR /brazo/cmd: [base, hombro, codo, garra] en grados
// (NaN = dejar esa articulación como está)
// =========================================================
static void brazo_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;

    if (m->data.size != BRAZO_JUNTAS) {
        ESP_LOGW(TAG, "/brazo/cmd: se esperaban %d valores y llegaron %d",
                 BRAZO_JUNTAS, (int)m->data.size);
        return;
    }
    for (int i = 0; i < BRAZO_JUNTAS; i++) {
        if (!isnan(m->data.data[i])) {
            servo_mover(brazo[i], m->data.data[i], SERVO_VEL_GPS);
        }
    }
}

// =========================================================
// PUBLICAR ESTADO cada 500 ms
// =========================================================
static void timer_callback(rcl_timer_t * t, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (t != NULL) {
        const char *estado = motores_en_falla() ? "FAILSAFE" : estado_actual;
        msg_estado.data.data = (char *)estado;
        msg_estado.data.size = strlen(estado);
        msg_estado.data.capacity = msg_estado.data.size + 1;
        RCSOFTCHECK(rcl_publish(&publisher, &msg_estado, NULL));
    }
}

// =========================================================
// PUBLICAR DISTANCIAS cada 200 ms: [izq, der, frente] en cm
// (-1 = sin eco, 400 = lejos)
// =========================================================
static void dist_timer_callback(rcl_timer_t * t, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (t != NULL) {
        sensores_leer(buf_dist);
        RCSOFTCHECK(rcl_publish(&pub_dist, &msg_dist, NULL));
    }
}

// =========================================================
// CREAR / DESTRUIR micro-ROS
// =========================================================
static bool crear_entidades(void)
{
    RCRET(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCRET(rclc_node_init_default(&node, "nodo_motores_esp32", "", &support));

    RCRET(rclc_publisher_init_default(&publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "estado_motores"));

    RCRET(rclc_publisher_init_default(&pub_dist, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "distancias"));

    RCRET(rclc_subscription_init_default(&sub_cmd_vel, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    RCRET(rclc_subscription_init_default(&sub_camara, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "camara/cmd"));

    RCRET(rclc_subscription_init_default(&sub_brazo, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "brazo/cmd"));

    RCRET(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), timer_callback));
    RCRET(rclc_timer_init_default(&timer_dist, &support, RCL_MS_TO_NS(DIST_PERIODO_MS),
          dist_timer_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCRET(rclc_executor_init(&executor, &support.context, 5, &allocator));  // 2 timers + 3 suscripciones
    RCRET(rclc_executor_add_timer(&executor, &timer));
    RCRET(rclc_executor_add_timer(&executor, &timer_dist));
    RCRET(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
          &cmd_vel_callback, ON_NEW_DATA));
    RCRET(rclc_executor_add_subscription(&executor, &sub_camara, &msg_camara,
          &camara_callback, ON_NEW_DATA));
    RCRET(rclc_executor_add_subscription(&executor, &sub_brazo, &msg_brazo,
          &brazo_callback, ON_NEW_DATA));
    return true;
}

static void destruir_entidades(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void) rcl_subscription_fini(&sub_cmd_vel, &node);
    (void) rcl_subscription_fini(&sub_camara, &node);
    (void) rcl_subscription_fini(&sub_brazo, &node);
    (void) rcl_publisher_fini(&publisher, &node);
    (void) rcl_publisher_fini(&pub_dist, &node);
    (void) rcl_timer_fini(&timer);
    (void) rcl_timer_fini(&timer_dist);
    (void) rclc_executor_fini(&executor);
    (void) rcl_node_fini(&node);
    (void) rclc_support_fini(&support);
}

// =========================================================
// TAREA DE MICRO-ROS con reconexión
// =========================================================
typedef enum {
    ESPERANDO_AGENTE,
    AGENTE_DISPONIBLE,
    CONECTADO,
    DESCONECTADO
} estado_uros_t;

static void micro_ros_task(void * arg)
{
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

    // El mensaje de distancias usa un buffer fijo: no se pide memoria dinámica
    msg_dist.data.data = buf_dist;
    msg_dist.data.size = NUM_SENSORS;
    msg_dist.data.capacity = NUM_SENSORS;

    // El mensaje de /brazo/cmd también usa un buffer fijo (layout.dim queda vacío:
    // quien publique no debe llenar el layout)
    msg_brazo.data.data = buf_brazo;
    msg_brazo.data.size = 0;
    msg_brazo.data.capacity = BRAZO_JUNTAS;

    estado_uros_t estado = ESPERANDO_AGENTE;
    int ms_desde_ping = 0;

    while (1) {
        switch (estado) {

        case ESPERANDO_AGENTE:
            if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK) {
                ESP_LOGI(TAG, "Agente encontrado");
                estado = AGENTE_DISPONIBLE;
            } else {
                ESP_LOGW(TAG, "Buscando agente en %s:%s ...",
                         CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case AGENTE_DISPONIBLE:
            if (crear_entidades()) {
                ESP_LOGI(TAG, "CONECTADO. Escuchando /cmd_vel, /camara/cmd y /brazo/cmd, publicando /distancias");
                estado = CONECTADO;
                ms_desde_ping = 0;
            } else {
                destruir_entidades();
                estado = ESPERANDO_AGENTE;
            }
            break;

        case CONECTADO:
            if (ms_desde_ping >= PING_PERIODO_MS) {
                ms_desde_ping = 0;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    estado = DESCONECTADO;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            vTaskDelay(pdMS_TO_TICKS(10));
            ms_desde_ping += 20;
            break;

        case DESCONECTADO:
            ESP_LOGE(TAG, "Agente PERDIDO -> FRENO y reconexion");
            motores_frenar();
            servos_detener();   // los servos se quedan donde están
            destruir_entidades();
            estado_actual = "ESPERANDO ORDENES";
            estado = ESPERANDO_AGENTE;
            break;
        }
    }
}

void app_main(void)
{
    // 1. Iniciar la librería de motores (arranca frenado)
    motores_init();

    // 1b. Iniciar los servos (van a su posición home sin saltos)
    servos_init();

    // 2. Iniciar los sensores (miden en su propia tarea, aunque no haya agente)
    ESP_ERROR_CHECK(sensores_init());

    // 3. Conectarse al WiFi
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    // Sin ahorro de energía: si no, la radio duerme ~300 ms entre escuchas,
    // los /cmd_vel llegan amontonados, salta el failsafe y se "pierde" el agente
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // 4. Iniciar micro-ROS
    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
}
