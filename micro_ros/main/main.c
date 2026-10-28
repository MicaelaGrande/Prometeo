#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"

// Nuestras librerías de motores, sensores y freno de emergencia
#include "motores.h"
#include "sensores.h"
#include "seguridad.h"

// Librerías base de micro-ROS
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// Stack y prioridad de las tareas
#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5

#define PING_PERIODO_MS 1000   // cada cuánto se chequea que el agente siga vivo

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
static rcl_publisher_t pub_freno;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_seguridad;
static rcl_timer_t timer;
static rclc_executor_t executor;
static std_msgs__msg__String msg_estado;
static std_msgs__msg__Float32MultiArray msg_dist;
static float buf_dist[NUM_SENSORS];
static uint32_t ultima_medicion = 0;   // última medición publicada en /distancias
static std_msgs__msg__Bool msg_freno;
static int freno_publicado = -1;       // último valor publicado en /freno_emergencia (-1 = ninguno)
static std_msgs__msg__Bool msg_seguridad;
static geometry_msgs__msg__Twist msg_cmd_vel;

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

    // + adelante, - atrás. Pasa por el freno de emergencia: si hay pared cerca, 0
    float avance = seguridad_filtrar((float)m->linear.x);
    float giro   = (float)m->angular.z;   // + izquierda, - derecha

    // Cinemática diferencial (versión simple, valores -1..1)
    float izq = avance - giro;
    float der = avance + giro;

    motores_set(izq, der);   // la librería limita a -1..1 y alimenta el failsafe
    estado_actual = "MOVIENDO";
}

// =========================================================
// LLEGA /seguridad_habilitada: false = modo recoger (sin freno)
// =========================================================
static void seguridad_callback(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    seguridad_habilitar(m->data);
}

// =========================================================
// PUBLICAR ESTADO cada 500 ms
// =========================================================
static void timer_callback(rcl_timer_t * t, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (t != NULL) {
        const char *estado = motores_en_falla()        ? "FAILSAFE"
                           : !seguridad_habilitada()   ? "SEGURIDAD OFF"
                           : seguridad_freno_activo()  ? "FRENO EMERGENCIA"
                           : estado_actual;
        msg_estado.data.data = (char *)estado;
        msg_estado.data.size = strlen(estado);
        msg_estado.data.capacity = msg_estado.data.size + 1;
        RCSOFTCHECK(rcl_publish(&publisher, &msg_estado, NULL));
    }
}

// =========================================================
// PUBLICAR DISTANCIAS apenas hay una medición nueva (~cada 120 ms):
// [izq, der, frente] en cm (-1 = sin eco, 400 = lejos)
// Se llama desde micro_ros_task: rcl no se puede usar desde otra tarea.
// =========================================================
static void publicar_distancias_si_hay_nuevas(void)
{
    uint32_t n = sensores_leer(buf_dist);
    if (n != ultima_medicion) {
        ultima_medicion = n;
        RCSOFTCHECK(rcl_publish(&pub_dist, &msg_dist, NULL));
    }
}

// =========================================================
// PUBLICAR /freno_emergencia solo cuando cambia (y una vez al conectar)
// =========================================================
static void publicar_freno_si_cambio(void)
{
    int activo = seguridad_freno_activo();
    if (activo != freno_publicado) {
        freno_publicado = activo;
        msg_freno.data = activo;
        RCSOFTCHECK(rcl_publish(&pub_freno, &msg_freno, NULL));
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

    // Best effort en /distancias y /cmd_vel: si se pierde un paquete no se reenvía,
    // porque el siguiente ya trae el dato nuevo. En ROS, escuchar /distancias con
    // QoS best effort (qos_profile_sensor_data) o no llega nada.
    RCRET(rclc_publisher_init_best_effort(&pub_dist, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "distancias"));

    RCRET(rclc_subscription_init_best_effort(&sub_cmd_vel, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    // Reliable: son eventos poco frecuentes que no se pueden perder
    RCRET(rclc_publisher_init_default(&pub_freno, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "freno_emergencia"));

    RCRET(rclc_subscription_init_default(&sub_seguridad, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "seguridad_habilitada"));

    RCRET(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), timer_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCRET(rclc_executor_init(&executor, &support.context, 3, &allocator));  // timer + 2 suscripciones
    RCRET(rclc_executor_add_timer(&executor, &timer));
    RCRET(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
          &cmd_vel_callback, ON_NEW_DATA));
    RCRET(rclc_executor_add_subscription(&executor, &sub_seguridad, &msg_seguridad,
          &seguridad_callback, ON_NEW_DATA));

    freno_publicado = -1;   // al (re)conectar se publica el estado actual
    return true;
}

static void destruir_entidades(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void) rcl_subscription_fini(&sub_cmd_vel, &node);
    (void) rcl_subscription_fini(&sub_seguridad, &node);
    (void) rcl_publisher_fini(&publisher, &node);
    (void) rcl_publisher_fini(&pub_dist, &node);
    (void) rcl_publisher_fini(&pub_freno, &node);
    (void) rcl_timer_fini(&timer);
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
                ESP_LOGI(TAG, "CONECTADO. Escuchando /cmd_vel, publicando /distancias");
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
            publicar_distancias_si_hay_nuevas();
            publicar_freno_si_cambio();
            vTaskDelay(pdMS_TO_TICKS(10));
            ms_desde_ping += 20;
            break;

        case DESCONECTADO:
            ESP_LOGE(TAG, "Agente PERDIDO -> FRENO y reconexion");
            motores_frenar();
            seguridad_habilitar(true);   // sin ROS nadie puede pedir modo recoger: freno activo
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

    // 2. Iniciar los sensores (miden en su propia tarea, aunque no haya agente)
    ESP_ERROR_CHECK(sensores_init());

    // 2b. Freno de emergencia: vigila el FRENTE aunque no haya agente
    ESP_ERROR_CHECK(seguridad_init());

    // 3. Conectarse al WiFi
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    // Sin ahorro de energía: si no, la radio duerme ~300 ms entre escuchas,
    // los /cmd_vel llegan amontonados, salta el failsafe y se "pierde" el agente
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // 4. Iniciar micro-ROS
    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
}
