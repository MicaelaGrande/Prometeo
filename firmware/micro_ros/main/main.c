#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

// Drivers (drivers/) y logica del robot (logica/)
#include "motores.h"
#include "mg90s.h"
#include "encoder.h"
#include "sensores.h"
#include "seguridad.h"
#include "centrado.h"
#include "brazo.h"
#include "ruedas.h"
#include "red.h"

// Librerías base de micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// Stack y prioridad de las tareas
#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5

#define ENTIDAD_TIMEOUT_MS 5000   // espera máxima del agente al crear cada entidad
#define PING_PERIODO_MS 1000  // cada cuánto se chequea que el agente siga vivo

static const char *TAG = "prometeo_body";

static const char *volatile estado_actual = "ESPERANDO ORDENES";

#define ROSDOMAIN_ID 33

// IP donde se busca/usa el agente. Con PROMETEO_AGENTE_AUTO se reemplaza por la que responda.
static char micro_ros_agent_ip_buf[16] = CONFIG_MICRO_ROS_AGENT_IP;
char * micro_ros_agent_ip = micro_ros_agent_ip_buf;

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
static rcl_publisher_t pub_vel;
static rcl_publisher_t pub_servos;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_seguridad;
static rcl_subscription_t sub_liberar;
static rcl_subscription_t sub_camara;
static rcl_subscription_t sub_brazo;
static rcl_timer_t timer;
static rclc_executor_t executor;
static std_msgs__msg__String msg_estado;
static std_msgs__msg__Float32MultiArray msg_dist;
static float buf_dist[NUM_SENSORS];
static uint32_t ultima_medicion = 0;   // última medición publicada en /distancias
static std_msgs__msg__Bool msg_freno;
static geometry_msgs__msg__Twist msg_vel;                 // velocidad estimada (lazo abierto)
static std_msgs__msg__Float32MultiArray msg_servos;      // ángulos actuales, en orden SERVO_1..SERVO_5
static float buf_servos[CANT_SERVOS];
static int freno_publicado = -1;       // último valor publicado en /freno_emergencia (-1 = ninguno)
static std_msgs__msg__Bool msg_seguridad;
static std_msgs__msg__Empty msg_liberar;
static geometry_msgs__msg__Twist msg_cmd_vel;
static std_msgs__msg__Float32 msg_camara;
static std_msgs__msg__Float32MultiArray msg_brazo;
static float buf_brazo[BRAZO_JUNTAS];

// /velocidad_ruedas: [vel_izq m/s, vel_der m/s, rpm_izq, rpm_der]
#define VEL_RUEDAS_LEN RUEDAS_TELEMETRIA_LEN
static rcl_publisher_t pub_vel_ruedas;
static rcl_timer_t timer_ruedas;
static std_msgs__msg__Float32MultiArray msg_vel_ruedas;
static float buf_vel_ruedas[VEL_RUEDAS_LEN];

static void log_heap(const char *donde)
{
    ESP_LOGI(TAG, "HEAP %s: libre=%u min=%u mayor_bloque=%u",
             donde,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

#define RCRET(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { \
    ESP_LOGE(TAG, "Fallo en linea %d: rc=%d", __LINE__, (int)temp_rc); log_heap("al fallar"); return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { \
    ESP_LOGW(TAG, "Fallo en linea %d: %d. Continuando.", __LINE__, (int)temp_rc); } }

// =========================================================
// LLEGA UNA ORDEN POR /cmd_vel
// =========================================================
static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

    // Un NaN/inf en el objetivo se queda pegado en la rampa y deja los motores muertos
    if (!isfinite(m->linear.x) || !isfinite(m->angular.z)) {
        ESP_LOGW(TAG, "/cmd_vel con NaN/inf: ignorado");
        return;
    }

    // linear.x en m/s (+ adelante), angular.z en rad/s (+ izquierda): igual que Gazebo.
    // El freno de emergencia usa el avance normalizado (-1..1) y devuelve 0 si avanzar es peligroso.
    // Solo bloquea el avance: girar y retroceder pasan siempre.
    float lineal = (float)m->linear.x;
    float avance_norm = seguridad_filtrar(lineal / (CONFIG_PROMETEO_VEL_MAX_MM_S / 1000.0f));
    if (lineal > 0.0f && avance_norm <= 0.0f) {
        lineal = 0.0f;
    }
    // Centrado en el pasillo: si avanza y nadie pide girar, suma el giro que lo deja en el medio
    float angular = (float)m->angular.z;
    angular += centrado_corregir(lineal, angular);
    // Cada rueda recibe su propia velocidad (m/s). Siempre las 4 ruedas; al girar
    // el giro va reforzado (PROMETEO_TURN_BOOST_PCT). El PID de encoders de motores.c
    // ajusta el duty de cada motor por separado hasta que cada rueda gire a su velocidad pedida.
    // motores_set_ruedas() también alimenta el failsafe.
    float v_rueda[CANT_MOTORES];
    motores_twist_a_ruedas(lineal, angular, v_rueda);
    motores_set_ruedas(v_rueda);
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
// LLEGA /motores/liberar: suelta lo que dejó frenados a los motores
//  - fallas latcheadas (atasco / sobreesfuerzo del puente), con motores_liberar_fallas()
//  - freno de emergencia por pared, con seguridad_liberar(): NO lo suelta mientras el
//    frente siga demasiado cerca; retroceder y girar siempre se pueden
// =========================================================
static void liberar_callback(const void *msgin)
{
    (void) msgin;
    bool fallas_ok = motores_liberar_fallas();
    bool freno_ok  = seguridad_liberar();
    ESP_LOGW(TAG, "Liberar: fallas %s, freno de pared %s",
             fallas_ok ? "liberadas" : "puente todavia caliente",
             freno_ok  ? "libre"     : "SIGUE (frente muy cerca)");
}

// =========================================================
// LLEGA UNA ORDEN POR /camara/cmd: ángulo de la cámara (grados)
// =========================================================
static void camara_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;

    camara_mover(m->data);
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
    brazo_mover(m->data.data);
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
// PUBLICAR VELOCIDAD DE LAS RUEDAS cada ENC_PERIODO_MS (lo mismo que tarda el
// driver en recalcularla); ver ruedas_leer() en logica/ruedas.c.
// =========================================================

static void timer_ruedas_callback(rcl_timer_t * t, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (t != NULL) {
        ruedas_leer(buf_vel_ruedas);
        RCSOFTCHECK(rcl_publish(&pub_vel_ruedas, &msg_vel_ruedas, NULL));
    }
}

// =========================================================
// PUBLICAR DISTANCIAS apenas hay una medición nueva (~cada 60-85 ms):
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
// PUBLICAR /velocidad y /servos/estado cada TELEMETRIA_PERIODO_MS
// velocidad: ESTIMADA a lazo abierto desde el duty de los motores (no hay encoders)
// servos/estado: ángulos actuales [camara, base, hombro, codo, garra] (SERVO_1..SERVO_5)
// =========================================================
#define TELEMETRIA_PERIODO_MS 200

static void publicar_telemetria_si_toca(void)
{
    static TickType_t ultima = 0;
    TickType_t ahora = xTaskGetTickCount();
    if ((ahora - ultima) < pdMS_TO_TICKS(TELEMETRIA_PERIODO_MS)) {
        return;
    }
    ultima = ahora;

    float v, w;
    motores_get_velocidad(&v, &w);
    msg_vel.linear.x  = v;
    msg_vel.angular.z = w;
    RCSOFTCHECK(rcl_publish(&pub_vel, &msg_vel, NULL));

    for (int s = 0; s < CANT_SERVOS; s++) {
        buf_servos[s] = servo_get_actual((servo_id_t)s);
    }
    RCSOFTCHECK(rcl_publish(&pub_servos, &msg_servos, NULL));
}

// =========================================================
// CREAR / DESTRUIR micro-ROS
// =========================================================
static bool crear_entidades(void)
{
    log_heap("PRE");
    RCRET(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    // Por WiFi cada entidad espera la respuesta del agente: el default (1000 ms) no alcanza
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_creation_session_timeout(rmw_context, ENTIDAD_TIMEOUT_MS);

    RCRET(rclc_node_init_default(&node, "body", "prometeo", &support));

    RCRET(rclc_publisher_init_default(&publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "estado_motores"));

    // Best effort en /distancias y /cmd_vel: si se pierde un paquete no se reenvía,
    // porque el siguiente ya trae el dato nuevo. En ROS, escuchar /distancias con
    // QoS best effort (qos_profile_sensor_data) o no llega nada.
    RCRET(rclc_publisher_init_best_effort(&pub_dist, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "distancias"));

    // Igual que /distancias: escuchar con QoS best effort
    RCRET(rclc_publisher_init_best_effort(&pub_vel_ruedas, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "velocidad_ruedas"));

    RCRET(rclc_subscription_init_best_effort(&sub_cmd_vel, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    // Reliable: son eventos poco frecuentes que no se pueden perder
    RCRET(rclc_publisher_init_default(&pub_freno, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "freno_emergencia"));

    // Telemetría periódica: best effort, la siguiente reemplaza a la que se pierda
    RCRET(rclc_publisher_init_best_effort(&pub_vel, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "velocidad"));
    RCRET(rclc_publisher_init_best_effort(&pub_servos, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "servos/estado"));

    RCRET(rclc_subscription_init_default(&sub_seguridad, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "seguridad_habilitada"));
    RCRET(rclc_subscription_init_default(&sub_liberar, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "motores/liberar"));
    RCRET(rclc_subscription_init_default(&sub_camara, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "camara/cmd"));

    RCRET(rclc_subscription_init_default(&sub_brazo, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "brazo/cmd"));

    RCRET(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(500), timer_callback, true));
    RCRET(rclc_timer_init_default2(&timer_ruedas, &support, RCL_MS_TO_NS(ENC_PERIODO_MS), timer_ruedas_callback, true));

    executor = rclc_executor_get_zero_initialized_executor();
    RCRET(rclc_executor_init(&executor, &support.context, 7, &allocator));  // 2 timers + 5 suscripciones
    RCRET(rclc_executor_add_timer(&executor, &timer));
    RCRET(rclc_executor_add_timer(&executor, &timer_ruedas));
    RCRET(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
          &cmd_vel_callback, ON_NEW_DATA));
    RCRET(rclc_executor_add_subscription(&executor, &sub_seguridad, &msg_seguridad,
          &seguridad_callback, ON_NEW_DATA));
    RCRET(rclc_executor_add_subscription(&executor, &sub_liberar, &msg_liberar,
          &liberar_callback, ON_NEW_DATA));
    RCRET(rclc_executor_add_subscription(&executor, &sub_camara, &msg_camara,
          &camara_callback, ON_NEW_DATA));
    RCRET(rclc_executor_add_subscription(&executor, &sub_brazo, &msg_brazo,
          &brazo_callback, ON_NEW_DATA));

    log_heap("POST");
    freno_publicado = -1;   // al (re)conectar se publica el estado actual
    return true;
}

static void destruir_entidades(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_subscription_fini(&sub_cmd_vel, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_seguridad, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_liberar, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_camara, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_brazo, &node));
    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_dist, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_freno, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_vel_ruedas, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_vel, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_servos, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rcl_timer_fini(&timer_ruedas));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
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
    RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
    RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, ROSDOMAIN_ID));   // mismo ROS_DOMAIN_ID que el agente y la PC
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(micro_ros_agent_ip, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

    // El mensaje de distancias usa un buffer fijo: no se pide memoria dinámica
    msg_dist.data.data = buf_dist;
    msg_dist.data.size = NUM_SENSORS;
    msg_dist.data.capacity = NUM_SENSORS;

    msg_vel_ruedas.data.data = buf_vel_ruedas;
    msg_vel_ruedas.data.size = VEL_RUEDAS_LEN;
    msg_vel_ruedas.data.capacity = VEL_RUEDAS_LEN;
    msg_servos.data.data = buf_servos;
    msg_servos.data.size = CANT_SERVOS;
    msg_servos.data.capacity = CANT_SERVOS;

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
                         micro_ros_agent_ip, CONFIG_MICRO_ROS_AGENT_PORT);
#ifdef CONFIG_PROMETEO_AGENTE_AUTO
                if (red_buscar_agente(rmw_options, micro_ros_agent_ip_buf, sizeof(micro_ros_agent_ip_buf))) {
                    ESP_LOGI(TAG, "Agente encontrado por barrida en %s", micro_ros_agent_ip);
                    estado = AGENTE_DISPONIBLE;
                    break;
                }
                // la barrida dejó cambiada la dirección: volver a la última conocida
                rmw_uros_options_set_udp_address(micro_ros_agent_ip, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
#endif
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case AGENTE_DISPONIBLE:
            if (crear_entidades()) {
                ESP_LOGI(TAG, "CONECTADO. Escuchando /cmd_vel, /camara/cmd y /brazo/cmd, publicando /distancias y /velocidad_ruedas");
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
            publicar_telemetria_si_toca();
            vTaskDelay(pdMS_TO_TICKS(10));
            ms_desde_ping += 20;
            break;

        case DESCONECTADO:
            ESP_LOGE(TAG, "Agente PERDIDO -> FRENO y reconexion");
            motores_frenar();
            seguridad_habilitar(true);   // sin ROS nadie puede pedir modo recoger: freno activo
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
    // 1. Iniciar los encoders (cuentan pulsos por interrupción aunque no haya agente).
    //    Van ANTES que los motores: la protección de atasco de motores.c lee los encoders
    //    desde que arranca la tarea de rampa (ver motores.h, "SEGURIDAD").
    encoders_init();

    // 1a. Iniciar la librería de motores (arranca frenado)
    motores_init();

    // 1b. Iniciar los servos (van a su posición home sin saltos)
    servos_init();

    // 2. Iniciar los sensores (miden en su propia tarea, aunque no haya agente)
    ESP_ERROR_CHECK(sensores_init());

    // 2b. Freno de emergencia: vigila el FRENTE aunque no haya agente
    ESP_ERROR_CHECK(seguridad_init());

    // 3. Conectarse al WiFi (reintenta para siempre; no importa quien arranque primero)
    ESP_ERROR_CHECK(red_wifi_init());

    // 4. Iniciar micro-ROS
    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
}
