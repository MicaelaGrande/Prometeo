#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motores.h"
#include "sensores.h"
#include "seguridad.h"

#define SEGURIDAD_TASK_STACK 3072
#define SEGURIDAD_TASK_PRIO  6     // arriba de micro-ROS (5): el freno no espera al WiFi
#define SEGURIDAD_PERIODO_MS 20

static const char *TAG_SEG = "seguridad";

/* ================== Umbral de freno (cm). Cambiar aca. ================== */

/* Tiempo hasta darse cuenta: ciclo de sensores (120 ms) + esta tarea (20 ms).
 * Con mediana seria 0.26 (un ciclo mas de atraso). */
#define T_REACCION_S   0.14f

/* Velocidad con avance = 1. ESTIMADO (motor TT a 6 V, rueda 65 mm):
 * medir cronometrando cuanto tarda en recorrer 1 m a fondo. */
#define VEL_MAX_CM_S   70.0f

/* Lo que sigue rodando despues de cortar, en "segundos de marcha" (por eso
 * crece con la velocidad). ESTIMADO: medir lanzandolo contra una marca. */
#define T_FRENADO_S    0.08f

/* Ruido del sensor + error de las estimaciones de arriba */
#define MARGEN_CM      1.0f

/* Debajo de ~2,5 cm el HC-SR04 deja de medir bien */
#define UMBRAL_MIN_CM  3.0f

/* Libera recien a umbral + esto, para no frenar/soltar en el borde */
#define HISTERESIS_CM  2.0f

/* Mediciones seguidas con el frente libre para soltar el freno (~120 ms c/u).
 * Frena con 1 lectura (rapido) pero suelta con varias (lento): de cerca el
 * HC-SR04 da lecturas falsas y una sola no tiene que soltarlo */
#define CICLOS_PARA_LIBERAR 3

/* umbral = avance * VEL_MAX_CM_S * (T_REACCION_S + T_FRENADO_S) + MARGEN_CM
 *          (nunca menos que UMBRAL_MIN_CM)
 * Ej: avance 0.2 -> ~4,1 cm | 0.5 -> ~8,7 cm | 1.0 -> ~16,4 cm */

/* ==================================================================== */

/* Los escribe cmd_vel (tarea de micro-ROS) y los lee la tarea de seguridad.
 * Son de 32 bits: se leen y escriben enteros, no hace falta mux. */
static volatile float avance_pedido = 0.0f;
static volatile bool freno_activo = false;
static volatile bool habilitada = true;
static volatile int ciclos_libre = 0;   /* mediciones seguidas con el frente libre */

static float umbral_cm(float avance)
{
    float u = avance * VEL_MAX_CM_S * (T_REACCION_S + T_FRENADO_S) + MARGEN_CM;
    return u > UMBRAL_MIN_CM ? u : UMBRAL_MIN_CM;
}

/* FRENTE del ultimo ciclo, sin mediana: reacciona un ciclo antes.
 * Si num_medicion no es NULL, devuelve ahi el numero de medicion. */
static float frente_cm(uint32_t *num_medicion)
{
    float d[NUM_SENSORS];
    uint32_t n = sensores_leer_crudo(d);
    if (num_medicion != NULL) {
        *num_medicion = n;
    }
    return d[SENSOR_FRENTE];
}

/* Avanzar sin ver (FRENTE sin dato) tambien cuenta como peligro */
static bool hay_peligro(float avance, float frente)
{
    return avance > 0.0f && (frente < 0.0f || frente < umbral_cm(avance));
}

static void activar_freno(float avance, float frente)
{
    motores_frenar();
    freno_activo = true;
    ciclos_libre = 0;
    if (frente < 0.0f) {
        ESP_LOGW(TAG_SEG, "FRENO EMERGENCIA: FRENTE sin dato (avance %.2f)", avance);
    } else {
        ESP_LOGW(TAG_SEG, "FRENO EMERGENCIA: frente %.1f cm < umbral %.1f cm (avance %.2f)",
                 frente, umbral_cm(avance), avance);
    }
}

// Frena si el robot ya viene avanzando y aparece la pared, aunque no llegue
// ningun /cmd_vel nuevo (por ejemplo, si se corto el WiFi).
static void seguridad_task(void *arg)
{
    TickType_t t = xTaskGetTickCount();
    uint32_t ultima_medicion = 0;
    while (1) {
        uint32_t n;
        float frente = frente_cm(&n);
        float avance = avance_pedido;
        /* La tarea gira cada 20 ms pero el sensor mide cada ~120 ms:
         * para soltar se cuentan mediciones nuevas, no vueltas */
        bool medicion_nueva = (n != ultima_medicion);
        ultima_medicion = n;

        if (habilitada && !freno_activo && hay_peligro(avance, frente)) {
            activar_freno(avance, frente);
        } else if (freno_activo && medicion_nueva) {
            if (frente >= 0.0f && frente > umbral_cm(avance) + HISTERESIS_CM) {
                if (++ciclos_libre >= CICLOS_PARA_LIBERAR) {
                    freno_activo = false;
                    ESP_LOGI(TAG_SEG, "Freno liberado: frente %.1f cm", frente);
                }
            } else {
                ciclos_libre = 0;   /* una lectura cerca: empieza a contar de nuevo */
            }
        }

        vTaskDelayUntil(&t, pdMS_TO_TICKS(SEGURIDAD_PERIODO_MS));
    }
}

esp_err_t seguridad_init(void)
{
    if (xTaskCreate(seguridad_task, "seguridad_task", SEGURIDAD_TASK_STACK, NULL,
                    SEGURIDAD_TASK_PRIO, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

float seguridad_filtrar(float avance)
{
    avance_pedido = avance;
    if (!habilitada || avance <= 0.0f) {
        return avance;      // retroceder, quedarse quieto o modo recoger: pasa tal cual
    }
    if (!freno_activo) {
        float frente = frente_cm(NULL);
        if (!hay_peligro(avance, frente)) {
            return avance;
        }
        activar_freno(avance, frente);   // la orden nueva lo mandaria contra la pared
    }
    return 0.0f;            // con freno: no avanza (el giro lo maneja quien llama)
}

bool seguridad_freno_activo(void) { return freno_activo; }

void seguridad_habilitar(bool on)
{
    if (on == habilitada) {
        return;
    }
    habilitada = on;
    if (!on) {
        freno_activo = false;
    }
    ESP_LOGW(TAG_SEG, "%s", on ? "Freno de emergencia ACTIVADO"
                               : "Freno de emergencia DESACTIVADO (modo recoger)");
}

bool seguridad_habilitada(void) { return habilitada; }
