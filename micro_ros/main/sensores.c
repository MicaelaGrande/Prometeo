#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hcsr04.h"
#include "sensores.h"

#define SENSORES_TASK_STACK  4096
#define SENSORES_TASK_PRIO   4     // debajo de micro-ROS: nunca le gana a /cmd_vel

#define LOG_CADA_CICLOS 8      // log de sensores cada ~1 s (cada ciclo dura ~120 ms)

static const char *TAG_SENS = "sensores";

static const char *NOMBRES[NUM_SENSORS] = { "IZQ", "DER", "FRENTE" };

/* ================== Calibracion (cm). Cambiar aca. ================== */

/* 1) Error propio de cada modulo (hardware). Poner un objeto a distancia
 *    conocida (medida con regla) y anotar: offset = crudo - real.
 *    Ej: el monitor muestra "crudo 21.0" y la regla dice 20.0 -> 1.0f */
static float offset_sensor_cm[NUM_SENSORS] = {
    [SENSOR_IZQ]    = 0.0f,
    [SENSOR_DER]    = 0.0f,
    [SENSOR_FRENTE] = 0.0f,
};

/* 2) Distancia desde el frente del sensor hasta el borde del robot
 *    (positivo si el sensor esta metido hacia adentro del robot).
 *    Todavia no definido: dejar en 0 hasta armar el robot. */
static float pos_sensor_cm[NUM_SENSORS] = {
    [SENSOR_IZQ]    = 0.0f,
    [SENSOR_DER]    = 0.0f,
    [SENSOR_FRENTE] = 0.0f,
};

/* distancia final = mediana(crudo) - offset_sensor_cm - pos_sensor_cm */

/* ==================================================================== */

/* Cantidad de lecturas que usa la mediana (impar) */
#define VENTANA 3

/* Ultimas lecturas de cada sensor, para la mediana */
typedef struct {
    float lecturas[VENTANA];
    int cant;       /* cuantas lecturas validas hay guardadas (hasta VENTANA) */
    int prox;       /* posicion donde se escribe la proxima */
} historial_t;

static hcsr04_handle_t sensors[NUM_SENSORS] = { NULL };

/* Ultima distancia de cada sensor: la escribe la tarea de sensores y la lee
 * quien llame a sensores_leer() (el timer de micro-ROS), por eso el mux */
static float distancias_cm[NUM_SENSORS] = { DIST_SIN_DATO, DIST_SIN_DATO, DIST_SIN_DATO };
/* Ultima lectura sin mediana (ya calibrada): la usa el freno de emergencia */
static float crudas_cm[NUM_SENSORS] = { DIST_SIN_DATO, DIST_SIN_DATO, DIST_SIN_DATO };
static uint32_t num_medicion = 0;   /* aumenta con cada juego de distancias nuevo */
static portMUX_TYPE dist_mux = portMUX_INITIALIZER_UNLOCKED;

static void historial_agregar(historial_t *h, float cm)
{
    h->lecturas[h->prox] = cm;
    h->prox = (h->prox + 1) % VENTANA;
    if (h->cant < VENTANA) {
        h->cant++;
    }
}

/* Ordena una copia de las lecturas y devuelve la del medio */
static float historial_mediana(const historial_t *h)
{
    float v[VENTANA];
    for (int i = 0; i < h->cant; i++) {
        v[i] = h->lecturas[i];
    }
    /* Insercion: con 3 valores es lo mas simple */
    for (int i = 1; i < h->cant; i++) {
        float x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = x;
    }
    return v[h->cant / 2];
}

/* Arma el texto de un sensor: distancia final y cruda, o el motivo del error */
static void formatear(char *buf, size_t len, bool hay_dato, esp_err_t err,
                      float crudo, float final)
{
    if (!hay_dato) {
        snprintf(buf, len, "%s", err == ESP_ERR_TIMEOUT ? "sin eco" : esp_err_to_name(err));
    } else if (crudo >= DIST_LEJOS_CM) {
        snprintf(buf, len, "lejos");
    } else {
        snprintf(buf, len, "%5.1f cm (crudo %5.1f)", final, crudo);
    }
}

// Mide sin parar. Casi todo el tiempo está bloqueada esperando el eco (RMT),
// así que no consume CPU. No toca micro-ROS: solo deja los valores en distancias_cm.
static void sensores_task(void *arg)
{
    float cm[NUM_SENSORS] = { 0 };
    esp_err_t errs[NUM_SENSORS];
    historial_t hist[NUM_SENSORS] = { 0 };
    int ciclo = 0;

    while (1) {
        /* Turno 1: los laterales juntos (apuntan en direcciones opuestas, no se interfieren) */
        hcsr04_measure_group_cm(&sensors[SENSOR_IZQ], 2, &cm[SENSOR_IZQ], &errs[SENSOR_IZQ]);
        /* Turno 2: el frente solo, 60 ms despues para que se apaguen los rebotes */
        hcsr04_measure_group_cm(&sensors[SENSOR_FRENTE], 1, &cm[SENSOR_FRENTE], &errs[SENSOR_FRENTE]);

        bool loguear = (++ciclo >= LOG_CADA_CICLOS);
        char txt[NUM_SENSORS][40];
        float nuevas[NUM_SENSORS];
        float crudas[NUM_SENSORS];

        for (int i = 0; i < NUM_SENSORS; i++) {
            /* Lectura de este ciclo, sin mediana, con la misma calibracion y codigos */
            if (errs[i] == ESP_OK && cm[i] < DIST_LEJOS_CM) {
                crudas[i] = cm[i] - offset_sensor_cm[i] - pos_sensor_cm[i];
            } else if (errs[i] == ESP_OK || errs[i] == ESP_ERR_INVALID_RESPONSE) {
                crudas[i] = DIST_LEJOS_CM;
            } else {
                crudas[i] = DIST_SIN_DATO;
            }

            /* Guardar la lectura: valida tal cual, fuera de rango como "lejos".
             * Un timeout (sin eco) no se guarda: es un problema de cableado. */
            if (errs[i] == ESP_OK) {
                historial_agregar(&hist[i], cm[i]);
            } else if (errs[i] == ESP_ERR_INVALID_RESPONSE) {
                historial_agregar(&hist[i], DIST_LEJOS_CM);
            }

            bool hay_dato = hist[i].cant > 0 && errs[i] != ESP_ERR_TIMEOUT;
            float crudo = hay_dato ? historial_mediana(&hist[i]) : 0.0f;
            float final = crudo - offset_sensor_cm[i] - pos_sensor_cm[i];

            if (!hay_dato) {
                nuevas[i] = DIST_SIN_DATO;
            } else if (crudo >= DIST_LEJOS_CM) {
                nuevas[i] = DIST_LEJOS_CM;
            } else {
                nuevas[i] = final;
            }

            if (loguear) {
                formatear(txt[i], sizeof(txt[i]), hay_dato, errs[i], crudo, final);
            }
        }

        taskENTER_CRITICAL(&dist_mux);
        memcpy(distancias_cm, nuevas, sizeof(distancias_cm));
        memcpy(crudas_cm, crudas, sizeof(crudas_cm));
        num_medicion++;
        taskEXIT_CRITICAL(&dist_mux);

        if (loguear) {
            ciclo = 0;
            ESP_LOGI(TAG_SENS, "%s: %s | %s: %s | %s: %s",
                     NOMBRES[SENSOR_IZQ], txt[SENSOR_IZQ],
                     NOMBRES[SENSOR_DER], txt[SENSOR_DER],
                     NOMBRES[SENSOR_FRENTE], txt[SENSOR_FRENTE]);
        }
    }
}

esp_err_t sensores_init(void)
{
    /* Un config por sensor: TRIG y ECHO propios */
    hcsr04_config_t cfg[NUM_SENSORS] = {
        [SENSOR_IZQ]    = { .trig_gpio = GPIO_NUM_18, .echo_gpio = GPIO_NUM_19 },
        [SENSOR_DER]    = { .trig_gpio = GPIO_NUM_21, .echo_gpio = GPIO_NUM_22 },
        [SENSOR_FRENTE] = { .trig_gpio = GPIO_NUM_25, .echo_gpio = GPIO_NUM_26 },
    };
    for (int i = 0; i < NUM_SENSORS; i++) {
        esp_err_t err = hcsr04_new(&cfg[i], &sensors[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_SENS, "No se pudo crear el sensor %s: %s", NOMBRES[i], esp_err_to_name(err));
            return err;
        }
    }

    if (xTaskCreate(sensores_task, "sensores_task", SENSORES_TASK_STACK, NULL,
                    SENSORES_TASK_PRIO, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

uint32_t sensores_leer(float out_cm[NUM_SENSORS])
{
    taskENTER_CRITICAL(&dist_mux);
    memcpy(out_cm, distancias_cm, sizeof(distancias_cm));
    uint32_t n = num_medicion;
    taskEXIT_CRITICAL(&dist_mux);
    return n;
}

uint32_t sensores_leer_crudo(float out_cm[NUM_SENSORS])
{
    taskENTER_CRITICAL(&dist_mux);
    memcpy(out_cm, crudas_cm, sizeof(crudas_cm));
    uint32_t n = num_medicion;
    taskEXIT_CRITICAL(&dist_mux);
    return n;
}
