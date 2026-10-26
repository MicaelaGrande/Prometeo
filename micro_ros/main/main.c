#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hcsr04.h"

static const char *TAG = "main";

enum { IZQ, DER, FRENTE, NUM_SENSORS };

static const char *NOMBRES[NUM_SENSORS] = { "IZQ", "DER", "FRENTE" };

/* ================== Calibracion (cm). Cambiar aca. ================== */

/* 1) Error propio de cada modulo (hardware). Poner un objeto a distancia
 *    conocida (medida con regla) y anotar: offset = crudo - real.
 *    Ej: el monitor muestra "crudo 21.0" y la regla dice 20.0 -> 1.0f */
float offset_sensor_cm[NUM_SENSORS] = {
    [IZQ]    = 0.0f,
    [DER]    = 0.0f,
    [FRENTE] = 0.0f,
};

/* 2) Distancia desde el frente del sensor hasta el borde del robot
 *    (positivo si el sensor esta metido hacia adentro del robot).
 *    Todavia no definido: dejar en 0 hasta armar el robot. */
float pos_sensor_cm[NUM_SENSORS] = {
    [IZQ]    = 0.0f,
    [DER]    = 0.0f,
    [FRENTE] = 0.0f,
};

/* distancia final = mediana(crudo) - offset_sensor_cm - pos_sensor_cm */

/* ==================================================================== */

/* Cantidad de lecturas que usa la mediana (impar) */
#define VENTANA 3

/* Valor que se guarda cuando el sensor no ve nada (fuera de rango): asi
 * "no hay pared" cuenta como lejos y la mediana lo trata como un dato mas */
#define DIST_LEJOS_CM 400.0f

/* Ultimas lecturas de cada sensor, para la mediana */
typedef struct {
    float lecturas[VENTANA];
    int cant;       /* cuantas lecturas validas hay guardadas (hasta VENTANA) */
    int prox;       /* posicion donde se escribe la proxima */
} historial_t;

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

void app_main(void)
{
    /* Un config por sensor: TRIG y ECHO propios */
    hcsr04_config_t cfg[NUM_SENSORS] = {
        [IZQ]    = { .trig_gpio = GPIO_NUM_18, .echo_gpio = GPIO_NUM_19 },
        [DER]    = { .trig_gpio = GPIO_NUM_21, .echo_gpio = GPIO_NUM_22 },
        [FRENTE] = { .trig_gpio = GPIO_NUM_25, .echo_gpio = GPIO_NUM_26 },
    };
    hcsr04_handle_t sensors[NUM_SENSORS] = { NULL };

    for (int i = 0; i < NUM_SENSORS; i++) {
        ESP_ERROR_CHECK(hcsr04_new(&cfg[i], &sensors[i]));
    }

    float cm[NUM_SENSORS] = { 0 };
    esp_err_t errs[NUM_SENSORS];
    historial_t hist[NUM_SENSORS] = { 0 };

    while (1) {
        /* Turno 1: los laterales juntos (apuntan en direcciones opuestas, no se interfieren) */
        hcsr04_measure_group_cm(&sensors[IZQ], 2, &cm[IZQ], &errs[IZQ]);
        /* Turno 2: el frente solo, 60 ms despues para que se apaguen los rebotes */
        hcsr04_measure_group_cm(&sensors[FRENTE], 1, &cm[FRENTE], &errs[FRENTE]);

        char txt[NUM_SENSORS][40];
        for (int i = 0; i < NUM_SENSORS; i++) {
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
            formatear(txt[i], sizeof(txt[i]), hay_dato, errs[i], crudo, final);
        }
        ESP_LOGI(TAG, "%s: %s | %s: %s | %s: %s",
                 NOMBRES[IZQ], txt[IZQ], NOMBRES[DER], txt[DER], NOMBRES[FRENTE], txt[FRENTE]);
    }
}
