/*
 * Driver del sensor ultrasonico HC-SR04 usando el periferico RMT del ESP32.
 * Pensado para ESP-IDF 5.4 (API nueva de RMT).
 *
 *   - Canal RMT TX -> pin TRIG (pulso de 10 us)
 *   - Canal RMT RX -> pin ECHO (mide el ancho del pulso de eco)
 *
 * IMPORTANTE: ECHO sale a ~5 V. Usar divisor resistivo (ej. 1k + 2k) antes del GPIO.
 */
#pragma once

#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Configuracion del sensor: solo hay que elegir los pines. */
typedef struct {
    gpio_num_t trig_gpio;   /*!< GPIO conectado a TRIG (salida del ESP32) */
    gpio_num_t echo_gpio;   /*!< GPIO conectado a ECHO (entrada del ESP32, con divisor resistivo) */
} hcsr04_config_t;

/** Configuracion por defecto: TRIG = GPIO18, ECHO = GPIO19. */
#define HCSR04_CONFIG_DEFAULT() { \
    .trig_gpio = GPIO_NUM_18,     \
    .echo_gpio = GPIO_NUM_19,     \
}

typedef struct hcsr04_t *hcsr04_handle_t;

/**
 * @brief Crea los canales RMT (TX y RX) y deja el sensor listo para medir.
 *
 * @return ESP_OK si salio bien; otro codigo si fallo la reserva de canales, etc.
 */
esp_err_t hcsr04_new(const hcsr04_config_t *config, hcsr04_handle_t *out_handle);

/**
 * @brief Hace UNA medicion y devuelve la distancia en centimetros.
 *
 * Respeta internamente los 60 ms minimos entre disparos que pide el datasheet
 * (contados desde el ultimo disparo de CUALQUIER sensor, no solo de este).
 * La funcion bloquea hasta ~50 ms mientras espera el eco.
 *
 * @return ESP_OK                    medicion valida
 *         ESP_ERR_TIMEOUT           no llego ningun eco (sensor desconectado o mal cableado)
 *         ESP_ERR_INVALID_RESPONSE  eco fuera del rango 2 cm - 400 cm (sin objeto, muy cerca o muy lejos)
 */
esp_err_t hcsr04_measure_cm(hcsr04_handle_t handle, float *distance_cm);

/**
 * @brief Dispara VARIOS sensores a la vez y devuelve la distancia de cada uno.
 *
 * Usar solo con sensores que no se interfieran entre si (por ejemplo los dos
 * laterales, que apuntan en direcciones opuestas). Respeta los 60 ms minimos
 * desde el ultimo disparo de cualquier sensor. Bloquea hasta ~50 ms.
 *
 * @param hs           handles de los sensores del grupo
 * @param n            cantidad de sensores
 * @param distance_cm  salida: distancia de cada sensor (valida solo si errs[i] == ESP_OK)
 * @param errs         salida: resultado de cada sensor, con los mismos codigos que hcsr04_measure_cm()
 * @return ESP_OK, o ESP_ERR_INVALID_ARG si algun puntero es nulo
 */
esp_err_t hcsr04_measure_group_cm(const hcsr04_handle_t *hs, size_t n,
                                  float *distance_cm, esp_err_t *errs);

/** Libera los canales RMT y la memoria. */
esp_err_t hcsr04_del(hcsr04_handle_t handle);

#ifdef __cplusplus
}
#endif
