/*
 * Sensores de distancia del robot: 3 HC-SR04 (izquierda, derecha y frente).
 *
 * Una tarea propia mide sin parar, filtra con mediana y aplica la calibracion
 * (ver sensores.c). Desde afuera solo se pide la ultima distancia de cada uno.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

enum { SENSOR_IZQ, SENSOR_DER, SENSOR_FRENTE, NUM_SENSORS };

/* Valor cuando el sensor no ve nada (no hay pared): cuenta como lejos */
#define DIST_LEJOS_CM 400.0f

/* Valor cuando el sensor no tiene dato (sin eco: problema de cableado) */
#define DIST_SIN_DATO -1.0f

/**
 * @brief Crea los 3 sensores y arranca la tarea que mide.
 *        Mide aunque no haya agente de micro-ROS.
 */
esp_err_t sensores_init(void);

/**
 * @brief Copia la ultima distancia de cada sensor (en cm, ya calibrada).
 *        Indices: SENSOR_IZQ, SENSOR_DER, SENSOR_FRENTE.
 * @return Numero de medicion: aumenta en 1 cada vez que hay distancias nuevas
 *         (~cada 120 ms). Sirve para saber si cambio desde la ultima lectura.
 */
uint32_t sensores_leer(float out_cm[NUM_SENSORS]);

/**
 * @brief Igual que sensores_leer() pero sin mediana: la lectura del ultimo ciclo.
 *        Reacciona un ciclo antes (~120 ms) pero tiene mas ruido. La usa el freno
 *        de emergencia; para navegar conviene sensores_leer().
 */
uint32_t sensores_leer_crudo(float out_cm[NUM_SENSORS]);
