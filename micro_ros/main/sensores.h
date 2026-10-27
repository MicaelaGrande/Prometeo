/*
 * Sensores de distancia del robot: 3 HC-SR04 (izquierda, derecha y frente).
 *
 * Una tarea propia mide sin parar, filtra con mediana y aplica la calibracion
 * (ver sensores.c). Desde afuera solo se pide la ultima distancia de cada uno.
 */
#pragma once

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
 */
void sensores_leer(float out_cm[NUM_SENSORS]);
