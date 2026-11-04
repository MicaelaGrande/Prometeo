#pragma once
#include "driver/gpio.h"

// ===== PINES =====
// 34 y 35 solo sirven como entrada: ideales para encoders
#define PIN_ENC_DER     GPIO_NUM_34
#define PIN_ENC_IZQ     GPIO_NUM_35

// ===== DATOS DEL ENCODER =====
#define PPR             20      // cantidad de ranuras del disco
#define PERIOD_MS       100     // cada cuánto se calcula la velocidad

// ===== DATOS DE LA RUEDA =====
#define DIAMETRO_RUEDA_M  0.065f   // 65 mm (hay que medir la rueda real)(no lo hice todavia xd)

// ===== FUNCIONES =====
void  encoders_init(void);          // configura pines, interrupciones y timer

float encoder_get_rpm_der(void);    // vueltas por minuto, rueda derecha
float encoder_get_rpm_izq(void);    // vueltas por minuto, rueda izquierda

float encoder_get_vel_der(void);    // metros por segundo, rueda derecha
float encoder_get_vel_izq(void);    // metros por segundo, rueda izquierda