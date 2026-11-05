#pragma once
#include <stdbool.h>
#include "sdkconfig.h"

// ===== PINES =====
// Puente H 1 (pines configurables: idf.py menuconfig -> Prometeo - Pines GPIO)
#define MOTOR_A1 CONFIG_PROMETEO_MOTOR_A1_PIN
#define MOTOR_A2 CONFIG_PROMETEO_MOTOR_A2_PIN
#define MOTOR_B1 CONFIG_PROMETEO_MOTOR_B1_PIN
#define MOTOR_B2 CONFIG_PROMETEO_MOTOR_B2_PIN
// Puente H 2
#define MOTOR_C1 CONFIG_PROMETEO_MOTOR_C1_PIN
#define MOTOR_C2 CONFIG_PROMETEO_MOTOR_C2_PIN
#define MOTOR_D1 CONFIG_PROMETEO_MOTOR_D1_PIN
#define MOTOR_D2 CONFIG_PROMETEO_MOTOR_D2_PIN

// ===== NOMBRES DE CADA MOTOR =====
typedef enum {
    MOTOR_A = 0,
    MOTOR_B,
    MOTOR_C,
    MOTOR_D,
    CANT_MOTORES
} motor_id_t;

// ===== FUNCIONES =====
void motores_init(void);                       // configura PWM y arranca frenado
void motor_set(motor_id_t m, float velocidad); // -1.0 atrás ... 0 ... +1.0 adelante
void motores_alimentar(void);                  // alimenta el failsafe sin cambiar objetivos (para usar motor_set suelto)
void motores_set(float izq, float der);        // lado izq y der (alimenta el failsafe)
void motores_set_twist(float v_mps, float w_rads); // cinemática diferencial de 4 ruedas: m/s y rad/s (+ adelante, + izquierda)
void motores_frenar(void);                     // freno de emergencia inmediato
void motores_get_velocidad(float *v_mps, float *w_rads); // velocidad ESTIMADA a lazo abierto (sin encoders): m/s y rad/s
bool motores_en_falla(void);                   // true si frenó por falta de órdenes
int  motor_get_sentido(motor_id_t m);          // +1 adelante / -1 atrás: último sentido aplicado (signo de los encoders)
