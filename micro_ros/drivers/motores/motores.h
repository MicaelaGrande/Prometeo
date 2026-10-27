#pragma once
#include <stdbool.h>

// ===== PINES =====
// Puente H 1
#define MOTOR_A1 16
#define MOTOR_A2 17
#define MOTOR_B1 27
#define MOTOR_B2 4
// Puente H 2
#define MOTOR_C1 32
#define MOTOR_C2 33
#define MOTOR_D1 13
#define MOTOR_D2 23

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
void motores_set(float izq, float der);        // lado izq y der (alimenta el failsafe)
void motores_frenar(void);                     // freno de emergencia inmediato
bool motores_en_falla(void);                   // true si frenó por falta de órdenes