#pragma once
#include <stdint.h>
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

// Duty maximo del PWM (segun PROMETEO_PWM_RES_BIT)
#define MOTORES_PWM_MAX  ((1 << CONFIG_PROMETEO_PWM_RES_BIT) - 1)

// ===== Settings del ramp =====
#define RAMP_PERIOD_MS   20     // CONFIG_PROMETEO_RAMP_PERIOD_MS
#define ACCEL_STEP       45     // CONFIG_PROMETEO_ACCEL_STEP    // ~450 ms 0->full
#define DECEL_STEP       100    // CONFIG_PROMETEO_DECEL_STEP    // ~200 ms full->0
#define MIN_DUTY         0   //280    // CONFIG_PROMETEO_MIN_DUTY      // measure on your robot

// ===== FALLAS DE SEGURIDAD (bits de motores_get_fallas) =====
#define MOTORES_FALLA_STALL(m)      (1u << (m))        // motor m: el encoder no da la velocidad minima
#define MOTORES_FALLA_SOBRE(lado)   (1u << (4 + (lado)))   // puente del lado (0 = izq A+B, 1 = der C+D): esfuerzo maximo superado
#define MOTORES_FALLA_STALL_TODOS   0x0Fu
#define MOTORES_FALLA_SOBRE_TODOS   0x30u

// ===== FUNCIONES =====
void motores_init(void);                       // configura PWM y arranca frenado
void motores_ramp_step(motor_id_t m, float velocidad); // -1.0 atrás ... 0 ... +1.0 adelante

void motores_unlock(void);                  // alimenta el failsafe sin cambiar objetivos (para usar motor_set suelto)
void motores_lock(bool falla);                     // freno de emergencia inmediato

bool motores_get_falla(void);                    // true si frenó por falta de órdenes
/*
 * @brief Devuelve el sentido del motor (1 adelante, -1 atrás) según el último duty aplicado.
 */
int  motores_get_sentido(motor_id_t m);
/*
 * @brief Devuelve la velocidad del motor (0..1) según el último duty aplicado.
 */
float  motores_get_duty(motor_id_t m);

/*
 * @brief Duty real de cada pin del puente (IA, IB), leido del periferico LEDC. 0..MOTORES_PWM_MAX.
 */
void motores_get_pwm(motor_id_t m, uint32_t *ia, uint32_t *ib);
