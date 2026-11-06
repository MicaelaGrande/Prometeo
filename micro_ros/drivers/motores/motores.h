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

// ===== FALLAS DE SEGURIDAD (bits de motores_get_fallas) =====
#define MOTORES_FALLA_STALL(m)      (1u << (m))        // motor m: el encoder no da la velocidad minima
#define MOTORES_FALLA_SOBRE(lado)   (1u << (4 + (lado)))   // puente del lado (0 = izq A+B, 1 = der C+D): esfuerzo maximo superado
#define MOTORES_FALLA_STALL_TODOS   0x0Fu
#define MOTORES_FALLA_SOBRE_TODOS   0x30u

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

// ===== SEGURIDAD (valores en menuconfig: "Prometeo - Seguridad motores") =====
// 1) ATASCO: con el motor andando (duty >= umbral) el encoder tiene que dar al menos N pulsos/s
//    en cada ventana. Si no, falla de ese motor. Hay que haber llamado a encoders_init() antes de
//    motores_init() (o desactivar PROMETEO_MOTORES_STALL_ENABLE).
// 2) ESFUERZO: sin sensor de corriente se estima por puente H como duty^2 de sus 2 motores,
//    promediado con constante de tiempo tau. Si pasa el limite, falla de ese puente.
// Con PROMETEO_MOTORES_STOP_ALL, cualquier falla frena los 4 motores. La falla queda latcheada:
// los motores siguen frenados aunque lleguen ordenes, hasta motores_liberar_fallas().
uint32_t motores_get_fallas(void);                 // bits MOTORES_FALLA_*
bool     motores_liberar_fallas(void);             // libera atascos y sobreesfuerzos; false si algun puente sigue
                                                   // caliente (esfuerzo > 50 % del limite): reintentar en unos segundos
float    motores_get_esfuerzo(int lado);           // esfuerzo medio del puente (0 = izq, 1 = der), 0..2
