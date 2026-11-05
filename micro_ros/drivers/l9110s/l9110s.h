#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Driver alternativo para 2 puentes H L9110S (2 canales cada uno) = 4 motores.
//   Puente izquierdo: motores FL (A) y RL (B)
//   Puente derecho:   motores FR (C) y RR (D)
//
// Pines (menuconfig "Prometeo - Pines GPIO" -> motores), los mismos que motores.c:
//   CONFIG_PROMETEO_MOTOR_{A,B,C,D}{1,2}_PIN   (1 = IA, 2 = IB)
//
// L9110S: IA=PWM,IB=0 -> adelante | IA=0,IB=PWM -> atras | ambos 0 -> parado.
// Nunca se saca PWM por los dos pines a la vez.
//
// IMPORTANTE:
//  - NO usar junto con motores.c: comparten los canales LEDC 0..7.
//  - Para la deteccion de atasco hay que llamar encoders_init() ANTES de l9110s_init().
//  - Un solo hilo debe llamar a las funciones de escritura de motores a la vez
//    (la tarea interna es la unica que escribe el PWM).

typedef enum {
    L9110S_FL = 0,   // delantero izquierdo (motor A)
    L9110S_RL,       // trasero izquierdo   (motor B)
    L9110S_FR,       // delantero derecho   (motor C)
    L9110S_RR,       // trasero derecho     (motor D)
    L9110S_N
} l9110s_motor_t;

typedef enum { L9110S_LEFT = 0, L9110S_RIGHT } l9110s_side_t;

// Bits de l9110s_fault_mask()
#define L9110S_FAULT_STALL(m)   (1u << (m))   // motor m atascado (latcheado)
#define L9110S_FAULT_TIMEOUT    (1u << 8)     // sin ordenes por TIMEOUT (se limpia con la proxima orden)
#define L9110S_FAULT_ANY_STALL  0x0Fu

// Configuracion propia de cada motor
typedef struct {
    bool     invertido;       // invierte el sentido de giro
    float    zona_muerta;     // 0..0.9  duty minimo con el que el motor arranca
    float    duty_max;        // (zona_muerta..1] tope de duty
    float    rampa_paso;      // (0..1]  maximo cambio por tick (20 ms)
    float    trim;            // 0.5..1.5 ganancia para igualar motores
    uint32_t deadtime_ms;     // 0..2000 pausa en 0 al invertir el sentido
    uint32_t stall_ms;        // 0 = sin deteccion; 100..10000 ms sin pulsos con duty alto
    float    stall_duty_min;  // 0..1 duty aplicado desde el cual se vigila el atasco
} l9110s_cfg_t;

// Valida pines (solo GPIO de salida, sin repetidos), configura LEDC (timer 1,
// 8 bit, 1 kHz), crea la tarea de control. Arranca frenado y en falla hasta la
// primera orden. ESP_ERR_INVALID_STATE si ya se inicializo.
esp_err_t l9110s_init(void);

// Configuracion por motor (valida rangos; ESP_ERR_INVALID_ARG si es invalida)
esp_err_t l9110s_cfg_set(l9110s_motor_t m, const l9110s_cfg_t *cfg);
void      l9110s_cfg_get(l9110s_motor_t m, l9110s_cfg_t *cfg);

// Ordenes (-1.0 atras ... +1.0 adelante; NaN -> 0). Solo l9110s_set/feed alimentan el watchdog.
void l9110s_motor_set(l9110s_motor_t m, float v);   // objetivo de un motor
void l9110s_side_set(l9110s_side_t lado, float v);  // objetivo de los 2 motores del lado
void l9110s_set(float izq, float der);              // ambos lados + alimenta watchdog
void l9110s_feed(void);                             // alimenta el watchdog (tras un timeout los objetivos ya quedaron en 0)

void l9110s_brake_all(void);                        // freno inmediato, sin rampa

// Fallas
uint32_t l9110s_fault_mask(void);
bool     l9110s_clear_fault(l9110s_motor_t m);      // libera atasco de m; false si no habia

// Estado
float l9110s_get_duty(l9110s_motor_t m);            // duty firmado realmente aplicado (-1..1)
