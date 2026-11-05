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
#define L9110S_FAULT_OVERLOAD(s) (1u << (9 + (s)))  // esfuerzo maximo superado en el puente s (L9110S_LEFT/RIGHT), latcheado
#define L9110S_FAULT_ANY_STALL  0x0Fu
#define L9110S_FAULT_ANY_OVERLOAD (L9110S_FAULT_OVERLOAD(0) | L9110S_FAULT_OVERLOAD(1))

// Configuracion propia de cada motor
typedef struct {
    bool     invertido;       // invierte el sentido de giro
    float    zona_muerta;     // 0..0.9  duty minimo con el que el motor arranca
    float    duty_max;        // (zona_muerta..1] tope de duty
    float    rampa_paso;      // (0..1]  maximo cambio por tick (20 ms)
    float    trim;            // 0.5..1.5 ganancia para igualar motores
    uint32_t deadtime_ms;     // 0..2000 pausa en 0 al invertir el sentido
    uint32_t stall_ms;        // 0 = sin deteccion; 100..10000: ventana (ms) en la que se mide la velocidad del encoder
    float    stall_duty_min;  // 0..1 duty aplicado desde el cual se vigila el atasco
    float    stall_min_pps;   // 0..1000 pulsos/s minimos del encoder en esa ventana (con duty >= stall_duty_min).
                              // 0 = solo se falla si no hay NINGUN pulso en la ventana
} l9110s_cfg_t;

// Seguridad global (ver l9110s_safety_set). Valores iniciales: menuconfig "Prometeo - Seguridad L9110S".
typedef struct {
    bool  stop_all;           // una falla latcheada (atasco o esfuerzo) frena los 4 motores, no solo el afectado
    bool  effort_enable;      // vigilar el esfuerzo de cada puente H
    float effort_limit;       // (0..2] limite del esfuerzo medio por puente (ver abajo)
    float effort_tau_s;       // [1..600] constante de tiempo del promedio, en segundos
} l9110s_safety_t;

// ESFUERZO: no hay sensor de corriente, se estima por el duty. Cada puente (2 motores) acumula
// P = duty_a^2 + duty_b^2 (proporcional al calor I^2*R en el peor caso, rueda trabada) promediado
// con constante de tiempo effort_tau_s. 1.0 = un motor a duty 1 sostenido, o los dos a duty 0.71.
// Maximo posible = 2.0. Si el promedio pasa effort_limit se latchea L9110S_FAULT_OVERLOAD(lado).

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

// Seguridad: con stop_all, cualquier atasco o sobreesfuerzo latcheado mantiene frenados los 4 motores
// hasta que se liberen todas las fallas (l9110s_clear_fault / l9110s_clear_overload) y llegue una orden nueva.
esp_err_t l9110s_safety_set(const l9110s_safety_t *s);   // ESP_ERR_INVALID_ARG si fuera de rango
void      l9110s_safety_get(l9110s_safety_t *s);

// Fallas
uint32_t l9110s_fault_mask(void);
bool     l9110s_clear_fault(l9110s_motor_t m);      // libera atasco de m; false si no habia
// Libera el sobreesfuerzo del puente. false si no habia o si todavia no se enfrio
// (esfuerzo medio > 50 % del limite): hay que esperar unos segundos.
bool     l9110s_clear_overload(l9110s_side_t lado);

// Estado
float l9110s_get_duty(l9110s_motor_t m);            // duty firmado realmente aplicado (-1..1)
float l9110s_get_effort(l9110s_side_t lado);        // esfuerzo medio actual del puente (0..2)
