#pragma once
#include <stdbool.h>
#include <stdint.h>

// ===== NOMBRES DE CADA SERVO =====
typedef enum {
    SERVO_1 = 0,
    SERVO_2,
    SERVO_3,
    SERVO_4,
    SERVO_5,
    CANT_SERVOS
} servo_id_t;

// ===== CONFIGURACIÓN =====
void  servos_init(void);                     // configura PWM, deja cada servo en su posición home y arranca la rampa

// ===== MOVIMIENTO =====
void  servo_set_angulo(servo_id_t s, float grados);           // salto inmediato a esa posición
void  servo_mover(servo_id_t s, float grados, float vel_gps); // con rampa, en grados por segundo (<= 0 -> inmediato)
void  servos_mover_sincronizado(const float grados[], uint32_t tiempo_ms); // todos llegan a la vez (0 -> inmediato)
void  servos_home(float vel_gps);            // vuelve cada servo a su posición home con rampa
void  servos_detener(void);                  // congela cada servo donde está (objetivo = actual)

// ===== ESTADO =====
float servo_get_angulo(servo_id_t s);        // ángulo OBJETIVO (no el actual)
bool  servo_en_movimiento(servo_id_t s);     // true si todavía no llegó al objetivo
bool  servos_en_movimiento(void);            // true si algún servo se está moviendo

// ===== ENERGÍA =====
void  servo_soltar(servo_id_t s);            // deja de mandar pulsos (el servo queda sin fuerza)
void  servo_enganchar(servo_id_t s);         // vuelve a mandar pulsos desde la última posición

// ===== CALIBRACIÓN =====
void  servo_set_us(servo_id_t s, uint16_t us); // pulso en crudo (us), se salta el mapeo de ángulos; el próximo comando de ángulo lo cancela
