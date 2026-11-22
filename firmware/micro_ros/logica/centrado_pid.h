/*
 * Cuenta del PID de centrado en el pasillo. Solo matematica (sin ESP-IDF), asi se prueba
 * en la compu. La usa centrado.c.
 *
 * Error: e = (izq - der) / 2 / (ancho / 2), normalizado: 0 = en el medio, +1 = pegado a la
 * pared derecha, -1 = pegado a la izquierda. Salida: velocidad angular (rad/s, + izquierda).
 * Si el robot esta a la derecha del medio (izq > der), tiene que girar a la izquierda (+).
 */
#pragma once

#include <math.h>
#include <stdbool.h>

typedef struct {
    float integral;     // termino I ya multiplicado por Ki (rad/s)
    float error_prev;
    float deriv;        // derivada filtrada (1/s)
    bool  tiene_prev;   // false = primer paso: sin derivada
} centrado_pid_t;

typedef struct {
    float kp;           // rad/s por unidad de error
    float ki;           // rad/s por unidad de error por segundo
    float kd;           // rad/s por unidad de error por segundo
    float w_max;        // tope de la salida (y del termino I), rad/s
} centrado_gan_t;

// Filtro de la derivada: el ultrasonido tiene ruido de ~1 cm y derivarlo directo da picos
#define CENTRADO_DERIV_ALFA 0.4f

static inline void centrado_pid_reiniciar(centrado_pid_t *p)
{
    p->integral = 0.0f;
    p->error_prev = 0.0f;
    p->deriv = 0.0f;
    p->tiene_prev = false;
}

static inline float centrado_limitar(float x, float lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

// e: error normalizado, dt: segundos desde el paso anterior (> 0)
static inline float centrado_pid_paso(centrado_pid_t *p, const centrado_gan_t *g, float e, float dt)
{
    p->integral = centrado_limitar(p->integral + g->ki * e * dt, g->w_max);   // anti-windup
    float d = p->tiene_prev ? (e - p->error_prev) / dt : 0.0f;
    p->deriv += CENTRADO_DERIV_ALFA * (d - p->deriv);
    p->error_prev = e;
    p->tiene_prev = true;
    return centrado_limitar(g->kp * e + p->integral + g->kd * p->deriv, g->w_max);
}
