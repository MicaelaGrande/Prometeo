#include "brazo.h"

#include <math.h>
#include "mg90s.h"

// Que servo fisico es cada articulacion (AJUSTAR segun el cableado)
#define SERVO_CAMARA  SERVO_1  // giro izq/der de la camara
#define SERVO_BASE    SERVO_2  // giro izq/der del brazo
#define SERVO_HOMBRO  SERVO_3
#define SERVO_CODO    SERVO_4
#define SERVO_GARRA   SERVO_5

// Velocidad de cada servo, en grados por segundo (AJUSTAR probando).
// El driver (mg90s.c, cfg[].vel) las limita a la maxima de cada servo: base, hombro y codo
// no pasan de 45, por eso se piden 45 (pedir mas no los hace ir mas rapido).
#define SERVO_CAMARA_VEL  180
#define SERVO_BASE_VEL    45
#define SERVO_HOMBRO_VEL  45
#define SERVO_CODO_VEL    45
#define SERVO_GARRA_VEL   90

static const servo_id_t juntas[BRAZO_JUNTAS] = {
    SERVO_BASE, SERVO_HOMBRO, SERVO_CODO, SERVO_GARRA
};
static const float juntas_vel_gps[BRAZO_JUNTAS] = {
    SERVO_BASE_VEL, SERVO_HOMBRO_VEL, SERVO_CODO_VEL, SERVO_GARRA_VEL
};

void camara_mover(float grados)
{
    if (!isnan(grados)) {
        servo_mover(SERVO_CAMARA, grados, SERVO_CAMARA_VEL);   // la libreria limita al rango
    }
}

void brazo_mover(const float grados[BRAZO_JUNTAS])
{
    for (int i = 0; i < BRAZO_JUNTAS; i++) {
        if (!isnan(grados[i])) {
            servo_mover(juntas[i], grados[i], juntas_vel_gps[i]);
        }
    }
}
