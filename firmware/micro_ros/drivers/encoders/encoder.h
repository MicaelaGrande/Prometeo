#pragma once
#include "driver/gpio.h"

// ===== PINES =====
// Se eligen en menuconfig ("Prometeo - Pines GPIO" -> Encoders) y quedan en sdkconfig:
//   CONFIG_PROMETEO_ENCODER_DEL_IZQ_PIN, CONFIG_PROMETEO_ENCODER_DEL_DER_PIN,
//   CONFIG_PROMETEO_ENCODER_TRAS_IZQ_PIN, CONFIG_PROMETEO_ENCODER_TRAS_DER_PIN

// ===== DATOS DEL ENCODER =====
#define PPR             20      // cantidad de ranuras del disco
#define PERIOD_MS       100     // cada cuánto se calcula la velocidad
#define TIMEOUT_PERIODO_US 3000000LL // sin pulsos por más de 3 s = rueda parada (mín. ~1 rpm)

// ===== DATOS DE LA RUEDA =====
#define DIAMETRO_RUEDA_M  0.065f   // 65 mm (hay que medir la rueda real)(no lo hice todavia xd)

// ===== RUEDAS =====
typedef enum {
    ENC_DEL_IZQ = 0,   // delantera izquierda
    ENC_DEL_DER,       // delantera derecha
    ENC_TRAS_IZQ,      // trasera izquierda
    ENC_TRAS_DER,      // trasera derecha
    ENC_CANTIDAD
} encoder_id_t;

// ===== FUNCIONES =====
void  encoders_init(void);                  // configura pines, interrupciones y timer

// Por rueda (siempre >= 0: un solo canal, no sabe para qué lado gira)
float encoder_get_rpm(encoder_id_t e);      // vueltas por minuto
float encoder_get_vel(encoder_id_t e);      // metros por segundo
// RPM por tiempo entre pulsos: precisa a baja velocidad (la de arriba tiene
// resolución de 30 rpm: 1 pulso en 100 ms). Se actualiza en cada pulso.
float encoder_get_rpm_lenta(encoder_id_t e);

// Pulsos contados desde el arranque (no se resetea; da la vuelta a los 2^32)
uint32_t encoder_get_pulsos(encoder_id_t e);
// GPIO de esa rueda (el de menuconfig)
int      encoder_get_pin(encoder_id_t e);

// Por lado: promedio de la rueda delantera y la trasera
float encoder_get_rpm_der(void);
float encoder_get_rpm_izq(void);
float encoder_get_vel_der(void);
float encoder_get_vel_izq(void);
