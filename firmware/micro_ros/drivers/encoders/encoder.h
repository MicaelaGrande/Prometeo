// Encoders de las 4 ruedas con el contador de pulsos por hardware (PCNT).
// Paso 1: solo RPM. El conteo lo hace el periferico: no hay interrupcion por pulso,
// ni timer, ni tarea. El RPM se calcula al pedirlo, con los pulsos contados desde la
// ultima vez.
#pragma once
#include "sdkconfig.h"

// ===== PINES =====
// Se eligen en menuconfig ("Prometeo - Encoders") y quedan en sdkconfig:
//   CONFIG_PROMETEO_ENCODER_DEL_IZQ_PIN, CONFIG_PROMETEO_ENCODER_DEL_DER_PIN,
//   CONFIG_PROMETEO_ENCODER_TRAS_IZQ_PIN, CONFIG_PROMETEO_ENCODER_TRAS_DER_PIN

// ===== DATOS DEL ENCODER =====
#define ENC_PPR             CONFIG_PROMETEO_ENC_PPR
#define ENC_FLANCOS         CONFIG_PROMETEO_ENC_FLACOS
#define ENC_CUENTAS_VUELTA  (ENC_PPR * ENC_FLANCOS)
#define ENC_VENTANA_MIN_MS  100     // menor ventana de medicion (si se pide mas seguido, devuelve el ultimo valor)
#define ENC_TIMEOUT_MS      3000    // sin cuentas por mas de esto = rueda parada (RPM 0)
#define ENC_FILTRO_TAU_MS   CONFIG_PROMETEO_ENC_FILTRO_TAU_MS   // constante de tiempo del pasa-bajos (0 = sin filtro)

// ===== RUEDAS =====
typedef enum {
    ENC_DEL_IZQ = 0,   // delantera izquierda
    ENC_DEL_DER,       // delantera derecha
    ENC_TRAS_IZQ,      // trasera izquierda
    ENC_TRAS_DER,      // trasera derecha
    ENC_CANTIDAD
} encoder_id_t;

// ===== FUNCIONES =====
void  encoders_init(void);              // configura pines y las 4 unidades PCNT; empiezan a contar ya

// Vueltas por minuto de la rueda (siempre >= 0: un solo canal, no sabe para que lado gira),
// FILTRADAS con un pasa-bajos de primer orden de constante ENC_FILTRO_TAU_MS: suaviza el escalon de
// una cuenta por ventana, pero atrasa (llega al 63 % en tau). Con tau = 0 es igual a la cruda.
// Llamar desde UNA sola tarea (no tiene candados: es lo que lo hace barato).
float encoder_get_rpm(encoder_id_t e);

// Igual pero SIN filtrar (la medida de la ventana). Usar donde no se puede tolerar el atraso
// (arranque, deteccion de rueda quieta, tests).
float encoder_get_rpm_raw(encoder_id_t e);
