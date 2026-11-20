#include "sdkconfig.h"
#include "centrado.h"

#ifdef CONFIG_PROMETEO_CENTRADO

#include <stdint.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "sensores.h"
#include "centrado_pid.h"

static const char *TAG = "centrado";

#define ANCHO_CM        (CONFIG_PROMETEO_PASILLO_ANCHO_MM / 10.0f)
#define PARED_MAX_CM    (CONFIG_PROMETEO_CENTRADO_PARED_MAX_MM / 10.0f)   // mas lejos = salida
#define VEL_MIN_MPS     (CONFIG_PROMETEO_CENTRADO_VEL_MIN_MM_S / 1000.0f)
#define GIRO_PEDIDO_MAX 0.05f      // rad/s: si piden girar mas que esto, no se mete
#define DATO_VIEJO_US   500000     // sin mediciones nuevas por esto: se reinicia
#define DT_MIN_S        0.02f

static const centrado_gan_t gan = {
    .kp    = CONFIG_PROMETEO_CENTRADO_KP_X100 / 100.0f,
    .ki    = CONFIG_PROMETEO_CENTRADO_KI_X100 / 100.0f,
    .kd    = CONFIG_PROMETEO_CENTRADO_KD_X100 / 100.0f,
    .w_max = CONFIG_PROMETEO_CENTRADO_W_MAX_X100 / 100.0f,
};

static centrado_pid_t pid;
static uint32_t medicion_prev = 0;
static int64_t  hora_prev_us = 0;
static float    salida = 0.0f;      // ultima correccion (se mantiene entre mediciones)

static void reiniciar(void)
{
    centrado_pid_reiniciar(&pid);
    salida = 0.0f;
    hora_prev_us = 0;
}

float centrado_corregir(float lineal_mps, float angular_pedido)
{
    if (lineal_mps < VEL_MIN_MPS || fabsf(angular_pedido) > GIRO_PEDIDO_MAX) {
        reiniciar();                  // parado, retrocediendo o girando a proposito
        return 0.0f;
    }

    float d[NUM_SENSORS];
    uint32_t medicion = sensores_leer(d);
    float izq = d[SENSOR_IZQ], der = d[SENSOR_DER];
    bool pared_izq = izq >= 0.0f && izq <= PARED_MAX_CM;    // -1 = sin dato, 400 = no ve nada
    bool pared_der = der >= 0.0f && der <= PARED_MAX_CM;
    if (!pared_izq || !pared_der) {
        reiniciar();                  // una salida o sin dato: no corrige
        return 0.0f;
    }

    // El PID se actualiza una vez por medicion nueva; entre mediciones se mantiene la salida
    if (medicion == medicion_prev) {
        return salida;
    }
    medicion_prev = medicion;
    int64_t ahora = esp_timer_get_time();
    float dt = (ahora - hora_prev_us) / 1e6f;
    if (hora_prev_us == 0 || (ahora - hora_prev_us) > DATO_VIEJO_US) {
        centrado_pid_reiniciar(&pid);       // primera medicion despues de una pausa
        hora_prev_us = ahora;
        salida = 0.0f;
        return 0.0f;
    }
    hora_prev_us = ahora;
    if (dt < DT_MIN_S) return salida;

    float e = (izq - der) / 2.0f / (ANCHO_CM / 2.0f);
    salida = centrado_pid_paso(&pid, &gan, e, dt);
    ESP_LOGD(TAG, "izq %.1f der %.1f e %+.2f -> w %+.3f rad/s", (double)izq, (double)der,
             (double)e, (double)salida);
    return salida;
}

#else   // PROMETEO_CENTRADO apagado

float centrado_corregir(float lineal_mps, float angular_pedido)
{
    (void)lineal_mps; (void)angular_pedido;
    return 0.0f;
}

#endif
