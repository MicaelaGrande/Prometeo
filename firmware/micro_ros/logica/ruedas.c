#include "ruedas.h"

#include "motores.h"
#include "encoder.h"

// Que motor mueve cada rueda con encoder (segun los nombres del Kconfig).
// VERIFICAR en el robot: con /cmd_vel angular.z > 0, izq tiene que dar - y der +.
static const motor_id_t MOTOR_DE_RUEDA[ENC_CANTIDAD] = {
    [ENC_DEL_IZQ]  = MOTOR_A,
    [ENC_TRAS_IZQ] = MOTOR_B,
    [ENC_DEL_DER]  = MOTOR_C,
    [ENC_TRAS_DER] = MOTOR_D,
};

static float con_signo(encoder_id_t e, float valor)
{
    return (float)motor_get_sentido(MOTOR_DE_RUEDA[e]) * valor;
}

void ruedas_leer(float out[RUEDAS_TELEMETRIA_LEN])
{
    out[0] = (con_signo(ENC_DEL_IZQ, encoder_get_vel(ENC_DEL_IZQ)) +
              con_signo(ENC_TRAS_IZQ, encoder_get_vel(ENC_TRAS_IZQ))) / 2.0f;
    out[1] = (con_signo(ENC_DEL_DER, encoder_get_vel(ENC_DEL_DER)) +
              con_signo(ENC_TRAS_DER, encoder_get_vel(ENC_TRAS_DER))) / 2.0f;
    out[2] = (con_signo(ENC_DEL_IZQ, encoder_get_rpm(ENC_DEL_IZQ)) +
              con_signo(ENC_TRAS_IZQ, encoder_get_rpm(ENC_TRAS_IZQ))) / 2.0f;
    out[3] = (con_signo(ENC_DEL_DER, encoder_get_rpm(ENC_DEL_DER)) +
              con_signo(ENC_TRAS_DER, encoder_get_rpm(ENC_TRAS_DER))) / 2.0f;
}
