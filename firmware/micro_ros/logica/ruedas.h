/*
 * Velocidad medida de las ruedas (encoders + sentido de giro de cada motor).
 */
#pragma once

#define RUEDAS_TELEMETRIA_LEN 4

/**
 * @brief Velocidad de cada lado, CON SIGNO (+ adelante, - atras). El encoder tiene un solo
 *        canal y no sabe para que lado gira, asi que el sentido se toma del motor de esa
 *        rueda (lo que se le esta aplicando de verdad, despues de la rampa).
 * @param out  [vel_izq m/s, vel_der m/s, rpm_izq, rpm_der]; cada valor es el promedio de
 *             la rueda delantera y la trasera de ese lado.
 */
void ruedas_leer(float out[RUEDAS_TELEMETRIA_LEN]);
