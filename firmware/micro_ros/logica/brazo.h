/*
 * Camara y brazo: que servo es cada articulacion y a que velocidad se mueve.
 * Cableado y velocidades se ajustan en brazo.c. No sabe nada de ROS: main.c le pasa los
 * grados que llegan por /camara/cmd y /brazo/cmd.
 */
#pragma once

#define BRAZO_JUNTAS 4   // [base, hombro, codo, garra]

/**
 * @brief Mueve la camara (giro izq/der) al angulo pedido, en grados. NaN = no hace nada.
 */
void camara_mover(float grados);

/**
 * @brief Mueve el brazo: [base, hombro, codo, garra] en grados.
 *        NaN = dejar esa articulacion como esta.
 */
void brazo_mover(const float grados[BRAZO_JUNTAS]);
