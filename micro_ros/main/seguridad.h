/*
 * Freno de emergencia del robot: ultimo recurso para no chocar de frente.
 *
 * Corre en el ESP32 y no depende del WiFi ni de ROS. La navegacion es la que
 * frena y gira normalmente; esto solo actua si se esta por chocar. Usa el
 * FRENTE sin mediana y un umbral que crece con la velocidad (ver seguridad.c).
 * Solo bloquea el avance: girar y retroceder siempre se permite.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Arranca la tarea que vigila el FRENTE (cada 20 ms).
 *        Llamar despues de motores_init() y sensores_init().
 */
esp_err_t seguridad_init(void);

/**
 * @brief Pasa por aca cada orden de avance antes de mandarla a los motores.
 * @param avance El avance pedido (-1..1, + adelante).
 * @return El avance permitido: el mismo, o 0 si avanzar asi es peligroso.
 */
float seguridad_filtrar(float avance);

/** @brief true mientras el freno de emergencia esta activo. */
bool seguridad_freno_activo(void);

/**
 * @brief Activa o desactiva el freno (false = modo recoger: deja acercarse
 *        a objetos). Arranca activado.
 */
void seguridad_habilitar(bool on);

/** @brief true si el freno esta activado (no en modo recoger). */
bool seguridad_habilitada(void);
