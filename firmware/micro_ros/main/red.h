/*
 * Red del cuerpo: WiFi y busqueda del agente micro-ROS (ver red.c).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include <rmw_microros/rmw_microros.h>

/**
 * @brief Arranca el WiFi como cliente del hotspot de la camara y vuelve enseguida.
 *        Reintenta conectarse para siempre (no importa quien arranque primero).
 */
esp_err_t red_wifi_init(void);

/**
 * @brief Busca el agente micro-ROS en la red /24 de la ESP32 (solo con PROMETEO_AGENTE_AUTO).
 *        Si responde alguno, deja su IP en ip_out y devuelve true. Cambia la direccion de
 *        rmw_options: si devuelve false hay que volver a ponerle la ultima conocida.
 * @param rmw_options  Opciones RMW de micro-ROS (donde se prueba cada candidata).
 * @param ip_out       Buffer donde se escribe la IP encontrada (16 bytes alcanzan).
 * @param ip_len       Tamano de ip_out.
 */
bool red_buscar_agente(rmw_init_options_t *rmw_options, char *ip_out, size_t ip_len);
