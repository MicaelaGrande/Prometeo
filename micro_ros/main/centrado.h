/*
 * Centrado en el pasillo: un PID que mantiene al robot en el medio de un pasillo
 * (PROMETEO_PASILLO_ANCHO_MM, 400 mm) usando los ultrasonidos izquierdo y derecho.
 *
 * Corre en el ESP32 y no depende de ROS. Suma una velocidad angular a la que llega por
 * /cmd_vel, solo si el robot avanza y nadie le pidio girar. Si un costado no ve pared
 * (una salida) o no hay dato, no corrige: no se tira al hueco.
 */
#pragma once

/**
 * @brief Correccion de velocidad angular (rad/s, + izquierda) para centrar el robot.
 *        Llamar desde el callback de /cmd_vel con la orden que se va a mandar.
 * @param lineal_mps  Avance pedido (despues del freno de emergencia).
 * @param angular_pedido  Giro pedido por quien maneja (rad/s).
 * @return Lo que hay que SUMAR al giro pedido; 0 si el centrado no esta activo.
 */
float centrado_corregir(float lineal_mps, float angular_pedido);
