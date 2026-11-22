#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_camera.h"
#include "esp_err.h"

// Inicializa la camara y comprueba que entregue al menos un JPEG completo.
// Si ya estaba lista no hace nada. Si falla, deja todo liberado para reintentar.
esp_err_t camera_manager_init(void);

// Apaga la camara (para volver a llamar a camera_manager_init).
void camera_manager_deinit(void);

// true si la camara esta inicializada y validada.
bool camera_manager_lista(void);

camera_fb_t *camera_manager_capture(void);

void camera_manager_release(camera_fb_t *frame);

// Capturas fallidas seguidas (se pone en 0 con la primera que sale bien).
// Sirve para decidir cuando reiniciar la camara.
unsigned camera_manager_fallos_seguidos(void);

// Pide un cuadro de prueba y lo descarta. Devuelve true si salio un JPEG completo.
// Cuenta como falla si no. Util para vigilar la camara cuando nadie mira el video.
bool camera_manager_probar(void);

// Milisegundos desde la ultima captura (de un cliente o de una prueba).
int64_t camera_manager_inactiva_ms(void);

// Cuantas veces fallo la inicializacion desde que arranco la placa.
unsigned camera_manager_inicios_fallidos(void);

#endif
