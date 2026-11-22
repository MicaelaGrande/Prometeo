#ifndef STREAM_SERVER_H
#define STREAM_SERVER_H

#include "esp_err.h"

// Arranca el servidor HTTP: /stream (video MJPEG, puerto 80) y /status (estado en JSON, puerto 81).
esp_err_t stream_server_start(void);

// Texto del motivo del ultimo reinicio (POWERON, BROWNOUT, PANIC...), para el log y /status.
const char *stream_server_razon_reinicio(void);

#endif
