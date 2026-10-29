# Librería de servos MG90S (`drivers/mg90s`)

Driver para 5 servos MG90S (PWM de 50 Hz, analógicos, sin realimentación de posición) en un ESP32 clásico.
Archivos: `micro_ros/drivers/mg90s/mg90s.h` y `mg90s.c`.

- **Sin tarea de FreeRTOS:** la rampa corre en un callback periódico de `esp_timer` (20 ms). Lo único de FreeRTOS que se usa es una sección crítica (`portMUX`) para proteger el estado compartido.
- **Un solo escritor:** solo el callback de la rampa llama a `ledc_set_duty` / `ledc_update_duty`. Las funciones públicas solo actualizan el estado, así que se pueden llamar desde cualquier tarea.
- **No toca los motores:** usa el LEDC en modo *high-speed* (timer 1, canales 0–4); `motores.c` usa los 8 canales *low-speed* y el timer low-speed 0.

## 1. Hardware

### Pines (tabla `cfg` al principio de `mg90s.c`)

| Servo | GPIO | Nota |
|-------|------|------|
| `SERVO_1` | 5  | pin de arranque (strapping) |
| `SERVO_2` | 14 | |
| `SERVO_3` | 15 | pin de arranque (imprime en el log de arranque) |
| `SERVO_4` | 2  | pin de arranque (LED integrado en muchas placas) |
| `SERVO_5` | 12 | pin de arranque (MTDI): el más delicado, no debe estar en alto al encender |

### Conexión

- Cable de señal (naranja/amarillo) al GPIO; rojo a 5 V; marrón a GND.
- **GND común** entre la alimentación de los servos y el ESP32.
- Los servos pueden consumir bastante corriente al moverse o al forzarse: conviene alimentarlos con una fuente de 5 V aparte (no desde el pin 3V3 ni desde el regulador de la placa) y poner un capacitor en la alimentación.
- Los pines de arranque no deben tener nada que los fuerce a un nivel al encender (un servo conectado normalmente no molesta, pero verificalo).

## 2. Cómo funciona

### PWM

| Parámetro | Valor |
|-----------|-------|
| Modo / timer | `LEDC_HIGH_SPEED_MODE`, `LEDC_TIMER_1` |
| Frecuencia / resolución | 50 Hz, 16 bits (período 20 000 µs = 65 536 cuentas) |
| Canales | 0–4 (uno por servo) |
| Desfase de pulsos | `hpoint = i * 4000` (los pulsos no arrancan todos a la vez, se reparte el consumo) |
| Pulso → duty | `duty = (us * 65536 + 10000) / 20000` (enteros, redondeado) |

Ejemplos: 500 µs → 1638, 1450 µs → 4751, 2400 µs → 7864.

En `servos_init()` cada canal se configura con el duty de su posición *home*, así que el primer pulso ya es el correcto (sin salto al arrancar).

### Ángulo → pulso

1. Se limita el ángulo a `[lim_min, lim_max]`.
2. Si `invertido`, se usa `180 - ángulo`.
3. Se mapea linealmente 0–180° a `us_min`–`us_max`.

### Rampa

Cada 20 ms el callback, para cada servo enganchado, acerca `actual` a `objetivo` como máximo `vel_gps * 0.020` grados y escribe el duty (solo si cambió; el LEDC conserva el último valor). Un servo soltado recibe duty 0 (sin pulsos). Si `vel_gps <= 0`, el servo salta directo al objetivo.

El servo no informa su posición real: `actual` es la posición que se le **ordenó**. Si el servo se traba o no alcanza, la librería no se entera.

## 3. Configuración (`mg90s.c`)

Una fila por servo en `static const config_servo_t cfg[]` (comentada con `// AJUSTAR`):

| Campo | Significado | Por defecto |
|-------|-------------|-------------|
| `pin` | GPIO de la señal | ver tabla de pines |
| `us_min` | pulso a 0° (µs) | 500 |
| `us_max` | pulso a 180° (µs) | 2400 |
| `lim_min`, `lim_max` | límite por software (°) | 0, 180 |
| `home` | posición inicial (°) | 90 |
| `invertido` | invierte el sentido (`180 - ángulo`) | `false` |

Otras constantes: `RAMPA_PERIODO_US` (20 000), `UMBRAL_LLEGADA` (0,01°), `US_CRUDO_MIN/MAX` (400/2600 µs, límite de seguridad de `servo_set_us`).

## 4. API (`mg90s.h`)

Todas las funciones con `servo_id_t` ignoran ids inválidos. Los ángulos se limitan siempre a `[lim_min, lim_max]`.

### Configuración

| Función | Descripción |
|---------|-------------|
| `void servos_init(void)` | Configura timer y canales, deja cada servo en *home*, crea y arranca el timer de la rampa. Llamarla una vez (una segunda llamada no hace nada). |

### Movimiento

| Función | Descripción |
|---------|-------------|
| `servo_set_angulo(s, grados)` | Salto inmediato: `objetivo = actual = grados`. |
| `servo_mover(s, grados, vel_gps)` | Mueve con rampa a `vel_gps` grados/segundo. `vel_gps <= 0` → inmediato. |
| `servos_mover_sincronizado(grados[], tiempo_ms)` | Mueve los 5 servos (arreglo de `CANT_SERVOS` ángulos) para que lleguen juntos. Velocidad de cada uno = `|objetivo - actual| / (tiempo_ms/1000)`. `tiempo_ms == 0` → inmediato. |
| `servos_home(vel_gps)` | `servo_mover` de cada servo a su posición *home*. |
| `servos_detener()` | Congela cada servo donde está (`objetivo = actual`). |

### Estado

| Función | Descripción |
|---------|-------------|
| `float servo_get_angulo(s)` | Ángulo **objetivo** (no el actual). |
| `bool servo_en_movimiento(s)` | `true` si está enganchado y todavía no llegó (diferencia > 0,01°). |
| `bool servos_en_movimiento()` | `true` si algún servo se mueve. |

### Energía

| Función | Descripción |
|---------|-------------|
| `servo_soltar(s)` | Deja de mandar pulsos: el servo queda sin fuerza. |
| `servo_enganchar(s)` | Vuelve a mandar pulsos desde la última posición (`actual`). |

Los comandos de movimiento sobre un servo soltado actualizan su estado pero **no lo enganchan**: hace falta `servo_enganchar`.

### Calibración

| Función | Descripción |
|---------|-------------|
| `servo_set_us(s, us)` | Pulso en crudo (se limita a 400–2600 µs), se salta el mapeo de ángulos y los límites. Lo cancela el próximo comando de ángulo. No engancha un servo soltado. |

## 5. Uso

`drivers/CMakeLists.txt` ya incluye `mg90s/mg90s.c`. En `main.c`:

```c
#include "mg90s.h"

void app_main(void)
{
    servos_init();                          // todos en home (90°), sin salto

    servo_mover(SERVO_1, 45, 60);           // a 45° a 60 °/s
    servo_set_angulo(SERVO_2, 120);         // salto inmediato

    float pose[CANT_SERVOS] = { 0, 90, 45, 180, 90 };
    servos_mover_sincronizado(pose, 1500);  // todos llegan en 1,5 s
    while (servos_en_movimiento()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    servos_home(90);                        // vuelta a home a 90 °/s
}
```

## 6. Calibración de un servo

1. Poner `us_min = 500`, `us_max = 2400`, límites 0–180 (valores típicos del MG90S).
2. Con `servo_set_us(s, us)` probar pulsos desde ~500 µs hacia arriba hasta donde el servo llega al tope mecánico **sin zumbar** (si zumba, retroceder: forzarlo contra el tope calienta el motor y gasta engranajes). Ese es `us_min`; repetir hacia 2400 µs para `us_max`.
3. Si el servo trabaja contra un tope mecánico del montaje, reducir `lim_min` / `lim_max` en lugar de tocar `us_min/us_max`.
4. Si el sentido es el contrario al esperado, poner `invertido = true`.
5. Elegir `home` en una posición segura del mecanismo.

## 7. Limitaciones y notas

- **Sin realimentación:** `servo_get_angulo` y `actual` son posiciones ordenadas, no medidas.
- **Resolución:** el pulso se redondea al microsegundo (≈ 0,1°), más fino que la zona muerta del MG90S.
- **Hilos:** se puede llamar desde cualquier tarea; cada función toma una sección crítica corta (sin llamadas a LEDC ni logs dentro). El callback corre en la tarea de `esp_timer` (prioridad `CONFIG_ESP_TIMER_TASK_PRIORITY`); su trabajo por ciclo es de microsegundos.
- **El timer corre siempre:** cada 20 ms aunque no haya movimiento (el ciclo sin cambios no escribe en el LEDC).
- **`servos_mover_sincronizado`:** llega al destino con precisión de un ciclo (±20 ms). Un servo que ya está en destino no se mueve.
- **Solo ESP32 clásico:** el modo `LEDC_HIGH_SPEED_MODE` no existe en S2/S3/C3.
- **Pines de arranque** (2, 5, 12, 15): ver sección 1.
- **Pruebas:** la lógica (conversiones, rampa, sincronización, soltar/enganchar, pulso crudo) se verificó en el PC con cabeceras simuladas de ESP-IDF; falta probar con el hardware.
