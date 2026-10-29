#include "freertos/FreeRTOS.h"   // solo para la sección crítica (portMUX); no se crean tareas
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "mg90s.h"

// ===== PWM =====
// LEDC high-speed (los motores usan todos los canales low-speed y el timer low-speed 0)
#define PWM_FREC_HZ       50
#define PWM_PERIODO_US    20000          // 1 / 50 Hz
#define PWM_RESOLUCION    (1 << 16)      // 16 bits
#define PWM_HPOINT_PASO   4000           // desfasa los pulsos para no cargar todos a la vez

// ===== RAMPA =====
#define RAMPA_PERIODO_US  20000          // cada cuánto se actualiza (esp_timer, sin tarea propia)
#define RAMPA_DT_S        0.020f
#define UMBRAL_LLEGADA    0.01f          // grados: menos que esto se considera "llegó"

// ===== PULSO EN CRUDO =====
#define US_CRUDO_MIN      400
#define US_CRUDO_MAX      2600

// ===== CONFIGURACIÓN DE CADA SERVO =====
typedef struct {
    int   pin;
    int   us_min;      // pulso a 0°
    int   us_max;      // pulso a 180°
    float lim_min;     // límite por software (grados)
    float lim_max;
    float home;        // posición inicial (grados)
    bool  invertido;
} config_servo_t;

// AJUSTAR: pines libres (los de motores y sensores HC-SR04 ya están usados).
// 2, 5, 12 y 15 son pines de arranque: no conectar nada que los fuerce al encender.
// AJUSTAR: us_min / us_max y límites según cada servo (sacar el rango real probando con servo_set_us).
static const config_servo_t cfg[CANT_SERVOS] = {
    //  pin  us_min us_max lim_min lim_max home  invertido
    {   5,   500,   2400,  0,      180,    90,   false },  // SERVO_1
    {  14,   500,   2400,  0,      180,    90,   false },  // SERVO_2
    {  15,   500,   2400,  0,      180,    90,   false },  // SERVO_3
    {   2,   500,   2400,  0,      180,    90,   false },  // SERVO_4
    {  12,   500,   2400,  0,      180,    90,   false },  // SERVO_5 (GPIO12 = MTDI, el más delicado)
};

// ===== ESTADO COMPARTIDO =====
// Lo escriben las funciones públicas y lo lee/actualiza el callback de la rampa,
// siempre dentro de la sección crítica.
static float    objetivo[CANT_SERVOS];
static float    actual[CANT_SERVOS];
static float    vel_gps[CANT_SERVOS];       // <= 0 -> inmediato
static bool     enganchado[CANT_SERVOS];
static uint16_t us_crudo[CANT_SERVOS];      // 0 -> usar el ángulo
static portMUX_TYPE cerrojo = portMUX_INITIALIZER_UNLOCKED;

// Solo lo toca el callback de la rampa (único que escribe en LEDC)
static uint32_t duty_escrito[CANT_SERVOS];
static bool     iniciado = false;

// ===== CONVERSIONES =====
static float limitar(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float limitar_angulo(int s, float grados)
{
    return limitar(grados, cfg[s].lim_min, cfg[s].lim_max);
}

// Ángulo -> pulso en us: limita, invierte si hace falta y mapea 0..180 -> us_min..us_max
static uint32_t angulo_a_us(int s, float grados)
{
    grados = limitar_angulo(s, grados);
    if (cfg[s].invertido) grados = 180.0f - grados;
    float us = cfg[s].us_min + grados / 180.0f * (cfg[s].us_max - cfg[s].us_min);
    return (uint32_t)(us + 0.5f);
}

// Pulso en us -> duty (enteros, redondeado)
static uint32_t us_a_duty(uint32_t us)
{
    return (us * PWM_RESOLUCION + PWM_PERIODO_US / 2) / PWM_PERIODO_US;
}

static bool id_valido(servo_id_t s)
{
    return (unsigned)s < CANT_SERVOS;
}

// ===== RAMPA =====
// Único lugar que escribe en LEDC (un canal no se debe escribir desde varias tareas)
static void rampa_cb(void *arg)
{
    (void)arg;
    uint32_t duty[CANT_SERVOS];

    portENTER_CRITICAL(&cerrojo);
    for (int s = 0; s < CANT_SERVOS; s++) {
        if (!enganchado[s]) {            // soltado -> sin pulsos
            duty[s] = 0;
            continue;
        }
        float d = objetivo[s] - actual[s];
        float paso = vel_gps[s] * RAMPA_DT_S;
        if (vel_gps[s] <= 0 || (d <= paso && d >= -paso)) {
            actual[s] = objetivo[s];     // inmediato, o llega en este ciclo
        } else {
            actual[s] += (d > 0) ? paso : -paso;
        }
        duty[s] = us_a_duty(us_crudo[s] ? us_crudo[s] : angulo_a_us(s, actual[s]));
    }
    portEXIT_CRITICAL(&cerrojo);

    // LEDC conserva el último duty, así que solo se escribe si cambió
    for (int s = 0; s < CANT_SERVOS; s++) {
        if (duty[s] == duty_escrito[s]) continue;
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)s, duty[s]);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)s);
        duty_escrito[s] = duty[s];
    }
}

// ===== CONFIGURACIÓN =====
void servos_init(void)
{
    if (iniciado) return;

    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = PWM_FREC_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int s = 0; s < CANT_SERVOS; s++) {
        float home = limitar_angulo(s, cfg[s].home);
        objetivo[s] = actual[s] = home;
        vel_gps[s] = 0;
        enganchado[s] = true;
        us_crudo[s] = 0;
        duty_escrito[s] = us_a_duty(angulo_a_us(s, home));

        // El canal arranca ya con el duty de home: el primer pulso es el correcto (sin salto)
        ledc_channel_config_t canal = {
            .gpio_num   = cfg[s].pin,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel    = (ledc_channel_t)s,
            .timer_sel  = LEDC_TIMER_1,
            .duty       = duty_escrito[s],
            .hpoint     = s * PWM_HPOINT_PASO
        };
        ESP_ERROR_CHECK(ledc_channel_config(&canal));
    }

    esp_timer_create_args_t args = {
        .callback = rampa_cb,
        .name     = "servos"
    };
    esp_timer_handle_t t;
    ESP_ERROR_CHECK(esp_timer_create(&args, &t));
    ESP_ERROR_CHECK(esp_timer_start_periodic(t, RAMPA_PERIODO_US));
    iniciado = true;
}

// ===== MOVIMIENTO =====
void servo_set_angulo(servo_id_t s, float grados)
{
    if (!id_valido(s)) return;
    grados = limitar_angulo(s, grados);
    portENTER_CRITICAL(&cerrojo);
    objetivo[s] = actual[s] = grados;
    vel_gps[s] = 0;
    us_crudo[s] = 0;
    portEXIT_CRITICAL(&cerrojo);
}

void servo_mover(servo_id_t s, float grados, float vel)
{
    if (!id_valido(s)) return;
    grados = limitar_angulo(s, grados);
    portENTER_CRITICAL(&cerrojo);
    objetivo[s] = grados;
    vel_gps[s] = vel;
    us_crudo[s] = 0;
    portEXIT_CRITICAL(&cerrojo);
}

void servos_mover_sincronizado(const float grados[], uint32_t tiempo_ms)
{
    float meta[CANT_SERVOS];
    for (int s = 0; s < CANT_SERVOS; s++) meta[s] = limitar_angulo(s, grados[s]);

    portENTER_CRITICAL(&cerrojo);
    for (int s = 0; s < CANT_SERVOS; s++) {
        float dist = meta[s] - actual[s];
        if (dist < 0) dist = -dist;
        objetivo[s] = meta[s];
        // misma duración para todos: cada uno con su velocidad (0 = inmediato)
        vel_gps[s] = (tiempo_ms == 0) ? 0 : dist / (tiempo_ms / 1000.0f);
        us_crudo[s] = 0;
    }
    portEXIT_CRITICAL(&cerrojo);
}

void servos_home(float vel)
{
    for (int s = 0; s < CANT_SERVOS; s++) servo_mover((servo_id_t)s, cfg[s].home, vel);
}

void servos_detener(void)
{
    portENTER_CRITICAL(&cerrojo);
    for (int s = 0; s < CANT_SERVOS; s++) objetivo[s] = actual[s];
    portEXIT_CRITICAL(&cerrojo);
}

// ===== ESTADO =====
float servo_get_angulo(servo_id_t s)
{
    if (!id_valido(s)) return 0;
    portENTER_CRITICAL(&cerrojo);
    float v = objetivo[s];
    portEXIT_CRITICAL(&cerrojo);
    return v;
}

bool servo_en_movimiento(servo_id_t s)
{
    if (!id_valido(s)) return false;
    portENTER_CRITICAL(&cerrojo);
    float d = objetivo[s] - actual[s];
    bool mueve = enganchado[s] && (d > UMBRAL_LLEGADA || d < -UMBRAL_LLEGADA);
    portEXIT_CRITICAL(&cerrojo);
    return mueve;
}

bool servos_en_movimiento(void)
{
    for (int s = 0; s < CANT_SERVOS; s++) {
        if (servo_en_movimiento((servo_id_t)s)) return true;
    }
    return false;
}

// ===== ENERGÍA =====
void servo_soltar(servo_id_t s)
{
    if (!id_valido(s)) return;
    portENTER_CRITICAL(&cerrojo);
    enganchado[s] = false;
    portEXIT_CRITICAL(&cerrojo);
}

void servo_enganchar(servo_id_t s)
{
    if (!id_valido(s)) return;
    portENTER_CRITICAL(&cerrojo);
    enganchado[s] = true;
    portEXIT_CRITICAL(&cerrojo);
}

// ===== CALIBRACIÓN =====
void servo_set_us(servo_id_t s, uint16_t us)
{
    if (!id_valido(s)) return;
    if (us < US_CRUDO_MIN) us = US_CRUDO_MIN;
    if (us > US_CRUDO_MAX) us = US_CRUDO_MAX;
    portENTER_CRITICAL(&cerrojo);
    us_crudo[s] = us;
    portEXIT_CRITICAL(&cerrojo);
}
