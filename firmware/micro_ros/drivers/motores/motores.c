#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "motores.h"
#include "encoder.h"
#include "sdkconfig.h"

static const char *TAG = "motores";

typedef struct { ledc_channel_t ia, ib; float trim; float speed;} motor_t;

// {pin IA, pin IB} de cada motor
static const int pines[CANT_MOTORES][2] = {
    {MOTOR_A1, MOTOR_A2},
    {MOTOR_B1, MOTOR_B2},
    {MOTOR_C1, MOTOR_C2},
    {MOTOR_D1, MOTOR_D2},
};

// {IA, IB, trim} de cada motor
static motor_t motors[CANT_MOTORES] = {
    {LEDC_CHANNEL_0, LEDC_CHANNEL_1, 1.0f, 0.0f},  // front-left
    {LEDC_CHANNEL_2, LEDC_CHANNEL_3, 1.0f, 0.0f},  // rear-left
    {LEDC_CHANNEL_4, LEDC_CHANNEL_5, 1.0f, 0.0f},  // front-right
    {LEDC_CHANNEL_6, LEDC_CHANNEL_7, 1.0f, 0.0f},  // rear-right
};
static volatile bool en_falla = true;   // arranca frenado hasta la primera orden

// Qué motores van de cada lado (AJUSTAR según el chasis)
// A = delantero izq, B = trasero izq, C = delantero der, D = trasero der (igual que en menuconfig)
static const motor_id_t lado_izq[] = { MOTOR_A, MOTOR_B };
static const motor_id_t lado_der[] = { MOTOR_C, MOTOR_D };

#define PWM_FREC_HZ CONFIG_PROMETEO_PWM_FREQ_HZ
#define PWM_RES_BIT CONFIG_PROMETEO_PWM_RES_BIT
#define PWM_MAX     ((1 << PWM_RES_BIT) - 1)


static void _set_duty(ledc_channel_t canal, uint32_t valor)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, valor);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, canal); // Necesario para LOW_SPEED_MODE
}

/*
 * @brief Devuelve el duty que quedo aplicado (0..1, sin signo).
 * Aplica el duty de manera progresiva.
 */
static float _quick_set(motor_t * mot_s, int new_speed)
{
    new_speed = (int)(new_speed * mot_s->trim);
    // Clamps
    if (new_speed >  PWM_MAX) new_speed =  PWM_MAX;
    if (new_speed < -PWM_MAX) new_speed = -PWM_MAX;
    // Ramp (OLD)
    // int abs_diff = fabsf(new_speed - mot_s->speed);
    // new_speed = (abs_diff > RAMPA_PASO) ? mot_s->speed + RAMPA_PASO * ((new_speed > mot_s->speed) ? 1 : -1) : new_speed;
    // Set
    if (new_speed >= 0) {
        _set_duty(mot_s->ib, 0);
        _set_duty(mot_s->ia, new_speed);
    } else {
        _set_duty(mot_s->ia, 0);
        _set_duty(mot_s->ib, -new_speed);
    }
    mot_s->speed = new_speed;
    return new_speed;
}

static int _ramp(int current, int target) {
    int old = current;
    bool reversing = (target > 0 && current < 0) || (target < 0 && current > 0);
    bool slowing   = reversing || abs(target) < abs(current);
    int step = slowing ? DECEL_STEP : ACCEL_STEP;

    if (current == 0 && target != 0)                      // skip dead zone
        current = target > 0 ? MIN_DUTY : -MIN_DUTY;

    if (target > current) current = (current + step > target) ? target : current + step;
    else if (target < current) current = (current - step < target) ? target : current - step;

    // Never cross zero in one step: stop at 0 for one period first
    if ((old > 0 && current < 0) || (old < 0 && current > 0)) current = 0;

    // Slowing into the dead zone (to stop or reverse): go to 0 instead of humming
    if (abs(current) < MIN_DUTY && (abs(target) < MIN_DUTY || reversing)) current = 0;

    return current;
}

static float _ramp_set(motor_t * mot_s, int new_speed)
{
    new_speed = _ramp(mot_s->speed, new_speed);
    return _quick_set(mot_s, new_speed);
}

/*
 * @brief Funcion que configura los perifericos para
 * poder utilizar los motores.
 */
void motores_init(void)
{
    // Configurar timer del periferico
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RES_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREC_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Configurar canales del periferico
    for (int i = 0; i < CANT_MOTORES * 2; i++) {
        ledc_channel_config_t canal = {
            .gpio_num = pines[i / 2][i % 2],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&canal));
    }
    // bloquear los motores
    motores_lock(true);
}

/*
 * @brief funcion que setea un duty en un motor particular
 */
void motores_single_set_ramp(motor_id_t m, float v)
{
    if (en_falla) return;
    if (m < 0 || m >= CANT_MOTORES) return;
    _ramp_set(&(motors[m]), (int)(v * PWM_MAX));
    en_falla = false;
}

/*
 * @brief Funcion que intenta liberar motores bloqueados
 * por falla o falta de inrtrucciones
 */
void motores_unlock(void)
{
    en_falla = false;
    return;
}

void motores_lock(bool falla)
{
    for (int m = 0; m < CANT_MOTORES; m++) {
        _quick_set(&(motors[m]), 0);
    }
    en_falla = true;
}

bool motores_get_falla(void) { return en_falla; }

int motores_get_sentido(motor_id_t m) {
    if (m >= CANT_MOTORES) return 1;
    int speed = motors[m].speed;
    return (speed > 0) ? speed/abs(speed) : 1;
}

int motores_get_duty(motor_id_t m) { return (m < CANT_MOTORES) ? motors[m].speed/PWM_MAX : 1; }
