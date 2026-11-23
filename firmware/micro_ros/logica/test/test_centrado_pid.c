/*
 * Pruebas en la compu de logica/centrado_pid.h (solo matematica, sin ESP-IDF).
 * Correr con: firmware/scripts/test_logica.sh
 */
#include <math.h>
#include <stdio.h>

#include "../centrado_pid.h"

static int fallas = 0;
static int pruebas = 0;

#define CERCA(real, esperado, nombre) do {                                          \
    pruebas++;                                                                      \
    double r_ = (real), e_ = (esperado);                                            \
    if (fabs(r_ - e_) > 1e-4) {                                                     \
        printf("FALLA %s: dio %.6f, esperaba %.6f\n", nombre, r_, e_);              \
        fallas++;                                                                   \
    }                                                                               \
} while (0)

#define VERDAD(cond, nombre) do {                                                   \
    pruebas++;                                                                      \
    if (!(cond)) { printf("FALLA %s\n", nombre); fallas++; }                        \
} while (0)

static const centrado_gan_t SOLO_P = { .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .w_max = 0.4f };
static const centrado_gan_t SOLO_I = { .kp = 0.0f, .ki = 1.0f, .kd = 0.0f, .w_max = 0.4f };
static const centrado_gan_t SOLO_D = { .kp = 0.0f, .ki = 0.0f, .kd = 1.0f, .w_max = 10.0f };

static void en_el_medio_no_gira(void)
{
    centrado_pid_t p; centrado_pid_reiniciar(&p);
    CERCA(centrado_pid_paso(&p, &SOLO_P, 0.0f, 0.1f), 0.0, "en el medio no gira");
}

static void proporcional_con_el_signo_correcto(void)
{
    centrado_pid_t p; centrado_pid_reiniciar(&p);
    // e > 0 = pegado a la pared derecha: hay que girar a la izquierda (+)
    CERCA(centrado_pid_paso(&p, &SOLO_P, 0.3f, 0.1f), 0.3, "P: e>0 gira a la izquierda");
    centrado_pid_reiniciar(&p);
    CERCA(centrado_pid_paso(&p, &SOLO_P, -0.3f, 0.1f), -0.3, "P: e<0 gira a la derecha");
}

static void la_salida_tiene_tope(void)
{
    centrado_pid_t p; centrado_pid_reiniciar(&p);
    CERCA(centrado_pid_paso(&p, &SOLO_P, 1.0f, 0.1f), 0.4, "tope positivo");
    centrado_pid_reiniciar(&p);
    CERCA(centrado_pid_paso(&p, &SOLO_P, -1.0f, 0.1f), -0.4, "tope negativo");
}

static void la_integral_acumula_y_no_se_dispara(void)
{
    centrado_pid_t p; centrado_pid_reiniciar(&p);
    // ki*e*dt = 0.1 por paso
    CERCA(centrado_pid_paso(&p, &SOLO_I, 1.0f, 0.1f), 0.1, "I: primer paso");
    CERCA(centrado_pid_paso(&p, &SOLO_I, 1.0f, 0.1f), 0.2, "I: acumula");
    for (int i = 0; i < 100; i++) {
        centrado_pid_paso(&p, &SOLO_I, 1.0f, 0.1f);
    }
    CERCA(p.integral, 0.4, "anti-windup: la integral queda en w_max");
    // al cambiar el signo del error tiene que bajar enseguida, no desde 100
    centrado_pid_paso(&p, &SOLO_I, -1.0f, 0.1f);
    CERCA(p.integral, 0.3, "anti-windup: vuelve sin demora");
}

static void el_primer_paso_no_tiene_derivada(void)
{
    centrado_pid_t p; centrado_pid_reiniciar(&p);
    CERCA(centrado_pid_paso(&p, &SOLO_D, 0.5f, 0.1f), 0.0, "D: primer paso sin derivada");
}

static void la_derivada_esta_filtrada(void)
{
    centrado_pid_t p; centrado_pid_reiniciar(&p);
    centrado_pid_paso(&p, &SOLO_D, 0.0f, 0.1f);
    // d = (0.1 - 0) / 0.1 = 1; filtrada: 0.4 * 1 = 0.4
    CERCA(centrado_pid_paso(&p, &SOLO_D, 0.1f, 0.1f), CENTRADO_DERIV_ALFA * 1.0, "D: filtro paso 1");
    // error constante: d = 0, la derivada baja un 40% hacia 0: 0.4 * (1 - 0.4) = 0.24
    CERCA(centrado_pid_paso(&p, &SOLO_D, 0.1f, 0.1f), CENTRADO_DERIV_ALFA * (1 - CENTRADO_DERIV_ALFA), "D: filtro paso 2");
}

static void reiniciar_borra_todo(void)
{
    centrado_pid_t p; centrado_pid_reiniciar(&p);
    centrado_pid_paso(&p, &SOLO_I, 1.0f, 0.1f);
    centrado_pid_paso(&p, &SOLO_D, 0.5f, 0.1f);
    centrado_pid_reiniciar(&p);
    CERCA(p.integral, 0.0, "reiniciar: integral");
    CERCA(p.deriv, 0.0, "reiniciar: derivada");
    VERDAD(!p.tiene_prev, "reiniciar: sin paso previo");
}

static void limitar_es_simetrico(void)
{
    CERCA(centrado_limitar(5.0f, 2.0f), 2.0, "limitar: arriba");
    CERCA(centrado_limitar(-5.0f, 2.0f), -2.0, "limitar: abajo");
    CERCA(centrado_limitar(1.0f, 2.0f), 1.0, "limitar: dentro del rango");
}

int main(void)
{
    en_el_medio_no_gira();
    proporcional_con_el_signo_correcto();
    la_salida_tiene_tope();
    la_integral_acumula_y_no_se_dispara();
    el_primer_paso_no_tiene_derivada();
    la_derivada_esta_filtrada();
    reiniciar_borra_todo();
    limitar_es_simetrico();

    printf("%d comprobaciones, %d fallas\n", pruebas, fallas);
    return fallas ? 1 : 0;
}
