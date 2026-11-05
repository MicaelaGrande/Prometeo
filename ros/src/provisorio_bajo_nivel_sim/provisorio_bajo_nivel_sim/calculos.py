"""PROVISORIO: las cuentas del centrado y del freno, sin ROS.

Borrar junto con todo el paquete cuando la ESP32 se centre sola (T9). Ver LEEME_BORRAR.md.

Estan separadas del nodo para poder probarlas sin ROS ni Gazebo: reciben numeros y
devuelven numeros. Las distancias van en cm, como en la ESP32, para que sea facil
comparar las dos versiones. None significa "el sensor no tiene dato".

AVISO: el freno es una COPIA de micro_ros/main/seguridad.c, porque ese archivo esta en C
para la ESP32 y no compila en la PC. Si Bajo nivel cambia seguridad.c, actualizar aca.
El centrado no existe todavia en la ESP32: esta escrito de cero.
"""


# ===================== Freno (copia de seguridad.c) =====================

def umbral_freno_cm(velocidad_m_s, t_reaccion_s, t_frenado_s, margen_cm, umbral_min_cm):
    """A que distancia del frente hay que frenar, segun la velocidad.

    Copia de umbral_cm() en seguridad.c: lo que el robot recorre mientras se da cuenta
    (reaccion) y mientras frena, mas un margen. Mas rapido, mas lejos.
    Diferencia: la ESP32 recibe el avance de -1 a 1 y aca llega en m/s.
    """
    velocidad_cm_s = velocidad_m_s * 100.0
    umbral = velocidad_cm_s * (t_reaccion_s + t_frenado_s) + margen_cm
    return max(umbral, umbral_min_cm)


def hay_peligro(velocidad_m_s, frente_cm, umbral_cm):
    """True si avanzar asi es peligroso. Copia de hay_peligro() en seguridad.c.

    Solo cuenta si el robot va hacia adelante, y avanzar sin ver el frente (sin dato)
    tambien es peligro.
    """
    if velocidad_m_s <= 0.0:
        return False  # quieto o marcha atras: el sensor de adelante no ayuda
    return frente_cm is None or frente_cm < umbral_cm


class FrenoEmergencia:
    """El freno, con memoria: recuerda si esta activo. Copia de seguridad_task() en seguridad.c.

    Frena con una sola lectura peligrosa (rapido), pero suelta recien despues de varias
    lecturas libres seguidas (lento), porque de cerca el HC-SR04 da lecturas falsas.
    Ademas suelta a umbral + histeresis, para no frenar y soltar todo el tiempo en el borde.
    """

    def __init__(self, histeresis_cm, lecturas_para_liberar):
        self.histeresis_cm = histeresis_cm
        self.lecturas_para_liberar = lecturas_para_liberar
        self.activo = False
        self.habilitado = True  # False = modo recoger: deja acercarse a un objeto
        self.lecturas_libres = 0

    def actualizar(self, velocidad_m_s, frente_cm, umbral_cm):
        """Recibe una lectura nueva del frente y devuelve si el freno queda activo."""
        if not self.habilitado:
            self.activo = False
            return self.activo

        if not self.activo:
            if hay_peligro(velocidad_m_s, frente_cm, umbral_cm):
                self.activo = True
                self.lecturas_libres = 0
            return self.activo

        # Ya esta frenado: cuenta lecturas libres seguidas para soltar
        libre = frente_cm is not None and frente_cm > umbral_cm + self.histeresis_cm
        if libre:
            self.lecturas_libres += 1
            if self.lecturas_libres >= self.lecturas_para_liberar:
                self.activo = False
        else:
            self.lecturas_libres = 0  # una lectura cerca: empieza a contar de nuevo
        return self.activo


# ===================== Centrado (nuevo) =====================

def debe_centrarse(velocidad_m_s, giro_rad_s, giro_minimo_rad_s):
    """True si hay que corregir el giro para centrarse.

    Solo cuando el robot avanza derecho. Si quien manda (T2 o el teclado) ya esta
    girando, ese giro se respeta: este nodo no decide hacia donde doblar.
    """
    return velocidad_m_s > 0.0 and abs(giro_rad_s) < giro_minimo_rad_s


def giro_para_centrarse(izq_cm, der_cm, ganancia, giro_max_rad_s, distancia_salida_cm):
    """Cuanto girar (rad/s) para quedar en el medio del pasillo.

    Compara las dos paredes, sin suponer un ancho fijo. En ROS, giro positivo es a la
    izquierda: si a la izquierda hay mas lugar, gira a la izquierda, y al reves.

    Devuelve 0 (seguir derecho) si un costado no tiene dato o mide mas que
    distancia_salida_cm: ahi probablemente hay una salida, y centrarse tiraria al robot
    hacia el hueco. Hacia donde doblar lo deciden Decision (T7) y Exploracion (T4).
    Es una simplificacion de este nodo provisorio: T2 detecta salidas por el salto de
    la medicion, no por una distancia fija.
    """
    for lado in (izq_cm, der_cm):
        if lado is None or lado > distancia_salida_cm:
            return 0.0

    diferencia_m = (izq_cm - der_cm) / 100.0
    giro = ganancia * diferencia_m
    # Limitar, para que una lectura rara no lo haga girar de golpe
    return max(-giro_max_rad_s, min(giro_max_rad_s, giro))
