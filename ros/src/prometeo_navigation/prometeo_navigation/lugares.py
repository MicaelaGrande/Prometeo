# Deteccion de lugares: darse cuenta de que el robot llego a una curva, una
# bifurcacion o un callejon, mirando los ultrasonidos.
#
# El diseno esta en ros/TICKETS_NAVEGACION.md, T2, "Deteccion de lugares".
#
# Son solo cuentas, sin ROS: reciben numeros y devuelven numeros, asi se pueden
# probar sin robot y sin simulador. Las distancias van en cm.
# None significa "el sensor no tiene dato".


def entro_al_pasillo(izquierda_cm, derecha_cm, margen_cm):
    """True si el robot ya esta dentro de un pasillo: las dos paredes a distancias parecidas.

    Si un sensor no tiene dato (None), todavia no sabemos, asi que devuelve False.
    """
    if izquierda_cm is None or derecha_cm is None:
        return False
    return abs(izquierda_cm - derecha_cm) <= margen_cm


def costado_abierto(medida_cm, promedio_cm, salto_cm):
    """True si este costado mide mucho mas que el promedio del pasillo, False si no.

    None si no hay dato: no se sabe. inf (sin eco, nada cerca) cuenta como abierto,
    y no necesita un caso aparte: infinito menos el promedio sigue siendo infinito.
    """
    if medida_cm is None:
        return None
    return medida_cm - promedio_cm >= salto_cm


def lectura_vigente(medida_cm, hora_s, ahora_s, edad_maxima_s):
    """La medida si es reciente; None si nunca llego o es mas vieja que edad_maxima_s.

    Cada ultrasonido manda su lectura por separado. Si uno deja de mandar (por ejemplo,
    se corta el WiFi de la ESP32), el nodo no tiene que seguir confiando en la ultima:
    pasa a ser "no se sabe".
    """
    if hora_s is None or ahora_s - hora_s > edad_maxima_s:
        return None
    return medida_cm


class Confirmador:
    """Confirma que un costado esta abierto cuando lo ve N veces seguidas.

    Cada costado tiene su propio Confirmador, asi no comparten la cuenta.
    Abierto suma 1, cerrado vuelve a 0 y "no se sabe" (None) deja la cuenta igual.
    """

    def __init__(self, lecturas_necesarias):
        self.lecturas_necesarias = lecturas_necesarias
        self.cuenta = 0

    def agregar(self, abierto):
        """Recibe True, False o None. Devuelve True si ya esta confirmado."""
        if abierto is True:
            self.cuenta += 1
        elif abierto is False:
            self.cuenta = 0
        # None (no se sabe): la cuenta queda igual
        return self.cuenta >= self.lecturas_necesarias
