# Deteccion de lugares: darse cuenta de que el robot llego a una curva, una
# bifurcacion o un callejon, mirando los ultrasonidos.
#
# El diseno esta en ros/TICKETS_NAVEGACION.md, T2, "Deteccion de lugares".
#
# Son solo cuentas, sin ROS: reciben numeros y devuelven numeros, asi se pueden
# probar sin robot y sin simulador. Las distancias van en cm.
# None significa "el sensor no tiene dato".

import math


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



def frente_muy_cerca(frente_cm, distancia_cm):
    """True si el frente esta mas cerca que distancia_cm. None si no hay dato.

    Sirve para confirmar un costado abierto sin esperar las N lecturas: si el
    robot esperara, podria llegar a la pared antes de terminar de contar.
    Un "no se sabe" (None) no saltea la espera.
    """
    if frente_cm is None:
        return None
    return frente_cm <= distancia_cm

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



class Promedio:
    """Va sumando las lecturas de un costado y dice su promedio.

    Hay uno para cada costado: el robot no siempre va justo en el medio, y asi
    cada costado se compara contra lo que venia midiendo el mismo.
    """

    def __init__(self):
        self.suma = 0.0
        self.cantidad = 0

    def agregar(self, medida_cm):
        """Suma una lectura. Ignora None (sin dato) e inf (sin eco): no son una pared."""
        if medida_cm is None or math.isinf(medida_cm):
            return
        self.suma += medida_cm
        self.cantidad += 1

    def reiniciar(self):
        """Borra todo y arranca de cero (se usa cuando aparece una pared nueva)."""
        self.suma = 0.0
        self.cantidad = 0

    def valor(self):
        """El promedio, o None si todavia no sumo ninguna lectura."""
        if self.cantidad == 0:
            return None
        return self.suma / self.cantidad

# ===================== Maquina de estados =====================
# Se acuerda de en que etapa esta el robot, para avisar una sola vez por lugar
# (como tocar el timbre una vez y esperar, en vez de tocarlo sin parar).

# Las etapas del detector. Constantes, igual que PENDIENTE y PROBADA en libreta.py.
EN_PASILLO = 'en_pasillo'          # mirando los costados, cada uno con su promedio
MIDIENDO_LUGAR = 'midiendo_lugar'  # vio un costado abierto, decide que lugar es
AVISADO = 'avisado'                # ya aviso, espera a Decision (el timbre ya se toco)


class DetectorDeLugares:
    """Se acuerda de en que etapa esta el robot, para avisar una sola vez por lugar.

    Cada costado tiene su Promedio y su Confirmador. Con el mismo salto_cm:
    - si un costado sube de golpe respecto de su promedio, se abrio una salida;
    - si baja de golpe (bajon), aparecio una pared nueva: su promedio se borra y
      arranca de cero. Asi, despues de girar, el lado que miraba al pasillo de donde
      venia se arregla solo cuando aparece la pared.
    """

    def __init__(self, salto_cm, lecturas_necesarias, distancia_frente_cm):
        self.salto_cm = salto_cm
        self.distancia_frente_cm = distancia_frente_cm
        self.etapa = EN_PASILLO
        self.promedio_izquierda = Promedio()
        self.promedio_derecha = Promedio()
        self.confirmador_izquierda = Confirmador(lecturas_necesarias)
        self.confirmador_derecha = Confirmador(lecturas_necesarias)
        self.frente_inicial_cm = None  # F0: el frente cuando se confirmo el costado abierto

    def actualizar(self, izquierda_cm, derecha_cm, frente_cm):
        """Recibe las lecturas de los tres ultrasonidos y cambia de etapa si corresponde."""
        if self.etapa == EN_PASILLO:
            izquierda = self._mirar_costado(izquierda_cm, self.promedio_izquierda,
                                            self.confirmador_izquierda, frente_cm)
            derecha = self._mirar_costado(derecha_cm, self.promedio_derecha,
                                          self.confirmador_derecha, frente_cm)
            # F0 tiene que ser un numero de verdad: con None no se sabe, y con inf
            # F0 / 2 tambien seria infinito y creeria llegar a la esquina enseguida.
            # Si no lo es, sigue esperando aca con el costado ya confirmado.
            frente_sirve = frente_cm is not None and math.isfinite(frente_cm)
            if (izquierda or derecha) and frente_sirve:
                self.frente_inicial_cm = frente_cm
                self.etapa = MIDIENDO_LUGAR

    def _mirar_costado(self, medida_cm, promedio, confirmador, frente_cm):
        """True si este costado quedo confirmado como abierto."""
        valor = promedio.valor()
        if medida_cm is None:
            abierto = None                      # sin dato: no se sabe
        elif valor is None:
            promedio.agregar(medida_cm)         # primera pared de este lado
            abierto = None                      # sin promedio todavia: no se sabe
        elif valor - medida_cm >= self.salto_cm:
            promedio.reiniciar()                # bajon: aparecio una pared nueva
            promedio.agregar(medida_cm)
            abierto = False
        else:
            abierto = costado_abierto(medida_cm, valor, self.salto_cm)
            if abierto is False:
                promedio.agregar(medida_cm)     # solo se suma cuando esta cerrado
        confirmado = confirmador.agregar(abierto)
        if abierto is True and frente_muy_cerca(frente_cm, self.distancia_frente_cm):
            confirmado = True                   # frente cerca: no espera las N lecturas
        return confirmado
