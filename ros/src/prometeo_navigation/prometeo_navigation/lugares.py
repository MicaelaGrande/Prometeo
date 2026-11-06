# Deteccion de lugares: darse cuenta de que el robot llego a una curva, una
# bifurcacion o un callejon, mirando los ultrasonidos.
#
# El diseno esta en ros/TICKETS_NAVEGACION.md, T2, "Deteccion de lugares".
#
# Son solo cuentas, sin ROS: reciben numeros y devuelven numeros, asi se pueden
# probar sin robot y sin simulador. Las distancias van en cm.
# None significa "el sensor no tiene dato".
# De ROS solo usa las constantes de Lugar (los textos que entiende Decision).

import math

from prometeo_interfaces.msg import Lugar


def entro_al_pasillo(izquierda_cm, derecha_cm, margen_cm):
    """True si el robot ya esta dentro de un pasillo: las dos paredes a distancias parecidas.

    Si un sensor no tiene dato (None), todavia no sabemos, asi que devuelve False.
    """
    if izquierda_cm is None or derecha_cm is None:
        return False
    return abs(izquierda_cm - derecha_cm) <= margen_cm


# Lo que manda la ESP32 (y hcsr04_sim en Gazebo) en /distancias, en cm.
# Igual que en micro_ros/main/sensores.h y prometeo_sensores/distancias_a_range.py.
DIST_LEJOS_CM = 400.0   # esto o mas = no hubo eco (no hay nada adelante)


def distancia_a_lectura(cm):
    """Pasa un valor de /distancias a lo que entiende el detector.

    Negativo = el sensor fallo -> None (no se sabe).
    DIST_LEJOS_CM o mas = sin eco -> inf (lejos, abierto).
    Si no, el mismo numero en cm.
    """
    if cm is None or math.isnan(cm) or cm < 0:
        return None
    if cm >= DIST_LEJOS_CM:
        return math.inf
    return float(cm)


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


def armar_lugar(izquierda_abierta, adelante_abierto, derecha_abierta):
    """Con lo que quedo abierto, arma (tipo, salidas) para avisarle a Decision.

    Es el traductor entre el detector y Decision: recibe tres si/no y devuelve
    lo que va en /navegacion/lugar. El tipo sale de contar las salidas:
    0 = callejon, 1 = curva, 2 o mas = bifurcacion.
    Las salidas van siempre en el mismo orden: izquierda, adelante, derecha.
    Los textos son las constantes de Lugar.msg, escritos una sola vez para
    navegacion y Decision.
    """
    salidas = [nombre for nombre, abierta in (
        (Lugar.IZQUIERDA, izquierda_abierta),
        (Lugar.ADELANTE, adelante_abierto),
        (Lugar.DERECHA, derecha_abierta)) if abierta]
    if len(salidas) == 0:
        return Lugar.CALLEJON, salidas
    if len(salidas) == 1:
        return Lugar.CURVA, salidas
    return Lugar.BIFURCACION, salidas


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
        self.lecturas_necesarias = lecturas_necesarias
        self.distancia_frente_cm = distancia_frente_cm
        self.reiniciar()

    def reiniciar(self):
        """Vuelve a EN_PASILLO y se olvida de todo, como recien creado.

        Lo llama el nodo cuando el robot termino de salir del lugar (termino de
        girar o de pasar de largo), para buscar el siguiente.
        Tambien borra los promedios: despues de girar, los costados miran paredes
        nuevas. El lado que mira al pasillo de donde vino arranca su promedio con
        esa medida, y se arregla solo con el bajon cuando aparece la pared.
        """
        self.etapa = EN_PASILLO
        self.promedio_izquierda = Promedio()
        self.promedio_derecha = Promedio()
        self.confirmador_izquierda = Confirmador(self.lecturas_necesarias)
        self.confirmador_derecha = Confirmador(self.lecturas_necesarias)
        # Para el callejon: cuenta las lecturas con el frente muy cerca
        self.confirmador_frente = Confirmador(self.lecturas_necesarias)
        # Para la puerta: cuentan las lecturas "cerrado" de un costado que estaba abierto
        self.cierre_izquierda = Confirmador(self.lecturas_necesarias)
        self.cierre_derecha = Confirmador(self.lecturas_necesarias)
        self.puerta_confirmada = False
        self.frente_inicial_cm = None  # F0: el frente cuando se confirmo el costado abierto
        self.frente_final_cm = None    # F1: el frente cuando se confirmo la puerta
        # Que costados quedaron abiertos en este lugar (se anotan al pasar a MIDIENDO_LUGAR)
        self.izquierda_abierta = False
        self.derecha_abierta = False

    def actualizar(self, izquierda_cm, derecha_cm, frente_cm):
        """Recibe las lecturas de los tres ultrasonidos y cambia de etapa si corresponde.

        Devuelve (tipo, salidas) la UNICA vez que avisa un lugar (como lo arma
        armar_lugar). El resto del tiempo devuelve None.
        """
        if self.etapa == MIDIENDO_LUGAR:
            return self._medir_lugar(izquierda_cm, derecha_cm, frente_cm)
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
                self.izquierda_abierta = izquierda
                self.derecha_abierta = derecha
                self.etapa = MIDIENDO_LUGAR
                return None

            # Callejon: ningun costado abierto y el frente muy cerca, N lecturas
            # seguidas (una lectura mala no lo inventa). Si hay un cubo adelante, el
            # frente es su cilindro: igual avisa, y Decision, que esta acercandose
            # al cubo, no le hace caso.
            if not (izquierda or derecha):
                cerca = frente_muy_cerca(frente_cm, self.distancia_frente_cm)
                if self.confirmador_frente.agregar(cerca):
                    self.etapa = AVISADO
                    return armar_lugar(False, False, False)
        return None

    def _medir_lugar(self, izquierda_cm, derecha_cm, frente_cm):
        """MIDIENDO_LUGAR: sigue avanzando hasta saber que lugar es.

        Puerta: un costado que se habia abierto se vuelve a cerrar antes de la
        esquina. El pasillo sigue derecho: adelante abierto. Se anota F1.
        Esquina: el frente llega a F0 / 2. Adelante hay pared, y el robot quedo en
        el medio de la salida, listo para girar.
        Primero se mira la puerta: si el costado se cerro, ya no es una esquina.
        """
        # Si el otro costado se abre recien ahora, tambien cuenta: es una T.
        if not self.izquierda_abierta:
            self.izquierda_abierta = self._mirar_costado(
                izquierda_cm, self.promedio_izquierda, self.confirmador_izquierda, frente_cm)
        if not self.derecha_abierta:
            self.derecha_abierta = self._mirar_costado(
                derecha_cm, self.promedio_derecha, self.confirmador_derecha, frente_cm)

        # Puerta: confirmada con N lecturas "cerrado", igual que al abrirse, para que
        # una lectura mala no la invente.
        if self.izquierda_abierta and self._se_cerro(
                izquierda_cm, self.promedio_izquierda, self.cierre_izquierda):
            self.puerta_confirmada = True
        if self.derecha_abierta and self._se_cerro(
                derecha_cm, self.promedio_derecha, self.cierre_derecha):
            self.puerta_confirmada = True

        if self.puerta_confirmada:
            # F1 tiene que ser un numero, como F0: sin dato o sin eco, espera aca.
            if frente_cm is not None and math.isfinite(frente_cm):
                self.frente_final_cm = frente_cm
                self.etapa = AVISADO
                return armar_lugar(self.izquierda_abierta, True, self.derecha_abierta)
            return None

        # Esquina: el frente llego a la mitad de F0. Sin dato (None) espera, y sin
        # eco (inf) nunca es menor: adelante esta libre, no es una esquina.
        if frente_cm is not None and frente_cm <= self.frente_inicial_cm / 2:
            self.etapa = AVISADO
            return armar_lugar(self.izquierda_abierta, False, self.derecha_abierta)
        return None

    def _se_cerro(self, medida_cm, promedio, cierre):
        """True si un costado abierto quedo confirmado como cerrado otra vez (volvio la pared).

        Compara con el promedio del pasillo, que no cambio mientras estaba abierto
        (solo se suma cuando esta cerrado). Sin dato deja la cuenta igual.
        """
        abierto = None if medida_cm is None else costado_abierto(
            medida_cm, promedio.valor(), self.salto_cm)
        cerrado = None if abierto is None else not abierto
        return cierre.agregar(cerrado)

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
