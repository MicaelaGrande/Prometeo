# La logica de Decision: el "capitan" del robot.
#
# Decision no mueve motores. Mira lo que ve la camara (flechas y cubos) y los
# lugares que avisa Navegacion alta (curvas, bifurcaciones, callejones), y
# decide que hacer: tomar una salida, dejar que elija Exploracion o acercarse
# a un cubo.
#
# Reglas:
#   - Sigue un color por vez, siempre en el orden rojo, verde y azul.
#     Arranca con el primero que falte, cuando ve su flecha.
#   - Mientras sigue ese color, ignora las flechas y los cubos de los otros.
#   - El color se libera recien al depositar el cubo. Si el brazo falla,
#     lo intenta hasta MAXIMO_DE_INTENTOS veces en total. Si falla todas,
#     abandona ese color, vuelve al inicio y sigue con el proximo.
#   - Si no hay flecha de su color, o no se entiende hacia donde apunta,
#     decide Exploracion.
#
# Es Python comun, sin ROS, asi se puede probar sin robot y sin simulador.

from collections import deque
from dataclasses import dataclass
from typing import Optional

# --- Estados ---------------------------------------------------------------
# En que parte de la mision esta el robot.
BUSCANDO_COLOR = 'buscando_color'  # todavia no eligio que color seguir
SIGUIENDO = 'siguiendo'            # sigue las flechas de color_actual
ACERCANDO = 'acercando'            # vio su cubo y se esta acercando
VOLVIENDO_SIN_CUBO = 'volviendo_sin_cubo'  # abandono el color y vuelve al inicio
VOLVIENDO = 'volviendo'            # agarro el cubo y vuelve a depositarlo (T8)
TERMINADO = 'terminado'            # ya no le queda ningun color por buscar

# --- Colores ---------------------------------------------------------------
# Escritos igual que en huggin_vision_msgs/Detection.msg.
ROJO = 'red'
VERDE = 'green'
AZUL = 'blue'
TODOS_LOS_COLORES = {ROJO, VERDE, AZUL}

# En que orden se buscan los cubos. Es una lista (con corchetes) y no un
# conjunto (con llaves), porque en una lista el orden importa.
ORDEN_DE_COLORES = [ROJO, VERDE, AZUL]

# --- Tipos de deteccion (campo kind de Detection.msg) ----------------------
FLECHA = 'arrow'
CUBO = 'cube'

# --- Tipos de lugar (los avisa Navegacion alta por /navegacion/lugar) ------
CURVA = 'curva'
BIFURCACION = 'bifurcacion'
CALLEJON = 'callejon'

# --- Salidas, vistas desde el robot ----------------------------------------
IZQUIERDA = 'izquierda'
ADELANTE = 'adelante'
DERECHA = 'derecha'

# Como se traduce la direccion de una flecha en una salida.
# 'down' y 'UNK' no estan: no dicen por donde seguir, asi que decide Exploracion.
# Es la propuesta del tablero; falta confirmarla con el equipo.
SALIDA_SEGUN_FLECHA = {
    'left': IZQUIERDA,
    'right': DERECHA,
    'up': ADELANTE,
}

# --- Filtro "la mejor de 3" ------------------------------------------------
# A la camara le cuesta reconocer, asi que Decision le cree solo a lo que ve
# repetido: lo mismo en al menos 2 de las ultimas 3 imagenes.
IMAGENES_A_MIRAR = 3
MINIMO_PARA_CREER = 2

# --- Intentos de agarre ----------------------------------------------------
# Cuantas veces en total intenta agarrar el cubo antes de abandonar el color.
MAXIMO_DE_INTENTOS = 3

# --- Acciones que puede pedir Decision -------------------------------------
IR_A_SALIDA = 'ir_a_salida'  # tomar una salida concreta
EXPLORAR = 'explorar'        # que Exploracion elija por donde seguir
ACERCARSE = 'acercarse'      # acercarse al cubo que tiene adelante
AGARRAR = 'agarrar'          # pedirle al brazo que agarre el cubo
VOLVER = 'volver'            # ya tiene el cubo: volver al deposito (T8)


@dataclass
class Orden:
    """Lo que Decision le pide al resto del robot."""

    # Que hacer: IR_A_SALIDA, EXPLORAR, ACERCARSE, AGARRAR o VOLVER.
    accion: str

    # Solo para IR_A_SALIDA: por donde ir (IZQUIERDA, ADELANTE o DERECHA).
    salida: Optional[str] = None

    # Solo para IR_A_SALIDA: el largo de la flecha ('close', 'normal' o 'far').
    # No sirve para elegir; Navegacion alta lo usa para anticipar la proxima curva.
    length_class: Optional[str] = None


def clave(d):
    """Lo que tiene que coincidir entre imagenes para creerle a una deteccion.

    Tipo (flecha o cubo) y color siempre; la direccion solo en las flechas.
    """
    direccion = d.direction if d.kind == FLECHA else ''
    return (d.kind, d.color, direccion)


class Decision:
    """Recuerda que color sigue el robot y decide que hacer en cada momento."""

    def __init__(self, confianza_minima=0.6):
        # Las detecciones con menos confianza que esto se ignoran.
        self.confianza_minima = confianza_minima

        # Los colores cuyo cubo todavia no se deposito.
        # set(...) hace una copia: si no, se borrarian colores de TODOS_LOS_COLORES.
        self.cubos_faltantes = set(TODOS_LOS_COLORES)

        # El color que esta siguiendo. None significa que todavia no eligio.
        self.color_actual = None

        self.estado = BUSCANDO_COLOR

        # La ultima flecha de color_actual que vio la camara. La camara ve la
        # flecha antes de llegar a la bifurcacion, asi que se guarda hasta que
        # Navegacion avise que llego.
        self.flecha_pendiente = None

        # Las claves de lo que se vio en las ultimas imagenes. Guarda solo
        # las ultimas IMAGENES_A_MIRAR y tira la mas vieja sola.
        self.ultimas = deque(maxlen=IMAGENES_A_MIRAR)
        # Cuantas veces intento agarrar el cubo que tiene adelante.
        self.intentos = 0

    def proximo_color(self):
        """El primer color de ORDEN_DE_COLORES que todavia falta.

        Devuelve None si ya no falta ninguno.
        """
        for color in ORDEN_DE_COLORES:
            if color in self.cubos_faltantes:
                return color
        return None

    # -----------------------------------------------------------------------
    # Lo que llega de la camara
    # -----------------------------------------------------------------------

    def ver(self, detecciones):
        """Recibe la lista de detecciones de la camara (/vision/detections).

        De cada deteccion usa solo kind, color, direction, length_class y
        confidence. Por eso sirve tanto con huggin_vision_msgs/Detection como
        con objetos armados a mano en las pruebas.

        Devuelve una Orden si hay que hacer algo ya (acercarse a un cubo),
        o None si no.
        """
        # Solo importa la camara mientras busca color o lo sigue.
        if self.estado not in (BUSCANDO_COLOR, SIGUIENDO):
            return None

        # Se queda con las detecciones confiables. Las zonas se ignoran,
        # porque las maneja Navegacion alta.
        utiles = [
            d for d in detecciones
            if d.confidence >= self.confianza_minima and d.kind in (FLECHA, CUBO)
        ]

        # Filtro "la mejor de 3": anota lo que vio en esta imagen y se queda
        # solo con lo que aparece en al menos MINIMO_PARA_CREER de las ultimas.
        self.ultimas.append({clave(d) for d in utiles})
        utiles = [
            d for d in utiles
            if sum(clave(d) in imagen for imagen in self.ultimas) >= MINIMO_PARA_CREER
        ]

        # 1) Si todavia no eligio color, espera la flecha del proximo color
        #    del orden. Las flechas de los otros colores no lo hacen arrancar.
        if self.estado == BUSCANDO_COLOR:
            siguiente = self.proximo_color()
            for d in utiles:
                if d.kind == FLECHA and d.color == siguiente:
                    self.color_actual = d.color
                    self.estado = SIGUIENDO
                    break

        # Si sigue sin color, no hay nada mas que hacer con esta imagen.
        if self.estado != SIGUIENDO:
            return None

        # 2) Ya tiene color: mira solo lo de su color. Lo demas se ignora.
        for d in utiles:
            if d.color != self.color_actual:
                continue

            if d.kind == CUBO:
                # Encontro su cubo: deja de seguir flechas y se acerca.
                self.estado = ACERCANDO
                self.flecha_pendiente = None
                return Orden(ACERCARSE)

            if d.kind == FLECHA:
                # La guarda para cuando llegue a la bifurcacion.
                self.flecha_pendiente = d

        return None

    # -----------------------------------------------------------------------
    # Lo que llega de Navegacion alta
    # -----------------------------------------------------------------------

    def llegar_a_lugar(self, tipo, salidas):
        """Navegacion alta avisa que llego a un lugar (/navegacion/lugar).

        tipo es CURVA, BIFURCACION o CALLEJON. salidas es la lista de salidas
        posibles desde el robot, por ejemplo [IZQUIERDA, ADELANTE].

        Devuelve la Orden a seguir, o None si no hay nada que decidir.
        """
        # Si se esta acercando o volviendo, no decide por donde ir.
        if self.estado not in (BUSCANDO_COLOR, SIGUIENDO):
            return None

        # En una curva hay una sola salida: no hay nada que elegir.
        if tipo == CURVA:
            return None

        # En un callejon hay que volver: Exploracion sabe a donde.
        if tipo == CALLEJON:
            return Orden(EXPLORAR)

        # En una bifurcacion: si hay una flecha de su color que indique una
        # salida que existe, la toma.
        if self.flecha_pendiente is not None:
            flecha = self.flecha_pendiente
            self.flecha_pendiente = None  # una flecha sirve para una sola bifurcacion
            salida = SALIDA_SEGUN_FLECHA.get(flecha.direction)
            if salida in salidas:
                return Orden(IR_A_SALIDA, salida, flecha.length_class)

        # No hay flecha de su color, o no se entiende hacia donde apunta.
        return Orden(EXPLORAR)

    def llegue_al_cubo(self):
        """Navegacion alta avisa que termino de acercarse al cubo.

        Devuelve la Orden de agarrarlo, o None si no se estaba acercando.
        """
        # Solo tiene sentido si se estaba acercando a su cubo.
        if self.estado != ACERCANDO:
            return None

        # Es el primer intento. El estado sigue en ACERCANDO: pasa a VOLVIENDO
        # recien cuando el brazo avisa que lo agarro (cubo_agarrado).
        self.intentos = 1
        return Orden(AGARRAR)

    # -----------------------------------------------------------------------
    # Lo que llega del brazo y de Volver al inicio
    # -----------------------------------------------------------------------

    def cubo_agarrado(self):
        """El brazo avisa que agarro el cubo. Ahora vuelve al deposito (T8).

        Devuelve la Orden de volver, o None si no se estaba acercando.
        """
        # Solo tiene sentido si se estaba acercando a su cubo.
        if self.estado != ACERCANDO:
            return None

        self.estado = VOLVIENDO
        return Orden(VOLVER)

    def cubo_no_agarrado(self):
        """El brazo avisa que no pudo agarrar el cubo.

        Si le quedan intentos, pide agarrarlo otra vez. Si ya fallo
        MAXIMO_DE_INTENTOS veces, abandona el color y pide volver al inicio.
        Devuelve None si no se estaba acercando.
        """
        # Solo tiene sentido si se estaba acercando a su cubo.
        if self.estado != ACERCANDO:
            return None

        # Le quedan intentos: lo vuelve a intentar, con el mismo color.
        if self.intentos < MAXIMO_DE_INTENTOS:
            self.intentos += 1
            return Orden(AGARRAR)

        # Fallo todas las veces: abandona este color para siempre.
        self.cubos_faltantes.discard(self.color_actual)
        self.color_actual = None
        self.flecha_pendiente = None
        self.ultimas.clear()
        self.intentos = 0
        self.estado = VOLVIENDO_SIN_CUBO
        return Orden(VOLVER)

    def cubo_depositado(self):
        """Avisan que el cubo quedo en su deposito. Libera el color."""
        if self.estado != VOLVIENDO:
            return

        # discard saca el color sin dar error si ya no estaba.
        self.cubos_faltantes.discard(self.color_actual)
        self.color_actual = None
        self.flecha_pendiente = None
        self.ultimas.clear()  # el color nuevo empieza a contar de cero

        if self.cubos_faltantes:
            self.estado = BUSCANDO_COLOR
        else:
            self.estado = TERMINADO

    def llegue_al_inicio(self):
        """Volver al inicio avisa que el robot llego a la entrada sin cubo.

        Pasa a buscar el proximo color, o termina si ya no queda ninguno.
        """
        if self.estado != VOLVIENDO_SIN_CUBO:
            return

        if self.cubos_faltantes:
            self.estado = BUSCANDO_COLOR
        else:
            self.estado = TERMINADO
