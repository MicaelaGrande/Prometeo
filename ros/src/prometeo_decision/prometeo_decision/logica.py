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
#   - Si el brazo no puede agarrar el cubo, lo intenta hasta MAXIMO_DE_INTENTOS
#     veces en total. Si falla todas, abandona ese color y sigue con el proximo.
#   - El color se libera recien al depositar el cubo: el brazo lo suelta en el
#     deposito. Si no puede soltarlo despues de MAXIMO_DE_INTENTOS_SOLTAR veces,
#     igual sigue con el proximo color.
#   - Si no hay flecha de su color, o no se entiende hacia donde apunta,
#     decide Exploracion.
#
# Es Python comun, sin nodos de ROS, asi se puede probar sin robot y sin simulador.
# Solo usa las constantes de prometeo_interfaces (hay que compilarlo y hacer
# source install/setup.bash antes de correr las pruebas).

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

# Los textos que viajan por ROS estan escritos una sola vez, en prometeo_interfaces
# (Lugar.msg y Mover.action). Aca se usan esas constantes: asi navegacion y
# Decision no pueden escribirlos distinto.
from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar

# --- Estados ---------------------------------------------------------------
# En que parte de la mision esta el robot.
BUSCANDO_COLOR = 'buscando_color'  # todavia no eligio que color seguir
SIGUIENDO = 'siguiendo'            # sigue las flechas de color_actual
ACERCANDO = 'acercando'            # vio su cubo y se esta acercando
VOLVIENDO_SIN_CUBO = 'volviendo_sin_cubo'  # abandono el color y vuelve al inicio
VOLVIENDO = 'volviendo'            # agarro el cubo y vuelve a depositarlo (T8)
SOLTANDO = 'soltando'              # llego al deposito y el brazo esta soltando el cubo
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
CURVA = Lugar.CURVA
BIFURCACION = Lugar.BIFURCACION
CALLEJON = Lugar.CALLEJON

# --- Salidas, vistas desde el robot ----------------------------------------
IZQUIERDA = Lugar.IZQUIERDA
ADELANTE = Lugar.ADELANTE
DERECHA = Lugar.DERECHA

# Como se traduce la direccion de una flecha en una salida.
# 'down' y 'UNK' no estan: no dicen por donde seguir, asi que decide Exploracion.
# Es la propuesta del tablero; falta confirmarla con el equipo.
SALIDA_SEGUN_FLECHA = {
    'left': IZQUIERDA,
    'right': DERECHA,
    'up': ADELANTE,
}

# --- Filtro de imagenes repetidas -------------------------------------------
# A la camara le cuesta reconocer, asi que Decision le cree solo a lo que ve
# repetido en las ultimas imagenes.
# Flechas: lo mismo en al menos 2 de las ultimas 3. Ya cuesta verlas (son
# chicas y estan lejos), y equivocarse sale barato.
IMAGENES_FLECHA = 3
MINIMO_FLECHA = 2
# Cubos: al menos 4 de las ultimas 5. Equivocarse sale caro (el robot deja todo
# y va hacia el cubo), y en la competencia el fondo azul parecia un cubo azul.
IMAGENES_CUBO = 5
MINIMO_CUBO = 4

# --- Distancia creible de un cubo (metros) ---------------------------------
# La camara no mide la distancia: la adivina por el tamaño, suponiendo que
# todo cubo mide 2 cm. Una mancha enorme (el fondo azul) parece un cubo pegado
# a la camara, y una mancha chiquita parece un cubo lejisimos.
# Menos de 7 cm: el robot nunca ve un cubo tan cerca (mover se arrima hasta 10 cm).
# Mas de 1 m: vision no detecta un cubo real de 2 cm tan lejos.
DISTANCIA_MIN_CUBO = 0.07
DISTANCIA_MAX_CUBO = 1.0

# --- Intentos de agarre ----------------------------------------------------
# Cuantas veces en total intenta agarrar el cubo antes de abandonar el color.
MAXIMO_DE_INTENTOS = 3

# --- Intentos de soltar ----------------------------------------------------
# Cuantas veces intenta soltar el cubo en el deposito. Si falla todas, igual
# lo da por depositado: es mejor seguir con el proximo color que quedar quieto.
MAXIMO_DE_INTENTOS_SOLTAR = 2

# --- Acciones que puede pedir Decision -------------------------------------
IR_A_SALIDA = Mover.Goal.IR_A_SALIDA  # tomar una salida concreta
EXPLORAR = 'explorar'        # que Exploracion elija por donde seguir
ACERCARSE = Mover.Goal.ACERCARSE      # acercarse al cubo que tiene adelante
AGARRAR = 'agarrar'          # pedirle al brazo que agarre el cubo
VOLVER = 'volver'            # ya tiene el cubo: volver al deposito (T8)
SOLTAR = 'soltar'            # llego al deposito: pedirle al brazo que suelte el cubo
RETOMAR = 'retomar'          # abandono el color: volver al ultimo lugar y seguir desde ahi
SEGUIR = 'seguir'            # falsa alarma (no llego al cubo): seguir por el pasillo


@dataclass
class Orden:
    """Lo que Decision le pide al resto del robot."""

    # Que hacer: IR_A_SALIDA, EXPLORAR, ACERCARSE, AGARRAR, VOLVER, SOLTAR, RETOMAR
    # o SEGUIR.
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

    def __init__(self, confianza_minima=0.6, imagenes_cubo=IMAGENES_CUBO,
                 minimo_cubo=MINIMO_CUBO, distancia_min_cubo=DISTANCIA_MIN_CUBO,
                 distancia_max_cubo=DISTANCIA_MAX_CUBO):
        # Las detecciones con menos confianza que esto se ignoran.
        self.confianza_minima = confianza_minima

        # Cuantas de las ultimas imagenes mira para creerle a un cubo, y en
        # cuantas lo tiene que ver (el nodo los puede cambiar con parametros).
        self.imagenes_cubo = imagenes_cubo
        self.minimo_cubo = minimo_cubo

        # Entre que distancias (metros) un cubo es creible, y cuantos se
        # descartaron por estar fuera (se ve en /decision/estado).
        self.distancia_min_cubo = distancia_min_cubo
        self.distancia_max_cubo = distancia_max_cubo
        self.cubos_descartados = 0

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

        # Las claves de lo que se vio en las ultimas imagenes. Guarda las que
        # hacen falta para el filtro mas largo y tira la mas vieja sola.
        self.ultimas = deque(maxlen=max(IMAGENES_FLECHA, imagenes_cubo))
        # Cuantas veces intento agarrar el cubo que tiene adelante.
        self.intentos = 0
        # Cuantas veces intento soltar el cubo en el deposito.
        self.intentos_soltar = 0

        # True despues de una falsa alarma (no llego al cubo): ignora los cubos
        # hasta el proximo lugar, asi lo mismo (por ejemplo el fondo azul) no lo
        # vuelve a frenar enseguida.
        self.ignorar_cubos = False

    def proximo_color(self):
        """El primer color de ORDEN_DE_COLORES que todavia falta.

        Devuelve None si ya no falta ninguno.
        """
        for color in ORDEN_DE_COLORES:
            if color in self.cubos_faltantes:
                return color
        return None

    def es_creible(self, d):
        """True si la deteccion puede ser real.

        Las flechas siempre. Un cubo, solo si esta adelante del robot y a una
        distancia entre distancia_min_cubo y distancia_max_cubo. Si no, lo
        cuenta en cubos_descartados.
        """
        if d.kind != CUBO:
            return True
        # La pose viene con x hacia adelante e y a la izquierda, en metros.
        x, y = d.pose.position.x, d.pose.position.y
        distancia = math.hypot(x, y)
        if x > 0 and self.distancia_min_cubo <= distancia <= self.distancia_max_cubo:
            return True
        self.cubos_descartados += 1
        return False

    def le_creo(self, d):
        """True si la deteccion d se repitio lo suficiente en las ultimas imagenes.

        Flecha: en MINIMO_FLECHA de las ultimas IMAGENES_FLECHA.
        Cubo: en minimo_cubo de las ultimas imagenes_cubo.
        """
        if d.kind == FLECHA:
            cuantas, minimo = IMAGENES_FLECHA, MINIMO_FLECHA
        else:
            cuantas, minimo = self.imagenes_cubo, self.minimo_cubo
        # list(...)[-cuantas:] se queda con las ultimas "cuantas" imagenes.
        mirar = list(self.ultimas)[-cuantas:]
        veces = sum(clave(d) in imagen for imagen in mirar)
        return veces >= minimo

    # -----------------------------------------------------------------------
    # Lo que llega de la camara
    # -----------------------------------------------------------------------

    def ver(self, detecciones):
        """Recibe la lista de detecciones de la camara (/vision/detections).

        De cada deteccion usa solo kind, color, direction, length_class,
        confidence y (en los cubos) pose. Por eso sirve tanto con huggin_vision_msgs/Detection como
        con objetos armados a mano en las pruebas.

        Devuelve una Orden si hay que hacer algo ya (acercarse a un cubo),
        o None si no.
        """
        # Solo importa la camara mientras busca color o lo sigue.
        if self.estado not in (BUSCANDO_COLOR, SIGUIENDO):
            return None

        # Se queda con las detecciones confiables y creibles. Las zonas se
        # ignoran, porque las maneja Navegacion alta. Un cubo a una distancia
        # imposible ni siquiera cuenta como visto para el filtro.
        utiles = [
            d for d in detecciones
            if d.confidence >= self.confianza_minima and d.kind in (FLECHA, CUBO)
            and self.es_creible(d)
        ]

        # Filtro de imagenes repetidas: anota lo que vio en esta imagen y se
        # queda solo con lo que se repite lo suficiente (ver le_creo).
        self.ultimas.append({clave(d) for d in utiles})
        utiles = [d for d in utiles if self.le_creo(d)]

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
                if self.ignorar_cubos:
                    continue  # hubo una falsa alarma en este pasillo
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

        # Es otro pasillo: vuelve a mirar los cubos.
        self.ignorar_cubos = False

        # Hay flecha en cada curva y en cada bifurcacion (no en las islas).
        # La flecha guardada sirve para UN lugar solo, sea curva, bifurcacion o
        # callejon: se saca y se borra siempre, asi no se usa en el lugar siguiente.
        flecha = self.flecha_pendiente
        self.flecha_pendiente = None
        salida_flecha = SALIDA_SEGUN_FLECHA.get(flecha.direction) if flecha else None

        # En un callejon hay que volver: Exploracion sabe a donde.
        if tipo == CALLEJON:
            return Orden(EXPLORAR)

        # En una curva hay una sola salida: va por ahi siempre.
        # Si la flecha apunta a esa salida, le pasa su largo (cuanto falta hasta la
        # proxima curva). Si no hay flecha, o apunta a otro lado (la camara se
        # equivoco), va igual, pero sin largo.
        if tipo == CURVA:
            if not salidas:
                return Orden(EXPLORAR)  # no deberia pasar: una curva sin salida
            salida = salidas[0]
            largo = flecha.length_class if salida_flecha == salida else None
            return Orden(IR_A_SALIDA, salida, largo)

        # En una bifurcacion: si hay una flecha de su color que indique una
        # salida que existe, la toma.
        if salida_flecha is not None and salida_flecha in salidas:
            return Orden(IR_A_SALIDA, salida_flecha, flecha.length_class)

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

    def no_llegue_al_cubo(self):
        """Navegacion alta avisa que NO pudo llegar al cubo (bloqueada o cancelada).

        Lo mas probable es que no fuera un cubo (por ejemplo, el fondo azul del
        laberinto). Es una falsa alarma: sigue con el mismo color por el pasillo e
        ignora los cubos hasta el proximo lugar.
        Devuelve Orden(SEGUIR), o None si no se estaba acercando.
        """
        # Solo tiene sentido si se estaba acercando a su cubo.
        if self.estado != ACERCANDO:
            return None

        self.estado = SIGUIENDO
        self.intentos = 0
        self.flecha_pendiente = None
        self.ultimas.clear()
        self.ignorar_cubos = True
        return Orden(SEGUIR)

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

        # Fallo todas las veces: abandona este color, igual que el boton Abandonar.
        return self.abandonar_color()

    def abandonar_color(self):
        """Abandona el color actual: el operador aprieta Abandonar, o fallo el agarre.

        Lo descarta para siempre y sigue buscando el proximo color DESDE DONDE ESTA:
        no vuelve a la largada, para no perder tiempo. Devuelve Orden(RETOMAR):
        Navegacion da media vuelta y vuelve al ultimo lugar, y ahi se decide de nuevo
        (la flecha del color nuevo, o Exploracion prueba otro camino, o retrocede al
        lugar con opciones mas cercano).
        Si ya no queda ningun color, vuelve al inicio (Orden VOLVER).
        Devuelve None si no hay un color que abandonar.
        """
        # VOLVIENDO tambien: con la garra a mano (sin_brazo) Decision ya pidio volver
        # cuando se dan cuenta de que no lo pueden agarrar.
        if self.color_actual is None or self.estado not in (SIGUIENDO, ACERCANDO, VOLVIENDO):
            return None
        self.cubos_faltantes.discard(self.color_actual)
        self.color_actual = None
        self.flecha_pendiente = None
        self.ultimas.clear()
        self.intentos = 0
        self.ignorar_cubos = False
        if not self.cubos_faltantes:
            self.estado = VOLVIENDO_SIN_CUBO
            return Orden(VOLVER)
        self.estado = BUSCANDO_COLOR
        return Orden(RETOMAR)

    def llegue_al_deposito(self):
        """Volver avisa que el robot llego al deposito con el cubo.

        Devuelve la Orden de soltarlo, o None si no estaba volviendo con un cubo.
        """
        if self.estado != VOLVIENDO:
            return None

        # Es el primer intento de soltar.
        self.estado = SOLTANDO
        self.intentos_soltar = 1
        return Orden(SOLTAR)

    def cubo_soltado(self):
        """El brazo avisa que solto el cubo en el deposito. Libera el color."""
        if self.estado != SOLTANDO:
            return
        self.cubo_depositado()

    def cubo_no_soltado(self):
        """El brazo avisa que no pudo soltar el cubo.

        Si le quedan intentos, pide soltarlo otra vez. Si ya fallo
        MAXIMO_DE_INTENTOS_SOLTAR veces, igual lo da por depositado y sigue con
        el proximo color (quedarse quieto en el deposito no sirve para nada).
        Devuelve None si no estaba soltando.
        """
        if self.estado != SOLTANDO:
            return None

        if self.intentos_soltar < MAXIMO_DE_INTENTOS_SOLTAR:
            self.intentos_soltar += 1
            return Orden(SOLTAR)

        self.cubo_depositado()
        return None

    def cubo_depositado(self):
        """El cubo quedo en su deposito. Libera el color.

        Lo llama cubo_soltado() (o cubo_no_soltado() si se rinde). Vale tambien
        desde VOLVIENDO, para probar sin el paso de soltar.
        """
        if self.estado not in (VOLVIENDO, SOLTANDO):
            return

        # discard saca el color sin dar error si ya no estaba.
        self.cubos_faltantes.discard(self.color_actual)
        self.color_actual = None
        self.flecha_pendiente = None
        self.ultimas.clear()  # el color nuevo empieza a contar de cero
        self.ignorar_cubos = False
        self.intentos = 0
        self.intentos_soltar = 0

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
