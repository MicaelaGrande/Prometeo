# La logica de la LLAVE de modo: decide que orden de movimiento le llega a la ESP32.
#
# Es de UNA SOLA VIA, como la llave de arranque de un auto:
#
#   MANUAL ──[ "auto" ]──► AUTOMATICO ──[ "parar" o termino el algoritmo ]──► TERMINADO
#     ▲                        │
#     └──── no se puede volver ┘
#
#   - MANUAL: arranca asi. Lo manejan a mano (panel o joystick), por ejemplo para
#     llevarlo a la entrada. Si nunca pasan a automatico, es manual todo el recorrido.
#   - AUTOMATICO: se pasa UNA sola vez. Desde ahi solo cuenta lo que manda la
#     navegacion: el manejo a mano queda bloqueado (aunque toquen el joystick).
#   - TERMINADO: el robot queda quieto para siempre. Se llega con PARAR (el boton del
#     panel) o cuando el algoritmo termina. Para volver a empezar hay que relanzar.
#
# Por que asi: con un ida y vuelta entre manual y automatico, cada nodo tenia que
# adivinar que paso mientras estaba en manual (pausas, "se movio o no se movio").
# Con una sola via no hay nada que adivinar.
#
# Es Python comun, sin nodos de ROS, asi se puede probar sin robot y sin simulador.
# Solo usa las constantes de prometeo_interfaces (hay que compilarlo y hacer
# source install/setup.bash antes de correr las pruebas).
# El nodo que la usa esta en selector_modo.py.

# Los textos estan escritos una sola vez, en prometeo_interfaces/msg/Modo.msg.
from prometeo_interfaces.msg import Modo

MANUAL = Modo.MANUAL
AUTO = Modo.AUTO
TERMINADO = Modo.TERMINADO

# Lo que se puede pedir por el topic "modo"
PEDIR_AUTO = Modo.PEDIR_AUTO      # pasar de manual a automatico (una sola vez)
PEDIR_PARAR = Modo.PEDIR_PARAR    # terminar la corrida (desde manual o desde automatico)
PEDIDOS = (PEDIR_AUTO, PEDIR_PARAR)

# De donde puede venir una orden de movimiento
NAVEGACION = 'navegacion'   # el automatico (mover)
COMPU = 'compu'             # el panel o el teclado
JOYSTICK = 'joystick'
ORIGENES = (NAVEGACION, COMPU, JOYSTICK)
ORIGENES_MANUALES = (COMPU, JOYSTICK)

# Quien esta manejando ahora, para el cartel del panel
NADIE = 'nadie'


class Llave:
    """Se acuerda del modo y de la ultima orden de cada origen.

    Arranca en MANUAL: el robot no se mueve solo hasta que alguien pide "auto".
    """

    def __init__(self, edad_maxima_s):
        # Una orden mas vieja que esto ya no vale: el robot para (por ejemplo, se
        # corto el WiFi del panel o la navegacion dejo de mandar).
        self.edad_maxima_s = edad_maxima_s
        self.modo = MANUAL
        self._borrar_ordenes()

    def _borrar_ordenes(self):
        # La ultima orden de cada origen y cuando llego: (orden, hora_s)
        self.ultima = {origen: (None, None) for origen in ORIGENES}

    def pedir(self, pedido):
        """Atiende un pedido: PEDIR_AUTO o PEDIR_PARAR. Devuelve True si cambio el modo.

        Todo lo demas se ignora (por ejemplo, pedir automatico estando en
        automatico, o un texto desconocido que llego del panel), para que un boton
        apretado de mas no deje al robot en un estado raro.
        Al cambiar se borran las ordenes guardadas: el robot queda quieto hasta que
        llegue una orden nueva del modo nuevo, y nunca repite una vieja.
        """
        if pedido == PEDIR_AUTO and self.modo == MANUAL:
            self.modo = AUTO
        elif pedido == PEDIR_PARAR and self.modo != TERMINADO:
            self.modo = TERMINADO
        else:
            return False
        self._borrar_ordenes()
        return True

    def guardar_orden(self, origen, orden, ahora_s):
        """Guarda la ultima orden que llego de un origen (NAVEGACION, COMPU o JOYSTICK)."""
        self.ultima[origen] = (orden, ahora_s)

    def _hora_vigente(self, origen, ahora_s):
        """La hora de la orden de ese origen si todavia vale, o None."""
        hora_s = self.ultima[origen][1]
        if hora_s is None or ahora_s - hora_s > self.edad_maxima_s:
            return None
        return hora_s

    def fuente(self, ahora_s):
        """Quien esta manejando ahora: NAVEGACION, COMPU, JOYSTICK o NADIE.

        En manual, si la compu y el joystick mandan a la vez, gana el que mando ultimo.
        """
        if self.modo == TERMINADO:
            return NADIE
        if self.modo == AUTO:
            vigente = self._hora_vigente(NAVEGACION, ahora_s) is not None
            return NAVEGACION if vigente else NADIE
        horas = {}
        for origen in ORIGENES_MANUALES:
            hora_s = self._hora_vigente(origen, ahora_s)
            if hora_s is not None:
                horas[origen] = hora_s
        if not horas:
            return NADIE
        return max(horas, key=horas.get)

    def orden_a_publicar(self, ahora_s):
        """La orden de quien maneja, o None si hay que quedarse quieto.

        Las ordenes de un origen que no corresponde al modo nunca pasan.
        """
        quien = self.fuente(ahora_s)
        if quien == NADIE:
            return None
        return self.ultima[quien][0]
