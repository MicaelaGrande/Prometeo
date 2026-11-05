# La logica del selector de modo (T11): decidir que orden de movimiento le llega a
# la ESP32, la del automatico o la del manual (compu o joystick), y el freno PARAR.
#
# Es Python comun, sin ROS, asi se puede probar sin robot y sin simulador.
# El nodo que la usa esta en selector_modo.py.

AUTO = 'auto'
MANUAL = 'manual'
MODOS = (AUTO, MANUAL)

# Pedidos que no son modos: el freno PARAR. Como el boton rojo de una maquina,
# queda puesto hasta que una persona lo suelta a proposito, y no cambia el modo.
PARAR = 'parar'
SOLTAR = 'soltar'

# De donde puede venir una orden de movimiento
COMPU = 'compu'         # el panel o el teclado
JOYSTICK = 'joystick'
ORIGENES = (AUTO, COMPU, JOYSTICK)
ORIGENES_MANUALES = (COMPU, JOYSTICK)

# Quien esta manejando, para el cartel del panel
NADIE = 'nadie'
FRENO = 'freno'


class Selector:
    """Se acuerda del modo, del freno y de la ultima orden de cada origen.

    Arranca en MANUAL y sin freno: el robot no se mueve solo hasta que alguien elige
    "auto" o lo maneja.
    """

    def __init__(self, edad_maxima_s):
        self.edad_maxima_s = edad_maxima_s
        self.modo = MANUAL
        self.freno_puesto = False
        self._borrar_ordenes()

    def _borrar_ordenes(self):
        # La ultima orden de cada origen y cuando llego: (orden, hora_s)
        self.ultima = {origen: (None, None) for origen in ORIGENES}

    def pedir(self, texto):
        """Atiende un pedido: "auto", "manual", "parar" o "soltar". True si cambio algo.

        No importan mayusculas ni espacios. Cualquier otro texto se ignora, para que un
        error de tipeo no deje al robot en un estado raro.
        Al cambiar de modo, poner o soltar el freno se borran las ordenes guardadas: el
        robot queda quieto hasta que llegue una orden nueva, y no repite una vieja.
        """
        pedido = texto.strip().lower()
        if pedido == PARAR:
            if self.freno_puesto:
                return False
            self.freno_puesto = True
            self._borrar_ordenes()
            return True
        if pedido == SOLTAR:
            if not self.freno_puesto:
                return False
            self.freno_puesto = False
            self._borrar_ordenes()
            return True
        if pedido not in MODOS or pedido == self.modo:
            return False
        self.modo = pedido
        self._borrar_ordenes()
        return True

    def guardar_orden(self, origen, orden, ahora_s):
        """Guarda la ultima orden que llego de un origen (AUTO, COMPU o JOYSTICK)."""
        self.ultima[origen] = (orden, ahora_s)

    def _hora_vigente(self, origen, ahora_s):
        """La hora de la orden de ese origen si todavia vale, o None."""
        hora_s = self.ultima[origen][1]
        if hora_s is None or ahora_s - hora_s > self.edad_maxima_s:
            return None
        return hora_s

    def fuente(self, ahora_s):
        """Quien esta manejando ahora: FRENO, AUTO, COMPU, JOYSTICK o NADIE.

        En manual, si la compu y el joystick mandan a la vez, gana el que mando ultimo.
        """
        if self.freno_puesto:
            return FRENO
        if self.modo == AUTO:
            return AUTO if self._hora_vigente(AUTO, ahora_s) is not None else NADIE
        horas = {}
        for origen in ORIGENES_MANUALES:
            hora_s = self._hora_vigente(origen, ahora_s)
            if hora_s is not None:
                horas[origen] = hora_s
        if not horas:
            return NADIE
        return max(horas, key=horas.get)

    def orden_a_publicar(self, ahora_s):
        """La orden de quien maneja, o None si hay que parar.

        None con el freno puesto, o si no hay ninguna orden reciente del modo elegido
        (por ejemplo, se corto el teclado o el automatico dejo de mandar).
        Las ordenes del otro modo nunca pasan.
        """
        quien = self.fuente(ahora_s)
        if quien in (FRENO, NADIE):
            return None
        return self.ultima[quien][0]
