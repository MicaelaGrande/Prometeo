# La logica del selector de modo (T11): decidir que orden de movimiento le llega a
# la ESP32, la del automatico o la del manual.
#
# Es Python comun, sin ROS, asi se puede probar sin robot y sin simulador.
# El nodo que la usa esta en selector_modo.py.

AUTO = 'auto'
MANUAL = 'manual'
MODOS = (AUTO, MANUAL)
# Boton de panico: no es un modo, es un pedido. Pasa a MANUAL y frena aunque ya
# estuviera en manual (descarta la orden manual que venia llegando).
PARAR = 'parar'


class Selector:
    """Se acuerda del modo elegido y de la ultima orden de cada origen.

    Arranca en MANUAL: asi el robot no se mueve solo hasta que alguien elige "auto".
    """

    def __init__(self, edad_maxima_s):
        self.edad_maxima_s = edad_maxima_s
        self.modo = MANUAL
        # La ultima orden de cada origen y cuando llego: (orden, hora_s)
        self.ultima = {AUTO: (None, None), MANUAL: (None, None)}

    def cambiar_modo(self, texto):
        """Cambia al modo pedido. Devuelve True si cambio.

        Acepta "auto", "manual" o "parar" (boton de panico), sin importar mayusculas ni
        espacios. Cualquier otro texto se ignora, para que un error de tipeo no deje al robot en un modo raro.
        Al cambiar se borran las ordenes guardadas: el robot arranca quieto en el modo
        nuevo hasta que llegue una orden nueva, y no repite una vieja.
        """
        modo = texto.strip().lower()
        if modo == PARAR:
            self.modo = MANUAL
            self.ultima = {AUTO: (None, None), MANUAL: (None, None)}
            return True
        if modo not in MODOS or modo == self.modo:
            return False
        self.modo = modo
        self.ultima = {AUTO: (None, None), MANUAL: (None, None)}
        return True

    def guardar_orden(self, origen, orden, ahora_s):
        """Guarda la ultima orden que llego de un origen (AUTO o MANUAL)."""
        self.ultima[origen] = (orden, ahora_s)

    def orden_a_publicar(self, ahora_s):
        """La orden del modo elegido, o None si hay que parar.

        None si nunca llego una orden de ese modo o si es mas vieja que edad_maxima_s
        (por ejemplo, se corto el teclado o el automatico dejo de mandar).
        La orden del otro modo nunca pasa.
        """
        orden, hora_s = self.ultima[self.modo]
        if hora_s is None or ahora_s - hora_s > self.edad_maxima_s:
            return None
        return orden
