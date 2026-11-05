# La logica de los botones del joystick que cambian de modo (T11), sin ROS.
# El nodo que la usa esta en joystick_modo.py.
#
# Un boton dispara su pedido una sola vez, cuando se aprieta (no mientras se mantiene).
# El de soltar el freno hay que mantenerlo apretado un rato, para no soltarlo sin querer.


class BotonesJoystick:
    """Convierte los botones apretados en pedidos para el selector ("parar", "auto", etc.)."""

    def __init__(self, botones, boton_soltar, tiempo_soltar_s):
        """botones: {numero_de_boton: pedido} que se disparan al apretar.

        boton_soltar: numero del boton que suelta el freno al mantenerlo tiempo_soltar_s.
        """
        self.botones = botones
        self.boton_soltar = boton_soltar
        self.tiempo_soltar_s = tiempo_soltar_s
        self.antes = []              # botones de la lectura anterior
        self.soltar_desde_s = None   # cuando se empezo a mantener el boton de soltar
        self.soltar_disparado = False

    @staticmethod
    def _apretado(lista, numero):
        return 0 <= numero < len(lista) and lista[numero] == 1

    def actualizar(self, botones_ahora, ahora_s):
        """Recibe la lista de botones del joystick (1 = apretado). Devuelve los pedidos."""
        pedidos = []
        for numero, pedido in self.botones.items():
            if self._apretado(botones_ahora, numero) and not self._apretado(self.antes, numero):
                pedidos.append(pedido)

        if self._apretado(botones_ahora, self.boton_soltar):
            if self.soltar_desde_s is None:
                self.soltar_desde_s = ahora_s
            mantenido = ahora_s - self.soltar_desde_s >= self.tiempo_soltar_s
            if mantenido and not self.soltar_disparado:
                pedidos.append('soltar')
                self.soltar_disparado = True
        else:
            self.soltar_desde_s = None
            self.soltar_disparado = False

        self.antes = list(botones_ahora)
        return pedidos
