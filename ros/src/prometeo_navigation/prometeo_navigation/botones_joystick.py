# La logica de los botones del joystick que le hablan a la llave de modo, sin ROS.
# El nodo que la usa esta en joystick_modo.py.
#
# Dos clases de boton:
#   - instantaneos: disparan su pedido una sola vez, al apretarlos (no mientras se
#     mantienen). Es el caso de PARAR: tiene que actuar enseguida.
#   - el de mantener: hay que tenerlo apretado un rato. Es el de pasar a automatico,
#     porque no tiene vuelta atras: asi no se pasa sin querer por rozarlo.


class BotonesJoystick:
    """Convierte los botones apretados en pedidos para la llave ("parar", "auto")."""

    def __init__(self, botones, boton_mantener, pedido_mantener, tiempo_mantener_s):
        """botones: {numero_de_boton: pedido} que se disparan al apretar.

        boton_mantener: numero del boton que dispara pedido_mantener si se lo
        mantiene apretado tiempo_mantener_s segundos.
        """
        self.botones = botones
        self.boton_mantener = boton_mantener
        self.pedido_mantener = pedido_mantener
        self.tiempo_mantener_s = tiempo_mantener_s
        self.antes = []                # botones de la lectura anterior
        self.mantener_desde_s = None   # cuando se empezo a mantener el boton
        self.mantener_disparado = False

    @staticmethod
    def _apretado(lista, numero):
        return 0 <= numero < len(lista) and lista[numero] == 1

    def actualizar(self, botones_ahora, ahora_s):
        """Recibe la lista de botones del joystick (1 = apretado). Devuelve los pedidos."""
        pedidos = []
        for numero, pedido in self.botones.items():
            if self._apretado(botones_ahora, numero) and not self._apretado(self.antes, numero):
                pedidos.append(pedido)

        if self._apretado(botones_ahora, self.boton_mantener):
            if self.mantener_desde_s is None:
                self.mantener_desde_s = ahora_s
            mantenido = ahora_s - self.mantener_desde_s >= self.tiempo_mantener_s
            if mantenido and not self.mantener_disparado:
                pedidos.append(self.pedido_mantener)
                self.mantener_disparado = True
        else:
            self.mantener_desde_s = None
            self.mantener_disparado = False

        self.antes = list(botones_ahora)
        return pedidos
