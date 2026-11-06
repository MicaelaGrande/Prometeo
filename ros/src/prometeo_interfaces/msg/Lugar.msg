# Navegacion alta avisa que llego a un lugar del laberinto.
#
# Los textos de tipo y salidas estan escritos UNA sola vez, aca, como constantes.
# En el codigo se usan como Lugar.CURVA, Lugar.IZQUIERDA, etc.: si alguien tipea
# mal el nombre, Python da error enseguida (en vez de mandar un texto que nadie
# entiende sin avisar).

# Tipos de lugar
string CURVA="curva"
string BIFURCACION="bifurcacion"
string CALLEJON="callejon"

# Salidas, vistas desde el robot
string IZQUIERDA="izquierda"
string ADELANTE="adelante"
string DERECHA="derecha"

string tipo            # CURVA, BIFURCACION o CALLEJON
float64 x              # donde estaba el robot, en metros, segun /odom
float64 y
float64 orientacion    # hacia donde miraba, en radianes, segun /odom
string[] salidas       # IZQUIERDA, ADELANTE o DERECHA, sin contar por donde vino

# Lo que media el frente (cm) al detectar el lugar. -1 = no hay.
# Los usa /mover en una puerta: el detector la ve recien cuando la paso, asi que
# retrocede hasta el medio, donde el frente mide (frente_inicial + frente_final) / 2.
float64 frente_inicial   # F0: cuando se abrio el costado
float64 frente_final     # F1: cuando se volvio a cerrar (solo en una puerta)
