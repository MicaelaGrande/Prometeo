# Navegacion alta avisa que llego a un lugar del laberinto.
string tipo            # curva, bifurcacion o callejon
float64 x              # donde estaba el robot, en metros, segun /odom
float64 y
float64 orientacion    # hacia donde miraba, en radianes, segun /odom
string[] salidas       # izquierda, adelante o derecha, sin contar por donde vino