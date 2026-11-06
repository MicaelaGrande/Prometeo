# Exploracion con memoria (T4): elige por donde seguir cuando no hay flecha.
# Lo usa el nodo mapeo cuando Decision pide explorar (/decision/explorar).
#
# Son solo cuentas sobre la libreta, sin ROS: se prueban sin robot.
# La logica es la de TICKETS_NAVEGACION.md, "Como funciona la exploracion":
#   1. Si en este lugar hay una salida que todavia no se probo, tomar esa.
#   2. Si no (callejon, o ya probo todo), volver hacia el lugar mas cercano que
#      tenga una salida sin probar, por los caminos ya conocidos.
#   3. Si no queda ninguna en toda la libreta, dar media vuelta.
# Asi, al rodear una isla y volver a un lugar conocido, no repite la vuelta.

from collections import deque

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar as AvisoLugar
from prometeo_navigation.libreta import (
    LARGADA, PENDIENTE, direccion_de_salida, salida_hacia)

# PROVISORIO: el orden en que prueba las salidas pendientes. Da lo mismo cual, porque
# la libreta despues hace que pruebe las otras; se puede cambiar si conviene.
ORDEN_DE_PRUEBA = (AvisoLugar.DERECHA, AvisoLugar.ADELANTE, AvisoLugar.IZQUIERDA)


def tiene_pendientes(hoja):
    return any(s.estado == PENDIENTE for s in hoja.salidas)


def primer_paso_hacia_pendiente(libreta, desde):
    """Por que direccion salir de 'desde' para llegar al lugar mas cercano con pendientes.

    Recorre la libreta por los caminos ya conocidos (las salidas con lleva_a), de a un
    lugar por vez y empezando por los mas cercanos: es una busqueda en anchura (BFS),
    asi el primero que encuentra es el que esta a menos lugares de distancia.
    Devuelve el punto cardinal, o None si no hay ningun lugar con pendientes.
    """
    por_numero = {hoja.numero: hoja for hoja in libreta.lugares}
    visitados = {desde.numero}
    # Cada elemento: (hoja, direccion del primer paso desde 'desde')
    cola = deque()
    visitados.add(LARGADA)   # la largada no es un lugar de la libreta
    for camino in desde.salidas:
        if camino.lleva_a is not None and camino.lleva_a not in visitados:
            visitados.add(camino.lleva_a)
            cola.append((por_numero[camino.lleva_a], camino.direccion))
    while cola:
        hoja, primer_paso = cola.popleft()
        if tiene_pendientes(hoja):
            return primer_paso
        for camino in hoja.salidas:
            if camino.lleva_a is not None and camino.lleva_a not in visitados:
                visitados.add(camino.lleva_a)
                cola.append((por_numero[camino.lleva_a], primer_paso))
    return None


def elegir_salida(libreta, orientacion):
    """La salida (izquierda, adelante, derecha o VUELTA) para pedirle a /mover.

    orientacion: hacia donde mira el robot en el lugar actual (la del aviso).
    None si todavia no hay lugar actual en la libreta.
    """
    hoja = libreta.actual
    if hoja is None:
        return None
    # 1. Una salida sin probar aca
    for salida in ORDEN_DE_PRUEBA:
        direccion = direccion_de_salida(orientacion, salida)
        if any(s.direccion == direccion and s.estado == PENDIENTE for s in hoja.salidas):
            return salida
    # 2. Volver hacia el lugar con pendientes mas cercano
    direccion = primer_paso_hacia_pendiente(libreta, hoja)
    if direccion is not None:
        return salida_hacia(orientacion, direccion)
    # 3. No queda nada sin probar: media vuelta
    return Mover.Goal.VUELTA
