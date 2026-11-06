# Vuelta al inicio (T8): el camino mas rapido hasta la largada, por la libreta.
#
# Son solo cuentas, sin ROS: se prueban sin robot.
#
# Usa Dijkstra: como un GPS, busca el camino que suma menos tiempo, probando
# siempre primero el lugar al que se llega mas rapido. El "largo" de cada tramo es
# lo que tardo el robot en recorrerlo (Salida.segundos), no los metros: sin
# odometria confiable no sabemos los metros, y lo que importa para ganar es el tiempo.
# Solo usa caminos conocidos (lleva_a): nunca entra a un callejon ni a algo sin probar.

import heapq

from prometeo_navigation.libreta import LARGADA

# Un tramo que se conoce pero no se midio (por ejemplo, se recorrio solo de ida y
# no se tomo la hora) cuenta como esto. PROVISORIO: ajustar con los tiempos reales.
SEGUNDOS_SI_NO_SE_SABE = 5.0


def camino_a_la_largada(libreta, desde):
    """Los pasos para volver al inicio: [(numero de lugar, direccion), ...].

    En cada lugar de la lista hay que salir por esa direccion (punto cardinal). El
    ultimo paso es en el primer lugar, por donde arranco el robot.
    None si no hay camino conocido (por ejemplo, la libreta quedo partida por un
    tramo en manual).
    """
    por_numero = {hoja.numero: hoja for hoja in libreta.lugares}
    mejor = {desde.numero: 0.0}
    de_donde = {}            # lugar -> (lugar anterior, direccion por la que se salio)
    cola = [(0.0, desde.numero)]
    while cola:
        tiempo, numero = heapq.heappop(cola)
        if numero == LARGADA:
            return _armar_pasos(de_donde, numero)
        if tiempo > mejor.get(numero, float('inf')):
            continue                      # ya se llego antes por un camino mas rapido
        for camino in por_numero[numero].salidas:
            if camino.lleva_a is None:
                continue
            nuevo = tiempo + (camino.segundos if camino.segundos is not None
                              else SEGUNDOS_SI_NO_SE_SABE)
            if nuevo < mejor.get(camino.lleva_a, float('inf')):
                mejor[camino.lleva_a] = nuevo
                de_donde[camino.lleva_a] = (numero, camino.direccion)
                heapq.heappush(cola, (nuevo, camino.lleva_a))
    return None


def _armar_pasos(de_donde, llegada):
    pasos = []
    while llegada in de_donde:
        anterior, direccion = de_donde[llegada]
        pasos.append((anterior, direccion))
        llegada = anterior
    return list(reversed(pasos))
