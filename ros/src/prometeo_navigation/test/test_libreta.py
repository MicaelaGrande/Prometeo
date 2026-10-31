# Pruebas de la libreta. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_libreta.py -v

from prometeo_navigation.libreta import Lugar, PENDIENTE, Salida


def test_una_salida_nueva_arranca_pendiente():
    salida = Salida('norte')

    assert salida.direccion == 'norte'
    assert salida.estado == PENDIENTE
    assert salida.lleva_a is None  # todavia no sabemos adonde lleva


def test_un_lugar_guarda_su_posicion_y_sus_salidas():
    lugar = Lugar(1, 1.20, 0.40, 'bifurcacion', [Salida('norte'), Salida('este')])

    assert lugar.numero == 1
    assert lugar.x == 1.20
    assert lugar.y == 0.40
    assert lugar.tipo == 'bifurcacion'
    assert len(lugar.salidas) == 2  # len cuenta los elementos de una lista
    assert lugar.salidas[0].direccion == 'norte'  # la primera de la lista es la 0


def test_cada_lugar_tiene_su_propia_lista_de_salidas():
    lugar_1 = Lugar(1, 0.0, 0.0, 'curva')
    lugar_2 = Lugar(2, 1.0, 0.0, 'curva')

    lugar_1.salidas.append(Salida('norte'))  # append agrega al final de la lista

    assert len(lugar_1.salidas) == 1
    assert len(lugar_2.salidas) == 0
