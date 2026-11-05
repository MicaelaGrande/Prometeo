# Pruebas de la logica de Decision. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_decision con:
#   python3 -m pytest test/test_logica.py -v

from types import SimpleNamespace

from prometeo_decision.logica import (
    ACERCANDO, ACERCARSE, AGARRAR, BIFURCACION, BUSCANDO_COLOR, CALLEJON, CURVA,
    DERECHA, Decision, EXPLORAR, IR_A_SALIDA, IZQUIERDA, ADELANTE,
    MAXIMO_DE_INTENTOS, SIGUIENDO, TERMINADO, VOLVER, VOLVIENDO,
    VOLVIENDO_SIN_CUBO,
)


# --- Ayudas: arman detecciones "de mentira", como las que manda la camara ---

def flecha(color, direccion='left', largo='normal', confianza=0.9):
    return SimpleNamespace(kind='arrow', color=color, direction=direccion,
                           length_class=largo, confidence=confianza)


def cubo(color, confianza=0.9):
    return SimpleNamespace(kind='cube', color=color, direction='',
                           length_class='', confidence=confianza)


def ver_varias(d, detecciones, veces=2):
    """Muestra lo mismo en varias imagenes seguidas, para pasar el filtro.

    Devuelve la Orden de la ultima imagen.
    """
    orden = None
    for _ in range(veces):
        orden = d.ver(detecciones)
    return orden


# --- 1. Elegir color ---------------------------------------------------------

def test_la_primera_flecha_elige_el_color():
    d = Decision()
    ver_varias(d, [flecha('red')])

    assert d.color_actual == 'red'
    assert d.estado == SIGUIENDO


def test_una_flecha_dudosa_no_elige_color():
    d = Decision()
    ver_varias(d, [flecha('red', confianza=0.3)])  # menos que confianza_minima (0.6)

    assert d.color_actual is None
    assert d.estado == BUSCANDO_COLOR


def test_con_las_3_flechas_arranca_por_rojo():
    d = Decision()
    # La roja va ultima en la lista a proposito: igual tiene que elegirla.
    ver_varias(d, [flecha('blue'), flecha('green'), flecha('red')])

    assert d.color_actual == 'red'


def test_sin_flecha_roja_no_arranca_con_otro_color():
    d = Decision()
    ver_varias(d, [flecha('blue'), flecha('green')])

    assert d.color_actual is None
    assert d.estado == BUSCANDO_COLOR


def test_despues_del_rojo_sigue_el_verde_y_no_el_azul():
    d = Decision()
    ver_varias(d, [flecha('red')])
    ver_varias(d, [cubo('red')])
    d.cubo_agarrado()
    d.cubo_depositado()
    # Ya deposito el rojo. La azul va primero en la lista a proposito.
    ver_varias(d, [flecha('blue'), flecha('green')])

    assert d.color_actual == 'green'


# --- 2. Ignorar los otros colores -------------------------------------------

def test_siguiendo_rojo_ignora_la_flecha_verde():
    d = Decision()
    ver_varias(d, [flecha('red', 'left')])
    ver_varias(d, [flecha('green', 'right')])

    assert d.color_actual == 'red'  # no cambio de color
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, DERECHA])
    assert orden.salida == IZQUIERDA  # siguio la roja, no la verde


# --- 3. Seguir la flecha en la bifurcacion ----------------------------------

def test_flecha_roja_izquierda_manda_a_la_izquierda():
    d = Decision()
    ver_varias(d, [flecha('red', 'left', 'far')])
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, ADELANTE])

    assert orden.accion == IR_A_SALIDA
    assert orden.salida == IZQUIERDA
    assert orden.length_class == 'far'  # le pasa el largo a Navegacion


def test_flecha_para_arriba_manda_derecho():
    d = Decision()
    ver_varias(d, [flecha('red', 'up')])
    orden = d.llegar_a_lugar(BIFURCACION, [ADELANTE, DERECHA])

    assert orden.salida == ADELANTE


# --- 4. Cuando decide Exploracion -------------------------------------------

def test_bifurcacion_sin_flecha_de_mi_color_pide_explorar():
    d = Decision()
    ver_varias(d, [flecha('red')])
    d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, ADELANTE])  # usa la flecha roja
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, ADELANTE])  # ya no hay flecha

    assert orden.accion == EXPLORAR


def test_flecha_sin_direccion_clara_pide_explorar():
    d = Decision()
    ver_varias(d, [flecha('red', 'UNK')])
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, DERECHA])

    assert orden.accion == EXPLORAR


def test_callejon_pide_explorar_y_curva_no_pide_nada():
    d = Decision()
    ver_varias(d, [flecha('red')])

    assert d.llegar_a_lugar(CALLEJON, []).accion == EXPLORAR
    assert d.llegar_a_lugar(CURVA, [IZQUIERDA]) is None


# --- 5. Cubos ----------------------------------------------------------------

def test_siguiendo_rojo_ignora_el_cubo_verde():
    d = Decision()
    ver_varias(d, [flecha('red')])
    orden = ver_varias(d, [cubo('green')])

    assert orden is None
    assert d.estado == SIGUIENDO


def test_ve_el_cubo_rojo_y_se_acerca():
    d = Decision()
    ver_varias(d, [flecha('red')])
    orden = ver_varias(d, [cubo('red')])

    assert orden.accion == ACERCARSE
    assert d.estado == ACERCANDO


def test_llegar_al_cubo_pide_agarrarlo():
    d = Decision()
    ver_varias(d, [flecha('red')])
    ver_varias(d, [cubo('red')])
    orden = d.llegue_al_cubo()

    assert orden.accion == AGARRAR
    assert d.estado == ACERCANDO   # sigue aca hasta que el brazo confirme


def test_llegar_al_cubo_sin_estar_acercandose_no_hace_nada():
    d = Decision()
    orden = d.llegue_al_cubo()

    assert orden is None
    assert d.estado == BUSCANDO_COLOR


def test_si_no_lo_agarra_lo_intenta_de_nuevo():
    d = Decision()
    ver_varias(d, [flecha('red')])
    ver_varias(d, [cubo('red')])
    d.llegue_al_cubo()
    orden = d.cubo_no_agarrado()

    assert orden.accion == AGARRAR     # lo vuelve a intentar
    assert d.estado == ACERCANDO       # no vuelve sin el cubo
    assert d.color_actual == 'red'     # sigue con el mismo color

    # Al segundo intento lo agarra y ahi si vuelve.
    assert d.cubo_agarrado().accion == VOLVER


def test_no_agarrado_sin_estar_acercandose_no_hace_nada():
    d = Decision()
    orden = d.cubo_no_agarrado()

    assert orden is None
    assert d.estado == BUSCANDO_COLOR


def test_agarrar_el_cubo_pide_volver():
    d = Decision()
    ver_varias(d, [flecha('red')])
    ver_varias(d, [cubo('red')])
    d.llegue_al_cubo()
    orden = d.cubo_agarrado()

    assert orden.accion == VOLVER
    assert d.estado == VOLVIENDO
    assert d.color_actual == 'red'   # el color se libera recien al depositar


def test_agarrar_sin_estar_acercandose_no_hace_nada():
    d = Decision()
    orden = d.cubo_agarrado()

    assert orden is None
    assert d.estado == BUSCANDO_COLOR


# --- 6. Agarrar, depositar y terminar ---------------------------------------

def test_depositar_libera_el_color():
    d = Decision()
    ver_varias(d, [flecha('red')])
    ver_varias(d, [cubo('red')])
    d.cubo_agarrado()
    assert d.estado == VOLVIENDO
    d.cubo_depositado()

    assert d.cubos_faltantes == {'green', 'blue'}
    assert d.color_actual is None
    assert d.estado == BUSCANDO_COLOR


def test_despues_de_los_3_cubos_termina():
    d = Decision()
    for color in ['red', 'green', 'blue']:
        ver_varias(d, [flecha(color)])
        ver_varias(d, [cubo(color)])
        d.cubo_agarrado()
        d.cubo_depositado()

    assert d.cubos_faltantes == set()
    assert d.estado == TERMINADO

# --- 7. Filtro "la mejor de 3" ----------------------------------------------

def test_una_sola_imagen_no_alcanza_para_elegir_color():
    d = Decision()
    d.ver([flecha('red')])

    assert d.color_actual is None
    assert d.estado == BUSCANDO_COLOR


def test_dos_de_tres_imagenes_alcanzan():
    d = Decision()
    d.ver([flecha('red')])
    d.ver([])                 # en esta imagen la camara no vio nada
    d.ver([flecha('red')])

    assert d.color_actual == 'red'


def test_gana_la_direccion_que_mas_se_repite():
    d = Decision()
    d.ver([flecha('red', 'left')])
    d.ver([flecha('red', 'right')])   # la camara se equivoco una vez
    d.ver([flecha('red', 'left')])
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, DERECHA])

    assert orden.salida == IZQUIERDA


def test_un_cubo_en_una_sola_imagen_no_alcanza():
    d = Decision()
    ver_varias(d, [flecha('red')])

    assert d.ver([cubo('red')]) is None   # 1 vez: todavia no le cree
    assert d.estado == SIGUIENDO
    orden = d.ver([cubo('red')])          # 2 veces: ahora si
    assert orden.accion == ACERCARSE


# --- 8. Intentos de agarre y abandonar el color ------------------------------

def llegar_al_cubo_rojo():
    """Arma una Decision que siguio el rojo y ya esta frente a su cubo."""
    d = Decision()
    ver_varias(d, [flecha('red')])
    ver_varias(d, [cubo('red')])
    d.llegue_al_cubo()  # intento 1
    return d


def test_falla_3_veces_y_abandona_el_color():
    d = llegar_al_cubo_rojo()
    assert d.cubo_no_agarrado().accion == AGARRAR  # intento 2
    assert d.cubo_no_agarrado().accion == AGARRAR  # intento 3
    orden = d.cubo_no_agarrado()                   # fallo el 3: abandona

    assert orden.accion == VOLVER
    assert d.estado == VOLVIENDO_SIN_CUBO
    assert d.color_actual is None
    assert d.cubos_faltantes == {'green', 'blue'}  # el rojo quedo descartado


def test_al_llegar_al_inicio_sigue_con_el_verde():
    d = llegar_al_cubo_rojo()
    for _ in range(MAXIMO_DE_INTENTOS):
        d.cubo_no_agarrado()
    d.llegue_al_inicio()
    assert d.estado == BUSCANDO_COLOR

    ver_varias(d, [flecha('red'), flecha('blue'), flecha('green')])
    assert d.color_actual == 'green'  # el rojo ya no cuenta


def test_con_el_color_nuevo_los_intentos_empiezan_de_cero():
    d = llegar_al_cubo_rojo()
    for _ in range(MAXIMO_DE_INTENTOS):
        d.cubo_no_agarrado()
    d.llegue_al_inicio()
    ver_varias(d, [flecha('green')])
    ver_varias(d, [cubo('green')])
    d.llegue_al_cubo()

    assert d.cubo_no_agarrado().accion == AGARRAR  # no abandona el verde enseguida


def test_llegar_al_inicio_sin_haber_abandonado_no_hace_nada():
    d = Decision()
    d.llegue_al_inicio()

    assert d.estado == BUSCANDO_COLOR
