# Pruebas de las cuentas de /mover (maniobras.py). Sin robot y sin simulador.
# Correr con: source install/setup.bash && python3 -m pytest src/prometeo_navigation/test

import math

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar
from prometeo_navigation.maniobras import (
    CON_COSTADO, CON_FRENTE, SIN_CONFIRMAR, PosicionLaberinto, angulo_objetivo, como_confirmar,
    cubo_mas_cercano, orden_para_acercarse,
    giro_confirmado, es_puerta, frente_abierto, frente_en_el_medio_de_la_puerta,
    giro_de_salida, normalizar_angulo, velocidad_de_giro)


def cerca(a, b):
    return abs(a - b) < 1e-9


def test_normalizar_deja_igual_un_angulo_chico():
    assert cerca(normalizar_angulo(1.0), 1.0)


def test_normalizar_da_la_vuelta_pasando_pi():
    assert cerca(normalizar_angulo(math.pi + 0.1), -math.pi + 0.1)


def test_izquierda_suma_90_grados():
    assert cerca(angulo_objetivo(0.0, Lugar.IZQUIERDA), math.pi / 2)


def test_derecha_resta_90_grados():
    assert cerca(angulo_objetivo(0.0, Lugar.DERECHA), -math.pi / 2)


def test_adelante_no_gira():
    assert cerca(angulo_objetivo(1.0, Lugar.ADELANTE), 1.0)


def test_vuelta_es_media_vuelta():
    assert cerca(abs(angulo_objetivo(0.0, Mover.Goal.VUELTA)), math.pi)


def test_izquierda_mirando_casi_atras_pasa_al_otro_lado():
    # Mirando a 170 grados, la izquierda queda en 260 = -100 grados
    objetivo = angulo_objetivo(math.radians(170), Lugar.IZQUIERDA)
    assert cerca(objetivo, math.radians(-100))


def test_salida_mal_escrita_es_none():
    assert angulo_objetivo(0.0, 'izqierda') is None


def test_ya_llego_no_gira():
    assert velocidad_de_giro(0.01, 2.0, 0.35, 0.9, 0.05) == 0.0


def test_falta_mucho_gira_al_maximo_hacia_la_izquierda():
    assert velocidad_de_giro(1.5, 2.0, 0.35, 0.9, 0.05) == 0.9


def test_falta_mucho_a_la_derecha_gira_negativo():
    assert velocidad_de_giro(-1.5, 2.0, 0.35, 0.9, 0.05) == -0.9


def test_falta_poco_gira_despacio_pero_no_menos_que_el_minimo():
    assert velocidad_de_giro(0.1, 2.0, 0.35, 0.9, 0.05) == 0.35


def test_en_el_medio_es_proporcional():
    assert cerca(velocidad_de_giro(0.3, 2.0, 0.35, 0.9, 0.05), 0.6)


def test_medio_de_la_puerta():
    assert frente_en_el_medio_de_la_puerta(190, 150) == 170


def test_medio_de_la_puerta_sin_dato_es_none():
    assert frente_en_el_medio_de_la_puerta(190, -1) is None


def test_es_puerta_si_tiene_f1():
    assert es_puerta(Lugar(frente_inicial=190.0, frente_final=150.0))


def test_esquina_no_es_puerta():
    assert not es_puerta(Lugar(frente_inicial=80.0, frente_final=-1.0))


def test_sin_lugar_no_es_puerta():
    assert not es_puerta(None)


def test_giro_de_salida():
    assert cerca(giro_de_salida(Lugar.IZQUIERDA), math.pi / 2)
    assert cerca(giro_de_salida(Lugar.DERECHA), -math.pi / 2)
    assert giro_de_salida('cualquiera') is None


def test_frente_abierto_es_pasillo():
    assert frente_abierto(120, 40) is True
    assert frente_abierto(math.inf, 40) is True


def test_frente_con_pared_no_es_pasillo():
    assert frente_abierto(20, 40) is False


def test_frente_sin_dato_no_se_sabe():
    assert frente_abierto(None, 40) is None


# ===== Como se confirma el giro (con el robot quieto) =====

def test_esquina_se_confirma_con_el_costado():
    assert como_confirmar(Lugar.IZQUIERDA, 20.0, 40) == CON_COSTADO


def test_puerta_no_se_puede_confirmar():
    assert como_confirmar(Lugar.IZQUIERDA, 150.0, 40) == SIN_CONFIRMAR
    assert como_confirmar(Lugar.DERECHA, math.inf, 40) == SIN_CONFIRMAR
    assert como_confirmar(Lugar.DERECHA, None, 40) == SIN_CONFIRMAR


def test_media_vuelta_se_confirma_con_el_frente():
    assert como_confirmar(Mover.Goal.VUELTA, 15.0, 40) == CON_FRENTE


def test_doblo_a_la_izquierda_la_pared_queda_a_la_derecha():
    # Antes: pared adelante a 20. Despues: el sensor derecho mide 22
    assert giro_confirmado(CON_COSTADO, Lugar.IZQUIERDA, 60.0, 22.0, 150.0, 20.0, 8, 40)


def test_doblo_a_la_derecha_la_pared_queda_a_la_izquierda():
    assert giro_confirmado(CON_COSTADO, Lugar.DERECHA, 18.0, 60.0, 150.0, 20.0, 8, 40)


def test_torcido_el_costado_no_coincide():
    # Quedo torcido: el sensor ve la pared en diagonal (mas lejos, o sin eco)
    assert not giro_confirmado(CON_COSTADO, Lugar.IZQUIERDA, 60.0, 35.0, 150.0, 20.0, 8, 40)
    assert not giro_confirmado(CON_COSTADO, Lugar.IZQUIERDA, 60.0, math.inf, 150.0, 20.0, 8, 40)


def test_costado_sin_dato_no_se_sabe():
    assert giro_confirmado(CON_COSTADO, Lugar.IZQUIERDA, 60.0, None, 150.0, 20.0, 8, 40) is None


def test_media_vuelta_confirma_con_el_pasillo_adelante():
    assert giro_confirmado(CON_FRENTE, Mover.Goal.VUELTA, 12.0, 12.0, 150.0, 15.0, 8, 40)
    assert not giro_confirmado(CON_FRENTE, Mover.Goal.VUELTA, 12.0, 12.0, 20.0, 15.0, 8, 40)


def test_sin_confirmar_siempre_da_bien():
    assert giro_confirmado(SIN_CONFIRMAR, Lugar.IZQUIERDA, None, None, None, None, 8, 40)


def test_si_no_giro_el_frente_sigue_igual_y_no_confirma():
    # No giro: el sensor derecho ve la pared del pasillo (12, parecida a 20) y
    # adelante sigue la misma pared a 20
    assert not giro_confirmado(CON_COSTADO, Lugar.IZQUIERDA, 400.0, 12.0, 20.0, 20.0, 8, 40)


def test_esquina_con_el_frente_sin_dato_no_se_sabe():
    assert giro_confirmado(CON_COSTADO, Lugar.IZQUIERDA, 60.0, 22.0, None, 20.0, 8, 40) is None


# ===== Posicion de laberinto =====

def test_posicion_arranca_en_cero_mirando_al_norte():
    p = PosicionLaberinto()
    assert (p.x, p.y, p.orientacion()) == (0.0, 0.0, 0.0)


def test_avanzar_al_norte_suma_en_x():
    p = PosicionLaberinto()
    p.avanzar(0.5)
    assert cerca(p.x, 0.5) and cerca(p.y, 0.0)


def test_girar_a_la_izquierda_y_avanzar_suma_en_y():
    p = PosicionLaberinto()
    p.girar(math.pi / 2)
    p.avanzar(1.0)
    assert cerca(p.x, 0.0) and cerca(p.y, 1.0)


def test_el_giro_se_redondea_a_cuartos_de_vuelta():
    p = PosicionLaberinto()
    p.girar(math.radians(73))     # confirmado aunque quedo torcido
    assert cerca(p.orientacion(), math.pi / 2)


def test_media_vuelta_y_retroceder():
    p = PosicionLaberinto()
    p.avanzar(1.0)
    p.girar(math.pi)
    p.avanzar(0.4)
    assert cerca(p.x, 0.6)


def test_cuatro_giros_a_la_derecha_vuelven_al_norte():
    p = PosicionLaberinto()
    for _ in range(4):
        p.girar(-math.pi / 2)
    assert p.cuartos == 0


def test_poner_corrige_la_posicion():
    p = PosicionLaberinto()
    p.poner(2.0, -1.0, -math.pi / 2)
    assert (p.x, p.y) == (2.0, -1.0) and cerca(p.orientacion(), -math.pi / 2)


# ===== Acercarse al cubo =====

class Det:
    """Una deteccion de mentira, con lo que usa cubo_mas_cercano."""

    def __init__(self, kind, color, x, y=0.0, confidence=0.9):
        self.kind, self.color, self.confidence = kind, color, confidence
        self.pose = type('Pose', (), {})()
        self.pose.position = type('Punto', (), {'x': x, 'y': y})()


def acercarse(x, y):
    return orden_para_acercarse(x, y, distancia_final=0.10, velocidad=0.08,
                                velocidad_min=0.03, ganancia_giro=2.0, giro_max=0.6,
                                frenar_desde=0.15)


def test_elige_el_cubo_mas_cercano():
    cubos = [Det('cube', 'red', 0.5), Det('cube', 'blue', 0.3), Det('arrow', 'red', 0.1)]
    assert cubo_mas_cercano(cubos, 0.6).color == 'blue'


def test_cubo_de_un_color():
    cubos = [Det('cube', 'red', 0.5), Det('cube', 'blue', 0.3)]
    assert cubo_mas_cercano(cubos, 0.6, 'red').color == 'red'


def test_cubo_con_poca_confianza_no_cuenta():
    assert cubo_mas_cercano([Det('cube', 'red', 0.5, confidence=0.3)], 0.6) is None


def test_lejos_y_derecho_avanza_a_velocidad_normal():
    avance, giro, llego = acercarse(0.6, 0.0)
    assert (avance, giro, llego) == (0.08, 0.0, False)


def test_cerca_va_mas_lento():
    avance, _, _ = acercarse(0.16, 0.0)        # le faltan 6 cm
    assert 0.03 <= avance < 0.08


def test_nunca_mas_lento_que_el_minimo():
    avance, _, _ = acercarse(0.101, 0.0)
    assert avance == 0.03


def test_cubo_a_la_izquierda_gira_a_la_izquierda():
    _, giro, _ = acercarse(0.5, 0.05)
    assert giro > 0


def test_cubo_muy_de_costado_gira_sin_avanzar():
    avance, giro, _ = acercarse(0.2, 0.2)       # 45 grados
    assert avance == 0.0 and giro > 0


def test_a_la_distancia_final_llego():
    assert acercarse(0.10, 0.0) == (0.0, 0.0, True)
