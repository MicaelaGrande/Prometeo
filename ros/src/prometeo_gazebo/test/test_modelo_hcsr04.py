"""Pruebas del modelo del HC-SR04 con geometria conocida.

Se corren sin Gazebo:  python3 -m pytest src/prometeo_gazebo/test
"""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import modelo_hcsr04 as modelo  # noqa: E402

ANGULOS = np.radians(np.linspace(-15.0, 15.0, 21))
SIN_RUIDO = modelo.Parametros(ruido_base_cm=0.0, ruido_proporcional=0.0, prob_eco_perdido=0.0)


def rayos_contra_rectas(paredes):
    """Simula el abanico contra rectas infinitas dadas como (punto, normal)."""
    rangos = np.full(len(ANGULOS), np.inf)
    for i, angulo in enumerate(ANGULOS):
        rayo = np.array([math.cos(angulo), math.sin(angulo)])
        for punto, normal in paredes:
            normal = np.asarray(normal, dtype=float)
            denominador = rayo @ normal
            if abs(denominador) > 1e-9:
                t = (np.asarray(punto, dtype=float) @ normal) / denominador
                if t > 0:
                    rangos[i] = min(rangos[i], t)
    return rangos


def pared_a(distancia, inclinacion_deg):
    """Pared a esa distancia, girada respecto de estar perfectamente de frente."""
    a = math.radians(inclinacion_deg)
    normal = (math.cos(a), math.sin(a))
    return (distancia * normal[0], distancia * normal[1]), normal


def eco(rangos):
    return modelo.eco_mas_cercano(rangos, ANGULOS, SIN_RUIDO.angulo_max_rebote_deg)


def test_pared_de_frente():
    assert eco(rayos_contra_rectas([pared_a(1.0, 0)])) == pytest.approx(1.0)


def test_pared_poco_inclinada_da_la_distancia_mas_corta():
    # Un rayo del cono pega derecho, asi que mide 1 m y no lo que hay sobre el eje
    rangos = rayos_contra_rectas([pared_a(1.0, 10)])
    assert rangos[10] > 1.01
    assert eco(rangos) == pytest.approx(1.0, abs=0.005)


def test_pared_muy_inclinada_no_devuelve_eco():
    assert eco(rayos_contra_rectas([pared_a(1.0, 40)])) is None


def test_paredes_de_un_pasillo_visto_a_lo_largo_no_dan_eco():
    # Es la diferencia con un simple "minimo del cono", que daria unos 77 cm
    rangos = rayos_contra_rectas([((0, 0.2), (0, 1)), ((0, -0.2), (0, 1))])
    assert np.nanmin(rangos) == pytest.approx(0.2 / math.sin(math.radians(15)), abs=0.01)
    assert eco(rangos) is None


def test_esquina_interior_devuelve_eco_aunque_las_paredes_esten_inclinadas():
    # Sensor mirando a una esquina a 45 grados: ninguna pared esta de frente
    d = 1.0 / math.sqrt(2)
    rangos = rayos_contra_rectas([pared_a(d, 45), pared_a(d, -45)])
    assert eco(rangos) == pytest.approx(1.0, abs=0.01)


def test_punta_hacia_afuera_no_cuenta_como_esquina():
    # La punta de una isla: el rayo del medio es el mas cercano, no el mas lejano
    rangos = 0.70 /(np.cos(ANGULOS) + np.abs(np.sin(ANGULOS)))
    assert eco(rangos) is None


def test_pared_cercana_tapando_a_otra_lejana():
    # Solo los rayos de la izquierda tocan la pared cercana, de frente al sensor
    rangos = np.where(ANGULOS > 0, 0.5 / np.cos(ANGULOS), 2.0 / np.cos(ANGULOS))
    assert eco(rangos) == pytest.approx(0.5, abs=0.01)


def test_nada_a_la_vista():
    assert eco(np.full(21, np.inf)) is None
    lectura = modelo.lectura_cm(np.full(21, np.inf), ANGULOS, SIN_RUIDO, np.random.default_rng(0))
    assert lectura == modelo.DIST_LEJOS_CM


@pytest.mark.parametrize('eco_us, esperado', [
    (5800, 100.0),
    (116, 2.0),
    (115, 400.0),     # pegado a la pared: se lee como lejos, igual que el firmware
    (23200, 400.0),
    (23201, 400.0),
])
def test_conversion_del_firmware(eco_us, esperado):
    assert modelo.eco_a_cm(eco_us) == pytest.approx(esperado)


def test_lectura_a_20_grados_casi_sin_error():
    rangos = rayos_contra_rectas([pared_a(1.0, 0)])
    lectura = modelo.lectura_cm(rangos, ANGULOS, SIN_RUIDO, np.random.default_rng(0))
    # A 20 grados el sonido es un poco mas lento de lo que asume el firmware
    assert lectura == pytest.approx(100.0, abs=0.5)


def test_ruido_tiene_el_tamano_pedido():
    rangos = rayos_contra_rectas([pared_a(1.0, 0)])
    parametros = modelo.Parametros(prob_eco_perdido=0.0)
    generador = np.random.default_rng(1)
    lecturas = [modelo.lectura_cm(rangos, ANGULOS, parametros, generador) for _ in range(500)]
    assert np.std(lecturas) == pytest.approx(0.5, abs=0.1)


def test_eco_perdido_da_lejos():
    rangos = rayos_contra_rectas([pared_a(1.0, 0)])
    parametros = modelo.Parametros(prob_eco_perdido=1.0)
    assert modelo.lectura_cm(rangos, ANGULOS, parametros, np.random.default_rng(0)) == 400.0


def test_mediana_descarta_un_400_suelto():
    mediana = modelo.Mediana()
    assert mediana.agregar(50.0) == 50.0
    assert mediana.agregar(400.0) == 400.0    # con 2 lecturas toma la de mas arriba
    assert mediana.agregar(51.0) == 51.0
