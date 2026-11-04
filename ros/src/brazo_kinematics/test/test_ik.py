import math

import pytest

from brazo_kinematics.ik import ArmParams, forward, solve


@pytest.fixture
def p():
    # rango de servo ancho: estos tests verifican la matemática, no la calibración
    return ArmParams(servo_min_deg=(-360.0,) * 3, servo_max_deg=(360.0,) * 3)


@pytest.mark.parametrize('x,y,z', [(0.12, 0.0, 0.08), (0.10, 0.06, 0.10), (0.09, -0.05, 0.05),
                                   (0.14, 0.02, 0.12)])
def test_roundtrip(p, x, y, z):
    res = solve(p, x, y, z)
    assert res.ok, res.message
    fx, fy, fz = forward(p, res.theta1, res.alpha, res.phi)
    assert (fx, fy, fz) == pytest.approx((x, y, z), abs=1e-9)


def test_servo3_is_phi_not_gamma(p):
    res = solve(p, 0.12, 0.0, 0.08)
    assert res.servo_deg[2] == pytest.approx(math.degrees(res.phi) + p.offset_deg[2])
    assert res.servo_deg[2] != pytest.approx(math.degrees(res.gamma))


def test_unreachable(p):
    assert not solve(p, 1.0, 0.0, 0.0).ok
    assert not solve(p, p.e + p.d, 0.0, p.h - p.c).ok       # D = 0
    assert not solve(p, math.nan, 0, 0).ok


def test_gamma_limit(p):
    q = ArmParams(gamma_min_deg=100.0, gamma_max_deg=150.0,
                  servo_min_deg=(-360.0,) * 3, servo_max_deg=(360.0,) * 3)
    reach = p.L1 + p.L2 - 1e-6                                # codo casi estirado: gamma ~ 180
    assert not solve(q, p.e + reach, 0.0, p.h - p.c).ok


def test_servo_range(p):
    q = ArmParams(servo_min_deg=(-360.0,) * 3, servo_max_deg=(10.0, 360.0, 360.0))
    assert not solve(q, 0.12, 0.0, 0.08).ok                   # base = 90° > 10°
