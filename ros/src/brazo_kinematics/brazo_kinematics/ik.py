"""Cinemática inversa pura (sin ROS) del brazo de paralelogramo.

Unidades: metros y radianes dentro de las funciones; los servos salen en grados.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass
class ArmParams:
    L1: float = 0.080      # brazo superior
    L2: float = 0.080      # antebrazo
    h: float = 0.070       # altura del pivote del hombro
    d: float = 0.0         # offset horizontal del pivote respecto del eje z
    e: float = 0.055       # offset horizontal muñeca -> punta
    c: float = 0.060       # offset vertical muñeca -> punta (+ = punta debajo)
    gamma_min_deg: float = 25.0
    gamma_max_deg: float = 150.0
    # servo = sign * angulo_deg + offset ; rango en grados de servo
    sign: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    offset_deg: Tuple[float, float, float] = (90.0, 0.0, 0.0)
    servo_min_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    servo_max_deg: Tuple[float, float, float] = (180.0, 180.0, 180.0)


@dataclass
class IKResult:
    ok: bool
    message: str = ''
    servo_deg: Tuple[float, float, float] = (math.nan,) * 3
    theta1: float = math.nan
    alpha: float = math.nan
    phi: float = math.nan
    gamma: float = math.nan


def _fail(msg: str) -> IKResult:
    return IKResult(False, msg)


def solve(p: ArmParams, x: float, y: float, z: float) -> IKResult:
    """IK codo-arriba para la punta de la garra (x, y, z) en el frame de la base."""
    if not all(math.isfinite(v) for v in (x, y, z)):
        return _fail('objetivo no finito')

    theta1 = math.atan2(y, x)
    r = math.hypot(x, y) - p.d - p.e
    s = z - p.h + p.c
    D = math.hypot(r, s)

    eps = 1e-9
    if D < abs(p.L1 - p.L2) - eps or D > p.L1 + p.L2 + eps:
        return _fail(f'fuera de alcance: D={D:.4f} m, rango '
                     f'[{abs(p.L1 - p.L2):.4f}, {p.L1 + p.L2:.4f}]')
    if D < eps:
        return _fail('singularidad: D = 0')

    def _acos(v):  # tolera error numérico en los bordes del alcance
        return math.acos(max(-1.0, min(1.0, v)))

    gamma = _acos((p.L1**2 + p.L2**2 - D**2) / (2 * p.L1 * p.L2))
    alpha = math.atan2(s, r) + _acos((p.L1**2 + D**2 - p.L2**2) / (2 * p.L1 * D))
    phi = alpha + gamma - math.pi          # antebrazo, absoluto desde la horizontal

    g_deg = math.degrees(gamma)
    if not (p.gamma_min_deg <= g_deg <= p.gamma_max_deg):
        return _fail(f'ángulo del codo {g_deg:.1f}° fuera de '
                     f'[{p.gamma_min_deg}, {p.gamma_max_deg}]')

    angles = (theta1, alpha, phi)           # servo3 usa phi, NO gamma
    servos = []
    for i, (name, a) in enumerate(zip(('base', 'hombro', 'codo'), angles)):
        v = p.sign[i] * math.degrees(a) + p.offset_deg[i]
        if not (p.servo_min_deg[i] <= v <= p.servo_max_deg[i]):
            return _fail(f'servo {name} = {v:.1f}° fuera de '
                         f'[{p.servo_min_deg[i]}, {p.servo_max_deg[i]}]')
        servos.append(v)

    return IKResult(True, 'ok', tuple(servos), theta1, alpha, phi, gamma)


def forward(p: ArmParams, theta1: float, alpha: float, phi: float) -> Tuple[float, float, float]:
    """Cinemática directa (para verificar la IK). Ángulos en radianes."""
    r = p.d + p.L1 * math.cos(alpha) + p.L2 * math.cos(phi) + p.e
    z = p.h + p.L1 * math.sin(alpha) + p.L2 * math.sin(phi) - p.c
    return r * math.cos(theta1), r * math.sin(theta1), z
