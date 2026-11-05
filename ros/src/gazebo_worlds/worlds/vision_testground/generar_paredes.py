#!/usr/bin/env python3
"""Genera paredes.xacro con las paredes del laberinto de ejemplo.

El laberinto se describe por el centro de sus pasillos. Cada pasillo mide
ANCHO de ancho, y las paredes son el borde de la union de todos los pasillos.

Las medidas salen de la pista real: un tramo de pared mide 32 cm, asi que dos
curvas separadas por n tramos estan a 0,32 * n + 0,40 m entre centros.

Uso, desde esta carpeta:
    python3 generar_paredes.py
    xacro vision_testground.world.xacro -o vision_testground.world
"""
ANCHO = 0.40  # m, ancho libre del pasillo
ESPESOR = 0.01  # m

PASILLOS = [
    # Entrada: la pared abierta del lado de la partida
    ((1.28, 0), (0, 0)),
    # Rama sur, hacia el cubo azul
    ((0, 0), (0, -0.72)),
    ((0, -0.72), (-1.04, -0.72)),
    ((-1.04, -0.72), (-1.04, -1.44)),
    # Isla chica: su bloque mide 64 x 32 cm. La rodea un anillo de pasillos
    ((-1.56, -1.44), (-0.52, -1.44)),
    ((-1.56, -2.16), (-0.52, -2.16)),
    ((-1.56, -1.44), (-1.56, -2.16)),
    ((-0.52, -1.44), (-0.52, -2.16)),
    ((-1.04, -2.16), (-1.04, -3.20)),  # callejon del cubo azul
    # Rama norte, hacia los cubos rojo y verde
    ((0, 0), (0, 1.04)),
    ((0, 1.04), (-0.72, 1.04)),
    # Isla grande: su bloque mide 96 x 64 cm
    ((-0.72, 0.52), (-0.72, 1.56)),
    ((-2.08, 0.52), (-2.08, 1.56)),
    ((-0.72, 0.52), (-2.08, 0.52)),
    ((-0.72, 1.56), (-2.08, 1.56)),
    ((-2.08, 1.04), (-2.80, 1.04)),
    # Bifurcacion: al sur el callejon del cubo rojo, al norte sigue el verde
    ((-2.80, 1.04), (-2.80, -0.64)),
    ((-2.80, 1.04), (-2.80, 2.40)),
    ((-2.80, 2.40), (-4.16, 2.40)),
    ((-4.16, 2.40), (-4.16, 4.08)),  # callejon del cubo verde
    ((-4.16, 2.40), (-4.16, 1.04)),  # camino sin cubo, para explorar
    ((-4.16, 1.04), (-5.20, 1.04)),
]
ABIERTO = (1.28 + ANCHO / 2, 0.0)  # centro de la abertura de la entrada


def rectangulos():
    mitad = ANCHO / 2
    return [(min(a[0], b[0]) - mitad, min(a[1], b[1]) - mitad,
             max(a[0], b[0]) + mitad, max(a[1], b[1]) + mitad)
            for a, b in PASILLOS]


def libre(rects, x, y):
    return any(x0 < x < x1 and y0 < y < y1 for x0, y0, x1, y1 in rects)


def tramos_de_borde(rects):
    """Devuelve los bordes (eje, fijo, ini, fin, lado) entre libre y solido."""
    xs = sorted({round(v, 4) for r in rects for v in (r[0], r[2])})
    ys = sorted({round(v, 4) for r in rects for v in (r[1], r[3])})
    eps = 1e-3
    tramos = []
    for x in xs:
        for y0, y1 in zip(ys, ys[1:]):
            medio = (y0 + y1) / 2
            izq, der = libre(rects, x - eps, medio), libre(rects, x + eps, medio)
            if izq != der:
                tramos.append(('x', x, y0, y1, 1 if izq else -1))
    for y in ys:
        for x0, x1 in zip(xs, xs[1:]):
            medio = (x0 + x1) / 2
            aba, arr = libre(rects, medio, y - eps), libre(rects, medio, y + eps)
            if aba != arr:
                tramos.append(('y', y, x0, x1, 1 if aba else -1))
    return tramos


def unir(tramos):
    """Une los bordes consecutivos sobre la misma recta."""
    unidos = []
    for t in sorted(tramos):
        ultimo = unidos[-1] if unidos else None
        if ultimo and ultimo[:2] == t[:2] and ultimo[4] == t[4] \
                and abs(ultimo[3] - t[2]) < 1e-6:
            unidos[-1] = (*ultimo[:3], t[3], t[4])
        else:
            unidos.append(t)
    return unidos


def main():
    paredes = []
    for eje, fijo, ini, fin, lado in unir(tramos_de_borde(rectangulos())):
        largo = fin - ini + 2 * ESPESOR
        medio = (ini + fin) / 2
        centro = fijo + lado * ESPESOR / 2
        if eje == 'x':
            if abs(fijo - ABIERTO[0]) < 1e-6 and abs(medio - ABIERTO[1]) < 1e-6:
                continue
            paredes.append((centro, medio, ESPESOR, largo))
        else:
            paredes.append((medio, centro, largo, ESPESOR))
    lineas = ['<?xml version="1.0" ?>',
              '<!-- Generado por generar_paredes.py: no editar a mano -->',
              '<sdf version="1.9" xmlns:xacro="http://www.ros.org/wiki/xacro">',
              '  <xacro:macro name="paredes">']
    for i, (x, y, sx, sy) in enumerate(paredes):
        lineas.append(f'    <xacro:wall_box name="{i}" x="{x:.4f}" y="{y:.4f}"'
                      f' sx="{sx:.4f}" sy="{sy:.4f}"/>')
    lineas += ['  </xacro:macro>', '</sdf>', '']
    with open('paredes.xacro', 'w') as archivo:
        archivo.write('\n'.join(lineas))
    print(len(paredes), 'paredes')


if __name__ == '__main__':
    main()
