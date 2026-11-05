#!/usr/bin/env python3
"""Revisa conflictos de pines GPIO definidos en drivers/Kconfig.projbuild.

Toma el valor de cada PROMETEO_*_PIN de sdkconfig si existe; si no, el default
del Kconfig. Detecta:
  - ERROR: mismo GPIO usado por dos funciones (salvo grupos compartidos a propósito)
  - ERROR: pin fuera de rango, pines de flash (6-11), pin solo-entrada usado como salida
  - WARN : pines de arranque (0, 2, 12, 15), UART0 (1, 3)

Uso: python3 tools/check_pins.py [--kconfig RUTA] [--sdkconfig RUTA]
Sale con código 1 si hay errores (sirve en CI / pre-commit).
"""
import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent

SOLO_ENTRADA = {34, 35, 36, 39}
PINES_FLASH = set(range(6, 12))
ARRANQUE = {0: "strapping", 2: "strapping (LED)", 12: "strapping MTDI", 15: "strapping MTDO"}
UART0 = {1: "UART0 TX (consola/flasheo)", 3: "UART0 RX (consola/flasheo)"}

# Varias funciones pueden compartir un pin a propósito (los 3 HC-SR04 comparten TRIG)
COMPARTIDOS = {"PROMETEO_SENSOR_TRIG_PIN"}


def es_entrada(nombre):
    return "ECHO" in nombre or "ENCODER" in nombre or nombre.endswith("_RX_PIN")


def leer_kconfig(ruta):
    """Devuelve {nombre: (default, (min, max) | None, descripcion)}."""
    pines, actual = {}, None
    for linea in ruta.read_text().splitlines():
        s = linea.strip()
        m = re.match(r"config\s+(PROMETEO_\w+_PIN)\b", s)
        if m:
            actual = m.group(1)
            pines[actual] = {"default": None, "rango": None, "desc": actual}
            continue
        if actual is None:
            continue
        if (m := re.match(r'int\s+"(.*)"', s)):
            pines[actual]["desc"] = m.group(1)
        elif (m := re.match(r"range\s+(\d+)\s+(\d+)", s)):
            pines[actual]["rango"] = (int(m.group(1)), int(m.group(2)))
        elif (m := re.match(r"default\s+(\d+)", s)):
            pines[actual]["default"] = int(m.group(1))
    return pines


def leer_sdkconfig(ruta):
    valores = {}
    if ruta.exists():
        for linea in ruta.read_text().splitlines():
            m = re.match(r"CONFIG_(PROMETEO_\w+_PIN)=(\d+)", linea)
            if m:
                valores[m.group(1)] = int(m.group(2))
    return valores


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--kconfig", type=Path, default=RAIZ / "drivers" / "Kconfig.projbuild")
    ap.add_argument("--sdkconfig", type=Path, default=RAIZ / "sdkconfig")
    args = ap.parse_args()

    pines = leer_kconfig(args.kconfig)
    if not pines:
        print(f"No se encontraron PROMETEO_*_PIN en {args.kconfig}")
        return 2
    override = leer_sdkconfig(args.sdkconfig)

    valor, origen = {}, {}
    for nombre, info in pines.items():
        if nombre in override:
            valor[nombre], origen[nombre] = override[nombre], "sdkconfig"
        elif info["default"] is not None:
            valor[nombre], origen[nombre] = info["default"], "default"
    for nombre in override.keys() - pines.keys():
        print(f"AVISO: {nombre} está en sdkconfig pero no en el Kconfig (símbolo viejo)")

    errores, avisos = [], []

    # 1) Duplicados
    por_pin = defaultdict(list)
    for nombre, gpio in valor.items():
        por_pin[gpio].append(nombre)
    for gpio, nombres in sorted(por_pin.items()):
        if len(nombres) > 1 and not all(n in COMPARTIDOS for n in nombres):
            lista = ", ".join(f"{n} ({pines[n]['desc']})" for n in nombres)
            errores.append(f"GPIO{gpio} repetido: {lista}")

    # 2) Validez de cada pin
    for nombre, gpio in sorted(valor.items(), key=lambda kv: kv[1]):
        info = pines[nombre]
        if info["rango"] and not info["rango"][0] <= gpio <= info["rango"][1]:
            errores.append(f"GPIO{gpio} fuera de rango {info['rango']}: {nombre}")
        if gpio > 39 or gpio in (20, 24) or gpio in range(28, 32):
            errores.append(f"GPIO{gpio} no existe en el ESP32: {nombre}")
        if gpio in PINES_FLASH:
            errores.append(f"GPIO{gpio} está conectado a la flash: {nombre}")
        if gpio in SOLO_ENTRADA and not es_entrada(nombre):
            errores.append(f"GPIO{gpio} es solo-entrada y {nombre} es una salida")
        if gpio in ARRANQUE:
            avisos.append(f"GPIO{gpio} pin de {ARRANQUE[gpio]}: {nombre}")
        if gpio in UART0:
            avisos.append(f"GPIO{gpio} es {UART0[gpio]}: {nombre}")

    print(f"{len(valor)} pines revisados ({sum(o == 'sdkconfig' for o in origen.values())} desde sdkconfig, "
          f"{sum(o == 'default' for o in origen.values())} desde defaults)\n")
    for e in errores:
        print(f"ERROR  {e}")
    for a in avisos:
        print(f"AVISO  {a}")
    if not errores and not avisos:
        print("OK: sin conflictos")
    elif not errores:
        print("\nOK: sin conflictos (hay avisos)")
    else:
        print(f"\n{len(errores)} error(es)")
    return 1 if errores else 0


if __name__ == "__main__":
    sys.exit(main())
