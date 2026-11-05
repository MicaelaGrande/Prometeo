#!/usr/bin/env python3
"""Busca la ESP32-CAM en la red y arranca camera_bridge con su IP (T11, panel).

La IP de la camara la pone el router cada vez que se conecta (DHCP), asi que puede
cambiar. Este programa:
  1. Prueba primero la IP que se le pase (parametro camara_ip), si hay una.
  2. Si no responde, recorre la red de esta compu (las 254 direcciones) buscando la
     que transmite video en http://<ip>/stream.
     En la competencia van a estar las camaras de otros equipos en la misma red: por eso
     solo acepta la que tiene la MAC de la nuestra (el codigo de fabrica de la placa, que
     no cambia aunque cambie la IP). Parametro camara_mac; vacio = aceptar cualquiera.
  3. Cuando la encuentra, arranca camera_bridge con esa direccion.
  4. Si camera_bridge se cierra, o todavia no hay camara, vuelve a buscar cada pocos
     segundos.

Para solo ver la IP, sin arrancar nada:
    ros2 run my_robot_bringup buscar_camara.py --solo-buscar
"""

import ipaddress
import re
import socket
import subprocess
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor


def es_camara(ip, espera_s=0.8):
    """True si en esa IP hay un video MJPEG en /stream (lo que manda la ESP32-CAM)."""
    try:
        with urllib.request.urlopen(f'http://{ip}/stream', timeout=espera_s) as respuesta:
            return 'multipart' in respuesta.headers.get('Content-Type', '')
    except Exception:  # noqa: BLE001 (cualquier falla = no es la camara)
        return False


def red_local():
    """La red /24 de esta compu (por ejemplo 172.16.0.0/24), o None si no hay red."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))   # no manda nada: solo elige la interfaz
            mi_ip = s.getsockname()[0]
    except OSError:
        return None, None
    return mi_ip, ipaddress.ip_network(f'{mi_ip}/24', strict=False)


# La MAC de nuestra ESP32-CAM (vista el 4/11/2026 con "ip neigh")
MAC_NUESTRA = 'd4:e9:f4:8a:b6:68'


def mac_de(ip):
    """La MAC de una IP de la red local, o '' si no se sabe (sale de la tabla ARP)."""
    try:
        salida = subprocess.run(['ip', 'neigh', 'show', ip], capture_output=True,
                                text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return ''
    encontrada = re.search(r'lladdr ([0-9a-f:]{17})', salida)
    return encontrada.group(1) if encontrada else ''


def es_la_nuestra(ip, mac):
    if not mac:
        return True
    mac_ip = mac_de(ip)
    if mac_ip != mac.lower():
        print(f'[buscar_camara] En {ip} hay una camara, pero no es la nuestra '
              f'(MAC {mac_ip or "desconocida"}). La salteo.', flush=True)
        return False
    return True


def buscar(ip_preferida='', mac=MAC_NUESTRA):
    """Devuelve la IP de nuestra camara, o None si no la encuentra."""
    if ip_preferida and es_camara(ip_preferida) and es_la_nuestra(ip_preferida, mac):
        return ip_preferida
    mi_ip, red = red_local()
    if red is None:
        return None
    candidatas = [str(h) for h in red.hosts() if str(h) != mi_ip]
    with ThreadPoolExecutor(max_workers=64) as hilos:
        for ip, si in zip(candidatas, hilos.map(es_camara, candidatas)):
            if si and es_la_nuestra(ip, mac):
                return ip
    return None


def main():
    args = sys.argv[1:]
    solo_buscar = '--solo-buscar' in args
    ip_preferida, mac = '', MAC_NUESTRA
    for a in args:
        if a.startswith('camara_ip:='):
            ip_preferida = a.split(':=', 1)[1]
        if a.startswith('camara_mac:='):
            mac = a.split(':=', 1)[1]

    while True:
        print('[buscar_camara] Buscando la ESP32-CAM en la red...', flush=True)
        ip = buscar(ip_preferida, mac)
        if ip is None:
            if solo_buscar:
                print('[buscar_camara] No la encontre. Esta prendida y en la misma red WiFi?')
                return 1
            print('[buscar_camara] Todavia no aparece. Pruebo de nuevo en 5 s.', flush=True)
            time.sleep(5)
            continue
        url = f'http://{ip}/stream'
        print(f'[buscar_camara] Camara encontrada: {url}', flush=True)
        if solo_buscar:
            return 0
        ip_preferida = ip   # si se corta, probar primero la misma
        subprocess.run(['ros2', 'run', 'prometeo_camera_bridge', 'camera_bridge',
                        '--ros-args', '-p', f'stream_url:={url}'])
        print('[buscar_camara] camera_bridge se cerro. Vuelvo a buscar.', flush=True)
        time.sleep(2)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
