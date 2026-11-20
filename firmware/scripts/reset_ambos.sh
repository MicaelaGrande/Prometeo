#!/usr/bin/env bash
# Reinicia la ESP32-CAM y el cuerpo (ESP32) desde la compu, pulsando RTS (= pin EN).
# DTR queda en bajo (GPIO0 alto) para que arranquen normal y no entren al bootloader.
#
# Uso: firmware/scripts/reset_ambos.sh [-d SEGUNDOS] [-s cam|cuerpo]
#   -d N   reinicia primero la camara y el cuerpo N segundos despues (por defecto 0:
#          los dos a la vez). Sirve para probar que el cuerpo se una al hotspot aunque
#          la camara tarde en arrancar.
#   -s X   reinicia solo una placa (cam o cuerpo)
#
# Los puertos se buscan por /dev/serial/by-id (no cambian de numero al reconectar).
# Si hay un monitor serie abierto en alguna placa, cerrarlo antes.
# Necesita pyserial: se usa el del entorno de ESP-IDF del contenedor jazzyDesktop.
set -euo pipefail

CAM_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
CUERPO_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_70b5e628b399ef11b52c167161ce3355-if00-port0
CONTENEDOR=jazzyDesktop
PY_IDF='~/.espressif/tools/python/v5.5.4/venv/bin/python'

DEMORA=0; SOLO=""
while getopts "d:s:h" op; do
  case $op in
    d) DEMORA=$OPTARG ;;
    s) SOLO=$OPTARG ;;
    *) sed -n '2,13p' "$0"; exit 0 ;;
  esac
done
case "$SOLO" in ""|cam|cuerpo) ;; *) echo "-s debe ser cam o cuerpo"; exit 1 ;; esac

PYCODE='
import sys, time, threading, serial
demora = float(sys.argv[1]); solo = sys.argv[2]
cam, cuerpo = sys.argv[3], sys.argv[4]

def reset(puerto):
    s = serial.Serial(puerto, 115200)
    s.dtr = False          # GPIO0 alto: arranque normal
    s.rts = True           # EN a masa
    time.sleep(0.1)
    s.rts = False          # suelta EN: la placa arranca
    s.close()

def tarea(nombre, puerto, espera):
    time.sleep(espera)
    reset(puerto)
    print("reiniciada:", nombre)

plan = []
if solo in ("", "cam"):    plan.append(("camara", cam, 0.0))
if solo in ("", "cuerpo"): plan.append(("cuerpo", cuerpo, demora if solo == "" else 0.0))
hilos = [threading.Thread(target=tarea, args=p) for p in plan]
[h.start() for h in hilos]; [h.join() for h in hilos]
'

if python3 -c "import serial" 2>/dev/null; then
  python3 -c "$PYCODE" "$DEMORA" "$SOLO" "$CAM_PORT" "$CUERPO_PORT"
else
  distrobox enter "$CONTENEDOR" -- bash -lc "$PY_IDF -c '$PYCODE' '$DEMORA' '$SOLO' '$CAM_PORT' '$CUERPO_PORT'"
fi
