#!/usr/bin/env bash
# Conecta la PC al WiFi "prometeo" y levanta el agente micro-ROS dentro del
# contenedor distrobox "microros" (red host, asi que el ESP32 lo ve en la IP de la PC).
#
# Uso: firmware/scripts/agente.sh [puerto]
#   Variables opcionales: WIFI_SSID (default prometeo), WIFI_IFACE (default: primera wifi),
#                         CONTENEDOR (default microros), WIFI_PASS (si no hay perfil guardado)
#   Ctrl+C detiene el agente y vuelve al WiFi anterior.
# OJO: la red prometeo no tiene internet. Mientras corre el agente la PC queda sin
# internet si el WiFi es su unica conexion (no hace falta nada de internet aca).
set -euo pipefail

SSID=${WIFI_SSID:-prometeo}
CONTENEDOR=${CONTENEDOR:-microros}
PUERTO=${1:-8888}   # mismo default que CONFIG_MICRO_ROS_AGENT_PORT

IFACE=${WIFI_IFACE:-$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" {print $1; exit}')}
[ -n "$IFACE" ] || { echo "No encontre ninguna interfaz WiFi"; exit 1; }

# 1. WiFi: si ya estamos en la red no se toca nada
ACTUAL=$(nmcli -g GENERAL.CONNECTION device show "$IFACE" || true)
ANTERIOR=""
volver() {
  if [ -n "$ANTERIOR" ]; then
    echo; echo "WiFi: volviendo a $ANTERIOR..."
    nmcli connection up "$ANTERIOR" ifname "$IFACE" >/dev/null || true
  fi
}
if [ "$ACTUAL" = "$SSID" ]; then
  echo "WiFi: ya conectado a $SSID ($IFACE)"
else
  echo "WiFi: conectando a $SSID ($IFACE)... (esa red no tiene internet)"
  ANTERIOR=$ACTUAL
  if nmcli -g NAME connection show | grep -qx "$SSID"; then
    nmcli connection up "$SSID" ifname "$IFACE"
  elif [ -n "${WIFI_PASS:-}" ]; then
    nmcli device wifi connect "$SSID" password "$WIFI_PASS" ifname "$IFACE"
  else
    nmcli --ask device wifi connect "$SSID" ifname "$IFACE"
  fi
fi

IP=$(ip -4 -o addr show dev "$IFACE" | awk '{split($4,a,"/"); print a[1]; exit}')
[ -n "$IP" ] || { echo "Sin IPv4 en $IFACE"; exit 1; }
echo "PC: $IP  (el firmware debe tener CONFIG_MICRO_ROS_AGENT_IP=\"$IP\"; ver set_net_config.sh)"

# 2. Contenedor
command -v distrobox >/dev/null || { echo "Falta distrobox"; exit 1; }
podman container exists "$CONTENEDOR" 2>/dev/null || { echo "No existe el contenedor $CONTENEDOR"; exit 1; }

if PID=$(pgrep -f "micro_ros_agent udp4 --port $PUERTO" | head -1) && [ -n "$PID" ]; then
  echo "Ya hay un agente corriendo en el puerto $PUERTO (pid $PID)."
  echo "Para reiniciarlo: pkill -f 'micro_ros_agent udp4' y corre este script de nuevo."
  exit 0
fi

# 3. Agente (el contenedor lo trae compilado en /uros_ws)
trap volver EXIT
echo "Agente: udp4 puerto $PUERTO en $CONTENEDOR"
distrobox enter "$CONTENEDOR" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source /uros_ws/install/local_setup.bash
  ros2 run micro_ros_agent micro_ros_agent udp4 --port $PUERTO"
