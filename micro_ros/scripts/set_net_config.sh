#!/usr/bin/env bash
# Carga SSID/password del WiFi actual y la IP local de la PC (agente micro-ROS)
# en sdkconfig. Uso: scripts/set_net_config.sh [interfaz_para_IP]
#   Sin argumento: IP de la ruta por defecto. sdkconfig esta ignorado por git.
set -euo pipefail
cd "$(dirname "$0")/.."
CFG=sdkconfig
[ -f "$CFG" ] || { echo "No existe $CFG (corre idf.py menuconfig/build una vez)"; exit 1; }

# WiFi activo
read -r CON SSID < <(nmcli -t -f ACTIVE,NAME,TYPE con show --active | awk -F: '$3 ~ /wireless/ {print $2, $2; exit}') || true
[ -n "${CON:-}" ] || { echo "No hay WiFi conectado"; exit 1; }
SSID=$(nmcli -g 802-11-wireless.ssid con show "$CON")
PASS=$(nmcli -s -g 802-11-wireless-security.psk con show "$CON" || true)
[ -n "$PASS" ] || echo "Aviso: sin password (red abierta o sin permiso para leerla)"

# IP de la PC
IFACE=${1:-$(ip route show default | awk '{print $5; exit}')}
IP=$(ip -4 -o addr show dev "$IFACE" | awk '{split($4,a,"/"); print a[1]; exit}')
[ -n "$IP" ] || { echo "Sin IPv4 en $IFACE"; exit 1; }

esc() { printf '%s' "$1" | sed -e 's/[\\"]/\\&/g' -e 's/[&|]/\\&/g'; }
sed -i \
  -e "s|^CONFIG_ESP_WIFI_SSID=.*|CONFIG_ESP_WIFI_SSID=\"$(esc "$SSID")\"|" \
  -e "s|^CONFIG_ESP_WIFI_PASSWORD=.*|CONFIG_ESP_WIFI_PASSWORD=\"$(esc "$PASS")\"|" \
  -e "s|^CONFIG_MICRO_ROS_AGENT_IP=.*|CONFIG_MICRO_ROS_AGENT_IP=\"$IP\"|" \
  "$CFG"
echo "SSID=$SSID  agente IP=$IP ($IFACE)  -> $CFG"
echo "Recompila: idf.py build flash"
