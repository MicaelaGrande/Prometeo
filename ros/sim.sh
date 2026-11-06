#!/usr/bin/env bash
# Levanta la simulacion de Prometeo en Gazebo, en cualquier compu con ROS 2 Jazzy.
#
# Uso (desde cualquier carpeta):
#   ./sim.sh                      mundo vision_testground
#   ./sim.sh laberinto            otro mundo (ver la lista con ./sim.sh --mundos)
#   ./sim.sh laberinto --compilar compila antes de lanzar
#   ./sim.sh --sin-gpu            para compus sin placa de video o maquinas virtuales
#   Lo que venga con := pasa directo al launch, por ejemplo joystick:=true
#
# Que arregla:
#   - La terminal de VS Code (instalado como snap) mete variables que rompen Gazebo y
#     RViz con "symbol lookup error ... libpthread". Aca se borran.
#   - Si no esta compilado, compila solo.
#   - Si quedo un Gazebo colgado de antes, lo cierra.

set -e

CARPETA_ROS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MUNDOS="$CARPETA_ROS/src/gazebo_worlds/worlds"
MUNDO="vision_testground"
COMPILAR=0
EXTRA=()

for a in "$@"; do
  case "$a" in
    --compilar) COMPILAR=1 ;;
    --sin-gpu) export LIBGL_ALWAYS_SOFTWARE=1 ;;
    --mundos) ls "$MUNDOS"; exit 0 ;;
    *:=*) EXTRA+=("$a") ;;
    *) MUNDO="$a" ;;
  esac
done

# 1. Borrar las variables del snap de VS Code (la causa del "symbol lookup error")
for v in GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR GTK_IM_MODULE_FILE LOCPATH \
         GSETTINGS_SCHEMA_DIR GDK_BACKEND; do
  unset "$v"
done
export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share:/usr/share}"
export XDG_CONFIG_DIRS="${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-/etc/xdg}"

# 2. ROS 2 Jazzy y Gazebo instalados
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "No esta ROS 2 Jazzy (/opt/ros/jazzy). Instalarlo primero."; exit 1
fi
set +u; source /opt/ros/jazzy/setup.bash
if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "Falta Gazebo para ROS. Instalar con:"
  echo "  sudo apt install ros-jazzy-ros-gz"
  exit 1
fi

# 3. El mundo existe
if [ ! -d "$MUNDOS/$MUNDO" ]; then
  echo "No existe el mundo '$MUNDO'. Hay estos:"; ls "$MUNDOS"; exit 1
fi

# 4. Compilar si hace falta
cd "$CARPETA_ROS"
if [ "$COMPILAR" = 1 ] || [ ! -f install/setup.bash ]; then
  echo ">> Compilando..."
  colcon build --symlink-install
fi
source install/setup.bash

# 5. Cerrar un Gazebo que haya quedado colgado
for pid in $(pgrep -f "gz sim" || true); do
  [ "$pid" != "$$" ] && kill "$pid" 2>/dev/null || true
done

echo ">> Mundo: $MUNDO"
exec ros2 launch my_robot_bringup gazebo.launch.py world_name:="$MUNDO" joystick:=false "${EXTRA[@]}"
