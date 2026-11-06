#!/usr/bin/env bash
# Levanta la simulacion de Prometeo en Gazebo, en cualquier compu con ROS 2 Jazzy,
# con todo lo necesario para probar el detector de lugares (T2).
#
# Uso (desde cualquier carpeta):
#   ./sim.sh                      mundo vision_testground + todas las ventanas
#   ./sim.sh laberinto            otro mundo (ver la lista con ./sim.sh --mundos)
#   ./sim.sh --compilar           compila antes de lanzar
#   ./sim.sh --solo-gazebo        solo la simulacion, sin las otras ventanas
#   ./sim.sh --sin-gpu            para compus sin placa de video o maquinas virtuales
#   Lo que venga con := pasa directo al launch, por ejemplo joystick:=true
#
# Ventanas que abre (esta terminal queda con Gazebo):
#   1. Detector   el nodo detector_lugares: etapa y cada LUGAR con F0 y F1
#   2. Avisos     lo que sale por /navegacion/lugar (lo que recibe Decision)
#   3. Teclado    manejar el robot (hacer clic en esa ventana antes de apretar teclas)
#   4. Reiniciar  Enter = "ya sali del lugar" (reemplaza a /mover, que todavia no existe).
#                 Apretarlo cuando el robot termino de girar y esta derecho en el pasillo.
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
VENTANAS=1
EXTRA=()

for a in "$@"; do
  case "$a" in
    --compilar) COMPILAR=1 ;;
    --solo-gazebo) VENTANAS=0 ;;
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
source /opt/ros/jazzy/setup.bash
if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "Falta Gazebo para ROS. Instalar con:"
  echo "  sudo apt install ros-jazzy-ros-gz"
  exit 1
fi
if [ "$VENTANAS" = 1 ] && ! ros2 pkg prefix teleop_twist_keyboard >/dev/null 2>&1; then
  echo "Falta el control por teclado. Instalar con:"
  echo "  sudo apt install ros-jazzy-teleop-twist-keyboard"
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
  colcon build   # (mezclar con --symlink-install rompe)
fi
source install/setup.bash

# 5. Cerrar un Gazebo que haya quedado colgado
for pid in $(pgrep -f "gz sim" || true); do
  [ "$pid" != "$$" ] && kill "$pid" 2>/dev/null || true
done

# 6. Las otras ventanas. Cada una prepara ROS por su cuenta (algunas terminales no
#    heredan el entorno de esta) y queda abierta aunque el programa se cierre.
PREPARAR="cd '$CARPETA_ROS'; source /opt/ros/jazzy/setup.bash; source install/setup.bash;"
if [ -n "$ROS_DOMAIN_ID" ]; then PREPARAR="$PREPARAR export ROS_DOMAIN_ID=$ROS_DOMAIN_ID;"; fi

abrir() {   # abrir "titulo" "comando"
  local todo="$PREPARAR echo '== $1 =='; $2; echo; echo '(termino; Ctrl+D cierra)'; exec bash"
  if command -v gnome-terminal >/dev/null; then
    gnome-terminal --title="$1" -- bash -c "$todo"
  elif command -v konsole >/dev/null; then
    konsole -p tabtitle="$1" -e bash -c "$todo" &
  elif command -v xfce4-terminal >/dev/null; then
    xfce4-terminal --title="$1" -x bash -c "$todo" &
  elif command -v xterm >/dev/null; then
    xterm -T "$1" -e bash -c "$todo" &
  else
    return 1
  fi
}

DETECTOR="ros2 run prometeo_navigation detector_lugares --ros-args -p use_sim_time:=true"
AVISOS="ros2 topic echo /navegacion/lugar"
TECLADO="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.6"
REINICIAR="while read -p 'Enter = ya sali del lugar (reiniciar el detector) ' _; do \
ros2 topic pub --times 1 -w 1 /navegacion/reiniciar_detector std_msgs/msg/Empty >/dev/null \
&& echo '   listo, el detector vuelve a EN_PASILLO'; done"

if [ "$VENTANAS" = 1 ]; then
  (
    sleep 10   # que Gazebo arranque primero
    if abrir "1 Detector" "$DETECTOR"; then
      abrir "2 Avisos" "$AVISOS"
      abrir "3 Teclado" "$TECLADO"
      abrir "4 Reiniciar" "$REINICIAR"
    else
      echo
      echo "!! No encontre como abrir ventanas. Abri 4 terminales y en cada una:"
      echo "   cd $CARPETA_ROS && source install/setup.bash"
      echo "   1) $DETECTOR"
      echo "   2) $AVISOS"
      echo "   3) $TECLADO"
      echo "   4) ros2 topic pub --once /navegacion/reiniciar_detector std_msgs/msg/Empty"
      echo
    fi
  ) &
fi

echo ">> Mundo: $MUNDO"
exec ros2 launch my_robot_bringup gazebo.launch.py world_name:="$MUNDO" joystick:=false "${EXTRA[@]}"
