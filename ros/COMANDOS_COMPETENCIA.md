# Comandos para la competencia

Todo funciona **sin internet**. Los comandos se corren desde la carpeta del repo
(`~/Documentos/Robotica/Prometeo`).

## 0. En cada terminal nueva

```bash
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
```

Si se cambió código, compilar antes:

```bash
cd ros && colcon build && cd ..
```

## 1. Conectar el robot real

**Agente de micro-ROS** (deja que la ESP32 hable con ROS). Dejar esta terminal abierta:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
```

**Ver qué firmware tiene cargado la ESP32:**

```bash
ros2 node list
```

| Aparece | Firmware | Qué usar |
|---|---|---|
| `/prometeo/body` | nuevo (`navegacion_bajo`), en m/s, **sin freno de emergencia** | los comandos de abajo tal cual |
| `/nodo_motores_esp32` | viejo, valores de −1 a 1, con freno | agregar `ns:=''` al panel y sacar `/prometeo` de los topics |

## 2. Panel de navegación (manual y automático)

Arranca el selector de modo, rosbridge y la página, todo junto:

```bash
ros2 launch my_robot_bringup panel.launch.py
```

Abrir en el navegador: **http://localhost:8082**

Desde otra compu de la misma red: `http://<ip-de-esta-compu>:8082`

### Teclas del panel

| Tecla | Qué hace |
|---|---|
| **M** | modo manual |
| **A** | modo automático |
| **ESPACIO** | **PARAR** (pánico): pasa a manual y frena, aunque ya estuviera en manual |
| Flechas o **W / S** | avanzar / retroceder (solo en manual) |
| Flechas o **Q / D** | girar a la izquierda / derecha (solo en manual) |

El robot arranca en **manual y quieto**: no se mueve solo hasta que alguien elige automático.
Si se corta la conexión, frena solo.

**Probarlo primero con las ruedas en el aire**, con la velocidad baja (0,15 m/s).

## 3. Plan B si el panel no anda

Cambiar de modo desde una terminal:

```bash
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: manual}"
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: auto}"
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: parar}"
```

Selector solo (si no se usa el panel):

```bash
ros2 run prometeo_navigation selector_modo --ros-args -r __ns:=/prometeo
```

Manejar con el teclado en una terminal (la ventana tiene que estar seleccionada):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/prometeo/cmd_vel_manual -p speed:=0.15 -p turn:=0.6
```

Teclas: `i` adelante, `,` atrás, `j` y `l` girar, `k` parar, `q` y `z` más o menos velocidad.

## 4. Revisar que llegan los datos

```bash
ros2 topic echo /prometeo/modo_actual             # modo actual
ros2 topic echo /prometeo/orden_enviada           # lo que se le manda a la ESP32
ros2 topic echo /prometeo/distancias --qos-reliability best_effort   # [izq, der, frente] en cm
```

## 5. Simulación (Gazebo)

```bash
ros2 launch my_robot_bringup gazebo.launch.py world_name:=laberinto
ros2 launch my_robot_bringup panel.launch.py ns:=''      # en otra terminal
```

En Gazebo los topics no llevan `/prometeo` adelante (`/modo`, `/cmd_vel_manual`, etc.).

## 6. Cerrar todo

`Ctrl+C` en cada terminal. Después revisar que no quedó nada corriendo:

```bash
pgrep -af "selector_modo|rosbridge|http.server"
```

Si aparece algo, cerrarlo por su número: `kill <número>`.

## 7. Pruebas del código (sin robot)

```bash
cd ros/src/prometeo_navigation && python3 -m pytest test -v
```
