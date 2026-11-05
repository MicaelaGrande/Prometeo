# Comandos para la competencia

Todo funciona **sin internet**. Los comandos se corren desde la carpeta del repo
(`~/Documentos/Robotica/Prometeo`).

## Paso a paso: hacer andar el panel con el robot real

Hacen falta **2 terminales** y el navegador. Seguir los pasos en orden y revisar cada
"✔ Anduvo si…" antes de pasar al siguiente.

### Paso 1. La compu en la red correcta

La ESP32 se conecta a una red WiFi y busca al agente en **una IP fija**, que se elige al
cargarle el firmware. La compu tiene que estar en esa red y **tener esa IP**.

Según la última configuración guardada en git (`micro_ros/sdkconfig`, que ya no está en el repo):

| Dato | Valor | Dónde se cambia |
|---|---|---|
| Red WiFi | `Ingenieria` | `CONFIG_ESP_WIFI_SSID` |
| IP del agente (la compu) | `172.16.0.208` | `CONFIG_MICRO_ROS_AGENT_IP` |
| Puerto | `8888` | `CONFIG_MICRO_ROS_AGENT_PORT` |

Ver la IP de la compu:

```bash
hostname -I
```

✔ Anduvo si la primera IP que aparece es la del firmware. **Si no coincide, la ESP32 no se va a
conectar nunca.** Hay dos salidas: fijarle esa IP a la compu en la configuración del WiFi, o
volver a cargar el firmware con la IP de la compu. Preguntarle a quien cargó el firmware
(Natasha) qué red e IP usó, porque puede no ser la de esta tabla.

**En la competencia la red va a ser otra**, así que esto hay que resolverlo allá, antes de la
ronda.

### Paso 2. Terminal 1: el agente de micro-ROS

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documentos/Robotica/micro_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Dejar esta terminal abierta. Prender el robot.

✔ Anduvo si al rato aparecen líneas con `session established` y `create_publisher` /
`create_subscriber`. Si no aparece nada, volver al paso 1.

### Paso 3. Terminal 2: ver que la ESP32 está en ROS

```bash
cd ~/Documentos/Robotica/Prometeo
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 node list
```

✔ Anduvo si aparece `/prometeo/body`. Si aparece `/nodo_motores_esp32`, es el firmware viejo:
en el paso 4 agregar `ns:=''` al final del comando.

### Paso 4. Terminal 2: arrancar el panel

En la misma terminal 2:

```bash
ros2 launch my_robot_bringup panel.launch.py
```

✔ Anduvo si aparece `Rosbridge WebSocket server started on port 9092` y
`Selector listo en modo manual`.

### Paso 5. Abrir el panel

En el navegador de la misma compu: **http://localhost:8082**

✔ Anduvo si arriba los tres indicadores están en verde: **rosbridge**, **selector** y
**sensores**, y el modo dice **Manual**.

| Indicador en rojo | Qué pasa | Qué hacer |
|---|---|---|
| rosbridge | la página no habla con ROS | revisar que el paso 4 siga corriendo |
| selector | el selector no arrancó | mirar los errores en la terminal 2 |
| sensores | no llegan los ultrasonidos | revisar el agente (paso 2) y `ros2 node list` (paso 3) |

### Paso 6. Probar con las ruedas en el aire

1. Levantar el robot para que las ruedas no toquen nada.
2. Hacer clic en la página (para que lea las teclas) y apretar la flecha **↑**.
3. ✔ Anduvo si las ruedas giran hacia adelante y en "Orden que recibe la ESP32" dice
   `avance 0.15 m/s`.
4. Probar **ESPACIO**: las ruedas tienen que frenar.
5. Recién ahí, apoyarlo en el piso.

**Ojo:** con este firmware no hay freno de emergencia. Manejar despacio.

### Ver el panel desde otra compu o el celular (opcional)

Tiene que estar en la misma red WiFi que la compu. En el navegador: `http://<ip-de-la-compu>:8082`
(la IP del paso 1). Desde el celular **no se puede manejar** (no tiene teclado), pero funcionan
los botones Manual, Automático y PARAR. En la competencia, preguntar a los jueces antes de
conectar un celular a la red del robot.

---

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

**Agente de micro-ROS** (deja que la ESP32 hable con ROS). Está instalado en
`~/Documentos/Robotica/micro_ws`. Dejar esta terminal abierta:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documentos/Robotica/micro_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
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

## 5. Opción 2: probar en Gazebo, sin el robot

Sirve para practicar el manejo y probar el panel sin el robot. **En la competencia no hace
falta.**

**Qué se puede probar en Gazebo:**

| Qué | ¿Se puede? |
|---|---|
| Manejar el robot con el panel (manual, automático, PARAR) | sí |
| Ultrasonidos (izquierda, derecha y frente) | sí |
| Posición y rastro en el mapa del panel (`/odom`) | sí (el robot real todavía no lo tiene) |
| Centrado en el pasillo y freno (nodo provisorio que imita a la ESP32) | sí |
| Cámara y visión (flechas, cubos, zonas) | sí, en el mundo `vision_testground` |
| **Mover el brazo y la garra** | **no**: en esta rama el brazo está dibujado pero no se mueve. Ignacio tiene una garra a medio andar en `feature/ros/sim-merge` ("todavía rebota demasiado"), sin mergear |

**Terminal 1: Gazebo.** Elegir un mundo:

```bash
cd ~/Documentos/Robotica/Prometeo
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 launch my_robot_bringup gazebo.launch.py world_name:=laberinto           # el laberinto
ros2 launch my_robot_bringup gazebo.launch.py world_name:=vision_testground   # con flechas y cubos
```

✔ Anduvo si se abren Gazebo y RViz con el robot.

**Terminal 2: el panel**, sin namespace (en Gazebo los topics no llevan `/prometeo`):

```bash
cd ~/Documentos/Robotica/Prometeo
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
ros2 launch my_robot_bringup panel.launch.py ns:=''
```

Abrir **http://localhost:8082** y manejar con las flechas, igual que con el robot real.
✔ Anduvo si los tres indicadores están en verde y el robot se mueve en Gazebo.

**Ojo:** el launch de Gazebo también arranca el control por joystick, que manda directo a
`/cmd_vel` sin pasar por el selector. Si no hay un joystick conectado no molesta; si hay uno,
puede pisar las órdenes del panel.

**No correr Gazebo con el robot real conectado.** Con el firmware viejo los dos usan los mismos
topics y se mezclan las órdenes; con el nuevo no se mezclan, pero es fácil abrir el panel
equivocado y manejar el que no es.

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
