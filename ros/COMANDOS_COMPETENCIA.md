# Comandos para la competencia

Todo funciona **sin internet**. Los comandos se corren desde la carpeta del repo
(`~/Documentos/Robotica/Prometeo`).

## Paso a paso: hacer andar el panel con el robot real

Hacen falta **2 terminales** y el navegador. Seguir los pasos en orden y revisar cada
"✔ Anduvo si…" antes de pasar al siguiente.

### Paso 0. Una sola vez, después de traer cambios del código

```bash
cd ~/Documentos/Robotica/Prometeo/ros
source /opt/ros/jazzy/setup.bash
colcon build
```

Si ya había un panel abierto, cerrarlo con `Ctrl+C` y volver a arrancarlo (paso 4): el que
estaba corriendo sigue con el código viejo.

### Paso 1. Prender el robot y la cámara, con la compu en la misma red WiFi

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

La **cámara** no necesita nada de esto: el panel la busca sola en la red (paso 4).

### Paso 2. Terminal 1: el agente de micro-ROS

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documentos/Robotica/micro_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Dejar esta terminal abierta.

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

### Paso 4. Terminal 2: arrancar el panel (con la cámara)

En la misma terminal 2:

```bash
ros2 launch my_robot_bringup panel.launch.py
```

Este único comando arranca el selector de modo, rosbridge, la página, el joystick y **la
cámara**: busca la ESP32-CAM en la red (tarda unos 6 segundos), la reconoce por su MAC
(`d4:e9:f4:8a:b6:68`, así no se conecta a la cámara de otro equipo) y arranca `camera_bridge`
con su IP. Si la cámara se prende después, la sigue buscando cada 5 segundos.

✔ Anduvo si aparecen estas líneas:

- `Rosbridge WebSocket server started on port 9092`
- `Selector listo en modo manual`
- `[buscar_camara] Camara encontrada: http://172.16.0.117/stream` (la IP puede cambiar)
- `[camera_bridge]: Stream conectado`

Si no hay un joystick enchufado, el nodo del joystick avisa que no encuentra ninguno: es normal.

Opciones del comando, si hacen falta:

| Agregar | Para qué |
|---|---|
| `asistencia:=true` | **ayudas de la Cat. B**: freno por obstáculo y centrado en el pasillo, en la compu (ver abajo) |
| `camara_ip:=172.16.0.117` | probar primero esa IP (más rápido) |
| `camara:=false` | no usar la cámara (ni visión) |
| `vision:=false` | cámara sin visión: imagen cruda, sin recuadros |
| `joystick:=false` | no usar el joystick |
| `ns:=''` | firmware viejo o Gazebo (topics sin `/prometeo`) |

Para ver la IP de la cámara sin arrancar nada: `ros2 run my_robot_bringup buscar_camara.py --solo-buscar`

### Ayudas de la Categoría B (`asistencia:=true`)

Mientras ustedes manejan, el robot ayuda solo:

- **Freno por obstáculo:** si mandan "adelante" y el ultrasonido de adelante ve una pared
  cerca, **no avanza**. Retroceder y girar siempre pasan. La distancia depende de la velocidad:
  a 0,15 m/s frena a unos 9 cm.
- **Corrección de trayectoria:** si van derecho y se tuercen hacia una pared, **corrige solo**
  hacia el medio del pasillo. Si un costado mide más de 50 cm (una salida), no corrige, para no
  tirarse al hueco. Si ustedes están girando, no se mete.

Es la misma cuenta que se usa en Gazebo (`provisorio_bajo_nivel_sim`), probada, con la
configuración del robot real en `config/asistencia_robot_real.yaml`. No toca el firmware.

**Está apagada por defecto hasta probarla en el robot.** Para usarla:

```bash
ros2 launch my_robot_bringup panel.launch.py asistencia:=true
```

Probarla con las ruedas en el aire: avanzar despacio y poner la mano a unos 10 cm del sensor
de adelante. ✔ Anduvo si las ruedas frenan, en el panel aparece **"Freno por obstáculo: ACTIVO
(compu)"**, y retroceder sigue andando.

- **Apagar el freno** (por ejemplo, para acercarse al cubo): pestaña **Manual**, "Freno por
  obstáculo: Apagar". Después, "Prender".
- **Ojo:** si el ultrasonido de adelante deja de medir (sin dato), el freno no deja avanzar. Si
  pasa seguido, apagar el freno o arrancar el panel sin `asistencia:=true`.
- Si la ayuda trae problemas, arrancar el panel sin `asistencia:=true` y queda todo como antes.

### Paso 5. Abrir el panel

En el navegador de la misma compu: **http://localhost:8082**

✔ Anduvo si arriba están en verde **rosbridge**, **selector**, **ESP32** y **ultrasonidos**, y
el modo dice **Manual**, y a la derecha se ve **la cámara con los recuadros de lo que detecta
visión**. El panel arranca también el nodo de visión y le carga a la cámara su calibración
(`config/calibracion_camara_ov3660.npz`, copia de `firmware/esp32_cam/calibracion`): sin
calibración, visión no detecta nada. Si visión no anda, el panel muestra la imagen cruda.

**La ESP32-CAM acepta una sola conexión a la vez.** Si la imagen no llega, revisar que no haya
otro programa mirándola (otro panel, `camera_bridge` suelto o el navegador en `http://<ip>/stream`).

| Indicador en rojo | Qué pasa | Qué hacer |
|---|---|---|
| rosbridge | la página no habla con ROS | revisar que el paso 4 siga corriendo |
| selector | el selector no arrancó | mirar los errores en la terminal 2 |
| ESP32 / ultrasonidos | no llega nada del robot | revisar el agente (paso 2) y `ros2 node list` (paso 3) |
| cámara | no llega imagen | mirar en la terminal 2 qué dice `[buscar_camara]`; revisar que la cámara esté prendida y en la misma red |

En la pestaña **Monitoreo** hay una tabla "Qué está llegando" con cada dato y hace cuánto llegó.

### Paso 6. Probar con las ruedas en el aire

1. Levantar el robot para que las ruedas no toquen nada.
2. Hacer clic en la página (para que lea las teclas) y apretar la flecha **↑**.
3. ✔ Anduvo si las ruedas giran hacia adelante y en "Orden que recibe la ESP32" dice
   `avance 0.15 m/s`.
4. Probar **ESPACIO**: las ruedas tienen que frenar y aparece **FRENO PUESTO** en rojo.
5. Apretar **Soltar freno** y volver a probar la flecha.
6. Si hay joystick: mantener **LB** y mover el stick izquierdo. El cartel "Maneja" tiene que
   decir **JOYSTICK**. Probar **B** (freno) y **Y** mantenido 1 segundo (soltar).
7. Recién ahí, apoyarlo en el piso.

**Ojo:** con este firmware no funciona el freno por obstáculo (frenar solo antes de chocar). Manejar despacio.

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
| `/prometeo/body` | nuevo (`navegacion_bajo`), en m/s, **sin freno por obstáculo** (pendiente) | los comandos de abajo tal cual |
| `/nodo_motores_esp32` | viejo, valores de −1 a 1, con freno | agregar `ns:=''` al panel y sacar `/prometeo` de los topics |

## 2. Panel de navegación (manual y automático)

Arranca el selector de modo, rosbridge y la página, todo junto:

```bash
ros2 launch my_robot_bringup panel.launch.py
```

Abrir en el navegador: **http://localhost:8082**

Desde otra compu de la misma red: `http://<ip-de-esta-compu>:8082`

### Qué tiene el panel

Arriba, siempre visibles: el **modo**, el cartel **"Maneja: COMPU / JOYSTICK / AUTOMÁTICO"**, el
estado del freno por obstáculo y los botones **Manual**, **Automático**, **PARAR** y **Soltar
freno**. A la derecha, siempre visibles: la **cámara** (con lo que detecta visión dibujado) y los
**ultrasonidos** con su valor y el mínimo de los últimos 10 segundos.

| Pestaña | Qué tiene |
|---|---|
| Monitoreo | mapa de la libreta, posición, últimos avisos y la tabla "Qué está llegando" |
| Manual | manejo con teclado, guía del joystick, **Capturar** y **Abandonar**, prender/apagar el freno por obstáculo, lugar para la garra |
| Automático | botones de rescate (**−20 pts cada uno en Cat. A**): capturar a mano y abandonar este cubo |
| Motores | estado de la ESP32, orden enviada, velocidad estimada, encoders de las ruedas |
| Visión | lo que detecta la cámara: flechas, cubos y zonas, con color, dirección y confianza |
| Brazo | ángulo de cada servo (cámara, base, hombro, codo, garra) |
| Registro | todo lo que pasó, con la hora. **Descargarlo al terminar cada ronda** (sirve para el jurado) |

### PARAR es un freno de emergencia

- **PARAR** (o ESPACIO, o **B** en el joystick) frena y **queda puesto**: nadie mueve el robot, ni
  el automático, ni la compu, ni el joystick. **No cambia el modo.**
- Para seguir hay que soltarlo a propósito: **Soltar freno** (o mantener **Y** 1 segundo). Si está
  en automático, el panel pregunta antes, porque el robot arranca solo.

### Teclas del panel

| Tecla | Qué hace |
|---|---|
| **M** | modo manual |
| **A** | modo automático |
| **ESPACIO** | **PARAR**: freno puesto |
| Flechas o **W / S** | avanzar / retroceder (solo en manual) |
| Flechas o **Q / D** | girar a la izquierda / derecha (solo en manual) |

### Joystick Xbox

| Botón | Qué hace |
|---|---|
| **LB** mantener + stick izquierdo | manejar (si se suelta LB, frena) |
| **RB** además | más rápido |
| **B** | PARAR (freno puesto) |
| **Y** mantener 1 segundo | soltar el freno |
| **View** (Back) | modo manual |
| **Menu** (Start) | modo automático |

**Cómo se conecta:** enchufarlo a la compu **con el cable USB** (lo más seguro: Linux lo reconoce
solo). Por Bluetooth también puede andar, pero algunos controles Xbox necesitan un programa extra
en Linux y pueden numerar distinto los botones. Se puede enchufar antes o después de arrancar el
panel. Para ver si la compu lo reconoce:

```bash
ros2 run joy joy_enumerate_devices
```

✔ Anduvo si en el panel el indicador **joystick** se pone en verde (sin tocar nada, porque el
joystick manda su estado 20 veces por segundo).

Si un botón no responde: `ros2 topic echo /prometeo/joy`, apretarlo, ver qué número cambia y
ponerlo en `ros/src/my_robot_bringup/config/joystick_xbox.yaml` (después compilar).

El robot arranca en **manual y quieto**: no se mueve solo hasta que alguien elige automático.
Si se corta la conexión, frena solo.

**Probarlo primero con las ruedas en el aire**, con la velocidad baja (0,15 m/s).

### Para los otros nodos del equipo (garra, Decisión, Navegación)

**El selector es el único que decide el modo.** Los demás lo escuchan:

| Topic | Tipo | Qué dice |
|---|---|---|
| `/prometeo/modo_actual` | `std_msgs/String` | `auto` o `manual` |
| `/prometeo/freno_operador` | `std_msgs/Bool` | `true` si el freno PARAR está puesto: no mover nada |
| `/prometeo/pedido_operador` | `std_msgs/String` | `capturar` o `abandonar`, desde los botones del panel |

Los dos primeros se repiten una vez por segundo y se guardan para el que se conecta tarde.
Para escucharlos en Python:

```python
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
self.modo = 'manual'        # hasta que llegue el primer aviso, suponer lo seguro
self.freno = True
self.create_subscription(String, '/prometeo/modo_actual', self.cambio_de_modo, guardado)
self.create_subscription(Bool, '/prometeo/freno_operador', self.cambio_de_freno, guardado)

def cambio_de_modo(self, msg):
    self.modo = msg.data    # 'auto' o 'manual'

def cambio_de_freno(self, msg):
    self.freno = msg.data   # True: no mover nada
```

## 3. Plan B si el panel no anda

Cambiar de modo desde una terminal:

```bash
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: manual}"
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: auto}"
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: parar}"    # freno puesto
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: soltar}"   # soltar el freno
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
ros2 topic echo /prometeo/estado_selector         # modo, freno y quién maneja
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
ros2 launch my_robot_bringup gazebo.launch.py world_name:=laberinto joystick:=false           # el laberinto
ros2 launch my_robot_bringup gazebo.launch.py world_name:=vision_testground joystick:=false   # con flechas y cubos
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

`joystick:=false` apaga el joystick propio de Gazebo, que manda directo a `/cmd_vel` sin pasar por
el selector. Con el panel, el joystick lo arranca el panel y pasa por el selector.

**No correr Gazebo con el robot real conectado.** Con el firmware viejo los dos usan los mismos
topics y se mezclan las órdenes; con el nuevo no se mezclan, pero es fácil abrir el panel
equivocado y manejar el que no es.

## 6. Cerrar todo

`Ctrl+C` en cada terminal. Después revisar que no quedó nada corriendo:

```bash
pgrep -af "selector_modo|rosbridge|http.server|camara_panel|joystick_modo|game_controller|teleop"
```

Si aparece algo, cerrarlo por su número: `kill <número>`.

## 7. Pruebas del código (sin robot)

```bash
cd ros/src/prometeo_navigation && python3 -m pytest test -v
```
