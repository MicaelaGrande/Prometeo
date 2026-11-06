# Arquitectura del código de Prometeo

Robot para CAID 2026. ROS 2 Jazzy en la compu + ESP32 con micro-ROS.
Cómo se arranca todo: sección 0.

---

## 0. Cómo arrancar

En **cada terminal** (después de traer cambios, una vez: `cd ros && colcon build`):
```bash
cd ~/Documentos/Robotica/Prometeo
source /opt/ros/jazzy/setup.bash
source ros/install/setup.bash
export ROS_DOMAIN_ID=33          # el mismo que el firmware
```

| Terminal | Comando | Para qué |
|---|---|---|
| 1 | `source ~/Documentos/Robotica/micro_ws/install/local_setup.bash` y `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888` | Puente con la ESP32 |
| 2 | `ros2 launch my_robot_bringup panel.launch.py` | Selector, panel, joystick, cámara y visión |
| 3 | `ros2 launch my_robot_bringup autonomia.launch.py` | Detector, mover, mapeo y Decisión (automático) |

Panel en el navegador: **http://localhost:8082**. El robot no se mueve solo hasta poner el panel en automático.

Plan B sin panel:
```bash
ros2 topic pub --once /prometeo/modo std_msgs/msg/String "{data: manual}"   # auto | parar | soltar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/prometeo/cmd_vel_manual -p speed:=0.15 -p turn:=0.6
```

Revisar datos: `ros2 node list`, `ros2 topic echo /prometeo/estado_selector`,
`ros2 topic echo /prometeo/distancias --qos-reliability best_effort`.

Gazebo (sin robot): `cd ros && ./sim.sh` (o `./sim.sh laberinto`, `./sim.sh --mundos`).
Con el panel: `panel.launch.py ns:=''` y `autonomia.launch.py ns:=''`.

Firmware: `cd micro_ros && idf.py build && idf.py -p /dev/ttyUSB0 flash monitor`.

---

## 1. Vista general: tres pisos y un cuerpo

```
┌──────────────────────── COMPU (ROS 2 Jazzy) ────────────────────────┐
│                                                                     │
│  PISO 3  QUÉ HACER        nodo_decision                             │
│                             ▲ /navegacion/lugar   ▲ /vision/detections
│                             │ /mover  /volver     │                 │
│  PISO 2  CÓMO HACERLO     detector_lugares · mover · mapeo · visión │
│                             │ cmd_vel_auto                          │
│  PISO 1  QUIÉN MANDA      selector_modo ◄── panel web / joystick    │
│                             │ cmd_vel                               │
└─────────────────────────────┼───────────────────────────────────────┘
                     WiFi · micro-ROS agent (~/Documentos/Robotica/micro_ws)
┌─────────────────────────────┼───────────────────────────────────────┐
│  ESP32 (micro_ros/)         ▼                                       │
│  cmd_vel → freno → centrado → motores (PID por rueda, térmica)      │
│  3 ultrasonidos → /distancias      brazo + garra + cámara (servos)  │
└─────────────────────────────────────────────────────────────────────┘
```

- Los pisos de arriba **deciden**, los de abajo **ejecutan**.
- La seguridad (freno) vive en la ESP32: no depende del WiFi.
- **Automático y manual usan la misma arquitectura**: el selector solo cambia quién da la orden
  (la navegación o el operador). En manual el robot igual frena, se centra y ve flechas y cubos.

---

## 2. Estructura del repositorio

```
Prometeo/
├── micro_ros/                 Firmware de la ESP32 (ESP-IDF 5.5 + micro-ROS)
│   ├── main/                  main.c, sensores, seguridad (freno), centrado
│   └── drivers/               motores, hcsr04 (ultrasonidos), encoders, mg90s (servos), Kconfig
├── ros/
│   ├── sim.sh                 Levanta Gazebo con todo para probar
│   └── src/                   Paquetes de ROS 2 (tabla abajo)
└── Documentacion/             Esta documentación (ARQUITECTURA.md), reglamento, pinout, datasheets
```

| Paquete | Qué tiene |
|---|---|
| `prometeo_navigation` | Selector de modo, joystick, detector de lugares, mover, mapeo |
| `prometeo_decision` | La lógica de qué hacer (Natasha) |
| `prometeo_interfaces` | Mensajes propios: `Lugar.msg`, `Mover.action`, `Volver.action` |
| `huggin_vision` / `huggin_vision_msgs` | Visión: flechas, cubos y zonas por color |
| `prometeo_camera_bridge` | Pasa el video de la ESP32-CAM (MJPEG por HTTP) a ROS |
| `prometeo_sensores` | `distancias_a_range`: `/distancias` → `/ultrasonido/izq, der, frente` |
| `brazo_kinematics` / `brazo_interfaces` | Cinemática inversa del brazo |
| `my_robot_bringup` | Launches, configuración, panel web |
| `robot_description`, `gazebo_worlds` | Modelo del robot y mundos para Gazebo |
| `provisorio_bajo_nivel_sim` | PROVISORIO: imita en Gazebo el centrado y el freno de la ESP32 |

---

## 3. El cuerpo: firmware de la ESP32 (`micro_ros/`)

### Camino de una orden de movimiento (`main/main.c`, `cmd_vel_callback`)
1. Llega `cmd_vel` (avance m/s, giro rad/s).
2. **Freno** (`main/seguridad.c`, `seguridad_filtrar`): si el frente está más cerca de lo que el
   robot necesita para frenar a esa velocidad, no deja avanzar. Retroceder y girar sí.
3. **Centrado** (`main/centrado.c`): si avanza derecho y hay pared a los dos lados, suma un giro
   chico (PID) para ir por el medio del pasillo.
4. **Motores** (`drivers/motores/motores.c`): reparte en las 4 ruedas, mantiene la velocidad de
   cada una con su encoder (PID), refuerza el giro y cuida los puentes H.
5. Si pasan 300 ms sin órdenes, frena solo (failsafe).

### Ultrasonidos (`main/sensores.c`, `drivers/hcsr04/`)
- 3 HC-SR04 que comparten el cable TRIG: un pulso dispara los tres juntos, cada 60–85 ms.
- Mediana de las últimas 3 lecturas. `400` = no ve pared, `-1` = sin dato (cableado).
- El freno usa la lectura cruda (reacciona un ciclo antes); todo lo demás, la de mediana.

### Protecciones de motores (menuconfig → "Prometeo - Seguridad motores")
| Protección | Estado | Qué hace |
|---|---|---|
| Freno por obstáculo | Prendido | Corta el avance con pared adelante |
| Térmica (esfuerzo) | Prendida, 150 % | Estima el calor de cada puente H; si se pasa, corta ese motor |
| Atasco (stall) | **Apagada** | Necesita encoders confiables |
| Liberar | — | Una falla queda trabada hasta `/motores/liberar` (botón del panel) |

### Topics de la ESP32 (con namespace `/prometeo` en el robot real)
| Topic | Sentido | Qué lleva |
|---|---|---|
| `distancias` | ESP32 → compu | `[izq, der, frente]` en cm |
| `estado_motores`, `velocidad`, `velocidad_ruedas` | ESP32 → compu | Estado y telemetría |
| `freno_emergencia` | ESP32 → compu | `true` si el freno está actuando |
| `servos/estado` | ESP32 → compu | Ángulos de los servos |
| `cmd_vel` | compu → ESP32 | Avance y giro |
| `seguridad_habilitada` | compu → ESP32 | Prender o apagar el freno (Decisión lo apaga para arrimarse al cubo) |
| `motores/liberar` | compu → ESP32 | Soltar una falla trabada |
| `brazo/cmd` | compu → ESP32 | `[base, hombro, codo, pinza]` en grados |
| `camara/cmd` | compu → ESP32 | Ángulo del servo de la cámara |

> **Dominio ROS 33.** El firmware usa `ROS_DOMAIN_ID` 33 (`main.c`, `rcl_init_options_set_domain_id`).
> Toda terminal de la compu que hable con el robot necesita `export ROS_DOMAIN_ID=33`.

> **Límite de micro-ROS.** `app-colcon.meta` permite 6 publicadores y 6 suscripciones.
> Hoy hay 6/6 publicadores: agregar uno más hace que el robot no conecte.

---

## 4. Piso 1: quién manda (`panel.launch.py`)

### `selector_modo` (`prometeo_navigation/selector_modo.py`)
```
cmd_vel_auto   (navegación) ─┐
cmd_vel_manual (panel)      ─┼─► selector_modo ──► cmd_vel ──► ESP32
cmd_vel_joy    (joystick)   ─┘        ▲
                                     modo: "auto" | "manual" | "parar" | "soltar"
```
- Deja pasar una sola fuente según el modo. "parar" es un freno que solo suelta una persona.
- Repite la orden varias veces por segundo (si no, la ESP32 frena por failsafe).
- Publica `modo_actual` y `freno_operador` (guardados para quien se conecta tarde),
  `estado_selector` (para el panel) y `orden_enviada` (copia de lo que sale).

### Panel web (`my_robot_bringup/web/panel.html`)
Se sirve por HTTP y habla con ROS por **rosbridge** (WebSocket). Pestañas: modo y manejo,
sensores, mapa de la libreta, visión, brazo, registro. Botones clave: **Abandonar color**,
**Liberar motores**.

### Otros nodos de `panel.launch.py`
- `joystick_modo` + `joy` + `teleop_twist_joy`: manejo con joystick.
- `vision_node` (`huggin_vision`) y `camara_panel.py`.
- Con `asistencia:=true` (apagado por defecto): `distancias_a_range` y `centrado_simulado`,
  un centrado y freno hechos en la compu. Ahora que la ESP32 centra sola, no conviene prender los dos.

---

## 5. Visión (`huggin_vision`)

Nodo `vision_node` (`object_recognition.py`). Por cada imagen:

1. **Colores** (`segmentation.py`): pasa a HSV y separa rojo, verde y azul. El umbral de
   saturación se recalcula en cada imagen según el fondo, para adaptarse a la luz.
2. **Cada mancha** (`blobs.py`, `detect_blobs`) se prueba en orden: flecha → zona → cubo.
3. **Flecha** (`arrow.py`): necesita dos muescas (donde el cuerpo se une a la punta). Se
   compara con las 3 flechas reales (close, normal, far) en las dos direcciones; la que más se
   superpone da la dirección y el largo. Si sabe la distancia a la pared
   (`/ultrasonido/frente`), el largo sale en centímetros reales.
4. **Cubo** (`cube_fit.py`): mancha casi cuadrada y llena. Se ajusta un cubo virtual de 2 cm
   para saber a qué distancia y ángulo está.
5. Publica **`/vision/detections`**: tipo, color, dirección, largo, confianza y posición en metros
   respecto del robot (`base_footprint`: x adelante, y izquierda).

---

## 6. Piso 2: cómo hacerlo (`autonomia.launch.py`)

### `detector_lugares` (`detector_lugares.py` + `lugares.py`)
Lee `/distancias` y avisa en **`/navegacion/lugar`** (`Lugar.msg`) cuando hay una curva,
bifurcación o callejón, con sus salidas.

Etapas: `MIRANDO_ALREDEDOR` (al arrancar o volver a auto) → `EN_PASILLO` → `MIDIENDO_LUGAR` → `AVISADO`.
- Un costado que se abre (salto de distancia) = salida.
- Esquina: el frente llega a `F0 × fraccion_esquina` (0.65).
- Puerta: el costado se vuelve a cerrar antes de la esquina.
- Pide 3 lecturas seguidas para creer algo. En manual no avisa (salvo `modo_testeo:=true`).

### `mover` (`mover.py` + `maniobras.py`)
Servidor de la acción **`/mover`** (`Mover.action`): `ir_a_salida` (izquierda, adelante,
derecha, vuelta) o `acercarse` (al cubo). Publica `cmd_vel_auto`.

Etapas: `AVANZANDO`, `RETROCEDIENDO` (al medio de una puerta), `GIRANDO`, `ACERCANDO`, `ESPERANDO`.
- **Giro**: gira por tiempo (o con `/odom` si hay) y después, **con el robot quieto**, lo confirma
  con los ultrasonidos: en una esquina la pared de adelante tiene que pasar al costado contrario.
  Si no confirma, gira 10° más; hasta 45° extra, después pide ayuda.
  Calibración: `factor_giro_tiempo` (2.0), `tolerancia_costado_cm`, `paso_grados`.
- **Posición de laberinto** (`PosicionLaberinto`): las ruedas patinan, así que la posición sale de
  giros confirmados de a 90° y del tiempo avanzado. Se publica en `odom_laberinto`.
- **Acercarse al cubo**: se guía con la cámara hasta 10 cm (lo mínimo que ve), avanza 2,5 cm más
  a ciegas y **pasa a manual** para que el operador agarre con la garra.
- Si pasan a manual sin mover el robot, al volver a auto sigue donde estaba.

### `mapeo` (`mapeo.py` + `libreta.py` + `exploracion.py` + `vuelta.py`)
- **Libreta** (`libreta.py`): un grafo de lugares visitados. Cada lugar tiene sus salidas por punto
  cardinal (norte = hacia donde miraba al arrancar), su estado (pendiente, probada, trabada), a qué
  lugar lleva y cuántos segundos tardó el tramo. Se publica en `mapeo/libreta` (JSON, para el panel).
- **Exploración** (`exploracion.py`): cuando no hay flecha, toma una salida sin probar; si no hay,
  va hacia el lugar más cercano que tenga una (búsqueda en anchura, BFS); si no queda ninguna, media vuelta.
- **Volver** (`vuelta.py`): servidor de **`/volver`** (`Volver.action`). Busca el camino más
  rápido a la largada con **Dijkstra** sobre los segundos de cada tramo, y lo recorre pidiendo `/mover`.
- **Retomar**: después de Abandonar, sigue explorando desde el último lugar, sin volver al principio.

---

## 7. Piso 3: qué hacer (`prometeo_decision`)

`nodo_decision.py` conecta ROS con `logica.py`, que es una máquina de estados sin ROS (se prueba sola).

```
BUSCANDO_COLOR ──► SIGUIENDO ──► ACERCANDO ──► VOLVIENDO ──► (próximo color)
```
- **Ver** (`ver`): ignora lo de poca confianza y solo cree lo que vio en 2 de las últimas 3 imágenes.
  Espera la flecha del próximo color (rojo → verde → azul); después ignora los otros colores.
  Flecha de su color: la guarda para el próximo lugar. Cubo de su color: pide `acercarse`.
- **Llegar a un lugar** (`llegar_a_lugar`): curva → su única salida; bifurcación → la de la flecha;
  sin flecha o callejón → `/decision/explorar` (Mapeo elige).
- **Abandonar color** (botón del panel, `/prometeo/pedido_operador`): descarta el color y publica
  `/decision/retomar`. Si no quedan colores, vuelve.
- Apaga el freno de la ESP32 (`seguridad_habilitada`) para poder arrimarse al cubo.

---

## 8. Mensajes propios (`prometeo_interfaces`)

| Interfaz | Uso |
|---|---|
| `msg/Lugar` | Tipo de lugar (curva, bifurcación, callejón), salidas, posición, orientación, `frente_inicial`/`frente_final` (-1 = sin dato) |
| `action/Mover` | Goal: `accion`, `salida`, `length_class`. Result: `terminada`, `bloqueada` o `cancelada` |
| `action/Volver` | Goal: `color` (vacío = sin cubo). Result: igual que Mover |

---

## 9. Una ronda, paso a paso

1. Mover avanza; el detector avisa «bifurcación»; Mapeo la anota en la libreta.
2. Decisión mira si guardó una flecha de su color y pide `/mover` hacia esa salida (o explorar).
3. Mover gira, confirma con los ultrasonidos y sigue avanzando.
4. Visión ve el cubo; Decisión pide `acercarse`; Mover se arrima y pasa a manual.
5. El operador agarra el cubo con la garra y vuelve a automático.
6. Decisión pide `/volver`; Mapeo calcula el camino con Dijkstra y lo recorre.
7. En la largada pasa a manual: el operador deja el cubo en el contenedor.

---

## 10. Pruebas

```bash
cd ros && colcon build && source install/setup.bash
cd src/prometeo_navigation && python3 -m pytest -q test   # 182 pruebas
cd ../prometeo_decision     && python3 -m pytest -q test
```
Las cuentas (`lugares.py`, `maniobras.py`, `libreta.py`, `exploracion.py`, `vuelta.py`,
`logica.py`) no usan ROS: se prueban sin robot.

---

## 11. Pendientes conocidos

- Calibrar en el robot: `factor_giro_tiempo`, `tolerancia_costado_cm`, `paso_grados`, `empujon_cm`, `distancia_final_m`.
- Mover se acerca al cubo más cercano de **cualquier** color (PROVISORIO: el goal no lleva color).
- El largo de la flecha llega pero navegación todavía no lo usa.
- El centrado del firmware puede pelear con el acercamiento al cubo (los dos corrigen el giro).
- `/ultrasonido/frente` (para el largo real de la flecha) solo se publica con `asistencia:=true`.
