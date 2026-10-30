# Guía para continuar el ticket de navegación

Este archivo resume una sesión de trabajo con Claude Code hecha en otra computadora,
para poder seguir desde cualquier PC. Si sos Claude: leelo completo y seguí estas
indicaciones. Antes de avanzar, revisá el estado real del repo, porque este resumen
puede haber quedado desactualizado.

## Cómo trabajar conmigo

- Es mi primera vez con ROS 2, Nav2 y Gazebo. Explicame cada término nuevo con palabras
  simples y una comparación.
- Vamos despacio, un paso chico por vez, y comprobamos que funcione antes de seguir.
- Quiero entender lo que hacemos. Creá vos los archivos, con el código bien comentado
  para que lo entienda mientras lo leo.
- Yo corro los comandos que abren ventanas. Decime en qué carpeta tengo que estar.
- No mergees las ramas de otras personas. Si hace falta un archivo puntual, copialo.
- No cuentes como cumplido un criterio del ticket por algo que funcionó en la demo del TurtleBot.
- En los commits no agregues la línea `Co-Authored-By`.
- Comunicación en español.

## Ticket: Nodo de Navegación (alto nivel)

**Objetivo:** que el robot vaya de forma autónoma a un objetivo del mapa, siguiendo la ruta
y evitando colisiones en tiempo real, usando Nav2 sobre ROS 2.

**Responsabilidad:**
- Planificar la ruta global hacia el objetivo.
- Seguir la ruta y corregir la trayectoria ante obstáculos (controlador local).
- Replanificar si el camino queda bloqueado.
- Frenar o reducir velocidad ante colisión inminente (Collision Monitor).
- Recuperarse si el robot se traba.
- Ejecutar la lógica de misión de la competencia (secuencia de objetivos).

**Entradas:** `/map` (OccupancyGrid, de Mapeo), TF `map → odom` (Mapeo) y `odom → base_link`
(Bajo nivel), `/scan` (ver la corrección más abajo), `/odom` (Bajo nivel), objetivos por las
acciones `navigate_to_pose` y `follow_waypoints`.

**Salida:** `/cmd_vel` (Twist), hacia Bajo nivel.

**Fuera de alcance:** construir el mapa y localizar (Mapeo); motores, PID y odometría
(Bajo nivel); frenado de seguridad por hardware o por pérdida de comunicación (Bajo nivel).

**Criterios de aceptación:**
1. En simulación, al marcar un objetivo en RViz el robot llega solo.
2. Si se coloca un obstáculo en el camino, lo esquiva o replanifica sin chocar.
3. Si algo aparece de golpe delante, frena (Collision Monitor).
4. El nodo de misión recorre una secuencia de puntos desde código.
5. En el robot real, las mismas pruebas funcionan con los parámetros ajustados.

**Corrección:** `/scan` es la salida de un LiDAR, y este robot no tiene. Nav2 va a usar
los 3 ultrasonidos con mensajes `sensor_msgs/Range` (capa `RangeSensorLayer` en los costmaps
y fuentes `range` en el Collision Monitor). Hay que corregir el ticket.

## El robot

- 4 ruedas comunes, un motor cada una, comandadas por lado: `motores_set(izq, der)`.
  Para Nav2 es un robot diferencial.
- 4 encoders. Están en otra rama, todavía sin mergear.
- 3 ultrasonidos HC-SR04 en el parachoques delantero: el de adelante horizontal y los
  de los costados verticales. Publican `/distancias` (`Float32MultiArray`, en cm).
- ESP32 con micro-ROS y ESP32-CAM arriba del chasis, cerca de los sensores. La cámara
  gira con un servo y está en otra rama, todavía sin mergear.
- Brazo con garra arriba del robot. Junto con el servo de la cámara, son 5 servos.
  Órdenes del brazo: `/brazo/cmd` con `[base, hombro, codo, garra]`.

## Estado actual

Rama: `feature/ros/navigation`.

Hecho:
- Demo de Nav2 con TurtleBot, solo para aprender: objetivo, varios puntos, obstáculo,
  nodos y topics.
- Paquete `ros/src/prometeo_description`: `urdf/prometeo.urdf.xacro`, `meshes/`,
  `launch/display.launch.py` y `rviz/prometeo.rviz`.
- Los 5 STL de chasis y ruedas se copiaron, sin merge, de `feature/ros/gazebo_simulation`.
- El URDF pasó `check_urdf` y el paquete compila.

Sin confirmar: todavía no vi la última versión en RViz.

Ningún criterio de aceptación está cumplido.

## Medidas

Salen de los modelos 3D, en metros: largo 0.26, ancho 0.163, radio de rueda 0.0345,
ancho de rueda 0.03, distancia entre ejes 0.111, separación entre centros de ruedas 0.13.
Hay que medir la separación entre ruedas en el robot real: el 0.16 de la rama de
simulación es de borde a borde. Cambiar una variable no cambia el tamaño de un STL.

## Qué sigue

1. Ver el robot en RViz y compararlo con el robot real.
2. Sumar brazo y garra cuando consiga los STL. La carpeta de Drive que tengo solo trae
   fotos de armado. Si no aparecen, usar formas simples con articulaciones. No influye
   en la navegación, así que no frena lo demás.
3. Gazebo: mundo propio, tracción, puente de topics y mover el robot con el teclado.
4. Ultrasonidos simulados, publicando `sensor_msgs/Range`.
5. Nav2: mapa, TF `map → odom` temporal, `nav2_params.yaml` y objetivo desde RViz.
6. Obstáculos y replanificación.
7. Collision Monitor.
8. Nodo de misión en Python con `nav2_simple_commander`.
9. Robot real: nodo que pase `/distancias` a `Range` y ajuste de parámetros.
   Depende de que Bajo nivel publique `/odom` y Mapeo publique `/map`.

## Pendientes con el equipo

- Corregir `/scan` en el ticket.
- Medir la separación entre ruedas.
- Avisar a Bajo nivel que Nav2 manda m/s y rad/s, pero hoy `micro_ros/main/main.c`
  trata `/cmd_vel` como valores de -1 a 1.
- Si el brazo sobresale del cuerpo al navegar, agrandar el contorno del robot en Nav2.

## Qué se puede hacer en cada computadora

Tengo dos PCs:
- **PC principal:** corre Gazebo y RViz. Es donde se prueba todo.
- **PC liviana:** apenas corre RViz y no puede correr Gazebo. No propongas abrir Gazebo ahí.

En la PC liviana sí se puede:
- Revisar el robot en RViz y sumar brazo y garra.
- Escribir archivos: `nav2_params.yaml`, launch y el nodo de misión.
- Estudiar y preparar lo que después se prueba en la PC principal.

Opción sin probar para la PC liviana: `nav2_loopback_sim`, un simulador sin física que
Nav2 referencia en `tb3_loopback_simulation.launch.py`. No estaba instalado cuando se escribió
esta guía. Antes de planificar con él, comprobá si el paquete existe, si la PC lo soporta
y si sirve para este robot sin LiDAR.

Las pruebas de los criterios de aceptación se hacen en la PC principal.

## Preparar otra computadora

Necesita Ubuntu 24.04 con ROS 2 Jazzy. En la PC liviana alcanza con instalar lo del robot:

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

En la PC principal va todo:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard

cd <carpeta del repo>/ros
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch prometeo_description display.launch.py
```
