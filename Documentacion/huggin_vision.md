# Paquete `huggin_vision`

Nodo de ROS 2 (Jazzy, Python) que detecta **cubos, flechas y zonas** de color (rojo, verde y azul) en la imagen de la cámara ESP32-CAM y publica dónde están, en un sistema de referencia sobre el piso. Pensado para la competencia: el robot usa esas detecciones para agarrar cubos, seguir flechas y evitar o buscar zonas.

Paquetes involucrados (carpeta `ros/src/`):

| Paquete | Qué contiene |
|---------|--------------|
| `huggin_vision` | El nodo `object_recognition` y los módulos de detección |
| `huggin_vision_msgs` | Mensajes `Detection` y `DetectionArray` |
| `my_robot_bringup` | Parámetros (`config/vision.yaml`) y launch (`vision.launch.py`, `vision_real.launch.py`) |

## 1. Qué se detecta

| Objeto | Tamaño real | Qué hace el robot con él | Cómo se ubica |
|--------|-------------|--------------------------|---------------|
| Cubo | 2 × 2 × 2 cm | Agarrarlo con la garra y llevarlo al depósito del mismo color | Se ajusta la silueta de un cubo a la mancha y se obtiene posición y orientación |
| Flecha | Punta 70 mm de ancho × 50 mm de largo, cuerpo 30 mm de ancho; largo del cuerpo 30 mm (*close*), 60 mm (*normal*) o 100 mm (*far*) | Apunta hacia el cubo del mismo color | Por la forma en la imagen (la flecha está **en una pared vertical**) |
| Zona | 40 × 40 cm en el piso | Verde = partida, azul = llegada, roja = no cruzar | Se proyecta su contorno sobre el piso |

## 2. Cómo funciona

Para cada imagen:

1. **Preproceso** (`segmentation.py`): se redimensiona a `frame_width × frame_height` (640 × 480 por defecto), se desenfoca y se pasa a HSV.
2. **Máscaras de color**: una por color (`hsv.<color>.lower/upper`), con una apertura de 3 × 3 para sacar ruido.
3. **Clasificación de manchas** (`blobs.py`): cada mancha es una sola cosa, probando en este orden:
   1. **Zona** si el área es al menos `min_area.zone`.
   2. **Flecha** si tiene las dos muescas características (`arrow.py`).
   3. **Cubo** si es un cuadrado lleno de al menos `min_area.cube` (`cube_fit.py`).
4. **Pose del objeto** en el sistema de referencia del piso (`camera_model.py`).
5. **Publicación** de los mensajes, marcadores de RViz e imagen anotada.

### 2.1 Pose de la cámara sin Gazebo ni TF

La cámara está en un servo (giro horizontal). La pose de la cámara respecto del piso se arma con parámetros más el ángulo del servo:

- Parámetros `camera.x`, `camera.y`, `camera.height` (sobre el piso) y `camera.pitch_deg` (inclinación hacia abajo).
- El ángulo del servo se lee de `camara/cmd` (`std_msgs/Float32`, grados), el mismo tópico con el que el ESP32 mueve el servo. `camera.pan_center_deg` es el ángulo en que la cámara mira de frente (90 por defecto) y `camera.pan_sign` indica si un ángulo mayor gira la cámara a la izquierda (+1) o a la derecha (−1).
- Con `camera.use_tf: true` (simulación) la pose se lee de TF en lugar de usar los parámetros. `vision.launch.py` lo activa.

El sistema de referencia de salida (`output_frame`, por defecto `base_footprint`) tiene el origen en el piso, x hacia adelante, y a la izquierda, z hacia arriba.

### 2.2 Cubos

Se busca el cubo cuya silueta proyectada coincide mejor (1 − IoU) con la mancha. El centro se fija sobre el rayo que pasa por el centroide, y se buscan la profundidad y la rotación (varios arranques con Nelder-Mead). La profundidad es confiable; la orientación es aproximada (un cubo es simétrico y la silueta dice poco de la inclinación), por eso `angle` solo vale módulo 90°.

### 2.3 Zonas

Se toma el contorno convexo simplificado y se proyecta punto a punto sobre el piso con la pose de la cámara. Sobre el piso se calcula el rectángulo mínimo:

- Zona completa: ambos lados deben medir `zone_size` ± `zone_tolerance` (0,40 m ± 25 %). Se informa el centro, las 4 esquinas y el giro.
- Zona cortada por el borde de la imagen (`partial = true`): solo se exige que no sea más grande que `zone_size`; se informan los vértices visibles y la confianza es 0,5.
- Si algún punto del contorno está en el horizonte o por encima, no se puede proyectar y se descarta.

### 2.4 Flechas

Las flechas están en una pared, así que no se proyectan al piso; se miden en la imagen:

- Las dos muescas donde el cuerpo se une a la cabeza definen el eje de simetría. La mitad que termina en punta es la cabeza.
- **Dirección**: ángulo en la imagen (0 = derecha, 90° = arriba) y su nombre (`right`, `up`, `left`, `down`, o `UNK` si no está a menos de `arrow_tolerance_deg`).
- **Largo**: se compara largo / ancho de la cabeza con los valores reales de las tres flechas (1,14, 1,57 y 2,14). No depende de la distancia. La confianza es 1 si coincide exacto y baja a 0 a mitad de camino de la clase vecina.
- **Posición**: la distancia sale del ancho de la cabeza (70 mm) suponiendo que la flecha mira a la cámara. Si se la ve muy de costado, la distancia y el largo salen sesgados.

## 3. Interfaz ROS

### 3.1 Tópicos

| Tópico | Tipo | Sentido | Descripción |
|--------|------|---------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | entrada | Imagen (QoS *best effort*, profundidad 1: se descartan las viejas) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | entrada | Intrínsecos de la cámara |
| `camara/cmd` | `std_msgs/Float32` | entrada | Ángulo del servo de la cámara, en grados |
| `/vision/detections` | `huggin_vision_msgs/DetectionArray` | salida | **Salida principal**, en `output_frame` |
| `/vision/markers` | `visualization_msgs/MarkerArray` | salida | Cubos, contornos de zonas y flechas para RViz |
| `/vision/annotated` | `sensor_msgs/Image` | salida | Imagen con las detecciones dibujadas |
| `/vision/cubes/{red,green,blue}` | `geometry_msgs/PoseArray` | salida | Cubos por color, en el sistema de la cámara (depuración) |

### 3.2 Mensaje `Detection`

| Campo | Significado |
|-------|-------------|
| `kind` | `cube`, `arrow` o `zone` |
| `color` | `red`, `green` o `blue` |
| `pose` | Cubo: su centro. Zona: centro del cuadrado en el piso. Flecha: punto medio, sobre la pared |
| `angle` | Cubo y zona: giro sobre z en rad, módulo 90°. Flecha: dirección en la imagen en rad (0 = derecha, π/2 = arriba) |
| `direction` | Solo flecha: `right`, `up`, `left`, `down` o `UNK` |
| `length_class` | Solo flecha: `close`, `normal` o `far` |
| `polygon` | Zona: esquinas en el piso. Flecha: `[cola, punta]` |
| `confidence` | 0 a 1 |
| `partial` | Zona cortada por el borde de la imagen |

`DetectionArray.header.frame_id` es `output_frame` y `header.stamp` es el instante de captura de la imagen.

### 3.3 Parámetros principales (`my_robot_bringup/config/vision.yaml`)

| Parámetro | Por defecto | Descripción |
|-----------|-------------|-------------|
| `output_frame` | `base_footprint` | Sistema de referencia de salida |
| `frame_width`, `frame_height` | 640, 480 | Tamaño de trabajo |
| `max_fps` | 15 | Máximo de imágenes procesadas por segundo (0 = todas) |
| `camera.use_tf` | `false` | Pose de la cámara por TF (simulación) o por parámetros |
| `camera.x`, `camera.y`, `camera.height`, `camera.pitch_deg` | 0,09 / 0 / 0,127 / 0 | Montaje de la cámara. **Son los valores de la simulación: hay que medir los reales** |
| `camera.pan_topic`, `camera.pan_center_deg`, `camera.pan_sign` | `camara/cmd` / 90 / +1 | Lectura del servo |
| `cube_size`, `zone_size`, `zone_tolerance` | 0,02 / 0,40 / 0,25 | Tamaños reales |
| `arrow_tolerance_deg` | 15 | Tolerancia para nombrar la dirección de una flecha |
| `min_area.cube`, `.arrow`, `.zone` | 500 / 100 / 4000 | Áreas mínimas en píxeles |
| `hsv.<color>.lower/upper` | ver archivo | Umbrales HSV |
| `marker_color.<color>` | RGB | Colores de marcadores e imagen anotada |

Los parámetros se leen en cada imagen, así que se pueden ajustar en vivo con `ros2 param set`.

## 4. Estructura del código

`ros/src/huggin_vision/huggin_vision/`:

| Archivo | Responsabilidad |
|---------|-----------------|
| `object_recognition.py` | El nodo `VisionNode`: suscripciones, flujo por imagen, publicación |
| `segmentation.py` | Preproceso, HSV, máscaras, colores |
| `blobs.py` | Clasifica cada mancha como zona, flecha o cubo |
| `cube_fit.py` | Ajuste de la silueta del cubo y filtro de forma |
| `arrow.py` | Medición de flechas, clase de largo, nombre de dirección |
| `zone.py` | Detección de zonas por proyección al piso |
| `camera_model.py` | Pose de la cámara (montaje + servo), rayos y proyección al piso |
| `geometry.py` | Rotaciones, cuaterniones y vértices del cubo |
| `detections.py` | `Detection`: resultado en Python puro, independiente de los mensajes |
| `visualization.py` | Imagen anotada y marcadores de RViz |
| `camera_stream.py` | Visor web MJPEG independiente (no forma parte de la detección) |

## 5. Cómo usarlo

Compilar y probar (el ROS 2 está en el contenedor `jazzyDesktop`):

```bash
distrobox enter jazzyDesktop -- bash -lc 'source /opt/ros/jazzy/setup.bash && cd /run/media/storage1/06-ROS2/prometeo/ros && colcon build --packages-select huggin_vision_msgs huggin_vision my_robot_bringup && colcon test --packages-select huggin_vision && colcon test-result --verbose'
```

Simulación (Gazebo + RViz + nodo, la pose de la cámara sale de TF):

```bash
ros2 launch my_robot_bringup vision.launch.py
```

Robot real (sin Gazebo ni TF; la imagen del ESP32-CAM debe llegar como `/camera/image_raw` y `/camera/camera_info`):

```bash
ros2 launch my_robot_bringup vision_real.launch.py
ros2 topic echo /vision/detections
```

Para ver los tiempos de cada etapa, ejecutar el nodo con `--ros-args --log-level debug`.

## 6. Pruebas

En `ros/src/huggin_vision/test/` (se ejecutan con `colcon test` o `python3 -m pytest`):

| Archivo | Qué verifica |
|---------|--------------|
| `test_geometry.py` | Ida y vuelta cuaternión ↔ rotación, vértices del cubo |
| `test_segmentation.py` | Redimensionado y máscaras por color |
| `test_cube_fit.py` | Posición de un cubo sintético; se ignoran manchas chicas o alargadas |
| `test_arrow.py` | Clase de largo y dirección con flechas de dimensiones reales, a distintos ángulos y tamaños; rechazo de cuadrados |
| `test_camera_zone.py` | Proyección píxel ↔ piso, giro del servo, horizonte, zonas completas y cortadas |
| `test_blobs.py` | Una escena con zona, flecha y cubo da una detección por objeto, sin duplicados |

Además hay un humo con el nodo real y una imagen sintética: publicó la zona verde en (0,80; 0,00) m y una flecha azul "left, normal".

Los tests de estilo (`test_flake8`, `test_pep257`) fallan: el chequeo exige un estilo de docstrings y comillas que el resto del código del workspace tampoco cumple (por ejemplo `camera_stream.py`).

## 7. Limitaciones y pendientes

- **Calibración**: la ESP32-CAM no está calibrada. Hay que calibrarla (tablero de ajedrez, `camera_calibration`) y publicar el `camera_info`; las posiciones de zonas y cubos dependen de eso.
- **Montaje real**: medir altura, adelanto e inclinación de la cámara y cargarlos en `vision.yaml`.
- **Servo**: se asume que `camara/cmd` es el ángulo en grados, con 90° al frente y ángulo creciente hacia la izquierda. Verificar contra el firmware y ajustar `pan_center_deg` y `pan_sign`.
- **Rojo en HSV**: el rojo real también aparece en el tono 170–179; hoy solo se usa 0–10. Puede hacer falta un segundo rango.
- **Flechas**: la distancia supone que la flecha mira de frente a la cámara. Vista en ángulo se sesgan la distancia y la clase de largo.
- **Cubo sobre zona del mismo color**: se funde con la mancha de la zona y no se detecta como cubo.
- **Rendimiento**: el ajuste del cubo es el paso más caro. Con `max_fps` se limita la carga; el objetivo es 10–15 imágenes por segundo en una notebook. No se midió todavía con la cámara real.
- **Simulación**: las flechas del mundo `vision_testground` siguen comentadas en `vision_testground.world.xacro`; falta habilitarlas para probar flechas en Gazebo.
- El nodo procesa en el mismo hilo en que recibe la imagen; no hay hilos aparte.
