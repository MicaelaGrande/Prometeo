# Guía de la simulación en Gazebo

Explica qué se agregó en el paso 3 de `GUIA_TICKET_NAVEGACION.md`, qué hace cada archivo y
cómo se conectan. Para correrlo, andá a [Cómo se prueba](#cómo-se-prueba).

**Estado:** el laberinto con isla y los ultrasonidos ya se probaron con el teclado en Gazebo. Falta
calibrar los ultrasonidos con el robot real.

## La idea general

Gazebo es un videojuego con física donde vive una copia del robot. Sirve para probar la
navegación sin el robot real y sin riesgo de romper nada.

El objetivo es **mover el robot por un laberinto con el teclado y que mida como el robot real.**
Todavía no hay Nav2, cámara, flechas ni cubos.

Comparado con el robot real, cada parte tiene su reemplazo:

| En el robot real | En la simulación |
|---|---|
| El chasis, las ruedas y el piso | `prometeo.urdf.xacro`, con forma de choque y masa |
| El ESP32 y los motores | Plugin de tracción diferencial |
| Los encoders | La odometría y los ángulos que calcula Gazebo |
| Los HC-SR04 y su firmware | Rayos de Gazebo y el nodo `hcsr04_sim` |
| El laberinto de la competencia | `laberinto.sdf.xacro` |
| La conexión Wi-Fi entre robot y PC | El puente de topics |

## Palabras nuevas

- **Plugin:** una pieza que se le agrega a un programa para sumarle una función. Como una
  aplicación que instalás en el celular.
- **Topic:** un canal de mensajes con nombre. Como un grupo de WhatsApp: unos escriben y otros
  leen, sin conocerse entre sí.
- **Puente:** un traductor. Gazebo y ROS 2 tienen cada uno sus propios topics, y el puente pasa los
  mensajes de un lado al otro.
- **Odometría:** la estimación de cuánto se movió el robot según cuánto giraron las ruedas. Como
  contar tus pasos con los ojos cerrados.
- **TF:** la lista de cómo se ubica cada pieza respecto de otra. Como decir "la rueda está a 5 cm
  del cuerpo, y el cuerpo está a 1 m de donde arrancó".
- **Launch:** un archivo que arranca varios programas con un solo comando.
- **Xacro:** un archivo XML con variables y moldes, para no repetir lo mismo muchas veces.
- **SDF:** el formato que usa Gazebo para describir mundos.
- **Eco:** el sonido que rebota y vuelve al sensor. El HC-SR04 mide cuánto tarda en volver.
- **Isla:** un bloque de paredes suelto, sin conexión con el contorno. Se le puede dar la vuelta,
  así que seguir una pared puede dejarte girando para siempre.
- **Mediana:** el valor del medio de una lista ordenada. Sirve para ignorar una lectura rara suelta.
- **QoS best effort:** una forma de enviar mensajes sin reintentar los perdidos. Como una radio:
  si te perdiste una frase, seguís con la siguiente.

## Cómo está organizado

```text
ros/src/
├── prometeo_description/           El robot: cómo es
│   └── urdf/
│       ├── prometeo.urdf.xacro     Modificado: se sumó choque y masa
│       └── prometeo.gazebo.xacro   Nuevo: tracción, encoders y rayos simulados
│
├── prometeo_gazebo/                Nuevo paquete: dónde vive y cómo se conecta
│   ├── worlds/laberinto.sdf.xacro  El laberinto
│   ├── config/puente.yaml          Qué topics se traducen
│   ├── config/hcsr04.yaml          Valores del ultrasonido simulado
│   ├── scripts/hcsr04_sim.py       Nodo que reemplaza al ESP32
│   ├── scripts/modelo_hcsr04.py    Las cuentas del sonido y del firmware, sin ROS
│   ├── test/                       Pruebas del modelo, se corren sin Gazebo
│   ├── launch/simulacion.launch.py Arranca todo
│   ├── CMakeLists.txt              Qué carpetas se instalan al compilar
│   └── package.xml                 De qué otros paquetes depende
│
└── prometeo_sensores/              Nuevo paquete: corre también en el robot real
    └── prometeo_sensores/distancias_a_range.py   Convierte /distancias en Range
```

Los separamos así porque el robot real también va a usar `prometeo_description` y
`prometeo_sensores`, mientras que `prometeo_gazebo` solo sirve para simular.

## Qué hay en cada archivo

### `prometeo_description/urdf/prometeo.urdf.xacro` (modificado)

Es la descripción del robot. Antes solo decía cómo **se ve**, y a eso le sumamos cómo **se
comporta físicamente**:

- **Masas:** dos variables nuevas, `masa_cuerpo` y `masa_rueda`.
- **`<collision>`:** la forma con la que el robot choca. El cuerpo es una caja de 26 x 16,3 cm y
  cada rueda es un cilindro. Son formas simples porque calcular choques con el modelo 3D completo
  sería mucho más lento.
- **`<inertial>`:** la masa y qué tan difícil es hacerla girar. Sin esto, Gazebo no puede mover la
  pieza.
- **`<xacro:include>`:** al final del archivo, carga `prometeo.gazebo.xacro`.

Una diferencia importante: `<visual>` es lo que ves y `<collision>` es lo que choca. Por eso, el
modelo puede verse detallado y chocar como una caja.

La cámara sigue siendo solo un dibujo. Lo que miden los ultrasonidos se define en el archivo
siguiente.

### `prometeo_description/urdf/prometeo.gazebo.xacro` (nuevo)

Reúne lo que solo le importa a Gazebo. RViz lo ignora.

- **Plugin `DiffDrive` (tracción diferencial):** lee `/cmd_vel` y hace girar las ruedas de cada lado.
  Es el equivalente de `motores_set(izq, der)`. También calcula la odometría y la publica.
- **Plugin `JointStatePublisher`:** publica el ángulo de cada rueda, como harían los encoders.
- **Macro `agarre_rueda`:** define la fricción de cada rueda con el piso, con el valor `mu`.
- **Macro `rayos_ultrasonido`:** le pone a cada ultrasonido un abanico de 21 rayos que cubre ±15°.
  Gazebo no trae un sensor de ultrasonido, así que los rayos solo dicen a qué distancia hay algo.

El robot gira como un tanque: las ruedas de un lado van más rápido que las del otro y se arrastran
de costado. Por eso el agarre importa: si es muy alto, le cuesta doblar.

### `prometeo_gazebo/worlds/laberinto.sdf.xacro` (nuevo)

Es el mundo donde vive el robot.

```text
        +==========================+
        |      norte: 60 cm        |
        |    +==============+      |
        | 40 |              |  50  +=========+
        | cm |     ISLA     |  cm    rama 40 |  sin salida
        |    |              |      +=========+
        |    +==============+      |
+=======+                          |
| R     A     sur: 40 cm           |
+==================================+
```

El robot arranca en `R`, mirando hacia la derecha del dibujo. En `A` puede seguir recto o doblar a la
izquierda, y los dos caminos dan la vuelta a la isla. La rama del lado este es un callejón. Con esto
se prueban bifurcaciones, un callejón y pasillos de 40, 50 y 60 cm.

Dentro del archivo vas a encontrar:

- **Medidas:** `pasillo`, `pasillo_este`, `largo_isla`, `alto_pared`, `espesor` y otras. Si cambia una
  medida, se cambia acá.
- **Líneas del plano:** como `x_oeste` o `y_isla_norte`, calculadas a partir de las medidas.
- **Plugins del mundo:** física, agregar el robot mientras corre, dibujar la escena y hacer funcionar
  los sensores.
- **Luz y piso.**
- **Macro `pared`:** recibe los dos extremos de una pared y calcula sola su centro, largo y giro.
- **Las 14 paredes:** cada línea es una pared.

Para agregar una pared, sumá una línea como esta:

```xml
<xacro:pared nombre="nueva" x1="0" y1="0" x2="1" y2="0"/>
```

### `prometeo_gazebo/config/puente.yaml` (nuevo)

Lista los topics que el puente traduce. Cada bloque dice el nombre del topic, el tipo de mensaje de
cada lado y la dirección.

| Topic | Dirección | Para qué sirve |
|---|---|---|
| `/clock` | Gazebo → ROS | El reloj de la simulación |
| `/cmd_vel` | ROS → Gazebo | Las órdenes de velocidad |
| `/odom` | Gazebo → ROS | La posición estimada del robot |
| `/tf` | Gazebo → ROS | La relación entre `odom` y `base_footprint` |
| `/joint_states` | Gazebo → ROS | El ángulo de cada rueda |
| `/ultrasonido/*/rayos` | Gazebo → ROS | Los rayos de cada ultrasonido, para `hcsr04_sim` |

`/cmd_vel` es lo único que entra a Gazebo: todo lo demás es información que sale.

Si un topic no está en esta lista, no cruza. Cuando sumemos sensores, también habrá que agregarlos acá.

### `prometeo_gazebo/scripts/modelo_hcsr04.py` y `hcsr04_sim.py` (nuevos)

Un ultrasonido real no mide "lo que tiene adelante". Manda un cono de sonido, y ese sonido rebota en
una pared lisa como una pelota. Estos dos archivos reproducen eso:

- **`modelo_hcsr04.py`:** las cuentas, sin nada de ROS, para poder probarlas sin abrir Gazebo.
- **`hcsr04_sim.py`:** el nodo que reemplaza al ESP32. Lee los rayos y publica `/distancias`.

Efectos del sonido que simula:

| Efecto | Qué pasa |
|---|---|
| Cono | Toma el eco más cercano dentro de ±15°, no solo lo que hay sobre el eje |
| Rebote | Una pared solo devuelve eco si un rayo le pega casi de frente |
| Esquina interior | Devuelve el eco aunque ninguna de sus paredes esté de frente |
| Ruido, temperatura y eco perdido | Cada lectura varía un poco, y a veces se pierde |

Lo que copia del firmware (`hcsr04.c` y `sensores.c`):

- Mide izquierda y derecha juntos y el frente 60 ms después. Publica unas 8 veces por segundo.
- Convierte el tiempo de eco con `us / 58`.
- Si está fuera de rango o se pierde el eco, da 400. **Pegado a la pared también da 400.**
- Aplica una mediana de 3 lecturas, que además atrasa un poco cada cambio.

Comportamientos que hay que tener en cuenta al programar la navegación:

| Situación | Qué mide | Por qué importa |
|---|---|---|
| Robot girado 45° en un pasillo | 400 en los tres sensores | 400 no siempre significa camino libre |
| Frente al arrancar | 91 cm, aunque el pasillo sigue 220 cm | El borde del cono entra por `A` y pega en la isla |
| Lateral al pasar por `A` | Salta a 151 cm unos 3 cm después del final de la pared | La salida se detecta tarde |

### `prometeo_sensores/distancias_a_range.py` (nuevo)

Convierte `/distancias` en un mensaje `sensor_msgs/Range` por sensor, en `/ultrasonido/izq`, `/der` y
`/frente`. Es lo que esperan herramientas como Nav2. Un 400 se publica como `+inf`, que en ROS significa
"no hay nada dentro del rango". Corre igual en el robot real, porque solo depende de `/distancias`.

### `prometeo_gazebo/config/hcsr04.yaml` (nuevo)

Los valores del ultrasonido simulado. Se cambian acá sin tocar el código, y después hay que correr
`colcon build`.

### `prometeo_gazebo/launch/simulacion.launch.py` (nuevo)

Arranca todo con un solo comando. En orden:

1. **Prepara el mundo:** Gazebo no entiende xacro, así que convierte el laberinto a SDF y lo guarda
   en un archivo temporal.
2. **`ruta_modelos`:** le dice a Gazebo dónde están los modelos 3D del robot.
3. **`gazebo`:** abre el simulador con el mundo.
4. **`robot_state_publisher`:** publica la descripción del robot y dónde está cada pieza.
5. **`agregar_robot`:** lee esa descripción y pone el robot en el mundo, en el punto (0, 0).
6. **`puente`:** conecta Gazebo con ROS 2.
7. **`hcsr04_sim`:** convierte los rayos en `/distancias`.
8. **`distancias_a_range`:** convierte `/distancias` en tres topics `Range`.

`use_sim_time` hace que los nodos usen el reloj de Gazebo. Si no, se mezclarían la hora de la PC y la
de la simulación, que puede ir más lenta.

### `prometeo_gazebo/CMakeLists.txt` y `package.xml` (nuevos)

Son los archivos que todo paquete de ROS 2 necesita.

- **`CMakeLists.txt`:** copia `launch`, `config` y `worlds` a la carpeta `install/`, y los nodos de
  Python a `lib/`.
- **`package.xml`:** el nombre del paquete y de qué otros paquetes depende.

ROS busca los archivos en `install/`, no en `src/`. Por eso, **después de cambiar cualquier archivo
hay que correr `colcon build` de nuevo.**

## La lógica completa

Esto pasa cuando apretás la tecla `i`:

```text
Teclado
   │  tecla "i"
   ▼
teleop_twist_keyboard
   │  /cmd_vel: "avanzá a 0,25 m/s"
   ▼
Puente (ROS → Gazebo)
   ▼
Plugin DiffDrive
   │  calcula cuánto tiene que girar cada rueda
   ▼
Física de Gazebo
   │  las ruedas giran, empujan contra el piso y el robot avanza
   ▼
Plugin DiffDrive y JointStatePublisher
   │  /odom, /tf y /joint_states
   ▼
Puente (Gazebo → ROS)
   ▼
Tus herramientas y nodos de ROS 2
```

Es un ciclo: se manda una orden, el robot se mueve y vuelve la información de cómo se movió.

Lo importante es que **en el paso 5, Nav2 va a reemplazar al teclado**. Va a publicar en el mismo
`/cmd_vel` y leer el mismo `/odom`, así que estos archivos se siguen usando. En el robot real pasa
lo mismo: se reemplaza Gazebo por el ESP32 con micro-ROS, con los mismos topics. La diferencia
pendiente es que Nav2 manda m/s y rad/s, y hoy `micro_ros/main/main.c` trata `/cmd_vel` como
valores de -1 a 1.

## Cómo se prueba

Solo en la PC principal.

**Terminal 1:**

```bash
cd <carpeta del repo>
git switch feature/ros/navigation
git pull
cd ros
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch prometeo_gazebo simulacion.launch.py
```

**Terminal 2:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Dejá esta terminal seleccionada, porque ahí se leen las teclas. Apretá `x` unas 7 veces para bajar a
0,25 m/s.

| Tecla | Acción |
|---|---|
| `i` | Avanzar |
| `,` | Retroceder |
| `j` | Girar a la izquierda |
| `l` | Girar a la derecha |
| `k` | Frenar |

**Terminal 3:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /distancias --qos-reliability best_effort --field data
```

Muestra `[izq, der, frente]` en cm. Tiene que indicar la opción `best_effort`: sin ella no llega nada.

Tenés que comprobar que:

1. El robot aparece con su forma real y queda apoyado en el piso.
2. Avanza recto y frena con `k`.
3. Gira en el lugar.
4. Puede dar la vuelta a la isla por los dos lados sin atravesar las paredes.
5. Al arrancar la terminal 3 muestra unos `[12, 12, 91]`.
6. Al acercarte a una pared baja el número del sensor que corresponde.
7. Al pasar por `A`, el primer número salta a unos 150.

Las pruebas del modelo se corren sin abrir Gazebo, desde la carpeta `ros`:

```bash
python3 -m pytest src/prometeo_gazebo/test
```

## Qué valores son provisorios

| Valor | Dónde se cambia | Estado |
|---|---|---|
| Masa de 0,9 kg del cuerpo y 0,08 kg por rueda | `prometeo.urdf.xacro` | Estimado: hay que pesar el robot |
| Separación entre ruedas de 0,13 m | `prometeo.urdf.xacro` | Sale del modelo 3D: hay que medirla |
| Agarre `mu` de 0,8 | `prometeo.gazebo.xacro` | Estimado: se ajusta comparando con el robot real |
| Pasillos de 40, 50 y 60 cm | `laberinto.sdf.xacro` | Elegidos para probar: el laberinto real está hecho a mano y el pasillo mide 40 cm como mínimo, sin máximo conocido |
| Ángulo máximo de rebote de 20° | `hcsr04.yaml` | Estimado: depende del material de las paredes |
| Ruido de 0,3 cm + 0,2 % de la distancia | `hcsr04.yaml` | Estimado: hay que medirlo con el sensor real |
| Eco perdido en 1 % de las lecturas | `hcsr04.yaml` | Estimado: hay que medirlo con el sensor real |
| Error fijo de cada sensor en 0 cm | `hcsr04.yaml` | Sin medir: es `offset_sensor_cm` en `sensores.c` |
| Paredes de 20 cm de alto | `laberinto.sdf.xacro` | Sale del reglamento en Google Docs, que describe otro formato de pista |
| Paredes de 2 cm de espesor | `laberinto.sdf.xacro` | Estimado: no figura en el reglamento |

## Qué sigue

Lo que falta de T1 es la cámara simulada y los cubos de 20 mm. Después sigue T2, Navegación alta.

Cuando se pueda usar el robot real, hay que registrar `/distancias` frente a una pared, a distancias y
ángulos conocidos, y ajustar `hcsr04.yaml`.

**Qué no simula:** rebotes múltiples, ecos fantasma del ciclo anterior ni objetos más bajos que el
sensor, porque el abanico es horizontal. El freno de emergencia de `seguridad.c` tampoco está en la
simulación.
