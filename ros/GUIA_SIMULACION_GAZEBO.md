# Guía de la simulación en Gazebo

Explica qué se agregó en el paso 3 de `GUIA_TICKET_NAVEGACION.md`, qué hace cada archivo y
cómo se conectan. Para correrlo, andá a [Cómo se prueba](#cómo-se-prueba).

**Estado:** los archivos compilan y pasaron las validaciones, pero todavía no se probaron con
física. Eso se hace en la PC principal.

## La idea general

Gazebo es un videojuego con física donde vive una copia del robot. Sirve para probar la
navegación sin el robot real y sin riesgo de romper nada.

El objetivo de este paso es uno solo: **mover el robot por un laberinto con el teclado.** Todavía
no hay Nav2, ultrasonidos simulados, cámara, flechas ni cubos.

Comparado con el robot real, cada parte tiene su reemplazo:

| En el robot real | En la simulación |
|---|---|
| El chasis, las ruedas y el piso | `prometeo.urdf.xacro`, con forma de choque y masa |
| El ESP32 y los motores | Plugin de tracción diferencial |
| Los encoders | La odometría y los ángulos que calcula Gazebo |
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

## Cómo está organizado

```text
ros/src/
├── prometeo_description/           El robot: cómo es
│   └── urdf/
│       ├── prometeo.urdf.xacro     Modificado: se sumó choque y masa
│       └── prometeo.gazebo.xacro   Nuevo: tracción y encoders simulados
│
└── prometeo_gazebo/                Nuevo paquete: dónde vive y cómo se conecta
    ├── worlds/laberinto.sdf.xacro  El laberinto
    ├── config/puente.yaml          Qué topics se traducen
    ├── launch/simulacion.launch.py Arranca todo
    ├── CMakeLists.txt              Qué carpetas se instalan al compilar
    └── package.xml                 De qué otros paquetes depende
```

Los separamos así porque el robot real también va a usar `prometeo_description`, mientras que
`prometeo_gazebo` solo sirve para simular.

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

Los ultrasonidos y la cámara siguen siendo solo dibujos: no miden nada todavía.

### `prometeo_description/urdf/prometeo.gazebo.xacro` (nuevo)

Reúne lo que solo le importa a Gazebo. RViz lo ignora.

- **Plugin `DiffDrive` (tracción diferencial):** lee `/cmd_vel` y hace girar las ruedas de cada lado.
  Es el equivalente de `motores_set(izq, der)`. También calcula la odometría y la publica.
- **Plugin `JointStatePublisher`:** publica el ángulo de cada rueda, como harían los encoders.
- **Macro `agarre_rueda`:** define la fricción de cada rueda con el piso, con el valor `mu`.

El robot gira como un tanque: las ruedas de un lado van más rápido que las del otro y se arrastran
de costado. Por eso el agarre importa: si es muy alto, le cuesta doblar.

### `prometeo_gazebo/worlds/laberinto.sdf.xacro` (nuevo)

Es el mundo donde vive el robot.

```text
        +=======================+
        | rama izq  T  rama der |
        +=======+       +=======+
                |       |
                |       |   2 segmentos
        +=======+       |
        | R             |   3 segmentos y una curva
        +===============+
```

El robot arranca en `R`, mirando hacia la derecha del dibujo. Las dos ramas no tienen salida. Con
esto se prueban una recta, una curva y una bifurcación, que son las situaciones de la competencia.

Dentro del archivo vas a encontrar:

- **Medidas:** `segmento`, `pasillo`, `alto_pared` y `espesor`. Si cambia una medida, se cambia acá.
- **Líneas del plano:** como `x_curva` o `y_ramas`, calculadas a partir de las medidas.
- **Plugins del mundo:** física, agregar el robot mientras corre y dibujar la escena.
- **Luz y piso.**
- **Macro `pared`:** recibe los dos extremos de una pared y calcula sola su centro, largo y giro.
- **Las 10 paredes:** cada línea es una pared.

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

`/cmd_vel` es lo único que entra a Gazebo: todo lo demás es información que sale.

Si un topic no está en esta lista, no cruza. Cuando sumemos sensores, también habrá que agregarlos acá.

### `prometeo_gazebo/launch/simulacion.launch.py` (nuevo)

Arranca todo con un solo comando. En orden:

1. **Prepara el mundo:** Gazebo no entiende xacro, así que convierte el laberinto a SDF y lo guarda
   en un archivo temporal.
2. **`ruta_modelos`:** le dice a Gazebo dónde están los modelos 3D del robot.
3. **`gazebo`:** abre el simulador con el mundo.
4. **`robot_state_publisher`:** publica la descripción del robot y dónde está cada pieza.
5. **`agregar_robot`:** lee esa descripción y pone el robot en el mundo, en el punto (0, 0).
6. **`puente`:** conecta Gazebo con ROS 2.

`use_sim_time` hace que los nodos usen el reloj de Gazebo. Si no, se mezclarían la hora de la PC y la
de la simulación, que puede ir más lenta.

### `prometeo_gazebo/CMakeLists.txt` y `package.xml` (nuevos)

Son los archivos que todo paquete de ROS 2 necesita.

- **`CMakeLists.txt`:** copia `launch`, `config` y `worlds` a la carpeta `install/`.
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
ros2 topic echo /odom --field pose.pose.position
```

Tenés que comprobar que:

1. El robot aparece con su forma real y queda apoyado en el piso.
2. Avanza recto y frena con `k`.
3. Gira en el lugar.
4. Llega a la bifurcación sin atravesar las paredes.
5. Los números de la terminal 3 cambian cuando se mueve.

## Qué valores son provisorios

| Valor | Dónde se cambia | Estado |
|---|---|---|
| Masa de 0,9 kg del cuerpo y 0,08 kg por rueda | `prometeo.urdf.xacro` | Estimado: hay que pesar el robot |
| Separación entre ruedas de 0,13 m | `prometeo.urdf.xacro` | Sale del modelo 3D: hay que medirla |
| Agarre `mu` de 0,8 | `prometeo.gazebo.xacro` | Estimado: se ajusta comparando con el robot real |
| Segmento de 32 cm y pasillo de 40 cm | `laberinto.sdf.xacro` | Aproximados: el laberinto está hecho a mano y el pasillo mide 40 cm como mínimo, sin máximo conocido |
| Paredes de 20 cm de alto | `laberinto.sdf.xacro` | Sale del reglamento en Google Docs, que describe otro formato de pista |
| Paredes de 2 cm de espesor | `laberinto.sdf.xacro` | Estimado: no figura en el reglamento |

## Qué sigue

Cuando el robot se mueva bien, sigue el paso 4: simular los ultrasonidos publicando
`sensor_msgs/Range`. Eso va a sumar un sensor al robot, un plugin al mundo y más líneas al puente.
