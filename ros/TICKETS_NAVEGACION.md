# Tickets de navegación autónoma

El robot sigue flechas de un color hasta el cubo y explora cuando no hay una flecha de ese color.
Asumimos que no todas las bifurcaciones tienen flecha de cada color, así que Exploración con memoria
forma parte del mínimo. También aporta la libreta que hace falta para volver.

**Mínimo actual:** llegar a un cubo. Agarrarlo, depositarlo y volver quedan para después.

**Criterio de diseño:** el laberinto está hecho a mano, así que las medidas son aproximadas. Los
pasillos miden 40 cm como mínimo y no tienen un máximo conocido. Por eso, el robot se guía por lo
que miden los sensores y no por distancias fijas.

## Orden de trabajo

| Orden | Ticket | Hito | Depende de | Estado |
|---|---|---|---|---|
| 1 | T1. Base de simulación | Llegar al cubo | Nada | Falta calibrar ultrasonidos |
| 2 | T2. Navegación alta | Llegar al cubo | T1 | Nav2 descartado, sigue control por ultrasonidos |
| 3 | T3. Mapeo: libreta del recorrido | Llegar al cubo | T2 | Solo los datos |
| 4 | T4. Exploración con memoria | Llegar al cubo | T3 | Sin empezar |
| 3 y 4, en paralelo | T6. Flechas en visión | Llegar al cubo | Nada | Casi lista, sin mergear |
| 5 | T7. Decisión por colores | Llegar al cubo | T4 y T6 | Sin empezar |
| 6 | T5. Decisión básica | Llegar al cubo | T7 | Sin empezar |
| 7 | T8. Volver al inicio | Volver al inicio | T3 | Sin empezar |
| En paralelo | T9. Bajo nivel en el robot real | Robot real | Nada | Parcial, sin mergear |
| En paralelo | T10. Cámara en el robot real | Robot real | T6 | Parcial, sin mergear |

Los tickets conservan su número para no confundir al equipo: el orden de trabajo es el de la tabla.
T3 y T4 van juntos: la libreta de Mapeo no sirve sin alguien que la use, y Exploración no puede
funcionar sin la libreta.

El detalle de cada rama y las pruebas hechas están en [ESTADO_DEL_CODIGO.md](ESTADO_DEL_CODIGO.md).

**Antes de seguir:** mergear `feature/claude/vision` a `main` y después `main` a
`feature/ros/navigation`. Sin ese paso, las ramas no se pueden probar juntas.

## Cómo se conectan los nodos

```text
          cámara                          ultrasonidos y encoders
             │                                      │
             ▼                                      ▼
          Visión                               Bajo nivel ◄──── /cmd_vel ────┐
             │ cubos y flechas                      │ /odom y distancias     │
             ▼                                      ├───────────┐            │
          Decisión                                  ▼           ▼            │
   "qué objetivo tengo"                          Mapeo ◄──► Exploración      │
             │                                 "dónde ya        │            │
             │                                  estuve"         │            │
             │ qué color o dirección seguir                     │            │
             └──────────────────────────────────────────────────┤            │
                                                                ▼            │
                                                         Navegación alta ────┘
                                                       "moverse sin chocar"
```

Cada nodo responde una sola pregunta:

| Nodo | Pregunta que responde | Comparación |
|---|---|---|
| Visión | ¿Qué estoy viendo? | Los ojos |
| Decisión | ¿Qué objetivo tengo? | El capitán del equipo |
| Exploración | ¿Por qué pasillo sigo? | Quien recorre un edificio y prueba puertas |
| Mapeo | ¿Por dónde ya pasé? | Una libreta |
| Navegación alta | ¿Cómo llego ahí sin chocar? | Quien maneja |
| Bajo nivel | ¿Cuánto gira cada motor? | Los pies |

## Cómo funciona la exploración

Es lo que usa el robot cuando no hay una flecha de su color. Es como recorrer un edificio
desconocido con una libreta: en cada lugar con varios caminos, anotás cuáles probaste. Los pasos son:

1. Avanzar por el pasillo hasta encontrar una curva, una bifurcación o un callejón.
2. Si es una bifurcación nueva, anotarla con sus salidas.
3. Si Decisión indica una salida, tomar esa.
4. Si no, tomar una salida que todavía no se probó.
5. Si es un callejón, o ya no quedan salidas sin probar, volver a la última bifurcación con opciones.

Repite esto hasta que Decisión le pide parar, por ejemplo porque apareció un cubo.

Con una isla pasa esto:

```text
   entrada ──► A ───────► B
               ▲          │
               │  (isla)  │
               └───── C ◄─┘
                      │
                      ▼
                  camino sin probar
```

El robot sale de A, pasa por B y C, y vuelve a A. La libreta dice que ya estuvo ahí, así que no repite
la vuelta: vuelve a C y prueba el camino que quedó pendiente. Seguir una pared no alcanza, porque el
robot no tiene forma de darse cuenta de que está dando vueltas.

**Cómo sabe que ya estuvo:** compara su posición de `/odom` con los lugares anotados. Si está cerca de
uno y ve salidas parecidas, es el mismo lugar. La tolerancia se ajusta probando.

**Riesgo principal:** `/odom` acumula error, sobre todo con 4 ruedas que patinan al doblar. Si el error
es grande, el robot puede creer que un lugar es nuevo y dar una vuelta de más.

## Tickets

Los nombres de topics y acciones son propuestas, a confirmar entre los equipos.

### T1. Base de simulación

- **Objetivo:** tener el robot moviéndose en Gazebo y midiendo con sus 3 ultrasonidos.
- **Entradas:** `/cmd_vel`.
- **Salidas:** `/odom`, `/tf`, `/distancias` y 3 topics `sensor_msgs/Range`.
- **Terminado cuando:**
    - [x] El robot se mueve por el laberinto de prueba con el teclado.
    - [ ] Cada ultrasonido publica una distancia parecida a la real.
    - [x] Hay un laberinto con al menos una isla y pasillos de distintos anchos, desde 40 cm.
    - [x] `vision_testground` tiene cubos de 20 mm, flechas, islas, obstáculos y una zona roja, con las medidas reales.
    - [x] El robot tiene cámara simulada y `huggin_vision` funciona con ella.
    - [x] La simulación usa las medidas del robot real: largo 0,27 m, ancho 0,16 m, separación entre ruedas 0,135 m, radio 0,0325 m y 0,125 m entre ejes.

El laberinto con isla y los ultrasonidos simulados ya se probaron con el teclado en Gazebo. Falta
calibrar los ultrasonidos con el robot real.

`feature/ros/navegacion-alta` tiene la simulación unida, la visión nueva y `vision_testground` con las medidas reales. Se probó sin ventanas: el robot se mueve, publican los ultrasonidos y la cámara, y pasa al costado de los obstáculos. El laberinto es un ejemplo y no se precarga en el robot.

### T2. Navegación alta

- **Objetivo:** ofrecerles a los demás nodos órdenes simples para moverse por el laberinto: seguir el
  pasillo, girar y avisar qué lugar encontró.
- **Entradas:** una acción `/mover` (seguir el pasillo, girar o ir a un punto, con el largo de la flecha si hay una), `/odom` y ultrasonidos.
- **Salidas:** `/cmd_vel`, el resultado de la orden (terminada, bloqueada o cancelada) y el tipo de
  lugar detectado: pasillo, curva, bifurcación o callejón.
- **Terminado cuando:**
    - [ ] Gira 90 grados dentro del laberinto sin chocar.
    - [ ] Detecta curvas, bifurcaciones y callejones en pasillos de distintos anchos.
    - [ ] Usa el largo de la flecha para anticipar la próxima curva y ajustar su velocidad.
    - [ ] No pisa las zonas rojas de 40 × 40 cm. De cerca salen de la imagen, así que hay que recordar dónde estaban.

Una salida se detecta cuando un sensor pasa a medir mucho más que antes, no con una distancia fija.

Centrarse en el pasillo y frenar ante un obstáculo pasaron a Bajo nivel (T9), según lo acordado
con el equipo.

**Primera tarea:** probar Nav2 sin mapa en el laberinto simulado. Si marca los pasillos como
bloqueados, usar control por ultrasonidos. Los demás nodos no cambian, porque solo conocen `/mover`.

**Resultado de la primera tarea (4/11/2026): Nav2 descartado.** Probado en Gazebo con
`nav2_sin_mapa.launch.py` en `vision_testground`:

- Con 3 ultrasonidos el costmap queda casi vacío: las paredes inclinadas no devuelven eco y en
  las esquinas no se ve casi nada. Con `allow_unknown: true` la ruta atraviesa las paredes que no
  vio, y el robot choca en las esquinas de la isla. No es un parámetro: Nav2 necesita un mapa
  detallado y saber dónde está en él (AMCL), y las dos cosas piden un LiDAR.
- Tampoco sirve para volver con un mapa: para eso está la libreta de T3. Dijkstra corre sobre su
  grafo de lugares y el control de T2 recorre el camino. Las paredes no se guardan, se miden al
  avanzar.
- Se encontró que la odometría exageraba los giros en un 50 % (decía 166° cuando el robot giró
  108°), porque con 4 ruedas el robot patina al girar. En simulación se corrigió con
  `wheel_separation_efectiva = 0.21` en `robot_description/urdf/common_properties.xacro`, y
  quedó 165° contra 161°. El robot real necesita su propio valor: girarlo una vuelta según la
  odometría y medir cuánto giró de verdad.
- Falla en `nav2_sin_mapa.yaml`, que queda sin corregir porque ya no se usa: la capa de
  ultrasonidos se marca desactualizada al arrancar y el controlador no se mueve hasta borrar el
  costmap.

**Sigue:** control por ultrasonidos, en pasos: 1) avanzar por el pasillo (Bajo nivel lo mantiene
centrado), 2) detectar curva, bifurcación o callejón, 3) girar 90° con la odometría, 4) juntarlo
en la acción `/mover`. T2 no elige hacia dónde doblar: avisa qué lugar encontró y Decisión elige
según las flechas de visión.

**Detección de lugares (acordado el 5/11/2026):** va en `prometeo_navigation/lugares.py`.

- **Costado abierto:** un costado está abierto cuando su ultrasonido pega un salto respecto del
  promedio de ese costado, no cuando pasa una distancia fija. Cada costado tiene su propio
  promedio, porque el robot no siempre va justo en el medio. Solo se suman las lecturas de
  cuando está cerrado.
- **Bajón:** si un costado baja de golpe (el mismo salto, pero para abajo), apareció una pared
  nueva y su promedio se borra y arranca de cero. Así, después de girar, el lado que mira al
  pasillo de donde venía se arregla solo. Pendiente: si justo después de girar hay una salida,
  ese lado arranca el promedio con la salida y no la ve. Revisarlo con una grabación de Gazebo.
- **F0 tiene que ser un número:** si al confirmar el frente es `None` o `inf`, el robot espera
  con el costado confirmado hasta tener una medida, porque con `inf` F0 / 2 sería infinito.
- **Confirmación:** el salto tiene que repetirse en N lecturas seguidas (parámetro) para no
  confundirse con una lectura mala. Si el frente está muy cerca, no se espera la confirmación.
- **Puerta o esquina:** cuando se abre un costado, se anota la medida del frente (F0) y el robot
  sigue avanzando.
    - Si el costado se vuelve a cerrar, era una **puerta** (`bifurcacion`): el pasillo sigue
      derecho y tiene una salida al costado. Se anota el frente en ese momento (F1).
    - Si el costado sigue abierto y el frente llega a F0 / 2, es una **esquina** (`curva`). Ahí
      el robot ya está centrado para girar.
    - Si se abren los dos costados, es una **T** (`bifurcacion`, salidas `izquierda` y
      `derecha`). Se trata igual que una esquina: avanza hasta que el frente mida F0 / 2.
- **Doblar en una puerta:** cuando se confirma la puerta, el robot ya pasó la abertura. Si
  Decisión elige doblar, retrocede despacio hasta que el frente mida (F0 + F1) / 2, que es el
  medio de la salida, y recién ahí gira. Usa el ultrasonido de adelante, no la odometría, así
  que no le afecta que patinen las ruedas.
- **Cubo adelante:** si Visión ve un cubo, el frente que mide el ultrasonido es el cilindro, no
  una pared. Eso no es un callejón, y F0 y F1 dejan de servir.

Cuidados:

1. **El ultrasonido tiembla:** para retroceder se usa el promedio de varias lecturas y un margen
   (parámetro). Por ejemplo, frena entre 102 y 108 cm en vez de exactamente en 105.
2. **Al retroceder se puede torcer:** en la abertura hay pared de un solo lado y el centrado no
   ayuda del todo. Mirarlo en Gazebo. En el robot real, si se tuerce, bajar la velocidad de
   retroceso (parámetro) para que patine menos.
3. **El frente tiene que ser la misma pared:** si aparece un cubo entre F0 y F1, la cuenta no
   vale (ver "Cubo adelante").
4. **Lecturas viejas:** cada ultrasonido llega por separado. Si uno deja de mandar (por ejemplo,
   se corta el WiFi de la ESP32), su última lectura no se usa más: pasa a "no se sabe" con
   `lectura_vigente()` y `edad_maxima_s` (parámetro). Esto va en el nodo de T2, no en el
   provisorio, que se borra.
5. **Un mensaje por vez:** el nodo de T2 usa el executor por defecto, que atiende los callbacks
   de a uno. No pasarlo a `MultiThreadedExecutor` sin revisar los contadores, porque dos
   lecturas podrían sumar a la vez (condición de carrera).

**Aviso de lugar (acordado el 4/11/2026):** cada vez que T2 encuentra una curva, una
bifurcación o un callejón, publica en el topic `/navegacion/lugar`. Lo escuchan Mapeo (T3) y
Decisión (T7). Lleva:

| Campo | Ejemplo | Qué es |
|---|---|---|
| `tipo` | `bifurcacion` | `curva`, `bifurcacion` o `callejon` |
| `x`, `y` | `1.20`, `0.40` | Dónde estaba el robot, en metros, según `/odom` |
| `orientacion` | `0.0` | Hacia dónde miraba, en radianes, según `/odom` |
| `salidas` | `izquierda`, `adelante` | Por dónde se puede seguir, desde el robot. No incluye por dónde vino |

T2 manda las salidas desde su punto de vista y T3 las pasa a norte, sur, este y oeste para la
libreta. **Norte no es el norte real:** es hacia donde miraba el robot al arrancar (orientación 0
en `/odom`). Oeste es 90°, sur 180° y este 270°. T3 redondea al múltiplo de 90° más cercano,
porque la odometría se desvía al girar.

**Mejora posible, para después:** usar la cámara para ver paredes buscando la línea donde la
pared toca el piso, con la misma cuenta que `camera_model.py` usa para las zonas. No salva a Nav2
(ve 60° hacia adelante y no resuelve la localización), pero puede ayudar al control a anticipar
una pared o un callejón.

| Cuerpo de la flecha | `length_class` | Tramos de pared | Pared recta hasta la curva | Entre centros de curvas (pasillo de 40 cm) |
|---|---|---|---|---|
| 30 mm | `close` | 1 | Unos 32 cm | Unos 72 cm |
| 60 mm | `normal` | 2 | Unos 64 cm | Unos 104 cm |
| 100 mm | `far` | 3 o más | Desde unos 96 cm | Desde unos 136 cm |

Un segmento es un tramo de pared de 32 cm. Entre centros de curvas se suma el ancho del pasillo. `vision_testground` ya usa estas medidas. T2 debe tener largo de tramo y ancho de pasillo como parámetros.

Es una referencia y puede tener error: `far` es solo un mínimo. Sirve para ajustar la velocidad, pero la curva se confirma con los ultrasonidos. La punta mide lo mismo en todas las flechas, así que lo que distingue cada largo es el cuerpo.

**A tener en cuenta:** las zonas llegan por `/vision/detections`. El firmware ya tiene un freno de
emergencia propio: revisar que no frene de más en los pasillos angostos.

### T3. Mapeo: libreta del recorrido

- **Objetivo:** guardar por dónde pasó el robot y qué caminos probó. No hay ningún mapa precargado: la libreta empieza vacía en cada ronda.
- **Entradas:** `/odom`, ultrasonidos y los lugares que detecta Navegación alta, por `/navegacion/lugar` (ver T2).
- **Salidas:** un grafo de lugares, con su posición y el estado de cada salida.
- **Terminado cuando:**
    - [ ] Registra cada curva, bifurcación y callejón.
    - [ ] Marca las salidas probadas y las pendientes.
    - [ ] Reconoce un lugar al volver a pasar, por ejemplo al rodear una isla.

Un grafo es como un mapa de subte: importan las estaciones y cómo se conectan, no el dibujo exacto.
Con el reglamento actual no se puede sumar LiDAR, por eso conviene esto y no un mapa detallado.

**Estado:** `feature/ros/mapeo` tiene el paquete `prometeo_navigation`, con los datos `Lugar` y
`Salida` y 3 pruebas. Falta agregar lugares, marcar salidas, reconocer un lugar repetido y el nodo.

### T4. Exploración con memoria

- **Objetivo:** encontrar el camino cuando no hay flecha, sin dar vueltas alrededor de una isla.
- **Lógica:** la de [Cómo funciona la exploración](#cómo-funciona-la-exploración).
- **Entradas:** ultrasonidos, `/odom`, la libreta de T3 y órdenes de Decisión.
- **Salidas:** órdenes a `/mover`.
- **Terminado cuando:**
    - [ ] Sale de una isla y prueba los caminos pendientes.
    - [ ] Recorre todo el laberinto de prueba sin chocar.
    - [ ] Se detiene cuando Decisión se lo pide.

### T5. Decisión básica

- **Objetivo:** frenar al ver un cubo y acercarse.
- **Entradas:** `/vision/detections` con `kind` igual a `cube`, en `feature/claude/vision`.
  `/vision/cubes/{color}` queda solo para depuración.
- **Salidas:** orden de acercarse a `/mover` y un estado de misión.
- **Terminado cuando:**
    - [ ] Partiendo sin conocer el laberinto, el robot sigue un color y se detiene frente a su cubo.
    - [ ] Funciona con el cubo en al menos 3 posiciones distintas.
    - [ ] La distancia final le sirve al brazo.

**El cubo está sobre un cilindro negro a la altura del chasis (unos 10,5 cm), según el equipo.
Falta saber el diámetro.** El ultrasonido de adelante (a unos 8 cm del piso) va a ver el cilindro:
- Sirve para medir cuánto falta para llegar al cubo.
- Navegación alta no debe tomarlo como un callejón: si Visión ve un cubo adelante, lo que mide el
  frente es el cilindro, y Decisión toma el control para acercarse.
- El freno de emergencia lo va a frenar: hay que usar el modo recoger (`/seguridad_habilitada` en
  false) para acercarse.

Tampoco debe empujar el cilindro.

**Riesgo de visión:** con la cámara simulada mirando derecho, el cubo se ve desde unos 0,5 m y sale
de la imagen a unos 0,27 m de la cámara. Los últimos 14 cm hasta el cubo el robot no lo ve. Hay
que inclinar la cámara o resolver ese tramo de otra forma, y medirlo en el robot real.

### T6. Flechas en visión

- **Objetivo:** que Decisión reciba las flechas con su color, dirección y largo.
- **Entradas:** `/camera/image_raw` y `/camera/camera_info`.
- **Salidas:** `/vision/detections`, con `kind` igual a `arrow`, `color`, `direction` (`right`, `up`,
  `left`, `down` o `UNK`), `length_class` (`close`, `normal` o `far`), `angle` y `confidence`.
- **Terminado cuando:**
    - [x] Detecta flechas por su forma y las publica. Hecho en `feature/claude/vision`.
    - [x] De frente, clasifica los 3 largos entre 0,75 y 2 m, usando la proporción largo/cabeza.
    - [x] Cada flecha visible se publica como una detección propia.
    - [ ] `feature/claude/vision` está mergeada a `main`.
    - [x] Detecta flechas cercanas. Hoy se descartan como zonas a menos de 0,5 o 0,75 m. Corrección probada en una copia: en `detect_blobs`, mirar la forma antes que el tamaño. En `vision_testground`, las 11 flechas se leen bien a 39 y 52 cm de la cámara; a 30 cm fallan 2 azules bajas y a 70 cm falla 1; sin la corrección, ninguna entre 10 y 39 cm. Más cerca, las flechas bajas se salen de la imagen.
    - [ ] Da una dirección con flechas inclinadas hasta ±30 grados. Hoy `arrow_tolerance_deg` está en 15.
    - [ ] Se decide qué hacer con flechas vistas de costado. El largo falla desde unos 30 grados.
    - [x] Funciona en Gazebo con flechas habilitadas.

- **Medidas reales:** cabeza de 70 × 50 mm, cuerpo de 30 mm de ancho, pared de 20 cm y pasillo de 40 cm como mínimo. `vision_testground` ya las usa, igual que `arrow.py`.

El detector está hecho, pero no está en `main`: esa rama sigue con la versión vieja, que solo publica
cubos. Falta corregir los casos de arriba y probarlo fuera de las pruebas sintéticas.

### T7. Decisión por colores

- **Objetivo:** elegir un color y seguirlo hasta el cubo.
- **Entradas:** `/vision/detections` y el lugar detectado por Navegación alta.
- **Salidas:** órdenes a `/mover` y estado de la misión.
- **Terminado cuando:**
    - [ ] Pasa pruebas sin simulador para cada caso de la lógica de abajo.
    - [ ] Ignora las flechas de los cubos ya encontrados.
    - [ ] En simulación, sigue un color por varias bifurcaciones hasta llegar a su cubo.

**Primera tarea:** definir cómo se traduce una flecha en una salida. Si está de frente, la propuesta es: `left`, salida izquierda; `right`, salida derecha; `up`, seguir derecho. Por ahora solo se detectan flechas de frente, con la cámara fija.

### T8. Volver al inicio

- **Objetivo:** volver a la entrada después de cada cubo y depositarlo en el contenedor de su color.
- **Entradas:** la libreta de T3.
- **Salidas:** órdenes a `/mover`.
- **Terminado cuando:**
    - [ ] Vuelve a la entrada desde cualquier punto del laberinto de prueba y deja el cubo en su contenedor.

La libreta permite buscar el camino más corto de vuelta, sin repetir callejones.

**Recomendación:** girar y volver de frente. Los 3 ultrasonidos están adelante, así que en marcha atrás
el robot va a ciegas. Lo que falta medir es si puede girar en todos los lugares: en un pasillo de 40 cm
le quedan en teoría 4,6 cm por lado, sin contar el brazo.

### T9. Bajo nivel en el robot real

- **Objetivo:** que el robot real ofrezca los mismos topics que el simulador.
- **Entradas:** `/cmd_vel`, en m/s y rad/s.
- **Salidas:** `/odom`, TF `odom → base_footprint` y ultrasonidos como `sensor_msgs/Range`.
- **Terminado cuando:**
    - [ ] `main.c` deja de tratar `/cmd_vel` como valores de -1 a 1.
    - [ ] Los encoders de `driver_encoders` están mergeados.
    - [ ] Se sabe para qué lado gira cada rueda.
    - [ ] `/odom` publica la distancia recorrida con poco error.
    - [x] `/distancias` se convierte en un `Range` por sensor. Hecho en `feature/ros/navigation`, falta probarlo con el robot real.
    - [ ] Están medidas la separación entre ruedas, el diámetro de rueda y la masa del robot.
    - [ ] Sigue un pasillo sin tocar las paredes, sin importar su ancho.
    - [ ] Frena si aparece un obstáculo adelante.

Para centrarse compara la distancia a cada pared, sin suponer un ancho fijo. Estos dos criterios
eran de T2 y los toma Bajo nivel.

**En simulación:** mientras la ESP32 no se centre sola, `ros/src/provisorio_bajo_nivel_sim/`
imita el centrado y el freno en Gazebo, para poder probar T2. El freno es una copia de
`micro_ros/main/seguridad.c`. Es provisorio: cómo borrarlo está en su `LEEME_BORRAR.md`.

**Estado de los encoders:** `driver_encoders` tiene 2 sensores de un solo canal y publica
`/velocidad_ruedas`, siempre positiva. Con eso no se distingue adelante de atrás, y todavía no hay
`/odom`. Una opción es tomar el sentido de la orden de `/cmd_vel`. El código usa una rueda de 65 mm
sin medir, y el modelo 3D da 69 mm.

### T10. Cámara en el robot real

- **Objetivo:** que `huggin_vision` funcione con la imagen de la ESP32-CAM.
- **Entradas:** el video de la ESP32-CAM y `camara/cmd`.
- **Salidas:** `/camera/image_raw`, `/camera/camera_info` y `/vision/detections`.
- **Terminado cuando:**
    - [ ] `feature/esp32-camara` está mergeada.
    - [ ] La cámara está calibrada y se publica `/camera/camera_info`.
    - [ ] Están medidos altura, posición e inclinación de la cámara, y cargados en `vision.yaml`.
    - [ ] El ángulo del servo coincide con `pan_center_deg` y `pan_sign`.
    - [ ] Reconoce los 3 colores con la luz real. El rojo puede necesitar un segundo rango HSV.

**Bloqueante:** ninguna rama publica `camera_info` para la cámara real, y el nodo de visión descarta
las imágenes sin ese dato. Sin esto no detecta nada, aunque reciba video.

## Lógica de decisión

En cada bifurcación del camino hay al menos una flecha. Las islas no tienen flechas: en su entrada decide Exploración. Junto con los ultrasonidos, es lo que permite decidir por dónde seguir, así que T6 es necesario para llegar al cubo. Puede haber una sola, y puede no ser de `color_actual`: en ese caso decide Exploración, que sigue siendo parte del mínimo.

El robot toma una flecha y sigue ese color. El largo no sirve para elegir: indica la distancia a la
próxima curva, no al cubo, así que tomar la flecha más corta no acerca al cubo. Decisión se lo pasa
a Navegación alta en la orden `/mover`, y esta lo usa para ajustar `/cmd_vel`.

```text
cubos_faltantes = rojo, verde y azul
color_actual = ninguno

en cada lugar con flechas:
    flechas_utiles = flechas cuyo color está en cubos_faltantes

    si color_actual es ninguno y hay flechas_utiles:
        color_actual = color de cualquiera de ellas

    si hay una flecha de color_actual:
        seguir esa flecha y pasarle su largo a Navegación alta

en una bifurcación sin flecha de color_actual:
    Exploración elige una salida sin probar
    color_actual no cambia

al encontrar un cubo:
    acercarse
    sacar su color de cubos_faltantes
    color_actual = ninguno
```

En las curvas sin bifurcación no hace falta decidir nada, porque hay una sola salida.

Suposiciones que hay que confirmar:

- Si hay varias flechas, se elige cualquiera entre los colores que faltan.
- Para el mínimo, el robot se detiene en el primer cubo que encuentra, aunque no sea del color elegido.

## Preguntas abiertas

| Pregunta | Por qué importa | Quién responde |
|---|---|---|
| ¿Las flechas de las bifurcaciones quedan siempre de frente al robot? | Se entiende que sí. Define cómo busca la flecha y cómo la traduce en una salida. | Organización |
| ¿Se va a mover la cámara con el servo? | Si es sí, hay que sumar el servo a la simulación y calibrarlo en el robot real. | Equipo |
| ¿El robot gira y vuelve, o hace marcha atrás? | Define T8. Marcha atrás va sin sensores. | Equipo |
| ¿Quién toma cada ticket? | T1 a T7 forman el mínimo. | Equipo |
| ¿Desde dónde se cuentan los segmentos que indica la flecha? | Sin eso, Navegación alta no sabe cuánto falta para la curva. | Organización |
| ¿Cómo se traduce cada dirección de flecha en una salida? | T7 no puede elegir un camino sin esto. | Equipo |
| ¿Son 2 o 4 encoders? | El código tiene 2. Afecta la calidad de `/odom`. | Bajo nivel |
| ¿Cuánto mide de diámetro el cilindro que sostiene el cubo? | Define a qué distancia lo ve el ultrasonido y hace falta para ponerlo en la simulación. | Organización |
