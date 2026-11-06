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
| 2 | T2. Navegación alta | Llegar al cubo | T1 | Hecha (detector, `/mover`, acercarse al cubo), sin probar en Gazebo ni en el robot |
| 3 | T3. Mapeo: libreta del recorrido | Llegar al cubo | T2 | Hecha (nodo `mapeo`), sin probar en Gazebo ni en el robot |
| 4 | T4. Exploración con memoria | Llegar al cubo | T3 | Hecha (en el nodo `mapeo`), sin probar en Gazebo ni en el robot |
| 3 y 4, en paralelo | T6. Flechas en visión | Llegar al cubo | Nada | Casi lista, sin mergear |
| 5 | T7. Decisión por colores | Llegar al cubo | T4 y T6 | Hecha (Natasha, `decision_basica_colores`, unida) |
| 6 | T5. Decisión básica | Llegar al cubo | T7 | Hecha (Natasha, `decision_basica_colores`, unida) |
| 7 | T8. Volver al inicio | Volver al inicio | T3 | Hecha (`/volver` en el nodo `mapeo`), sin probar en Gazebo ni en el robot |
| En paralelo | T9. Bajo nivel en el robot real | Robot real | Nada | Parcial, sin mergear |
| En paralelo | T10. Cámara en el robot real | Robot real | T6 | Parcial, sin mergear |
| Antes de probar el robot real | T11. Modo manual de respaldo | Robot real | T2 | Hecho (selector, panel, joystick, pausa para la garra) |

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
- **Mirando alrededor (`MIRANDO_ALREDEDOR`):** es la etapa inicial. Se usa cuando el robot no
  tiene historia de los costados: al empezar la ronda y al volver de manual (T11), porque los
  promedios viejos ya no sirven y puede estar en el medio de una esquina. Sin promedio, compara
  cada costado contra el ancho del pasillo (parámetro), confirmando con N lecturas. Si un lado
  está abierto, arrancó en un lugar: lo avisa una vez y pasa a `AVISADO`. Si no, pasa a
  `EN_PASILLO`. No hace falta después de un giro automático: ahí lo resuelve el bajón.
- **En manual (T11):** el detector no avisa lugares, porque los giros y correcciones a mano
  darían saltos falsos y ensuciarían la libreta.
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

**Giro de `/mover` (6/11/2026):** las ruedas patinan y los encoders no son confiables, así que
el giro se confirma con los ultrasonidos, **con el robot quieto**: girando, una pared en
diagonal no devuelve bien el eco. En una esquina, la pared que estaba adelante tiene que quedar
al costado a la misma distancia, y el frente tiene que cambiar. Si no confirma, gira de a 10°;
si después de 45° de más sigue sin confirmar, contesta `bloqueada` y pide AYUDA en el panel.
Probado con un robot de mentira (`maniobras.py`, `mover.py`).

- [ ] **Pendiente, en el robot real:** ver cuánto queda torcido. En la prueba quedó en 73° en
  vez de 90°, porque confirma apenas el sensor del costado devuelve eco. Ver si el centrado lo
  endereza, y ajustar `tolerancia_costado_cm`, `paso_grados` y `factor_giro_tiempo`.

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

**Estado (6/11/2026):** hecho en `feature/ros/navegacion-alta`. `libreta.py` pasa las salidas a
norte, sur, este y oeste, anota lugares nuevos, reconoce los repetidos (posición con margen y
mismas salidas), marca las salidas probadas, une los lugares (`lleva_a`) y guarda cuántos segundos
tardó cada tramo. El nodo `mapeo` la llena con `/navegacion/lugar` y `/navegacion/salida_tomada`
y la publica en `mapeo/libreta` para el panel. El primer lugar lleva a la largada (`lleva_a = 0`).

**Posición sin odometría (6/11/2026):** las ruedas patinan y el robot real no publica `/odom`.
`/mover` calcula una **posición de laberinto** (`odom_laberinto`): la dirección sale de los giros
que confirmó con los ultrasonidos (cuartos de vuelta justos) y la distancia, del tiempo avanzando
por la velocidad. El detector la usa en vez de `/odom` (`autonomia.launch.py`).

**Manual sin moverse es una pausa (6/11/2026):** Mapeo y `/mover` miran `orden_enviada`. Si en
manual el robot no se movió (por ejemplo, solo movieron la garra), al volver a automático todo
sigue igual. Si se movió, recién ahí la salida queda `trabada` y el tramo no se anota.

**Con el modo manual (T11, acordado el 5/11/2026):**

- La libreta no se borra al pasar a manual ni al volver. `/odom` sigue sumando en manual, así
  que el robot sabe aproximadamente dónde quedó.
- Al pasar a manual, la salida que estaba recorriendo se marca como **`trabada`**, un tercer
  estado además de `pendiente` y `probada`. Dijkstra no la usa, salvo que no haya otro camino.
- Al volver a automático, el robot mira alrededor (T2). Si está cerca de **cualquier** lugar de
  la libreta, se une a ese. Para no confundirse tienen que coincidir dos cosas: la posición según
  `/odom`, con un margen (parámetro), y el parecido, o sea el mismo tipo y las mismas salidas en
  norte, sur, este y oeste.
- Si no está cerca de ninguno, el lugar nuevo queda en **otro grupo**: la libreta queda partida,
  y el tramo hecho a mano es el vacío entre los grupos. El tramo manual no se anota, porque solo
  se conoce por la odometría.

### T4. Exploración con memoria

**Estado (6/11/2026):** hecho en `feature/ros/navegacion-alta`, en el nodo `mapeo`
(`exploracion.py`), cuando Decisión pide `/decision/explorar`. Prueba una salida sin probar; si
no hay, vuelve por BFS al lugar con pendientes más cercano; si no queda nada, media vuelta. Al
**abandonar un color** (botón del panel o 3 intentos fallidos), Decisión publica
`/decision/retomar`: el robot vuelve al último lugar y sigue desde ahí, sin ir a la largada.

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

**Estado (6/11/2026):** hecho en `feature/ros/navegacion-alta`, en el nodo `mapeo` (acción
`/volver`). Dijkstra (`vuelta.py`) usa los segundos de cada tramo, no los metros. Si está en un
pasillo da media vuelta; en cada lugar recalcula el camino. Desde el primer lugar avanza lo que
tardó el primer tramo y **pasa a manual**: el cubo se deja a mano. Al volver a automático la
posición vuelve a (0, 0): hay que dejar el robot como arrancó la ronda. Pendiente: si la libreta
quedó partida, hoy pide AYUDA; falta que el operador diga en el panel en qué lugar dejó el robot.

- **Objetivo:** volver a la entrada después de cada cubo y depositarlo en el contenedor de su color.
- **Entradas:** la libreta de T3.
- **Salidas:** órdenes a `/mover`.
- **Terminado cuando:**
    - [ ] Vuelve a la entrada desde cualquier punto del laberinto de prueba y deja el cubo en su contenedor.

La libreta permite buscar el camino más corto de vuelta, sin repetir callejones.

**Si la libreta quedó partida por el modo manual (acordado el 5/11/2026):**

```text
INICIO ── A ── B ╌╌╌ (manual) ╌╌╌ C ── D ── cubo
└──── grupo 1 ────┘               └─── grupo 2 ───┘
```

1. La vuelta es siempre automática y con Dijkstra.
2. Si el inicio está en el mismo grupo que el robot, Dijkstra calcula el camino directo.
3. Si están en grupos distintos, el robot explora (T4) por salidas pendientes de su grupo hasta
   reconocer un lugar del grupo del inicio (como en T3: cerca y parecido). Ahí se unen los grupos
   y sigue con Dijkstra.
4. La exploración tiene un límite (parámetro, de tiempo o de lugares nuevos). Al pasarlo avisa
   "no encuentro el camino" y se pasa a manual para llevarlo a mano (T11). Sin límite, podría
   quedarse dando vueltas sin que nos demos cuenta.

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

### T11. Modo manual de respaldo

- **Objetivo:** si el robot se traba y no sabe cómo seguir, poder manejarlo a mano y volver a
  automático sin perder lo recorrido.
- **Entradas:** `/cmd_vel_auto` (Navegación alta), `/cmd_vel_manual` (teclado con
  `teleop_twist_keyboard` o joystick con `teleop_twist_joy`, ya instalados) y `/modo`.
- **Salidas:** `/cmd_vel`, el único que escucha la ESP32.
- **Terminado cuando:**
    - [ ] El nodo `selector_modo` deja pasar a `/cmd_vel` solo lo del modo elegido.
    - [ ] Se cambia de modo publicando `auto` o `manual` en `/modo`, y después con un botón o tecla.
    - [ ] En manual, si dejan de llegar órdenes (se cortó la conexión), manda parar.
    - [ ] En manual, el detector de T2 no avisa lugares y Mapeo marca la salida como `trabada`.
    - [ ] Al volver a automático, T2 arranca en `MIRANDO_ALREDEDOR` y Mapeo se ubica en la libreta (T3).

```text
Navegación alta (automático) ──► /cmd_vel_auto   ─┐
                                                   ├─► selector_modo ──► /cmd_vel ──► ESP32
Teclado o joystick (manual)  ──► /cmd_vel_manual ─┘         ▲
                                                             │
                                          /modo: "auto" o "manual"
```

| | Automático | Manual |
|---|---|---|
| `/odom` y sensores | siguen | siguen |
| La libreta y la posición | siguen | siguen |
| El detector avisa lugares | sí | pausado |
| Al volver a automático | — | `MIRANDO_ALREDEDOR` (T2) y ubicarse en la libreta (T3) |

El firmware no cambia: la ESP32 obedece a `/cmd_vel` venga de donde venga. Con el firmware de
`navegacion_bajo` el topic es `/prometeo/cmd_vel`, en m/s y rad/s, así que el selector se corre
con `--ros-args -r __ns:=/prometeo`. **Atención:** en ese firmware el freno de emergencia está
desactivado ("desactivado por ahora" en `cmd_vel_callback`, `micro_ros/main/main.c`), así que en
manual no hay freno: manejar despacio.
**Panel de navegación (hecho el 5/11/2026), funciona sin internet:**

```bash
ros2 launch my_robot_bringup panel.launch.py          # robot real (/prometeo)
ros2 launch my_robot_bringup panel.launch.py ns:=''   # Gazebo
# abrir http://localhost:8082
```

Arranca rosbridge, el selector y la página (`my_robot_bringup/web/panel.html`). Muestra el modo,
los ultrasonidos, la orden que sale a la ESP32, la posición y el rastro según `odom`, los avisos y
el mapa de la libreta. Teclas: **M** manual, **A** automático, **ESPACIO** parar (pánico, también
frena si ya estaba en manual), flechas o W, S, Q, D para manejar. rosbridge no tiene permiso
sobre `cmd_vel`: todo pasa por el selector, que publica una copia en `orden_enviada`.

Lo que el panel espera de los otros nodos (todavía no existen):

- `navegacion/aviso` (`std_msgs/String`): texto libre. Si empieza con `AYUDA`, se muestra en rojo
  arriba de todo. Ejemplo: `AYUDA: trabado en el lugar 4, llevalo hasta el lugar 2`.
- `mapeo/libreta` (`std_msgs/String` con JSON), cada vez que cambia:

```json
{"lugares": [{"numero": 1, "x": 1.2, "y": 0.4, "tipo": "bifurcacion",
              "salidas": [{"direccion": "norte", "estado": "pendiente", "lleva_a": null}]}],
 "actual": 1, "camino": [1, 0]}
```

`estado` es `probada`, `pendiente` o `trabada`. `camino` es el que eligió Dijkstra, de lugar en
lugar. **El robot real todavía no publica `odom`** (solo `velocidad`, estimada): sin eso no hay
posición ni rastro, y Mapeo tampoco puede anotar dónde está cada lugar.

**Pendiente para charlar con Ignacio y Natasha: el freno por obstáculo no funciona** en el
firmware de `navegacion_bajo`. Desde `dc3e843` (4/11), `cmd_vel_callback` en
`micro_ros/main/main.c` ya no llama a `seguridad_filtrar()`, así que la tarea de `seguridad.c`
nunca se entera de la velocidad y no frena. Siguen andando el failsafe de 300 ms y el PARAR del
selector. Arreglo propuesto, sin aplicar: pasar `linear.x` a la escala −1..1, filtrar y volver a m/s
(`seguridad_filtrar(v / vel_max) * vel_max`). Sin peligro da la misma velocidad; con peligro da
0; el giro y el retroceso no se tocan. Riesgos: que frene de más si el frente no mide (por eso
el botón del panel para apagarlo con `seguridad_habilitada`) y que frene a ~4 cm a 0,15 m/s.
Hay que compilarlo y cargarlo en la ESP32. En Cat. B, evitar choques es obligatorio (−15 si
falta).

**Actualización (5/11/2026, tarde):** el panel tiene pestañas (Monitoreo, Manual, Automático,
Motores, Visión, Brazo, Registro con descarga), la cámara siempre visible (`camara_panel.py`,
MJPEG en el puerto 8083, prefiere `/vision/annotated`) y el cartel de quién maneja. **PARAR es un
freno que queda puesto y no cambia el modo**; lo suelta una persona (`soltar`). El selector recibe
tres fuentes: `cmd_vel_auto`, `cmd_vel_manual` (compu) y `cmd_vel_joy` (joystick Xbox con
`game_controller_node` + `teleop_twist_joy` + `joystick_modo`; botones en
`config/joystick_xbox.yaml`). Avisa `modo_actual`, `freno_operador` y `estado_selector`. Los
botones Capturar (pasa a manual: la garra la mueven ellos) y Abandonar (en auto, Decisión vuelve a
un lugar conocido y toma otra flecha) publican en `pedido_operador`. Topics y código para los
demás nodos en `ros/COMANDOS_COMPETENCIA.md`.

**Asistencia de la Cat. B (5/11/2026):** `panel.launch.py asistencia:=true` corre en la compu el
freno por obstáculo y el centrado del nodo provisorio (`centrado_simulado`, con el nombre
`asistencia`) entre el selector y la ESP32, con `config/asistencia_robot_real.yaml`
(`use_sim_time: false`, reacción 0,30 s por el WiFi). Se conecta con remapeos, sin tocar su
código: `selector → cmd_vel_seleccionado → asistencia → /prometeo/cmd_vel`, y avisa en
`/prometeo/freno_asistencia`. Probado con sensores simulados: frena con la pared a 4 cm, deja
retroceder y girar, corrige hacia el medio y no corrige ante una salida. Apagada por defecto hasta
probarla en el robot. Resuelve "evitar choques" y "corrección de trayectoria" sin el freno del
firmware.

Navegación alta deja de publicar en `/cmd_vel` y pasa a `/cmd_vel_auto`. Para la vuelta con la
libreta partida, ver T8.

## Lógica de decisión

Siempre que el robot tiene que doblar hay una flecha, tanto en las curvas como en las bifurcaciones (confirmado el 5/11/2026). Las islas no tienen flechas: en su entrada decide Exploración. Junto con los ultrasonidos, es lo que permite decidir por dónde seguir, así que T6 es necesario para llegar al cubo. Puede haber una sola, y puede no ser de `color_actual`: en ese caso decide Exploración, que sigue siendo parte del mínimo.

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
