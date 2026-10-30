# Tickets de navegación autónoma

El robot sigue flechas de un color hasta el cubo y explora cuando no hay una flecha de ese color.
Asumimos que no todas las bifurcaciones tienen flecha de cada color, así que Exploración con memoria
forma parte del mínimo. También aporta la libreta que hace falta para volver.

**Mínimo actual:** llegar a un cubo. Agarrarlo, depositarlo y volver quedan para después.

**Criterio de diseño:** el laberinto está hecho a mano, así que las medidas son aproximadas. Los
pasillos miden 40 cm como mínimo y no tienen un máximo conocido. Por eso, el robot se guía por lo
que miden los sensores y no por distancias fijas.

## Orden de trabajo

| Orden | Ticket | Hito | Depende de |
|---|---|---|---|
| 1 | T1. Base de simulación | Llegar al cubo | Nada |
| 2 | T2. Navegación alta | Llegar al cubo | T1 |
| 3 | T3. Mapeo: libreta del recorrido | Llegar al cubo | T2 |
| 4 | T4. Exploración con memoria | Llegar al cubo | T3 |
| 3 y 4, en paralelo | T6. Flechas en visión | Llegar al cubo | Nada |
| 5 | T7. Decisión por colores | Llegar al cubo | T4 y T6 |
| 6 | T5. Decisión básica | Llegar al cubo | T7 |
| 7 | T8. Volver al inicio | Volver al inicio | T3 |
| En paralelo | T9. Bajo nivel en el robot real | Robot real | Nada |

Los tickets conservan su número para no confundir al equipo: el orden de trabajo es el de la tabla.
T3 y T4 van juntos: la libreta de Mapeo no sirve sin alguien que la use, y Exploración no puede
funcionar sin la libreta.

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
- **Salidas:** `/odom`, `/tf` y 3 topics `sensor_msgs/Range`.
- **Terminado cuando:**
    - [ ] El robot se mueve por el laberinto de prueba con el teclado.
    - [ ] Cada ultrasonido publica una distancia parecida a la real.
    - [ ] Hay un laberinto con al menos una isla y pasillos de distintos anchos, desde 40 cm.
    - [ ] Ese laberinto tiene cubos de 20 mm, y el robot tiene cámara simulada.

Ya están armados el robot, un primer laberinto y el puente, pero falta probarlos en la PC principal.

### T2. Navegación alta

- **Objetivo:** ofrecerles a los demás nodos órdenes simples para moverse sin chocar.
- **Entradas:** una acción `/mover` (seguir el pasillo, girar o ir a un punto), `/odom` y ultrasonidos.
- **Salidas:** `/cmd_vel`, el resultado de la orden (terminada, bloqueada o cancelada) y el tipo de
  lugar detectado: pasillo, curva, bifurcación o callejón.
- **Terminado cuando:**
    - [ ] Sigue un pasillo sin tocar las paredes, sin importar su ancho.
    - [ ] Gira 90 grados dentro del laberinto sin chocar.
    - [ ] Frena si aparece un obstáculo adelante.
    - [ ] Detecta curvas, bifurcaciones y callejones en pasillos de distintos anchos.

Para centrarse compara la distancia a cada pared, sin suponer un ancho fijo. Una salida se detecta
cuando un sensor pasa a medir mucho más que antes, no con una distancia fija.

**Primera tarea:** probar Nav2 sin mapa en el laberinto simulado. Si marca los pasillos como
bloqueados, usar control por ultrasonidos. Los demás nodos no cambian, porque solo conocen `/mover`.

### T3. Mapeo: libreta del recorrido

- **Objetivo:** guardar por dónde pasó el robot y qué caminos probó.
- **Entradas:** `/odom`, ultrasonidos y los lugares que detecta Navegación alta.
- **Salidas:** un grafo de lugares, con su posición y el estado de cada salida.
- **Terminado cuando:**
    - [ ] Registra cada curva, bifurcación y callejón.
    - [ ] Marca las salidas probadas y las pendientes.
    - [ ] Reconoce un lugar al volver a pasar, por ejemplo al rodear una isla.

Un grafo es como un mapa de subte: importan las estaciones y cómo se conectan, no el dibujo exacto.
Con el reglamento actual no se puede sumar LiDAR, por eso conviene esto y no un mapa detallado.

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
- **Entradas:** `/vision/cubes/{color}`, que ya existe en `feature/ros/vision_node`.
- **Salidas:** orden de acercarse a `/mover` y un estado de misión.
- **Terminado cuando:**
    - [ ] Partiendo sin conocer el laberinto, el robot sigue un color y se detiene frente a su cubo.
    - [ ] Funciona con el cubo en al menos 3 posiciones distintas.
    - [ ] La distancia final le sirve al brazo.

**Riesgo:** el cubo mide 2 cm y el ultrasonido frontal está a unos 8,5 cm del piso, según el
modelo. Probablemente no lo vea, así que frenar a tiempo depende de la cámara. Tampoco debe
pasarlo por encima ni empujarlo.

### T6. Flechas en visión

- **Objetivo:** publicar las flechas que hoy el código detecta pero no envía.
- **Entradas:** `/camera/image_raw`.
- **Salidas:** `/vision/flechas`, con color, dirección y tipo de largo: corta, media o larga.
  El largo indica la distancia a la próxima curva.
- **Terminado cuando:**
    - [ ] Clasifica los 3 largos a distintas distancias, usando la proporción cuerpo/punta.
    - [ ] Reconoce flechas inclinadas hasta ±30 grados. Hoy `arrow_tolerance_deg` está en 15.
    - [ ] Publica varias flechas si se ven juntas en una bifurcación.

### T7. Decisión por colores

- **Objetivo:** elegir un color y seguirlo hasta el cubo.
- **Entradas:** `/vision/flechas`, el lugar detectado por Navegación alta y `/vision/cubes/{color}`.
- **Salidas:** órdenes a `/mover` y estado de la misión.
- **Terminado cuando:**
    - [ ] Pasa pruebas sin simulador para cada caso de la lógica de abajo.
    - [ ] Ignora las flechas de los cubos ya encontrados.
    - [ ] En simulación, sigue un color por varias bifurcaciones hasta llegar a su cubo.

### T8. Volver al inicio

- **Objetivo:** salir del laberinto después de llegar al cubo.
- **Entradas:** la libreta de T3.
- **Salidas:** órdenes a `/mover`.
- **Terminado cuando:**
    - [ ] Vuelve a la zona de inicio desde cualquier punto del laberinto de prueba.

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
    - [ ] Los encoders están mergeados y `/odom` publica la distancia recorrida con poco error.
    - [ ] `/distancias` se convierte en un `Range` por sensor.
    - [ ] Están medidas la separación entre ruedas y la masa del robot.

## Lógica de decisión

Por ahora, el robot toma una flecha y sigue ese color. Más adelante se puede optimizar: elegir la
flecha más corta no sirve, porque el largo indica la distancia a la próxima curva y no al cubo.

```text
cubos_faltantes = rojo, verde y azul
color_actual = ninguno

en cada lugar con flechas:
    flechas_utiles = flechas cuyo color está en cubos_faltantes

    si color_actual es ninguno y hay flechas_utiles:
        color_actual = color de cualquiera de ellas

    si hay una flecha de color_actual:
        seguir esa flecha

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
| ¿Hay una flecha de cada color en todas las bifurcaciones? | Asumimos que no. Si la respuesta es sí, Exploración deja de ser parte del mínimo. | Organización |
| ¿El robot gira y vuelve, o hace marcha atrás? | Define T8. Marcha atrás va sin sensores. | Equipo |
| ¿Quién toma cada ticket? | T1 a T7 forman el mínimo. | Equipo |
