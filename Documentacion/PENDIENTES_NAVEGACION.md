# Pendientes de la navegación (rama `feature/navegacion-adaptativa`)

Lista para retomar en otra sesión o en otra compu. Actualizada el 2026-11-24.
Primero leer "Decisiones tomadas": explican por qué el código está como está.

---

## Decisiones tomadas (no volver a discutirlas)

- **Todo en un solo lugar, sin números mágicos.** Nada fijo de la pista: el robot la aprende.
  Solo las medidas del robot son fijas, en `my_robot_bringup/config/robot.yaml` (todavía no
  lo usa nadie: lo va a usar el mover nuevo).
- **No asumir NADA del laberinto.** Los cubos pueden estar en una zona abierta o en una
  esquina, no solo en callejones.
- **Llave de una sola vía** (`prometeo_navigation/modos.py`, clase `Llave`):
  MANUAL → AUTO → TERMINADO. Se pasa a automático una sola vez y no se vuelve a manual.
  PARAR es definitivo: termina porque el algoritmo terminó o porque se apretó PARAR.
  Para empezar de nuevo hay que relanzar (hasta que exista `/reiniciar`).
  Los textos están como constantes en `prometeo_interfaces/msg/Modo.msg` (nada de strings sueltos).
  Joystick: B = PARAR al instante; Start mantenido 1 s = automático.
- **La garra se maneja a mano** en el cubo y en el contenedor, con los botones Listo / No pude
  del panel, a través de la acción `/brazo`.
- **La largada es el lugar 0 de la libreta** (`libreta.largada`, fuera de `libreta.lugares`
  para que `buscar()` no la confunda con una curva cercana). `arrancar()` sale por el norte;
  `llegar_a_la_largada()` la usa mapeo. Por ahora, al llegar a la largada: media vuelta y listo.
  Más adelante la cámara tiene que reconocer el contenedor y acercarse (toca Decisión).
- **Una salida queda PROBADA solo al llegar al lugar siguiente** (`_unir`). Si sale y no llega
  (el giro falla, o entra a buscar un cubo y vuelve), sigue PENDIENTE.
- **La IMU solo se usa para girar** (regla del equipo): no para mantener el rumbo en recta ni
  para la odometría.
- **Panel**: sin pestañas. Cámara | cabina (el robot visto desde arriba en su pasillo) |
  controles; abajo mapa | todos los topics. En la cabina gira el autito, no el pasillo, y al
  terminar el giro se endereza. Nada de explicaciones al pasar el mouse: botón "❔ Explicar".

---

## Lo que ya está hecho (en el commit de hoy)

- Paso 0: se sacó todo lo de pasar de manual a automático y viceversa (llave, joystick,
  detector, libreta, mapeo, vuelta y nodo_decision). 184 tests de navegación y 63 de decisión pasan.
- Merge de `origin/decision_basica_colores` (Natasha): falsas alarmas, filtro 4 de 5,
  acción `/brazo`.
- Panel nuevo (`my_robot_bringup/web/panel.html`) conectado y probado con datos de ejemplo.
- Nodos nuevos: `scripts/monitor_topicos.py` (resume todos los topics en `/monitor/topicos`)
  y `scripts/estado_camara.py` (lee `http://192.168.4.1:81/status` de la ESP32-CAM y lo
  publica en `/camara/estado`). Los arranca `panel.launch.py`.

---

## Lo que falta, en orden

### 1. Terminar el paso 0
- [ ] **`/reiniciar`**: el botón del panel ya publica `/prometeo/reiniciar` (Empty), pero
  ningún nodo lo escucha. Hacer que la llave vuelva a MANUAL, que mapeo vacíe la libreta y que
  Decisión arranque de cero. **Toca mapeo: avisarle a Micaela antes.**
- [ ] **`garra_manual.py`**: un nodo que atiende la acción `/brazo` (AGARRAR / SOLTAR →
  LISTO / FALLO) y espera los botones Listo / No pude del panel. Si no contestan en 20 s,
  cuenta como No pude. Se lanza con `sin_brazo:=false`. Micaela dijo que capaz lo escribe ella.
  Después activar los botones de la garra en el panel (hoy están desactivados).
- [ ] **Chequeo antes de largar**: cada nodo avisa que está vivo, el panel lo muestra y la
  llave no deja pasar a AUTO si falta alguno.
- [ ] Sacar `pedido_operador` y `freno_operador` de `gazebo.launch.py` y `autonomia.launch.py`
  (remapeos y topics_glob), y lanzar la llave también en Gazebo.
- [ ] Documentación: `Documentacion/ARQUITECTURA.md` (por ejemplo la línea 220 todavía nombra
  `modo_testeo`).
- [ ] **Mensaje para Natasha** sobre `nodo_decision.py`: salieron freno_operador,
  pedido_operador (Abandonar), olvidar_lo_visto y cambio_de_freno; `puede_decidir` solo mira
  `Modo.AUTO`; el nodo nuevo `terminar_si_no_queda_nada` publica `PEDIR_PARAR` cuando la
  lógica llega a TERMINADO (termina al DEJAR el último cubo, no al agarrarlo). `logica.py` no
  se tocó. Le quedan dos comentarios para corregir en `logica.py`: "el cubo está en un callejón"
  (abandonar_color) y el estado VOLVIENDO de abandonar_color, que estaba solo por la garra a mano.
- [ ] `stash@{0}` tiene un cambio suelto de Micaela en `selector_modo.py` (de antes de esta
  rama). Ver si sigue haciendo falta o borrarlo.

### 2. IMU (MPU6050) en el firmware
- [ ] Driver con los pines expuestos en Kconfig (apagado por defecto). Solo queda libre el
  GPIO15 e I2C necesita dos: propuesta, mover dos encoders a GPIO36/39.
- [ ] Integrar el giroscopio Z a ~200 Hz, calibrar quieto al arrancar y con `imu/calibrar`,
  publicar `imu/angulo` (Float32, grados; el panel ya lo lee en `/prometeo/imu/angulo`).
- [ ] Hace falta subir `MAX_PUBLISHERS` a 7 (hoy usa los 6).
- [ ] La IMU en Gazebo (agregarla al xacro) con el mismo topic.
- ESP-IDF está en `~/.espressif/v5.5.5/esp-idf`. `check_pins.py` da un error falso en GPIO10.

### 3. Rehacer `maniobras.py` y `mover.py` desde cero
El problema de la competencia fueron los giros (sesgo de confirmación de 7 cm, el medio de la
puerta corrido 10 cm, patinaje del skid steer con 70 % de duty). El contrato con los demás:
- [ ] Acción `/mover`: IR_A_SALIDA (con salida), ACERCARSE, VUELTA, y SEGUIR (adelante en
  medio del pasillo). Resultados TERMINADA / BLOQUEADA / CANCELADA.
- [ ] Publicar `/navegacion/reiniciar_detector`, `/navegacion/salida_tomada` y `odom_laberinto`.
- [ ] **Conservar `/navegacion/poner_posicion`**: lo usa mapeo en la largada.
- [ ] Publicar en `cmd_vel_auto` (pasa por la llave).
- [ ] Usar la IMU solo mientras gira; corregir para los dos lados; tolerancia ±5°.
- [ ] Después de ACERCARSE, enderezarse al rumbo cardinal antes de volver (al acercarse al
  cubo queda mirando a cualquier ángulo).
- [ ] Usar la geometría de `robot.yaml`: sensor de adelante a 14,25 cm del centro; los de los
  costados a 7,25 cm al costado y 10,25 cm adelante; ancho del pasillo = izq + der + 14,5;
  punto de giro: frente = ancho/2 − 14,25. Skid steer de 4 ruedas: gira sobre el centro del
  rectángulo de las ruedas.
- [ ] La libreta debería avisar si la orientación queda a más de ~20° de un punto cardinal
  (con la IMU). Hoy `rumbo_de` redondea en silencio.

### 4. Después
- [ ] **Anotar cubos en la libreta**: el cubo confirmado (llegó a acercarse) se registra y se
  usa para decidir. Las falsas alarmas y los cubos vistos de paso se anotan solo para evaluar
  visión, SIN usarlos para decidir. Si visión se confunde poco, después se usan para navegar.
- [ ] Detectar callejones con seguridad (opción 3) para marcar PROBADA antes.
- [ ] Firmware: publicar las 4 ruedas por separado (hoy `/velocidad_ruedas` manda el promedio
  por lado, `main.c:268-275`); en el panel cambiar solo `leerRuedas()`. Opcional: publicar
  fallas y esfuerzo de los puentes.
- [ ] ESP32-CAM: listar las MAC de los conectados (para saber cuáles son). WiFi de respaldo
  si falla el hotspot (toca los dos firmwares).
- [ ] Hardware, cuando haya batería: que giren las 4 ruedas, probar el centro de giro,
  revisar cables y cortos.

---

## Para probar el panel

```
sudo apt install ros-jazzy-rosbridge-suite      # una sola vez
cd ros && colcon build && source install/setup.bash
ros2 launch my_robot_bringup panel.launch.py camara:=false joystick:=false
```

y abrir http://localhost:8082. Teclas: flechas o W A S D para manejar (solo en MANUAL),
J L / I K / U O para el brazo, P para la pinza, E R para el servo de la cámara, F para la
cámara grande y espacio para PARAR.
