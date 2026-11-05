# PROVISORIO: imitación del Bajo nivel en Gazebo

## Qué es

Un nodo que hace en la simulación lo que en el robot real va a hacer la ESP32:

- **Centrarse en el pasillo** comparando los ultrasonidos de la izquierda y la derecha.
- **Frenar ante un obstáculo** con el ultrasonido de adelante, igual que
  `micro_ros/main/seguridad.c`.

```
T2 / teclado ──/cmd_vel──► este nodo ──/cmd_vel_corregido──► Gazebo
```

## Por qué existe

El equipo de Bajo nivel se encarga de que el robot no choque con las paredes (T9 en
`ros/TICKETS_NAVEGACION.md`). En Gazebo no hay ESP32, así que sin este nodo el robot iría derecho
hasta chocar y no se podría probar Navegación alta (T2).

Este nodo **no decide hacia dónde doblar**. Eso lo deciden Decisión (T7) y Exploración (T4).

## Ojo: el freno es una copia

El freno de `provisorio_bajo_nivel_sim/calculos.py` copia `micro_ros/main/seguridad.c`, porque
ese archivo está en C para la ESP32 y no compila en la PC. Si Bajo nivel cambia los números de
`seguridad.c`, hay que actualizarlos acá. El centrado no es copia: todavía no existe en la ESP32.

## Ahora también se usa con el robot real (5/11/2026)

Desde el 5/11 este mismo nodo es la **asistencia de la Categoría B** en el robot real (freno por
obstáculo y centrado, en la compu): `panel.launch.py asistencia:=true` lo arranca con el nombre
`asistencia` y la configuración `my_robot_bringup/config/asistencia_robot_real.yaml`.

**No borrarlo** sin antes reemplazar eso (por ejemplo, con el freno y el centrado en la ESP32),
o sacar la opción `asistencia` de `panel.launch.py`.

## Cuándo borrarlo

Cuando la ESP32 del robot real se centre sola y T9 esté terminado.

## Cómo borrarlo

1. Borrar esta carpeta: `ros/src/provisorio_bajo_nivel_sim/`.
2. Buscar las líneas marcadas y deshacerlas:
   ```bash
   grep -rn PROVISORIO ros/
   ```
   En `my_robot_bringup/config/bridge.yaml`, `/cmd_vel_corregido` vuelve a ser `/cmd_vel`.
3. Compilar de nuevo: `colcon build`.
