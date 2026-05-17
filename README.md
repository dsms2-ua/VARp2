# Práctica 2 — Carrera de Robots

Sistema de navegación autónoma (VFH+) + detección de objetos (YOLOv8) para un TurtleBot3 Waffle en Gazebo.

---

## Arranque del entorno

Levanta el contenedor con todo dentro:

```bash
./run.sh
```

Las demás instancias con:
```bash
./connect.sh
```

Esto hace `docker compose up -d --build` y deja una shell dentro del contenedor con el workspace ya compilado.

---

## Lanzar la carrera (flujo principal)

```bash
ros2 launch vfh_navigation navigation.launch.py
```

Este launch arranca **todo a la vez**: Gazebo + SLAM + nodo VFH+ + métricas + grabador de rastro + visualización del histograma + RViz2, y coloca las ventanas en pantalla.

> No usar `ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py` directamente para la carrera, ya que solo arranca Gazebo sin navegación.

---

## Grabar waypoints (solo si se cambia el circuito)

```bash
# Terminal 1 — solo Gazebo
ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py

# Terminal 2 — teleop + grabador
ros2 run vfh_navigation waypoint_recorder
```

Controles dentro de `waypoint_recorder`:

| Tecla | Acción |
|---|---|
| W / S | acelerar / frenar |
| A / D | girar izq / der |
| Espacio | parar |
| M | guardar waypoint en la pose actual |
| Z | deshacer último waypoint |
| P | listar waypoints actuales |
| Q | guardar YAML y salir |

Salida: `ros2_ws/vfh_navigation/waypoints/circuit_waypoints.yaml`.

---

## Detección de objetos (YOLOv8)

Nodo independiente que se suscribe a `/camera/image_raw`, corre YOLOv8n y republica la imagen anotada en `/detections/image`.

```bash
ros2 launch vfh_navigation navigation.launch.py

ros2 run detector_pkg detector_node

# Ver la imagen anotada
ros2 run rqt_image_view rqt_image_view /detections/image
```

> La primera ejecución descarga `yolov8n.pt` desde Ultralytics

---

## Métricas en tiempo real

```bash
ros2 topic echo /lap/metrics --truncate-length 500
```

Devuelve JSON con tiempo de vuelta, velocidad media/máxima, distancia mínima a obstáculo, maniobras de esquiva y recuperaciones.

---

## Topics útiles

| Topic | Función |
|---|---|
| `/cmd_vel` | Comandos de velocidad publicados por VFH |
| `/scan` | LiDAR 360° |
| `/odom` | Odometría |
| `/map` | Mapa de SLAM |
| `/vfh/histogram` | Histograma polar VFH |
| `/vfh/steering` | Dirección elegida por VFH |
| `/lap/path` | Rastro acumulado de la vuelta |
| `/lap/metrics` | Métricas JSON |
| `/detections/image` | Imagen con bounding boxes |
