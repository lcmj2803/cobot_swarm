# Guía: simulación líder-seguidor en Gazebo

Workspace: `~/cobot_swarm/cobots_hero_swarm` · ROS 2 Humble · Gazebo Classic

Se usan 4 terminales. En **todas**, primero:

```bash
cd ~/cobot_swarm/cobots_hero_swarm
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Si cambiaste código, antes compila con `colcon build` y vuelve a hacer el `source install/setup.bash`.

---

## Paso 0 — Limpiar lo anterior

Cierra con Ctrl+C todo lo que esté corriendo. Si Gazebo no abre o se queda congelado, mata los procesos viejos:

```bash
killall -9 gzserver gzclient
```

- `gzserver` = la física; `gzclient` = la ventana.
- Si uno queda vivo, el siguiente Gazebo no arranca.

---

## Paso 1 · T1 — Gazebo con 6 robots

```bash
ros2 launch swarm_bb_description swarm_gazebo_6robots.launch.py
```

Espera a que aparezcan los 6 robots (`swarm_bb_01` … `swarm_bb_06`) antes de seguir.

---

## Paso 2 · T2 — Puente Zenoh + coordinador

```bash
ros2 launch swarm_bb_coordinator swarm_system.launch.py formation_type:=wedge
```

- `formation_type`: `wedge` (cuña), `column` o `line`.
- Deja `leader_id` en `swarm_bb_01`. La tabla de offsets está fija en el código y supone ese líder; con otro, ese robot se persigue a sí mismo y gira en su sitio.
- Ver todos los parámetros: `ros2 launch swarm_bb_coordinator swarm_system.launch.py --show-args`

---

## Paso 3 · T3 — Manejar al líder con el teclado

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/swarm_bb_01/cmd_vel -p speed:=0.2 -p turn:=0.8
```

- `teleop_twist_keyboard` va **dos veces**: `ros2 run <paquete> <ejecutable>`.
- `-r cmd_vel:=/swarm_bb_01/cmd_vel` manda las órdenes al líder (sin esto van a `/cmd_vel`, que nadie escucha).
- `speed:=0.2` queda por debajo de `v_max` = 0.22 m/s de los seguidores, para que puedan alcanzarlo.
- La terminal tiene que tener el foco para leer las teclas.

| Tecla | Acción |
|---|---|
| `i` / `,` | adelante / atrás |
| `j` / `l` | girar en el sitio |
| `u` / `o` | avanzar girando |
| `k` | detener |
| `z` / `q` | bajar / subir velocidad 10 % |

---

## Paso 4 · T4 — Reset y comprobaciones

### Reset del mundo

1. Presiona `k` en T3 para detener al líder.
2. Corre:

   ```bash
   ros2 service call /reset_world std_srvs/srv/Empty
   ```

   O desde la ventana de Gazebo: **Edit → Reset World**.

- El servicio lo da el plugin `gazebo_ros_init` (lo carga `gazebo.launch.py` de `gazebo_ros`).
  Comprobar que existe: `ros2 service list | grep reset` → deben salir `/reset_world` y `/reset_simulation`.
- Usa `/reset_world`, no `/reset_simulation`:
  - `/reset_world` regresa los robots a su posición inicial y deja el reloj de la simulación.
  - `/reset_simulation` además pone el reloj en 0; el tiempo hacia atrás suele dar avisos de TF.
- No hace falta reiniciar el coordinador: el diff-drive usa `odometry_source` = 1 (posición real del mundo), así que `odom` salta solo a la posición inicial.

### Comprobaciones

```bash
ros2 node list                        # deben estar zenoh_swarm_bridge y leader_follower_coordinator
ros2 topic echo /swarm_bb_01/cmd_vel  # lo que mandas con el teclado
ros2 topic echo /swarm_bb_02/cmd_vel  # lo que el coordinador manda a un seguidor
```

---

## Cerrar

Ctrl+C en T3, T2 y T1, en ese orden. Si Gazebo deja procesos colgados, repite el Paso 0.

---

## Problemas conocidos

| Síntoma | Causa | Arreglo |
|---|---|---|
| `bash: ~/ros2_ws/installsetup.bash: No such file or directory` al abrir terminal | Línea 123 de `~/.bashrc` sin `/` | `source ~/ros2_ws/install/setup.bash`, después del `source` de Humble, o comentarla |
| `[gzclient] libprotobuf ERROR ... Joint.name contains invalid UTF-8` en bucle | Joints con tilde en el URDF (`Rígida-10`, `Revolución-12`…) | Renombrarlos a ASCII en `<joint name>` **y** en `<left_joint>`/`<right_joint>` del plugin diff-drive |
| `ros2: error: unrecognized arguments: --ros-args` | Falta el nombre del ejecutable en `ros2 run` | `ros2 run teleop_twist_keyboard teleop_twist_keyboard ...` |
| `Package 'swarm_bb_...' not found` | Falta `source install/setup.bash` en esa terminal | Hacer los dos `source` |
