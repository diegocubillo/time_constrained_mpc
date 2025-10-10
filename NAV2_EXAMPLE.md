# Nav2 Integration Example

## Descripción

Este ejemplo muestra cómo integrar el controlador MPC con el stack completo de Nav2. Incluye:

- **Map Server**: Servidor de mapas con un mapa de prueba simple
- **Planner Server**: Planificador global (NavFn por defecto)
- **MPC Controller**: Nuestro controlador MPC personalizado
- **BT Navigator**: Árbol de comportamiento de Nav2
- **Lifecycle Manager**: Gestiona el ciclo de vida de todos los nodos
- **Loopback Simulator**: Simula el movimiento del robot para pruebas sin robot real

## Estructura de Archivos

```
time_constrained_mpc/
├── config/
│   ├── mpc_params.yaml       # Parámetros del controlador MPC
│   ├── nav2_params.yaml      # Parámetros de Nav2 completo
│   └── nav2_view.rviz        # Configuración de RViz2
├── launch/
│   └── nav2_mpc_example.launch.py  # Launch completo
└── maps/
    ├── map.yaml              # Metadatos del mapa
    └── map.pgm               # Imagen del mapa (200x200 píxeles)
```

## Mapa de Prueba

El mapa incluido es un espacio simple de 10m x 10m con:
- Paredes exteriores
- Un obstáculo cuadrado en el centro
- Resolución: 0.05m por píxel
- Tamaño: 200x200 píxeles

## Cómo Usar

### 1. Compilar el Paquete

```bash
cd ~/ros2_ws
colcon build --packages-select time_constrained_mpc
source install/setup.bash
```

### 2. Lanzar el Sistema Completo

```bash
ros2 launch time_constrained_mpc nav2_mpc_example.launch.py
```

Esto iniciará:
- Todos los servidores de Nav2
- El controlador MPC
- El simulador loopback
- RViz2 para visualización

### 3. Dar una Pose Inicial (en RViz2)

1. En RViz2, haz clic en el botón "2D Pose Estimate"
2. Haz clic en el mapa para establecer la posición inicial
3. Arrastra para establecer la orientación inicial

### 4. Enviar un Goal de Navegación

Opción A - **Usando RViz2**:
1. Haz clic en el botón "2D Goal Pose"
2. Haz clic en el destino deseado
3. Arrastra para establecer la orientación final
4. El robot planificará y seguirá el path usando MPC

Opción B - **Usando la línea de comandos**:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: 3.0
      y: 3.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"
```

## Verificar el Estado del Sistema

### Comprobar Nodos Lifecycle

```bash
# Listar todos los nodos lifecycle
ros2 lifecycle nodes

# Verificar el estado de cada nodo
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /mpc_controller
```

Todos deben estar en estado `active` si el `autostart` está habilitado.

### Monitorear Topics

```bash
# Ver el mapa
ros2 topic echo /map

# Ver el plan global
ros2 topic echo /plan

# Ver comandos de velocidad
ros2 topic echo /cmd_vel

# Ver el path de debug del MPC
ros2 topic echo /mpc_debug_path

# Ver odometría del simulador
ros2 topic echo /odom
```

### Monitorear Acciones

```bash
# Ver el estado de la acción de navegación
ros2 action list

# Ver información de una acción específica
ros2 action info /navigate_to_pose
ros2 action info /follow_path
```

## Flujo de Datos

```
Usuario (RViz2 Goal) 
    ↓
BT Navigator
    ↓
Planner Server → Plan Global (/plan)
    ↓
Controller Server
    ↓
MPC Controller (action /follow_path)
    ↓
Comandos de Velocidad (/cmd_vel)
    ↓
Loopback Simulator
    ↓
Odometría (/odom) + TF (map→odom→base_link)
```

## Configuración Importante

### Lifecycle Manager

El `lifecycle_manager` gestiona automáticamente todos los nodos:

```yaml
lifecycle_manager:
  ros__parameters:
    autostart: true
    node_names: 
      - 'map_server'
      - 'planner_server'
      - 'controller_server'
      - 'bt_navigator'
      - 'mpc_controller'  # ← Nuestro nodo MPC
```

**Importante**: Con `autostart: true`, todos los nodos se configuran y activan automáticamente. El controlador MPC ahora:
- Rechaza goals si está en estado `inactive`
- Solo acepta goals cuando está `active`
- Se desactiva correctamente cuando se detiene el lifecycle manager

### Manejo de Estado `initialized_`

El código ahora maneja correctamente el estado:

```cpp
// En on_configure(): initialized_ = false
// El action server está creado pero rechaza goals

// En on_activate(): initialized_ = true
// Ahora el controlador acepta goals

// En on_deactivate(): initialized_ = false
// Vuelve a rechazar goals y detiene el robot
```

## Parámetros Ajustables

### Parámetros del MPC (mpc_params.yaml)

```yaml
mpc_controller:
  ros__parameters:
    horizon_steps: 10
    max_linear_vel: 0.5
    max_angular_vel: 1.0
    # ... otros parámetros
```

### Parámetros de Nav2 (nav2_params.yaml)

Los más importantes:
- `controller_frequency`: Frecuencia del controlador (10 Hz por defecto)
- `xy_goal_tolerance`: Tolerancia de posición al goal (0.25m)
- `yaw_goal_tolerance`: Tolerancia de orientación al goal (0.25 rad)

## Troubleshooting

### El robot no se mueve

1. **Verificar que todos los nodos estén activos**:
   ```bash
   ros2 lifecycle nodes
   ```

2. **Verificar TF**:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

3. **Verificar comandos de velocidad**:
   ```bash
   ros2 topic echo /cmd_vel
   ```

### "Goal rejected - controller is not active"

Esto significa que el nodo MPC no está en estado `active`. Soluciones:

```bash
# Verificar estado
ros2 lifecycle get /mpc_controller

# Si está en 'inactive', activar manualmente
ros2 lifecycle set /mpc_controller activate

# O reiniciar el lifecycle manager con autostart
```

### El planificador no encuentra un path

- Verifica que la pose inicial esté en espacio libre
- Verifica que el goal esté en espacio libre
- Reduce la tolerancia del planificador
- Verifica el mapa con: `ros2 topic echo /map`

### Errores de compilación

Si falta `nav2_loopback_sim`:
```bash
sudo apt-get install ros-${ROS_DISTRO}-nav2-loopback-sim
```

Si faltan otros paquetes de Nav2:
```bash
sudo apt-get install ros-${ROS_DISTRO}-navigation2
sudo apt-get install ros-${ROS_DISTRO}-nav2-bringup
```

## Opciones de Lanzamiento

### Sin RViz2

```bash
ros2 launch time_constrained_mpc nav2_mpc_example.launch.py use_rviz:=false
```

### Con tiempo de simulación

```bash
ros2 launch time_constrained_mpc nav2_mpc_example.launch.py use_sim_time:=true
```

### Sin autostart (gestión manual del lifecycle)

```bash
ros2 launch time_constrained_mpc nav2_mpc_example.launch.py autostart:=false
```

Luego activar manualmente:
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
# ... repetir para cada nodo
```

## Próximos Pasos

Una vez que funcione este ejemplo:

1. **Crear tu propio mapa**: Usa `slam_toolbox` o `cartographer` para crear un mapa real
2. **Ajustar parámetros MPC**: Tuning de Q, R, R_d para tu robot específico
3. **Integrar con robot real**: Cambiar `loopback_simulator` por el driver de tu robot
4. **Añadir sensores**: Configurar láser scan u otros sensores en costmaps
5. **Behavior Trees personalizados**: Crear árboles de comportamiento más complejos

## Referencias

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tutorials](https://navigation.ros.org/tutorials/index.html)
- [Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)
