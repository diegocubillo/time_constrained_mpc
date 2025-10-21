# Implementación de Referencia Temporal Explícita - Resumen

## Cambios Implementados

Se ha implementado con éxito el seguimiento temporal explícito en el MPC. El controlador ahora utiliza los timestamps de cada pose en el path para calcular referencias temporales precisas en cada paso del horizonte de predicción.

---

## Cómo Funciona

### 1. **Estructura de Datos**

El path (`nav_msgs::msg::Path`) ahora debe contener poses con timestamps válidos en sus headers:

```cpp
nav_msgs::msg::Path path;
for (each waypoint) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Time(epoch_seconds, epoch_nanoseconds);
    pose.pose.position = ...;
    pose.pose.orientation = ...;
    path.poses.push_back(pose);
}
```

**Asunciones:**
- Los timestamps están en tiempo epoch (absoluto)
- Todos los timestamps están en el futuro cuando se recibe el path
- La desincronización de reloj entre planificador y controlador es despreciable

### 2. **Flujo de Ejecución**

#### Cuando se recibe un nuevo path (`handle_goal`):
```cpp
path_start_time_ = this->now();  // Marcar inicio de ejecución
```

Se registra el tiempo de inicio y se valida que los timestamps estén en el futuro.

#### En cada ciclo de control (`control_loop`):

1. **Calcular tiempo actual:**
   ```cpp
   rclcpp::Time current_time = this->now();
   ```

2. **Obtener trayectoria de referencia temporal:**
   ```cpp
   auto reference_trajectory = get_reference_trajectory_horizon(current_time, N, dt);
   // Retorna vector de N referencias [x, y, θ] interpoladas según timestamps
   ```

3. **Resolver MPC con referencias temporales:**
   ```cpp
   auto cmd = solve_mpc(pose, velocity, reference_trajectory);
   ```

4. **Calcular error temporal para monitoreo:**
   ```cpp
   double temporal_error = calculate_temporal_error();
   // Positivo = adelantado, Negativo = atrasado
   ```

---

## Funciones Clave

### `get_temporal_reference(target_time)`

**Propósito:** Obtener la pose de referencia que corresponde a un timestamp específico.

**Algoritmo:**
1. Si `target_time` < timestamp del primer waypoint → retorna primera pose
2. Si `target_time` > timestamp del último waypoint → retorna última pose
3. Si `target_time` está entre dos waypoints → **interpola linealmente**:
   ```
   ratio = (target_time - t0) / (t1 - t0)
   x_ref = x0 + ratio * (x1 - x0)
   y_ref = y0 + ratio * (y1 - y0)
   θ_ref = θ1  // Orientación del siguiente waypoint
   ```

**Ejemplo:**
```
Waypoint 0: (0, 0) @ t=10.0s
Waypoint 1: (4, 0) @ t=14.0s

target_time = 12.0s
ratio = (12.0 - 10.0) / (14.0 - 10.0) = 0.5
x_ref = 0 + 0.5 * 4 = 2.0
y_ref = 0 + 0.5 * 0 = 0.0
→ Referencia: (2.0, 0.0)
```

### `get_reference_trajectory_horizon(current_time, N, dt)`

**Propósito:** Calcular las N referencias futuras para el horizonte MPC.

**Algoritmo:**
```cpp
for i = 0 to N-1:
    target_time = current_time + i * dt
    reference[i] = get_temporal_reference(target_time)
```

**Ejemplo con N=3, dt=0.1s:**
```
current_time = 12.0s

Paso 0: target_time = 12.0s → referencia @ t=12.0s
Paso 1: target_time = 12.1s → referencia @ t=12.1s
Paso 2: target_time = 12.2s → referencia @ t=12.2s
```

### `calculate_temporal_error()`

**Propósito:** Calcular el error de sincronización temporal.

**Algoritmo:**
1. Encontrar la pose más cercana espacialmente en el path
2. Obtener su timestamp: `t_trajectory`
3. Calcular: `error = t_trajectory - t_actual`

**Interpretación:**
- `error > 0`: Robot adelantado (ya pasó por donde debería estar más adelante)
- `error < 0`: Robot atrasado (aún no ha llegado donde ya debería estar)
- `error ≈ 0`: Robot sincronizado temporalmente

---

## Modificación de `build_mpc_matrices`

### Cambio de Firma

**Antes:**
```cpp
void build_mpc_matrices(
    const Eigen::Vector3d &current_state,
    const Eigen::Vector3d &desired_state,  // ← Una sola referencia
    ...
)
```

**Después:**
```cpp
void build_mpc_matrices(
    const Eigen::Vector3d &current_state,
    const std::vector<Eigen::Vector3d> &reference_trajectory,  // ← N referencias
    ...
)
```

### Construcción del Vector de Error

**Antes (referencia única):**
```cpp
Eigen::Vector3d e = current_state - desired_state;
q = S_u^T * Q_bar * S_x * x_aug;
```

**Después (referencias múltiples):**
```cpp
// Construir vector de referencias para todo el horizonte
Eigen::VectorXd x_ref_vec(3*N);
for (int i = 0; i < N; ++i) {
    x_ref_vec.segment(3*i, 3) = reference_trajectory[i];
}

// Predecir estados futuros desde estado actual
Eigen::VectorXd x_predicted = S_x * x_aug;

// Calcular error para todo el horizonte
Eigen::VectorXd error_vec = x_predicted - x_ref_vec;

// Gradiente del QP
q = S_u^T * Q_bar * error_vec;
```

### Visualización Matemática

**Vector de referencias:**
$$
\bar{x}_{ref} = \begin{bmatrix}
x_0 \\ y_0 \\ \theta_0 \\
x_1 \\ y_1 \\ \theta_1 \\
\vdots \\
x_{N-1} \\ y_{N-1} \\ \theta_{N-1}
\end{bmatrix} \in \mathbb{R}^{3N}
$$

**Vector de predicción:**
$$
\bar{x}_{pred} = S_x \cdot \xi_0 = \begin{bmatrix}
\hat{x}_0 \\ \hat{y}_0 \\ \hat{\theta}_0 \\
\hat{x}_1 \\ \hat{y}_1 \\ \hat{\theta}_1 \\
\vdots
\end{bmatrix} \in \mathbb{R}^{3N}
$$

**Error por paso:**
$$
e_i = \begin{bmatrix}
\hat{x}_i - x_{ref,i} \\
\hat{y}_i - y_{ref,i} \\
\hat{\theta}_i - \theta_{ref,i}
\end{bmatrix} \quad \forall i \in [0, N-1]
$$

**Función de coste resultante:**
$$
J = \sum_{i=0}^{N-1} \left[ \|e_i\|_Q^2 + \|\Delta u_i\|_{R+R_d}^2 \right]
$$

Cada paso del horizonte tiene su propia referencia temporal, **adaptándose dinámicamente** según el tiempo transcurrido.

---

## Comportamiento del Sistema

### Escenario 1: Robot Sincronizado

```
t_actual = 12.0s
Pose actual: (2.0, 0.5)
Pose más cercana en path: (2.0, 0.0) @ t=12.0s
Error temporal = 0.0s ✓
```

**Acción:** El MPC mantiene velocidad nominal para seguir la trayectoria.

### Escenario 2: Robot Adelantado

```
t_actual = 12.0s
Pose actual: (3.0, 0.5)
Pose más cercana en path: (3.0, 0.0) @ t=13.0s
Error temporal = +1.0s (adelantado)
```

**Acción:** 
- Las referencias del horizonte están "más atrás" espacialmente
- El robot tendrá que reducir velocidad o incluso detenerse
- El MPC naturalmente genera comandos más conservadores

### Escenario 3: Robot Atrasado

```
t_actual = 14.0s
Pose actual: (2.0, 0.5)
Pose más cercana en path: (2.0, 0.0) @ t=12.0s
Error temporal = -2.0s (atrasado)
```

**Acción:**
- Las referencias del horizonte están "más adelante" espacialmente
- El robot necesita acelerar para recuperar sincronización
- El MPC genera comandos más agresivos (dentro de los límites)

---

## Logs y Debugging

El sistema imprime información útil:

### Al recibir un path:
```
[INFO] Received new path with 50 poses
[INFO] Path temporal span: 0.50 to 25.00 seconds from now
```

### Durante la ejecución:
```
[INFO] Temporal tracking: 0.35 s ahead of schedule
[WARN] Temporal tracking: 1.20 s behind schedule
```

Los logs se throttle a 2 segundos para no saturar la consola.

---

## Ventajas de Esta Implementación

### ✅ **Penalización Bidireccional Automática**
- Adelantarse es malo → referencias "atrás" → robot frena
- Atrasarse es malo → referencias "adelante" → robot acelera

### ✅ **Sin Modificar Pesos**
- No requiere ajuste dinámico de Q y R
- El comportamiento emerge naturalmente del MPC

### ✅ **Adaptación Continua**
- Cada ciclo recalcula referencias según tiempo real
- Se adapta a perturbaciones y retrasos

### ✅ **Suavidad Garantizada**
- Los límites de aceleración siguen aplicándose
- No genera comandos discontinuos

### ✅ **Predicción Coherente**
- El horizonte MPC "ve" hacia adelante en el tiempo correcto
- Las predicciones coinciden con lo que realmente pasará

---

## Limitaciones Actuales

### ⚠️ **Interpolación de Orientación Simplificada**
Actualmente se usa la orientación del siguiente waypoint sin SLERP:
```cpp
reference.pose.orientation = global_plan_.poses[i + 1].pose.orientation;
```

**Mejora futura:** Implementar SLERP (Spherical Linear Interpolation) para orientaciones suaves.

### ⚠️ **Manejo de Retrasos Excesivos**
Si el robot está muy atrasado, las referencias pueden quedarse en el último waypoint.

**Posible mejora:** Detectar retrasos irrecuperables y abortar la misión.

### ⚠️ **Sin Ajuste de Horizon Temporal**
El horizonte siempre usa `dt` fijo. 

**Posible mejora:** Ajustar `dt` dinámicamente según urgencia temporal.

---

## Próximos Pasos Sugeridos

1. **Validación Experimental:**
   - Probar con trayectorias temporales variadas
   - Medir precisión temporal real
   - Ajustar pesos Q, R, R_d si es necesario

2. **Visualización en RViz:**
   - Publicar referencias temporales como markers
   - Mostrar error temporal como texto

3. **Métricas de Desempeño:**
   - Calcular MAE (Mean Absolute Error) temporal
   - Registrar historial de errores temporales

4. **Mejoras Algorítmicas:**
   - Implementar SLERP para orientaciones
   - Añadir predicción de velocidad requerida
   - Considerar Fase 2: ajuste dinámico de pesos

---

## Archivos Modificados

- `time_constrained_mpc.hpp`: Nuevas declaraciones de métodos y variables
- `time_constrained_mpc.cpp`: Implementación completa de referencia temporal
  - `get_temporal_reference()`: Interpolación temporal
  - `get_reference_trajectory_horizon()`: Referencias del horizonte
  - `calculate_temporal_error()`: Cálculo de error
  - `build_mpc_matrices()`: Aceptar múltiples referencias
  - `solve_mpc()`: Usar referencias temporales
  - `control_loop()`: Calcular referencias en cada ciclo

---

## Testing Recomendado

### Test 1: Path con timestamps uniformes
```python
# Generar path de 10m en 20s
for i in range(20):
    pose = PoseStamped()
    pose.pose.position.x = i * 0.5
    pose.pose.position.y = 0.0
    pose.header.stamp = start_time + Duration(seconds=i)
    path.poses.append(pose)
```

### Test 2: Path con aceleración
```python
# Primera mitad lenta, segunda mitad rápida
for i in range(10):
    t = i * 2.0  # 2s por metro
for i in range(10, 20):
    t = 20.0 + (i-10) * 0.5  # 0.5s por metro
```

### Test 3: Path con parada intermedia
```python
# Parada en x=5m durante 5 segundos
for i in range(5):
    pose at (i, 0) @ t=i*1.0s
for i in range(5):
    pose at (5, 0) @ t=5.0s + i*1.0s  # Mismo lugar, tiempo avanza
for i in range(5, 10):
    pose at (i, 0) @ t=10.0s + (i-5)*1.0s
```

---

**Implementación completada exitosamente. El sistema está listo para pruebas.**
