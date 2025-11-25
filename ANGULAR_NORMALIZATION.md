# Estrategia de Normalización Angular para MPC

## Problema Original

La normalización estándar a [-π, π] causa problemas en el MPC cuando:
1. El robot está cerca de ±180° y la referencia cruza esta discontinuidad
2. Las matrices de linearización (A, B) tienen coeficientes discontinuos
3. Los errores angulares pueden ser de 2π cuando deberían ser pequeños
4. El blend de ángulos produce resultados incorrectos (ej: 170° y -170° → 68° en vez de ±176°)

## Solución Implementada: Normalización Adaptativa con `furthest_theta`

### Concepto Principal

En lugar de normalizar siempre a [-π, π], creamos una **ventana dinámica de 2π** que evita la discontinuidad en el rango de trabajo actual del MPC.

**Clave**: No necesitamos normalizar constantemente a [-π, π]. Solo necesitamos determinar **qué "lado" tiene el ángulo mayor que π** y crear una ventana que evite esa discontinuidad.

### Algoritmo

1. **Obtener ángulos directamente** (sin pre-normalización):
   - `theta_current` (pose del robot)
   - `theta_ref` (primer punto de referencia)

2. **Calcular `furthest_theta`** (bisectriz del ángulo mayor):
   ```cpp
   double diff = theta_ref - theta_current;
   
   if (diff > M_PI) {
     // Ángulo mayor está del lado "negativo"
     furthest_theta = theta_current + diff/2.0 - M_PI;
   } else if (diff < -M_PI) {
     // Ángulo mayor está del lado "positivo"
     furthest_theta = theta_current + diff/2.0 + M_PI;
   } else {
     // |diff| <= π, el camino más corto es 'diff'
     // El ángulo mayor está en el lado opuesto
     furthest_theta = theta_current + diff + M_PI;
   }
   ```
   
   **Ejemplo**:
   - Si `theta_current = 170°` y `theta_ref = -170°`
   - `diff = -340°` (< -π)
   - `furthest_theta = 170° + (-340°)/2 + 180° = 170° - 170° + 180° = 180°`

3. **Normalizar todo a `[furthest_theta - 2π, furthest_theta]`**:
   ```cpp
   while (angle > base) angle -= 2.0 * M_PI;
   while (angle < base - 2.0 * M_PI) angle += 2.0 * M_PI;
   ```
   
   Simple aritmética modular: solo verificamos si estamos arriba o abajo de la ventana.

4. **Operar en espacio continuo**:
   - Blend lineal: `theta_lin = 0.7 * theta_ref + 0.3 * theta_current` ✓
   - Error angular: `error = theta_pred - theta_ref` ✓ (ya es el camino más corto)
   - Sin saltos de 2π en ninguna operación

### Ventajas

✅ **Continuidad garantizada**: La discontinuidad está en `furthest_theta`, lejos de los ángulos de trabajo
✅ **Operaciones lineales válidas**: Podemos usar promedio ponderado directo
✅ **Matrices consistentes**: A y B no tienen saltos discontinuos en sus coeficientes
✅ **Simplicidad**: Solo aritmética modular simple, sin múltiples normalizaciones
✅ **Error correcto automático**: `theta_pred - theta_ref` da el camino más corto sin normalización adicional
✅ **Sin normalize_angle()**: No necesitamos la función legacy en el MPC

### Implementación Simplificada

```cpp
// 1. Calcular furthest_theta (sin pre-normalización)
double furthest_theta = calculate_furthest_theta(theta_current, theta_ref);

// 2. Normalizar todos los ángulos a [furthest_theta - 2π, furthest_theta]
double theta_curr_norm = normalize_angle_around(theta_current, furthest_theta);
double theta_ref_norm = normalize_angle_around(theta_ref, furthest_theta);

// 3. Blend lineal seguro (sin normalización adicional)
double theta_lin = 0.7 * theta_ref_norm + 0.3 * theta_curr_norm;

// 4. Normalizar referencias y predicciones a la misma ventana
for (int i = 0; i < N; ++i) {
  x_ref_vec(dim_x * i + 2) = normalize_angle_around(ref_angle_i, furthest_theta);
  x_predicted(dim_x * i + 2) = normalize_angle_around(pred_angle_i, furthest_theta);
}

// 5. Error angular correcto automáticamente (sin normalización)
error_vec = x_predicted - x_ref_vec;
```

**Nota**: La función `normalize_angle()` legacy a [-π, π] se eliminó del MPC. Solo se usa aritmética modular simple para detectar qué "lado" tiene el ángulo mayor que π.

## Alternativas Consideradas

### 1. Usar `theta_eucl` (dirección hacia la referencia)

**Idea**: Considerar también la dirección del vector de error posicional:
```cpp
double dx = x_ref - x_current;
double dy = y_ref - y_current;
double theta_eucl = atan2(dy, dx);
```

**Ventajas**:
- Anticipa la dirección de giro que el MPC elegirá
- Útil cuando la referencia angular apunta a una dirección pero el robot necesita girar hacia otra para acercarse

**Implementación posible**:
```cpp
// Blend ponderado por distancia
double dist = hypot(dx, dy);
double weight_eucl = 1.0 / (1.0 + dist);  // Más peso cuando está cerca
double weight_angular = 1.0 - weight_eucl;

double furthest_combined = calculate_furthest_from_three(
    theta_current, theta_ref, theta_eucl, weight_angular, weight_eucl);
```

**Desventajas**:
- Más complejo de implementar
- Puede causar inestabilidad si `theta_eucl` cambia rápidamente
- No está claro el beneficio práctico para trayectorias suaves

### 2. Quaternions en el MPC

**Idea**: Usar quaternions en vez de ángulos de Euler

**Ventajas**:
- No hay discontinuidades
- SLERP (interpolación esférica) es matemáticamente correcta

**Desventajas**:
- ❌ Las matrices de linearización se complican enormemente
- ❌ El modelo de differential drive es naturalmente en yaw, no en quaternions
- ❌ OSQP requiere formulación cuadrática, difícil con quaternions
- ❌ Overhead computacional significativo

### 3. Unwrapped angles (ángulos acumulativos)

**Idea**: Mantener un contador de "vueltas completas" y no normalizar nunca

**Ejemplo**: En vez de [170°, -170°, 10°], usar [170°, 190°, 370°]

**Ventajas**:
- Continuidad absoluta

**Desventajas**:
- ❌ Requiere estado adicional (contador de vueltas)
- ❌ Acumulación de error numérico a largo plazo
- ❌ Problemas con cambios abruptos en el path (saltos de más de π)
- ❌ Difícil de implementar con referencias temporales que pueden cambiar

### 4. Modelo de control en espacio tangente

**Idea**: Control basado en velocidad angular en vez de orientación absoluta

**Implementación**:
- Estado: `[x, y, ω]` en vez de `[x, y, θ]`
- Solo penalizar `Δω` (cambios en velocidad angular)

**Ventajas**:
- Evita por completo el problema angular en el estado

**Desventajas**:
- ❌ Pierde tracking de orientación absoluta
- ❌ No puede seguir referencias con orientación específica
- ❌ Deriva angular acumulativa
- ❌ Incompatible con el objetivo del MPC (seguir pose completa)

### 5. Dual-mode normalization (Dubins/Reeds-Shepp)

**Idea**: Cambiar estrategia según la distancia al objetivo

**Implementación**:
```cpp
if (dist_to_goal < threshold) {
  // Modo cercano: usar theta_eucl para guiar hacia el objetivo
  furthest_theta = calculate_furthest_theta(theta_current, theta_eucl);
} else {
  // Modo lejano: usar theta_ref para seguir la trayectoria
  furthest_theta = calculate_furthest_theta(theta_current, theta_ref);
}
```

**Ventajas**:
- Mejor comportamiento en aproximación final
- Puede mejorar convergencia en espacios confinados

**Desventajas**:
- Discontinuidad en el cambio de modo
- Requiere tuning del threshold
- Complejidad adicional

## Comparación con Métodos en la Literatura

### Nav2 MPPI Controller
- Usa muestreo estocástico, evita linearización
- No tiene el problema de discontinuidad angular
- Más caro computacionalmente

### TEB Local Planner
- Usa optimización no lineal (g2o)
- Puede manejar wrapping angular directamente en el optimizador
- Más lento que QP

### Pure Pursuit / DWA
- No lineariza, trabaja directamente con ángulos
- Menos sofisticado, no puede hacer optimización con horizonte

### Model Predictive Path Integral (MPPI)
- Similar a MPPI de Nav2
- Evita linearización mediante sampling
- Robusto pero computacionalmente intensivo

## Conclusión

La estrategia de **normalización adaptativa con `furthest_theta`** es:
- ✅ Simple de implementar
- ✅ Computacionalmente eficiente
- ✅ Matemáticamente sólida
- ✅ Compatible con el framework MPC linealizado
- ✅ No requiere cambios en el modelo dinámico

Es la mejor solución para MPC con linearización cuando se trabaja con ángulos de Euler.

## Referencias

1. Coulter, R. C. (1992). "Implementation of the pure pursuit path tracking algorithm"
2. Rösmann, C., et al. (2015). "Trajectory modification considering dynamic constraints of autonomous robots"
3. Williams, G., et al. (2017). "Information-theoretic model predictive control"
4. Pivtoraiko, M. & Kelly, A. (2005). "Efficient constrained path planning via search in state lattices"
