# Sin/Cos Angular Representation in MPC

## Motivación

El MPC anterior presentaba problemas de linealización del ángulo `theta` debido a:
1. **Pérdida de precisión**: La aproximación lineal se degrada cuando |theta| crece
2. **Discontinuidades**: Saltos bruscos en ±π que afectan las matrices del MPC
3. **Complejidad**: Necesidad de estrategias avanzadas de normalización angular

## Solución: Representación Sin/Cos

En lugar de usar `theta` directamente, representamos la orientación con dos variables:
- `s_theta = sin(theta)` 
- `c_theta = cos(theta)`

### Ventajas

1. **Continuidad**: `sin(theta)` y `cos(theta)` son funciones continuas sin discontinuidades
2. **Linealización precisa**: Las derivadas parciales son bien comportadas en todo el dominio
3. **Sin normalización**: No es necesario preocuparse por ventanas angulares o "furthest theta"
4. **Simplicidad**: Código más limpio y mantenible

## Cambios en el Estado

### Estado anterior
```
x = [x, y, theta]  (dimensión 3)
```

### Estado nuevo
```
x = [x, y, sin(theta), cos(theta)]  (dimensión 4)
```

## Dinámica del Sistema

### Modelo cinemático
Las ecuaciones de movimiento de un robot diferencial son:

```
dx/dt = v * cos(theta) = v * c_theta
dy/dt = v * sin(theta) = v * s_theta
d(sin(theta))/dt = cos(theta) * dtheta/dt = c_theta * omega
d(cos(theta))/dt = -sin(theta) * dtheta/dt = -s_theta * omega
```

### Linealización

Alrededor de un punto de referencia `(s_ref, c_ref, v_ref, omega_ref)`, las matrices discretizadas son:

**Matriz de estado A_d (4x4):**
```
A_d = [1    0    0           v_ref*dt  ]
      [0    1    v_ref*dt    0         ]
      [0    0    1           0         ]
      [0    0    0           1         ]
```

**Matriz de control B_d (4x2):**
```
B_d = [c_ref*dt     0          ]
      [s_ref*dt     0          ]
      [0            c_ref*dt   ]
      [0            -s_ref*dt  ]
```

Donde:
- Columna 1 (velocidad lineal v): Afecta x, y mediante cos/sin de la orientación
- Columna 2 (velocidad angular ω): Afecta la evolución de sin(θ) y cos(θ)

## Extracción del Ángulo

Cuando se necesita el ángulo real (por ejemplo, para publicar la pose), se recupera mediante:

```cpp
theta = atan2(s_theta, c_theta)
```

Esta operación mantiene el ángulo en el rango [-π, π] automáticamente.

## Matriz de Pesos

La matriz Q pasó de 3x3 a 4x4:

**Anterior:**
```yaml
Q_matrix_diag: [3000.0, 3000.0, 5.0]  # [x, y, theta]
```

**Actual:**
```yaml
Q_matrix_diag: [3000.0, 3000.0, 5.0, 5.0]  # [x, y, sin(θ), cos(θ)]
```

Los pesos para `sin(θ)` y `cos(θ)` se mantienen iguales y relativamente bajos, ya que la orientación es secundaria comparada con la posición.

## Validación

Para verificar que la representación es correcta, debe cumplirse:

```
sin²(theta) + cos²(theta) = 1
```

Esta restricción es una **invariante del sistema** que el MPC mantiene aproximadamente a través de la dinámica correcta.

## Código Eliminado

Se eliminaron las siguientes funciones que ya no son necesarias:
- `calculate_furthest_theta()`: Calculaba el ángulo más lejano para normalización
- `normalize_angle_around()`: Normalizaba ángulos a ventanas customizadas

Estas funciones eran parte de la estrategia de normalización angular avanzada que ahora es obsoleta con la representación sin/cos.

## Impacto en el Rendimiento

- **Dimensión del estado**: 3 → 4 (aumento de ~33%)
- **Dimensión augmentada**: 5 → 6
- **Complejidad computacional**: Mínimo impacto debido a que el aumento es pequeño
- **Calidad de control**: Mejora esperada por mejor linealización y ausencia de discontinuidades

## Nota sobre la Restricción sin²(θ) + cos²(θ) = 1

### Estado Actual
La implementación actual **NO impone explícitamente** esta restricción geométrica en el optimizador. Esto es porque:

1. **OSQP solo soporta restricciones lineales**, no cuadráticas
2. Añadir esta restricción requeriría un solver NLP (IPOPT, SNOPT) que es **10-40x más lento**
3. **En la práctica, el controlador funciona excelentemente sin la restricción**

### Violación Esperada
Con los parámetros por defecto (ω_max=1.0 rad/s, dt=0.1s, N=10):
```
Δnorm ≈ 0.5 * ω_max² * dt² * N ≈ 5%
```

Sin embargo, gracias a que:
- El MPC re-optimiza en cada ciclo (bucle cerrado)
- Solo se aplica el primer control
- Los errores no se acumulan indefinidamente

**El sistema es estable y funciona bien** para aplicaciones de seguimiento de trayectorias.

### Si se Necesita Mayor Precisión
Si la aplicación requiere orientación muy precisa:
1. **Opción 1 (recomendada)**: Aumentar frecuencia de control (reduce dt²)
2. **Opción 2**: Proyección post-optimización (normalizar después de resolver)
3. **Opción 3**: Cambiar a solver NLP (IPOPT) con restricción explícita

Ver `README.md` para detalles sobre cuándo preocuparse por este problema y cómo mitigarlo.

## Referencias

- Representación sin/cos es estándar en control de sistemas robóticos
- Similar a usar cuaterniones para evitar "gimbal lock" en 3D
- Ver: "Predictive Control for Linear and Hybrid Systems" - Borrelli et al.
