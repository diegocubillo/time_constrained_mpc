# Explicación Detallada del Funcionamiento del MPC

## Tabla de Nomenclatura

| Símbolo | Dimensión | Descripción |
|---------|-----------|-------------|
| $x_i$ | 3×1 | Estado del robot en el paso $i$: $[x, y, \theta]^T$ (posición x, y y orientación) |
| $u_i$ | 2×1 | Control aplicado en el paso $i$: $[v, \omega]^T$ (velocidad lineal y angular) |
| $\Delta u_i$ | 2×1 | Incremento de control: $u_i - u_{i-1}$ (cambio respecto al control anterior) |
| $\xi_k$ | 5×1 | Estado aumentado: $\xi_k = [x_k, y_k, \theta_k, u_{k-1,v}, u_{k-1,\omega}]^T = [x_k; u_{k-1}]$ donde $u_{k-1}$ es la velocidad **anterior** (NO el incremento) |
| $N$ | - | Número de pasos del horizonte de predicción (ej: 10) |
| $\Delta t$ | - | Paso de tiempo del control (ej: 0.1s) |
| $Q$ | 3×3 | Matriz de peso para el error de estado (penaliza desviaciones de posición/orientación) |
| $R$ | 2×2 | **NO USADO en Opción 2** - Matriz de peso para el control (penaliza uso de energía) |
| $R_d$ | 2×2 | Matriz de peso para cambios de control (penaliza aceleraciones bruscas/suavidad) |
| $A_d$ | 3×3 | Matriz de transición de estados discretizada |
| $B_d$ | 3×2 | Matriz de entrada de control discretizada |
| $A_{aug}$ | 5×5 | Matriz de transición del sistema aumentado |
| $B_{aug}$ | 5×2 | Matriz de entrada del sistema aumentado |
| $C$ | 3×5 | Matriz de salida (extrae solo $[x, y, \theta]$ del estado aumentado) |
| $S_x$ | 3N×5 | Matriz de predicción: efecto del estado inicial en estados futuros |
| $S_u$ | 3N×2N | Matriz de predicción: efecto de los controles en estados futuros |
| $P$ | 2N×2N | Matriz Hessiana del problema QP |
| $q$ | 2N×1 | Vector gradiente del problema QP |
| $\bar{Q}$ | 3N×3N | Matriz de peso del error de estado extendida para todo el horizonte |
| $\bar{R}_d$ | 2N×2N | Matriz de peso de incrementos de control extendida para todo el horizonte (solo $R_d$, no $R$) |
| $v_{ref}$ | - | Velocidad lineal de referencia (obtenida de la odometría actual del robot) |
| $\omega_{ref}$ | - | Velocidad angular de referencia (obtenida de la odometría actual del robot) |
| $\theta_{ref}$ | - | Orientación de referencia (obtenida del lookahead point en la trayectoria) |

### Nota importante sobre referencias y incrementos:

- **$v_{ref}$ y $\omega_{ref}$**: Son las velocidades actuales del robot obtenidas de la odometría (`current_odom_.twist.twist`). Se usan para linealizar el modelo dinámico alrededor del punto de operación actual.

- **$\theta_{ref}$**: Es la orientación del punto de lookahead en la trayectoria deseada (`desired_state(2)`). Se usa para linealizar las ecuaciones cinemáticas del robot.

- **$\Delta u_i$**: Son incrementos **respecto al control anterior**. Es decir:
  - $\Delta u_0 = u_0 - u_{-1}$ (control actual menos control del ciclo anterior)
  - $\Delta u_1 = u_1 - u_0$ (siguiente control menos control actual)
  - En el código, `du_prev_` almacena el incremento aplicado en el ciclo anterior

- **Estado aumentado $\xi_k$**: Almacena $u_{k-1}$ (velocidad anterior), NO $\Delta u_{k-1}$ (incremento):
  - La memoria guarda la **velocidad absoluta anterior**: $u_{k-1} = [v_{k-1}, \omega_{k-1}]^T$
  - Esto permite calcular correctamente: $u_k = u_{k-1} + \Delta u_k$
  - La dinámica queda: $x_{k+1} = A_d \cdot x_k + B_d \cdot u_k$ ✓ (correcta)

- **Aplicación del control**: El control final que se publica es:
  ```
  u_actual = u_ref + du_prev + Δu_0^*
  ```
  Donde $\Delta u_0^*$ es el primer elemento de la solución óptima del solver.

---

## 1. **Fundamentos del MPC como Problema Cuadrático**

El MPC (Model Predictive Control) resuelve un problema de optimización en cada paso de tiempo. El objetivo es encontrar una secuencia de controles futuros que minimice un costo mientras se predicen los estados futuros del robot.

La función de costo a minimizar es:

$$
\min_{u_0, u_1, ..., u_{N-1}} J = \sum_{i=0}^{N-1} \left[ \|x_i - x_{ref}\|_Q^2 + \|\Delta u_i\|_{R_d}^2 \right]
$$

**Nota**: La implementación actual usa **Opción 2** (solo penaliza incrementos), donde:
- $\|x_i - x_{ref}\|_Q^2$: Penaliza la desviación del estado respecto a la referencia
- $\|\Delta u_i\|_{R_d}^2$: Penaliza cambios bruscos en el control (suavidad)
- **NO se penaliza** $\|u_i - u_{ref}\|_R^2$ (esfuerzo de control absoluto)

Esto significa que el robot busca movimientos suaves pero puede mantener velocidades altas constantes sin penalización.

Donde cada término tiene un propósito específico:

- **$\|x_i - x_{ref}\|_Q^2$**: Penaliza la desviación del estado respecto a la referencia (queremos estar cerca del camino deseado)
- **$\|\Delta u_i\|_{R_d}^2$**: Penaliza cambios bruscos en el control (queremos movimientos suaves)

**Interpretación física:**
- Si $Q$ es grande: el robot se esfuerza más por seguir exactamente la trayectoria
- Si $R_d$ es grande: el robot evita aceleraciones bruscas, moviéndose más suavemente

**Consecuencias de la Opción 2:**
- ✅ El robot genera movimientos suaves (sin cambios bruscos)
- ⚠️ El robot **no optimiza energía** - puede mantener velocidades altas constantes sin penalización
- ✅ Formulación más simple (no requiere matriz de acumulación $T_{cum}$)
- ✅ Menos costo computacional

---

## 2. **El Estado Aumentado**

Para poder formular el MPC en términos de incrementos de control $\Delta u$, utilizamos un **estado aumentado** que incluye memoria de la velocidad anterior.

### Estado original del robot:
$$
x = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix} \in \mathbb{R}^3
$$

### Estado aumentado:
$$
\xi = \begin{bmatrix} x \\ y \\ \theta \\ u_{v,prev} \\ u_{\omega,prev} \end{bmatrix} = \begin{bmatrix} x \\ u_{-1} \end{bmatrix} \in \mathbb{R}^5
$$

**Crítico**: El estado aumentado almacena $u_{-1}$ (velocidad anterior), **NO** $\Delta u_{-1}$ (incremento anterior).

Esto permite:
1. Calcular la velocidad actual: $u_k = u_{k-1} + \Delta u_k$
2. Mantener la dinámica correcta: $x_{k+1} = A_d \cdot x_k + B_d \cdot u_k$
3. Trabajar con incrementos $\Delta u$ como variables de decisión

### Modelo del robot sin aumentar:

El robot diferencial sigue la cinemática:

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = 
\begin{bmatrix} v \cos(\theta) \\ v \sin(\theta) \\ \omega \end{bmatrix}
$$

Linealizando alrededor de una trayectoria de referencia $(v_{ref}, \omega_{ref}, \theta_{ref})$ y discretizando con paso $\Delta t$:

$$
x_{k+1} = A_d \cdot x_k + B_d \cdot u_k
$$

Donde:

$$
A_d = \begin{bmatrix}
1 & 0 & -v_{ref} \sin(\theta_{ref}) \Delta t \\
0 & 1 & v_{ref} \cos(\theta_{ref}) \Delta t \\
0 & 0 & 1
\end{bmatrix}
$$

$$
B_d = \begin{bmatrix}
\cos(\theta_{ref}) \Delta t & 0 \\
\sin(\theta_{ref}) \Delta t & 0 \\
0 & \Delta t
\end{bmatrix}
$$

**Explicación de $A_d$:**
- La diagonal principal (1, 1, 1) representa que el estado "persiste" al siguiente instante
- El término $-v_{ref} \sin(\theta_{ref}) \Delta t$ en $(1,3)$: muestra cómo un cambio en $\theta$ afecta a $x$
- El término $v_{ref} \cos(\theta_{ref}) \Delta t$ en $(2,3)$: muestra cómo un cambio en $\theta$ afecta a $y$

**Explicación de $B_d$:**
- La primera columna muestra cómo la velocidad lineal $v$ afecta a la posición
- La segunda columna muestra cómo la velocidad angular $\omega$ afecta a la orientación

### Modelo aumentado:

**Ecuación de evolución del estado aumentado:**

$$
\xi_{k+1} = \begin{bmatrix} x_{k+1} \\ u_k \end{bmatrix} = 
\begin{bmatrix}
A_d & B_d \\
0_{2×3} & I_2
\end{bmatrix}
\begin{bmatrix} x_k \\ u_{k-1} \end{bmatrix} +
\begin{bmatrix}
B_d \\
I_2
\end{bmatrix}
\Delta u_k
$$

**Nota crítica:** La submatriz inferior derecha es $I_2$ (identidad), **NO** $0_{2×2}$. Esto implementa la actualización de velocidad: $u_k = u_{k-1} + \Delta u_k$.

**Descomponiendo en dos ecuaciones separadas:**

#### **Ecuación 1: Dinámica del robot**
$$
x_{k+1} = A_d \cdot x_k + B_d \cdot u_{k-1} + B_d \cdot \Delta u_k
$$

Simplificando (usando $u_k = u_{k-1} + \Delta u_k$):
$$
x_{k+1} = A_d \cdot x_k + B_d \cdot u_k \quad \text{✓ Dinámica correcta}
$$

Esta es la cinemática correcta del robot: el siguiente estado solo depende del control actual $u_k$, **NO** de $u_{k-2}$.

#### **Ecuación 2: Actualización de velocidad**
$$
u_k = 0_{2×3} \cdot x_k + I_2 \cdot u_{k-1} + I_2 \cdot \Delta u_k
$$

Simplificando:
$$
u_k = u_{k-1} + \Delta u_k \quad \text{✓ Definición de incremento}
$$

**Interpretación física:**

La segunda ecuación implementa la definición de incremento de control:
> "La velocidad actual es la velocidad anterior más el cambio que aplicamos"

**Ejemplo numérico:**

```
k = 0:
  u_{-1} = [0, 0]          (velocidad inicial: robot en reposo)
  Δu_0 = [0.1, 0.05]       (optimizador decide primer incremento)
  
  Actualización: u_0 = u_{-1} + Δu_0 = [0.1, 0.05] ✓
  Dinámica: x_1 = A_d·x_0 + B_d·u_0 ✓

k = 1:
  u_0 = [0.1, 0.05]        (velocidad anterior)
  Δu_1 = [0.08, 0.03]      (optimizador decide)
  
  Actualización: u_1 = u_0 + Δu_1 = [0.18, 0.08] ✓
  Dinámica: x_2 = A_d·x_1 + B_d·u_1 ✓ (solo depende de u_1, NO de u_{-1})

k = 2:
  u_1 = [0.18, 0.08]       (velocidad anterior)
  Δu_2 = [0.05, 0.02]      (optimizador decide)
  
  Actualización: u_2 = u_1 + Δu_2 = [0.23, 0.10] ✓
  Dinámica: x_3 = A_d·x_2 + B_d·u_2 ✓ (solo depende de u_2, NO de u_0)
```

**Ventajas de este enfoque:**

1. ✅ **Dinámica correcta**: $x_{k+1}$ solo depende de $u_k$ (no hay dependencia espuria de $u_{k-2}$)
2. ✅ **Simplicidad**: La matriz $I_2$ implementa automáticamente $u_k = u_{k-1} + \Delta u_k$
3. ✅ **Modelo lineal**: El estado aumentado evoluciona linealmente con $\Delta u_k$
4. ✅ **Penalización directa**: Podemos penalizar $\Delta u$ directamente en la función de costo

**Analogía en código:**

```python
# Estado aumentado en k
x_k = [x, y, theta]
u_k_minus_1 = [v_{k-1}, ω_{k-1}]  # Velocidad anterior (NO incremento)

# Optimizador decide el incremento
delta_u_k = optimizador.solve()

# Actualización de velocidad (implementada por I_2 en A_aug)
u_k = u_k_minus_1 + delta_u_k

# Dinámica del robot (correcta)
x_k_plus_1 = A_d @ x_k + B_d @ u_k  # ✓ Solo depende de u_k

# Estado aumentado en k+1
xi_k_plus_1 = [x_k_plus_1, u_k]
```

---

## 3. **Matrices de Predicción (El Corazón del MPC)**

Las matrices de predicción relacionan **todos los estados futuros en el horizonte** con el **estado actual** y los **controles futuros que queremos encontrar**.

### Ecuación de predicción:

$$
\underbrace{\begin{bmatrix}
x_1 \\ x_2 \\ \vdots \\ x_N
\end{bmatrix}}_{X \in \mathbb{R}^{3N}} = 
\underbrace{\begin{bmatrix}
C A_{aug} \\
C A_{aug}^2 \\
\vdots \\
C A_{aug}^N
\end{bmatrix}}_{S_x \in \mathbb{R}^{3N×5}}
\xi_0 +
\underbrace{\begin{bmatrix}
C B_{aug} & 0 & \cdots & 0 \\
C A_{aug} B_{aug} & C B_{aug} & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
C A_{aug}^{N-1} B_{aug} & C A_{aug}^{N-2} B_{aug} & \cdots & C B_{aug}
\end{bmatrix}}_{S_u \in \mathbb{R}^{3N×2N}}
\underbrace{\begin{bmatrix}
\Delta u_0 \\ \Delta u_1 \\ \vdots \\ \Delta u_{N-1}
\end{bmatrix}}_{\Delta U \in \mathbb{R}^{2N}}
$$

Donde la matriz $C$ extrae solo la parte del estado (sin los incrementos):

$$
C = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0
\end{bmatrix} \in \mathbb{R}^{3×5}
$$

### Ejemplo concreto con N=3 pasos:

Simplificando la notación ($A = A_{aug}$, $B = B_{aug}$):

$$
S_u = \begin{bmatrix}
CB & 0 & 0 \\
CAB & CB & 0 \\
CA^2B & CAB & CB
\end{bmatrix} \in \mathbb{R}^{9×6}
$$

**Interpretación física de $S_u$:**
- **Primera fila** ($CB$): El control $\Delta u_0$ afecta inmediatamente al estado $x_1$
- **Segunda fila** ($CAB$, $CB$): El estado $x_2$ es afectado por $\Delta u_0$ (propagado a través de $A$) y directamente por $\Delta u_1$
- **Tercera fila** ($CA^2B$, $CAB$, $CB$): El estado $x_3$ es afectado por todos los controles anteriores, cada uno propagado según cuántos pasos han pasado

**Interpretación física de $S_x$:**
- Muestra cómo el estado inicial $\xi_0$ se propaga hacia el futuro
- Cada bloque $CA^i$ representa el estado inicial después de $i$ pasos sin aplicar controles nuevos

### En el código:

```cpp
// Construcción de S_x
Eigen::MatrixXd A_pow = Eigen::MatrixXd::Identity(dim_aug, dim_aug);
for (int i = 0; i < N; ++i) {
    A_pow = A_pow * A_aug;  // A^(i+1)
    S_x.block(dim_x * i, 0, dim_x, dim_aug) = C_aug * A_pow;
}

// Construcción de S_u
for (int i = 0; i < N; ++i) {
    for (int j = 0; j <= i; ++j) {
        Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(dim_aug, dim_aug);
        for (int k = 0; k < i - j; ++k) {
            temp = temp * A_aug;  // A^(i-j)
        }
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C_aug * temp * B_aug;
    }
}
```

---

## 4. **Reformulación como Problema Cuadrático Estándar**

Ahora queremos convertir nuestra función de costo original en la forma estándar de un problema cuadrático (QP) que el solver OSQP puede resolver.

### Paso 1: Expresar el costo en términos de $\Delta U$

Sustituyendo $X = S_x \xi_0 + S_u \Delta U$ en la función de costo:

$$
J = \sum_{i=0}^{N-1} \left[ \|x_i - x_{ref}\|_Q^2 + \|\Delta u_i\|_{R_d}^2 \right]
$$

**Nota Opción 2**: Solo usamos $R_d$ (no $R + R_d$) porque solo penalizamos suavidad, no energía.

Esto se puede escribir en forma matricial:

$$
J = \|X\|_{\bar{Q}}^2 + \|\Delta U\|_{\bar{R}_d}^2
$$

Donde las matrices de peso extendidas son:

$$
\bar{Q} = \begin{bmatrix}
Q & & & \\
& Q & & \\
& & \ddots & \\
& & & Q
\end{bmatrix} \in \mathbb{R}^{3N×3N}
$$

$$
\bar{R}_d = \begin{bmatrix}
R_d & & & \\
& R_d & & \\
& & \ddots & \\
& & & R_d
\end{bmatrix} \in \mathbb{R}^{2N×2N}
$$

**Ejemplo con N=2, Q=diag(10,10,1), R_d=diag(10,10):**

$$
\bar{Q} = \begin{bmatrix}
10 & 0 & 0 & 0 & 0 & 0 \\
0 & 10 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 10 & 0 & 0 \\
0 & 0 & 0 & 0 & 10 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}, \quad
\bar{R}_d = \begin{bmatrix}
10 & 0 & 0 & 0 \\
0 & 10 & 0 & 0 \\
0 & 0 & 10 & 0 \\
0 & 0 & 0 & 10
\end{bmatrix}
$$

**Comparación con Opción 1 (no implementada):**
- **Opción 1**: $\bar{R} = diag(R + R_d, ..., R + R_d)$ - penaliza tanto energía como suavidad
- **Opción 2 (implementada)**: $\bar{R}_d = diag(R_d, ..., R_d)$ - solo penaliza suavidad

### Paso 2: Expandir la función de costo

$$
J = (S_x \xi_0 + S_u \Delta U)^T \bar{Q} (S_x \xi_0 + S_u \Delta U) + \Delta U^T \bar{R}_d \Delta U
$$

Expandiendo los productos:

$$
J = \xi_0^T S_x^T \bar{Q} S_x \xi_0 + 2 \xi_0^T S_x^T \bar{Q} S_u \Delta U + \Delta U^T S_u^T \bar{Q} S_u \Delta U + \Delta U^T \bar{R}_d \Delta U
$$

Agrupando términos cuadráticos y lineales en $\Delta U$:

$$
J = \frac{1}{2} \Delta U^T \underbrace{2(S_u^T \bar{Q} S_u + \bar{R}_d)}_{P} \Delta U + \underbrace{2(S_u^T \bar{Q} S_x \xi_0)^T}_{q^T} \Delta U + \underbrace{\xi_0^T S_x^T \bar{Q} S_x \xi_0}_{\text{constante}}
$$

Simplificando (el factor 2 se cancela con el 1/2):

$$
J = \frac{1}{2} \Delta U^T P \Delta U + q^T \Delta U + \text{constante}
$$

### Forma estándar QP:

$$
\min_{\Delta U} \quad \frac{1}{2} \Delta U^T P \Delta U + q^T \Delta U
$$

Sujeto a:
$$
l \leq A_{constraint} \Delta U \leq u
$$

**Interpretación:**
- $P$: Captura cómo cada par de controles interactúa (Hessiano). Es siempre simétrica y positiva definida.
- $q$: Captura la dirección de descenso basada en el estado actual y el error actual.
- La constante no afecta al mínimo, por lo que se ignora.

---

## 5. **Dimensiones Reales en el Código**

Con `horizon_steps_ = 10` (N=10):

| Variable | Dimensión | Contenido | Significado |
|----------|-----------|-----------|-------------|
| $P$ | 20×20 | Matriz densa/sparse | Hessiano: cómo se relacionan los 20 controles entre sí |
| $q$ | 20×1 | Vector denso | Gradiente: dirección óptima para los controles |
| $\Delta U$ | 20×1 | **Variables de decisión** | $[\Delta v_0, \Delta \omega_0, \Delta v_1, \Delta \omega_1, ..., \Delta v_9, \Delta \omega_9]^T$ |
| $\xi_0$ | 5×1 | Estado inicial | $[x_0, y_0, \theta_0, \Delta u_{-1,v}, \Delta u_{-1,\omega}]^T$ |
| $S_x$ | 30×5 | Matriz de predicción | Proyecta el estado inicial a 10 estados futuros (3 vars × 10 pasos) |
| $S_u$ | 30×20 | Matriz de predicción | Mapea 20 controles a 30 estados futuros |
| $\bar{Q}$ | 30×30 | Diagonal por bloques | Peso del error de estado para 10 pasos |
| $\bar{R}_d$ | 20×20 | Diagonal por bloques | Peso de incrementos de control para 10 pasos (solo $R_d$) |
| $A_{constraint}$ | 20×20 | Matriz identidad | Restricciones de caja (box constraints) sobre $\Delta U$ |
| $l, u$ | 40×1 | Límites | Cotas inferiores y superiores |

**Ejemplo numérico del vector de decisión:**

Si el solver encuentra:
$$
\Delta U^* = \begin{bmatrix}
0.15 \\ 0.05 \\ 0.12 \\ 0.03 \\ 0.10 \\ 0.02 \\ ... \\ 0.05 \\ 0.01
\end{bmatrix}
$$

Esto significa:
- En el paso 0: incrementar $v$ en 0.15 m/s y $\omega$ en 0.05 rad/s
- En el paso 1: incrementar $v$ en 0.12 m/s y $\omega$ en 0.03 rad/s
- ... (y así sucesivamente para los 10 pasos)

**Pero solo aplicamos el primero** ($\Delta v_0 = 0.15$, $\Delta \omega_0 = 0.05$) debido al principio de "horizonte receding".

---

## 6. **Restricciones Actuales**

Las restricciones limitan los valores que pueden tomar las variables de decisión $\Delta U$.

### En el código:

```cpp
for (int i = 0; i < N; ++i) {
    // Linear velocity constraints (du)
    l(dim_u * i) = -0.2;      // mín Δv
    u(dim_u * i) = 0.2;       // máx Δv
    
    // Angular velocity constraints (du)
    l(dim_u * i + 1) = -0.3;  // mín Δω
    u(dim_u * i + 1) = 0.3;   // máx Δω
}
```

### Forma matricial:

La matriz de restricciones $A_{constraint}$ en este caso es simplemente la identidad (porque son restricciones de caja - box constraints):

$$
\begin{bmatrix}
-0.2 \\ -0.3 \\ -0.2 \\ -0.3 \\ \vdots \\ -0.2 \\ -0.3
\end{bmatrix} \leq
\begin{bmatrix}
\Delta v_0 \\ \Delta \omega_0 \\ \Delta v_1 \\ \Delta \omega_1 \\ \vdots \\ \Delta v_9 \\ \Delta \omega_9
\end{bmatrix} \leq
\begin{bmatrix}
0.2 \\ 0.3 \\ 0.2 \\ 0.3 \\ \vdots \\ 0.2 \\ 0.3
\end{bmatrix}
$$

**Interpretación física:**
- $-0.2 \leq \Delta v_i \leq 0.2$: El robot puede acelerar o desacelerar hasta 0.2 m/s² en cada paso
- $-0.3 \leq \Delta \omega_i \leq 0.3$: El robot puede cambiar su velocidad angular hasta 0.3 rad/s² en cada paso

**¿Por qué limitar incrementos y no velocidades absolutas?**

Porque:
1. **Suavidad**: Limitar incrementos garantiza movimientos suaves sin cambios bruscos
2. **Realismo**: Los motores reales tienen aceleraciones limitadas
3. **Seguridad**: Evita comandos que puedan desestabilizar el robot

**Nota importante**: El código también limita las velocidades absolutas ajustando dinámicamente los límites de $\Delta u$ para asegurar que $u_k = u_{k-1} + \Delta u_k$ no exceda los límites de velocidad máxima.

---

## Restricciones de Velocidad Absoluta

### Implementación Actual

El código implementa **restricciones dinámicas** que consideran tanto límites de incrementos como límites de velocidad absoluta.

**Lo que hace:**

1. Calcula la velocidad acumulada actual: `v_current = v_ref + du_prev(0)`
2. Ajusta dinámicamente los límites de $\Delta u$ para respetar límites absolutos:
   - $\Delta v_{min} = \max(-max\_accel, 0 - v_{current})$
   - $\Delta v_{max} = \min(+max\_accel, v_{max} - v_{current})$
3. El solver OSQP encuentra $\Delta u_0^*$ óptimo respetando **ambos** límites
4. Se calcula: `u_v = Δu_0* + du_prev + v_ref`
5. Se aplica saturación como medida de seguridad adicional (no debería ser necesaria)

```cpp
// En build_mpc_matrices():
double v_current = u_ref(0) + du_prev_(0);
double w_current = u_ref(1) + du_prev_(1);

for (int i = 0; i < N; ++i) {
    // Límites para Δv considerando ambas restricciones
    double delta_v_min = std::max(-max_linear_accel_, 0.0 - v_current);
    double delta_v_max = std::min(max_linear_accel_, max_linear_vel_ - v_current);
    
    l(dim_u * i) = delta_v_min;
    u(dim_u * i) = delta_v_max;
    
    // Similar para Δω...
}
```

### Ventajas de este enfoque:

1. ✅ **El optimizador conoce los límites**: Las restricciones están en el problema QP
2. ✅ **Predicción correcta**: El MPC sabe qué velocidades reales se aplicarán
3. ✅ **Optimalidad**: La solución es óptima considerando **todos** los límites
4. ✅ **No hay sorpresas**: No hay saturaciones inesperadas post-optimización

### Formulación matemática:

Las restricciones implementadas son:

$$
0 \leq v_{current} + \Delta v_i \leq v_{max} \quad \forall i \in [0, N-1]
$$

$$
-\omega_{max} \leq \omega_{current} + \Delta \omega_i \leq \omega_{max} \quad \forall i \in [0, N-1]
$$

Y además:

$$
-a_{v,max} \leq \Delta v_i \leq a_{v,max}
$$

$$
-a_{\omega,max} \leq \Delta \omega_i \leq a_{\omega,max}
$$

El límite efectivo es la **intersección** de ambas restricciones, implementado con `std::max` y `std::min`.

#### Código en build_mpc_matrices():

```cpp
// Código real en time_constrained_mpc.cpp (ya implementado):
double v_current = u_ref(0) + du_prev_(0);
double w_current = u_ref(1) + du_prev_(1);

for (int i = 0; i < N; ++i) {
    // Linear velocity constraints (Δv)
    // Intersección de: [-a_max, +a_max] y [0-v_current, v_max-v_current]
    double delta_v_min = std::max(-max_linear_accel_, 0.0 - v_current);
    double delta_v_max = std::min(max_linear_accel_, max_linear_vel_ - v_current);
    
    l(dim_u * i) = delta_v_min;
    u(dim_u * i) = delta_v_max;
    
    // Angular velocity constraints (Δω)
    double delta_w_min = std::max(-max_angular_accel_, -max_angular_vel_ - w_current);
    double delta_w_max = std::min(max_angular_accel_, max_angular_vel_ - w_current);
    
    l(dim_u * i + 1) = delta_w_min;
    u(dim_u * i + 1) = delta_w_max;
}
```

**Nota sobre simplificación**: El código actual aplica los mismos límites a todos los pasos del horizonte, asumiendo que `v_current` no cambia. Una implementación más sofisticada acumularía los incrementos: $v_i = v_{current} + \sum_{j=0}^{i-1} \Delta v_j$, pero esto requeriría restricciones acopladas (no "box constraints").

---

## Resumen del Flujo Completo

1. **Entrada**: Estado actual $\xi_0$ y trayectoria deseada
2. **Construcción**: Matrices $P$, $q$, $A$, $l$, $u$ usando el modelo linealizado
3. **Optimización**: OSQP resuelve el problema y devuelve $\Delta U^*$
4. **Aplicación**: Se aplica solo $\Delta u_0^*$ (primer elemento de la solución)
5. **Control publicado**: $u_{cmd} = u_{ref} + du_{prev} + \Delta u_0^*$
6. **Repetición**: En el siguiente ciclo se vuelve a resolver con el nuevo estado

---

**Archivo creado:** `EXPLICA_FUNCIONAMIENTO.md`

Este documento proporciona la base teórica necesaria para entender cómo funciona el MPC actual y será la referencia para implementar restricciones temporales en el futuro.
