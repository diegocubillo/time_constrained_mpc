# Explicación Detallada del Funcionamiento del MPC

## Tabla de Nomenclatura

| Símbolo | Dimensión | Descripción |
|---------|-----------|-------------|
| $x_i$ | 3×1 | Estado del robot en el paso $i$: $[x, y, \theta]^T$ (posición x, y y orientación) |
| $u_i$ | 2×1 | Control aplicado en el paso $i$: $[v, \omega]^T$ (velocidad lineal y angular) |
| $\Delta u_i$ | 2×1 | Incremento de control: $u_i - u_{i-1}$ (cambio respecto al control anterior) |
| $z_k$ | 2×1 | Variable de memoria del estado aumentado: $z_k = \Delta u_{k-1}$ (almacena el incremento anterior) |
| $\xi_k$ | 5×1 | Estado aumentado: $\xi_k = [x_k, y_k, \theta_k, z_{k,v}, z_{k,\omega}]^T = [x_k; z_k]$ donde $z_k = \Delta u_{k-1}$ |
| $N$ | - | Número de pasos del horizonte de predicción (ej: 10) |
| $\Delta t$ | - | Paso de tiempo del control (ej: 0.1s) |
| $Q$ | 3×3 | Matriz de peso para el error de estado (penaliza desviaciones de posición/orientación) |
| $R$ | 2×2 | Matriz de peso para el control (penaliza uso de energía) |
| $R_d$ | 2×2 | Matriz de peso para cambios de control (penaliza aceleraciones bruscas) |
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
| $\bar{R}$ | 2N×2N | Matriz de peso del control extendida para todo el horizonte |
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
\min_{u_0, u_1, ..., u_{N-1}} J = \sum_{i=0}^{N-1} \left[ \|x_i - x_{ref}\|_Q^2 + \|u_i - u_{ref}\|_R^2 + \|\Delta u_i\|_{R_d}^2 \right]
$$

Donde cada término tiene un propósito específico:

- **$\|x_i - x_{ref}\|_Q^2$**: Penaliza la desviación del estado respecto a la referencia (queremos estar cerca del camino deseado)
- **$\|u_i - u_{ref}\|_R^2$**: Penaliza el uso de control (queremos usar poca energía)
- **$\|\Delta u_i\|_{R_d}^2$**: Penaliza cambios bruscos en el control (queremos movimientos suaves)

**Interpretación física:**
- Si $Q$ es grande: el robot se esfuerza más por seguir exactamente la trayectoria
- Si $R$ es grande: el robot prefiere usar menos potencia aunque se desvíe un poco
- Si $R_d$ es grande: el robot evita aceleraciones bruscas, moviéndose más suavemente

---

## 2. **El Truco del Estado Aumentado**

Para manejar el término $\Delta u$ (cambios en el control) sin complicar matemáticamente el problema, se usa un "truco": **aumentar el vector de estado**.

### Estado original del robot:
$$
x = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix} \in \mathbb{R}^3
$$

### Estado aumentado:
$$
\xi = \begin{bmatrix} x \\ y \\ \theta \\ \Delta u_v \\ \Delta u_\omega \end{bmatrix} \in \mathbb{R}^5
$$

Ahora el "estado" incluye también el último cambio de control. Esto permite que el optimizador trabaje directamente con incrementos.

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

**Notación clara del estado aumentado:**

Definimos el estado aumentado como:
$$
\xi_k = \begin{bmatrix} x_k \\ z_k \end{bmatrix}
$$

Donde:
- $x_k \in \mathbb{R}^3$ es el estado físico del robot: $[x, y, \theta]^T$
- $z_k \in \mathbb{R}^2$ es la **memoria del incremento anterior**: $z_k = \Delta u_{k-1}$

**Ecuación de evolución del estado aumentado:**

$$
\xi_{k+1} = \begin{bmatrix} x_{k+1} \\ z_{k+1} \end{bmatrix} = 
\begin{bmatrix}
A_d & B_d \\
0_{2×3} & 0_{2×2}
\end{bmatrix}
\begin{bmatrix} x_k \\ z_k \end{bmatrix} +
\begin{bmatrix}
B_d \\
I_2
\end{bmatrix}
\Delta u_k
$$

**⚠️ Nota crítica:** La submatriz inferior derecha es $0_{2×2}$, **NO** $I_2$. Esto significa que $z_k$ NO se propaga al siguiente estado.

**Descomponiendo en dos ecuaciones separadas:**

#### **Ecuación 1: Dinámica del robot**
$$
x_{k+1} = A_d \cdot x_k + B_d \cdot z_k + B_d \cdot \Delta u_k
$$

Como $z_k = \Delta u_{k-1} = u_{k-1} - u_{k-2}$, esto es equivalente a:
$$
x_{k+1} = A_d \cdot x_k + B_d \cdot (u_{k-1} - u_{k-2}) + B_d \cdot (u_k - u_{k-1})
$$
$$
x_{k+1} = A_d \cdot x_k + B_d \cdot u_k - B_d \cdot u_{k-2}
$$

Esta ecuación describe la cinemática real del robot considerando el control actual y la "inercia" del control de hace 2 pasos.

#### **Ecuación 2: Actualización de memoria (simple asignación)**
$$
z_{k+1} = 0_{2×3} \cdot x_k + 0_{2×2} \cdot z_k + I_2 \cdot \Delta u_k
$$

Simplificando:
$$
z_{k+1} = \Delta u_k
$$

**Verificación en términos de velocidades absolutas:**
$$
z_{k+1} = u_k - u_{k-1} \quad \text{✓ Correcto por definición}
$$

**Interpretación rigurosa:**

La segunda ecuación **NO** es una ecuación dinámica. Es una **asignación directa** que dice:

> "La variable de memoria $z$ en el siguiente paso es simplemente el incremento de control que aplicamos ahora"

**Verificación completa en términos de velocidades absolutas:**

```
k = 0:
  u_{-1} = [0, 0]       (velocidad inicial)
  u_0 = [0.5, 0.1]      (optimizador decide)
  z_0 = u_{-1} - u_{-2} = [0, 0]
  Δu_0 = u_0 - u_{-1} = [0.5, 0.1]
  
  Actualización: z_1 = Δu_0 = u_0 - u_{-1} = [0.5, 0.1] ✓

k = 1:
  u_0 = [0.5, 0.1]      (control previo)
  u_1 = [0.6, 0.15]     (optimizador decide)
  z_1 = u_0 - u_{-1} = [0.5, 0.1]
  Δu_1 = u_1 - u_0 = [0.1, 0.05]
  
  Actualización: z_2 = Δu_1 = u_1 - u_0 = [0.1, 0.05] ✓

k = 2:
  u_1 = [0.6, 0.15]     (control previo)
  u_2 = [0.65, 0.18]    (optimizador decide)
  z_2 = u_1 - u_0 = [0.1, 0.05]
  Δu_2 = u_2 - u_1 = [0.05, 0.03]
  
  Actualización: z_3 = Δu_2 = u_2 - u_1 = [0.05, 0.03] ✓
```

**Por qué $z_k$ NO se suma:**

Si tuviéramos $z_{k+1} = z_k + \Delta u_k$, entonces:
$$z_{k+1} = (u_{k-1} - u_{k-2}) + (u_k - u_{k-1}) = u_k - u_{k-2}$$

Pero por definición queremos:
$$z_{k+1} = \Delta u_k = u_k - u_{k-1}$$

**Por lo tanto, la matriz correcta debe tener $0_{2×2}$ en la posición inferior derecha, NO $I_2$.**

**Formulación en términos de velocidades absolutas:**

La dinámica completa del sistema en términos de las velocidades absolutas $u_k$ es:

$$
\xi_{k+1} = \begin{bmatrix} 
A_d \cdot x_k + B_d \cdot u_k - B_d \cdot u_{k-2} \\ 
u_k - u_{k-1}
\end{bmatrix}
$$

Esto muestra claramente que:
1. El estado del robot depende del control actual $u_k$ y el control de hace 2 pasos $u_{k-2}$
2. La memoria simplemente almacena la diferencia entre controles consecutivos

**Analogía en código:**

```python
# Estado aumentado en k
x_k = [x, y, theta]
z_k = u_k_minus_1 - u_k_minus_2  # Memoria = Δu_{k-1}

# Optimizador decide el incremento
delta_u_k = optimizador.solve()

# Calculamos velocidad absoluta nueva
u_k = u_k_minus_1 + delta_u_k

# Dinámica del robot (usa u_k y u_k_minus_2)
x_k_plus_1 = A_d @ x_k + B_d @ u_k - B_d @ u_k_minus_2

# Actualización de memoria (simple asignación)
z_k_plus_1 = u_k - u_k_minus_1  # = delta_u_k

# Estado aumentado en k+1
xi_k_plus_1 = [x_k_plus_1, z_k_plus_1]

# Guardar para próxima iteración
u_k_minus_2 = u_k_minus_1
u_k_minus_1 = u_k
```

**Ventaja del estado aumentado:**

Al mantener $\Delta u_{k-1}$ como parte del estado $\xi_k$, podemos:
1. Expresar la predicción futura únicamente en términos de $\Delta u_0, \Delta u_1, ..., \Delta u_{N-1}$
2. Penalizar cambios bruscos de control sin necesitar almacenar histórico externo
3. Mantener un modelo lineal (estado aumentado evoluciona linealmente con $\Delta u_k$)

### Ejemplo Numérico Completo (con velocidades absolutas)

```
Instante k=0:
  u_{-1} = [0, 0]       (robot en reposo)
  u_0 = ?               (por determinar)
  x₀ = [0, 0, 0]ᵀ
  z₀ = Δu_{-1} = [0, 0]ᵀ
  
  Optimizador decide: Δu₀ = [0.1, 0.05]ᵀ
  Velocidad aplicada: u₀ = u_{-1} + Δu₀ = [0.1, 0.05]ᵀ
  
Instante k=1:
  u_0 = [0.1, 0.05]ᵀ    (velocidad anterior)
  u_1 = ?               (por determinar)
  
  Robot se movió: x₁ = A_d·x₀ + B_d·u₀ = [0.01, 0.0, 0.005]ᵀ
  Memoria: z₁ = Δu₀ = u₀ - u_{-1} = [0.1, 0.05]ᵀ
  Estado aumentado: ξ₁ = [0.01, 0.0, 0.005, 0.1, 0.05]ᵀ
  
  Optimizador decide: Δu₁ = [0.08, 0.03]ᵀ
  Velocidad aplicada: u₁ = u₀ + Δu₁ = [0.18, 0.08]ᵀ
  
Instante k=2:
  u_1 = [0.18, 0.08]ᵀ   (velocidad anterior)
  
  Robot se movió: x₂ = A_d·x₁ + B_d·u₁ - B_d·u_{-1} = [0.03, 0.001, 0.013]ᵀ
  Memoria: z₂ = Δu₁ = u₁ - u₀ = [0.08, 0.03]ᵀ
  Estado aumentado: ξ₂ = [0.03, 0.001, 0.013, 0.08, 0.03]ᵀ
```

**Observación clave:** La variable $z_k$ simplemente almacena $\Delta u_{k-1} = u_{k-1} - u_{k-2}$. La actualización es una asignación directa, **NO** una suma:

$$z_{k+1} = \Delta u_k \quad \text{(NO es } z_{k+1} = z_k + \Delta u_k \text{)}$$

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
J = \sum_{i=0}^{N-1} \left[ \|x_i - x_{ref}\|_Q^2 + \|\Delta u_i\|_{R+R_d}^2 \right]
$$

(Nota: asumimos $u_{ref} = 0$ en el espacio de incrementos)

Esto se puede escribir en forma matricial:

$$
J = \|X\|_{\bar{Q}}^2 + \|\Delta U\|_{\bar{R}}^2
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
\bar{R} = \begin{bmatrix}
R+R_d & & & \\
& R+R_d & & \\
& & \ddots & \\
& & & R+R_d
\end{bmatrix} \in \mathbb{R}^{2N×2N}
$$

**Ejemplo con N=2, Q=diag(10,10,1), R=diag(1,1), R_d=diag(10,10):**

$$
\bar{Q} = \begin{bmatrix}
10 & 0 & 0 & 0 & 0 & 0 \\
0 & 10 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 10 & 0 & 0 \\
0 & 0 & 0 & 0 & 10 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}, \quad
\bar{R} = \begin{bmatrix}
11 & 0 & 0 & 0 \\
0 & 11 & 0 & 0 \\
0 & 0 & 11 & 0 \\
0 & 0 & 0 & 11
\end{bmatrix}
$$

### Paso 2: Expandir la función de costo

$$
J = (S_x \xi_0 + S_u \Delta U)^T \bar{Q} (S_x \xi_0 + S_u \Delta U) + \Delta U^T \bar{R} \Delta U
$$

Expandiendo los productos:

$$
J = \xi_0^T S_x^T \bar{Q} S_x \xi_0 + 2 \xi_0^T S_x^T \bar{Q} S_u \Delta U + \Delta U^T S_u^T \bar{Q} S_u \Delta U + \Delta U^T \bar{R} \Delta U
$$

Agrupando términos cuadráticos y lineales en $\Delta U$:

$$
J = \frac{1}{2} \Delta U^T \underbrace{2(S_u^T \bar{Q} S_u + \bar{R})}_{P} \Delta U + \underbrace{2(S_u^T \bar{Q} S_x \xi_0)^T}_{q^T} \Delta U + \underbrace{\xi_0^T S_x^T \bar{Q} S_x \xi_0}_{\text{constante}}
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
| $\bar{R}$ | 20×20 | Diagonal por bloques | Peso del control para 10 pasos |
| $A_{constraint}$ | 40×20 | Matriz de restricciones | Dos restricciones (min/max) por cada una de las 20 variables |
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

**Nota sobre las restricciones duplicadas en el código:**

El código actual tiene un bug donde duplica las restricciones:
```cpp
// Duplicate for upper bound constraints
l(dim_u * N + dim_u * i) = -0.2;  // ← Esto es redundante
u(dim_u * N + dim_u * i) = 0.2;
```

Esto debería corregirse porque OSQP solo necesita un conjunto de restricciones por variable cuando se usan cotas inferiores (`l`) y superiores (`u`) separadas.

---

## Punto Crítico: Limitación de Velocidad Absoluta

### Problema Actual

El código actual tiene una **limitación importante**: solo restringe los **incrementos** $\Delta u$, pero **NO restringe las velocidades absolutas** dentro del optimizador.

**Lo que hace actualmente:**

1. El solver OSQP encuentra $\Delta u_0^*$ óptimo (solo considerando límites de incrementos)
2. Se calcula: `u_v = Δu_0* + du_prev + v_ref`
3. **Después** se satura: `cmd.linear.x = std::clamp(u_v, 0.0, max_linear_vel_)`

```cpp
// En solve_mpc():
double u_v = work->solution->x[0] + du_prev_(0) + u_ref(0);
double u_w = work->solution->x[1] + du_prev_(1) + u_ref(1);

// Saturate controls
cmd.linear.x = std::clamp(u_v, 0.0, max_linear_vel_);        // ← SATURACIÓN POST-HOC
cmd.angular.z = std::clamp(u_w, -max_angular_vel_, max_angular_vel_);
```

### ¿Cuál es el problema?

**El optimizador no sabe que hay un límite de velocidad máxima.** 

Imagina esta situación:
- `v_ref = 0.4 m/s` (velocidad actual)
- `du_prev = 0.0`
- `max_linear_vel_ = 0.5 m/s`
- El solver encuentra: `Δu_0* = 0.15 m/s` (dentro del límite de incremento ±0.2)
- Velocidad calculada: `u_v = 0.15 + 0.0 + 0.4 = 0.55 m/s`
- **Saturación**: `cmd.linear.x = 0.5 m/s`

**Consecuencias:**

1. **El control aplicado NO es el óptimo**: El solver pensaba que iba a aplicar 0.55 m/s, pero realmente se aplica 0.5 m/s
2. **Predicción incorrecta**: En el siguiente ciclo, el modelo interno del MPC estará desincronizado con la realidad
3. **Suboptimalidad**: La trayectoria predicha no es la que realmente va a seguir el robot

### Solución Correcta: Restricciones de Velocidad Absoluta

Para resolver esto, **las restricciones de velocidad máxima deben estar DENTRO del problema de optimización**.

#### Formulación matemática:

Añadir restricciones:

$$
0 \leq u_{ref}(v) + \Delta u_{prev}(v) + \Delta u_i(v) \leq v_{max} \quad \forall i \in [0, N-1]
$$

$$
-\omega_{max} \leq u_{ref}(\omega) + \Delta u_{prev}(\omega) + \Delta u_i(\omega) \leq \omega_{max} \quad \forall i \in [0, N-1]
$$

#### En forma matricial:

Para cada paso $i$, añadir dos restricciones (una para $v$, otra para $\omega$):

$$
\begin{bmatrix}
1 & 0 & 0 & 0 & \cdots & 0 & 0 \\
0 & 1 & 0 & 0 & \cdots & 0 & 0
\end{bmatrix}
\begin{bmatrix}
\Delta v_0 \\ \Delta \omega_0 \\ \Delta v_1 \\ \Delta \omega_1 \\ \vdots \\ \Delta v_9 \\ \Delta \omega_9
\end{bmatrix}
\leq
\begin{bmatrix}
v_{max} - v_{ref} - \Delta u_{prev}(v) \\
\omega_{max} - \omega_{ref} - \Delta u_{prev}(\omega)
\end{bmatrix}
$$

Y restricciones de cota inferior:

$$
\begin{bmatrix}
1 & 0 & 0 & 0 & \cdots & 0 & 0 \\
0 & 1 & 0 & 0 & \cdots & 0 & 0
\end{bmatrix}
\begin{bmatrix}
\Delta v_0 \\ \Delta \omega_0 \\ \Delta v_1 \\ \Delta \omega_1 \\ \vdots \\ \Delta v_9 \\ \Delta \omega_9
\end{bmatrix}
\geq
\begin{bmatrix}
0 - v_{ref} - \Delta u_{prev}(v) \\
-\omega_{max} - \omega_{ref} - \Delta u_{prev}(\omega)
\end{bmatrix}
$$

#### Implementación en código:

```cpp
void MPCController::build_mpc_matrices(
    const Eigen::Vector3d &current_state,
    const Eigen::Vector3d &desired_state,
    const Eigen::Vector2d &u_ref,  // Velocidades actuales [v_ref, ω_ref]
    Eigen::SparseMatrix<double> &P,
    Eigen::VectorXd &q,
    Eigen::SparseMatrix<double> &A,
    Eigen::VectorXd &l,
    Eigen::VectorXd &u)
{
  // ... código existente para construir P y q ...
  
  // RESTRICCIONES MEJORADAS:
  // 1. Límites en incrementos: -0.2 ≤ Δv ≤ 0.2, -0.3 ≤ Δω ≤ 0.3
  // 2. Límites en velocidades absolutas: 0 ≤ v ≤ v_max, -ω_max ≤ ω ≤ ω_max
  
  const int n_constraints = 2 * dim_u * N;  // Dos restricciones por cada control
  A.resize(n_constraints, dim_u * N);
  l.resize(n_constraints);
  u.resize(n_constraints);
  
  std::vector<Eigen::Triplet<double>> triplets;
  
  for (int i = 0; i < N; ++i) {
    // Restricción para Δv_i
    triplets.push_back(Eigen::Triplet<double>(2*i, 2*i, 1.0));
    
    // Restricción para Δω_i
    triplets.push_back(Eigen::Triplet<double>(2*i + 1, 2*i + 1, 1.0));
  }
  
  A.setFromTriplets(triplets.begin(), triplets.end());
  
  // Velocidad acumulada actual (incluyendo incremento previo)
  double v_current = u_ref(0) + du_prev_(0);
  double w_current = u_ref(1) + du_prev_(1);
  
  for (int i = 0; i < N; ++i) {
    // Límites para Δv_i:
    // Inferior: max(-0.2, 0 - v_current)         ← No puede bajar más de 0.2 ni hacer v negativa
    // Superior: min(0.2, v_max - v_current)       ← No puede subir más de 0.2 ni exceder v_max
    l(2*i) = std::max(-0.2, 0.0 - v_current);
    u(2*i) = std::min(0.2, max_linear_vel_ - v_current);
    
    // Límites para Δω_i:
    // Inferior: max(-0.3, -ω_max - w_current)
    // Superior: min(0.3, ω_max - w_current)
    l(2*i + 1) = std::max(-0.3, -max_angular_vel_ - w_current);
    u(2*i + 1) = std::min(0.3, max_angular_vel_ - w_current);
    
    // Actualizar velocidad acumulada para el siguiente paso
    // (aproximación: asumimos que se aplicará el incremento máximo permitido)
    // Esto es una simplificación; idealmente deberíamos usar la solución previa
    v_current += 0.0;  // En la primera iteración no sabemos qué Δu se aplicará
    w_current += 0.0;
  }
}
```

### Comparación de Enfoques

| Aspecto | Enfoque Actual (Saturación Post-hoc) | Enfoque Correcto (Restricciones en QP) |
|---------|--------------------------------------|----------------------------------------|
| **Optimalidad** | ❌ No óptimo si hay saturación | ✅ Óptimo considerando todos los límites |
| **Predicción** | ❌ Puede desincronizarse | ✅ Coherente con el modelo |
| **Suavidad** | ⚠️ Puede causar discontinuidades | ✅ Suave por diseño |
| **Simplicidad** | ✅ Más simple de implementar | ⚠️ Más complejo |
| **Garantías** | ❌ No garantiza que el solver "sepa" de los límites | ✅ Garantías matemáticas |

### Recomendación

**Para un MPC robusto, deberías implementar las restricciones de velocidad absoluta dentro del problema QP.** 

Sin embargo, si la velocidad raramente se acerca a `v_max` en tu aplicación, la saturación post-hoc puede ser suficiente como solución temporal.

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
