# Corrección de la Matriz del Estado Aumentado

## Problema Detectado

La matriz del sistema aumentado tenía un error matemático fundamental en su formulación.

### ❌ Formulación INCORRECTA (anterior):

```cpp
A_aug.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();
```

Esto implicaba:
$$
A_{aug} = \begin{bmatrix}
A_d & B_d \\
0_{2×3} & I_2
\end{bmatrix}
$$

Lo que resultaba en: $z_{k+1} = z_k + \Delta u_k$

### ✅ Formulación CORRECTA (actual):

```cpp
// bottomRightCorner remains 0_{2×2}, NOT I_2
// (Ya inicializado a cero con MatrixXd::Zero())
```

Esto da:
$$
A_{aug} = \begin{bmatrix}
A_d & B_d \\
0_{2×3} & 0_{2×2}
\end{bmatrix}
$$

Lo que resulta en: $z_{k+1} = \Delta u_k$ ✓

---

## Demostración Matemática

### Verificación en términos de velocidades absolutas:

**Definiciones:**
- $u_k = [v_k, \omega_k]^T$: velocidad absoluta en el instante $k$
- $\Delta u_k = u_k - u_{k-1}$: incremento de velocidad
- $z_k = \Delta u_{k-1} = u_{k-1} - u_{k-2}$: memoria del incremento anterior

**Con la formulación incorrecta** ($z_{k+1} = z_k + \Delta u_k$):
$$
z_{k+1} = (u_{k-1} - u_{k-2}) + (u_k - u_{k-1}) = u_k - u_{k-2}
$$
❌ **Esto NO es la definición de $\Delta u_k$**

**Con la formulación correcta** ($z_{k+1} = \Delta u_k$):
$$
z_{k+1} = u_k - u_{k-1}
$$
✓ **Esta SÍ es la definición correcta de $\Delta u_k$**

---

## Ejemplo Numérico

```
Instante k=0:
  u_{-1} = [0.0, 0.0]
  u_0 = [0.5, 0.1]
  z_0 = [0.0, 0.0]
  Δu_0 = [0.5, 0.1]
  
  ✓ z_1 = Δu_0 = [0.5, 0.1]
  ❌ Con I_2: z_1 = z_0 + Δu_0 = [0.5, 0.1] (coincide por casualidad)

Instante k=1:
  u_0 = [0.5, 0.1]
  u_1 = [0.6, 0.15]
  z_1 = [0.5, 0.1]
  Δu_1 = [0.1, 0.05]
  
  ✓ z_2 = Δu_1 = [0.1, 0.05]
  ❌ Con I_2: z_2 = z_1 + Δu_1 = [0.6, 0.15] (¡ERROR! Esto es u_1, no Δu_1)

Instante k=2:
  u_1 = [0.6, 0.15]
  u_2 = [0.65, 0.18]
  z_2 = [0.1, 0.05]
  Δu_2 = [0.05, 0.03]
  
  ✓ z_3 = Δu_2 = [0.05, 0.03]
  ❌ Con I_2: z_3 = z_2 + Δu_2 = [0.65, 0.18] (¡ERROR! Esto es u_2, no Δu_2)
```

**Conclusión:** Con $I_2$, después del primer paso la variable $z$ deja de almacenar $\Delta u$ y empieza a acumular las velocidades absolutas, lo cual es incorrecto.

---

## Cambios Realizados

### 1. En `src/time_constrained_mpc.cpp`:

```cpp
// ANTES:
A_aug.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

// DESPUÉS:
// Note: bottomRightCorner remains 0_{2×2}, NOT I_2
// This ensures z_{k+1} = Δu_k (not z_{k+1} = z_k + Δu_k)
// (Línea eliminada, la matriz ya está inicializada a cero)
```

### 2. En `EXPLICA_FUNCIONAMIENTO.md`:

- Reescrita completamente la sección "Modelo aumentado"
- Añadida notación clara con $z_k = \Delta u_{k-1}$
- Incluida verificación matemática en términos de velocidades absolutas
- Agregados ejemplos numéricos completos
- Demostración de por qué $0_{2×2}$ es correcto y $I_2$ es incorrecto

---

## Impacto en el Controlador

### Comportamiento con el error (antes):

Después del primer ciclo de control, la variable $z$ ya no representa $\Delta u_{k-1}$ sino que se convierte en $u_{k-1}$. Esto causa:

1. **Predicción incorrecta**: Las matrices de predicción $S_x$ asumen que $z$ es un incremento, no una velocidad absoluta
2. **Penalización incorrecta**: El término $R_d$ penaliza $z$ como si fuera un incremento
3. **Acumulación no deseada**: $z$ crece sin límite en lugar de oscilar alrededor de cero

### Comportamiento correcto (ahora):

- $z_k$ siempre representa $\Delta u_{k-1} = u_{k-1} - u_{k-2}$
- Las predicciones son matemáticamente consistentes
- El modelo aumentado funciona como se diseñó originalmente

---

## Fecha de Corrección

**21 de octubre de 2025**

## Validación

Para validar que el cambio es correcto, se puede verificar que:

1. $z_k$ permanece pequeño (del orden de las aceleraciones)
2. La optimización converge más rápidamente
3. El robot tiene movimientos más suaves
4. Los logs muestran que $\|z_k\| \approx \|\Delta u_{k-1}\|$

---

**Autor:** Corrección detectada mediante análisis matemático riguroso
