# Internal Logic & Sin/Cos Representation

This document details the mathematical formulation and internal logic of the `time_constrained_mpc` controller.

## 1. State Representation

To avoid angular discontinuities (wrapping at $\pm\pi$) and singularities, the controller represents orientation using both sine and cosine.

### State Vector ($x_{aug}$)
The solver uses an **augmented state vector** of dimension 6:

$$
x_{aug} = \begin{bmatrix} 
x \\ 
y \\ 
\sin(\theta) \\ 
\cos(\theta) \\ 
v_{prev} \\ 
\omega_{prev} 
\end{bmatrix}
$$

- $x, y$: Robot position in map frame.
- $\sin(\theta), \cos(\theta)$: Orientation components.
- $v_{prev}, \omega_{prev}$: Control inputs applied at the *previous* step.

### Control Vector ($\Delta u$)
The optimizer solves for **control increments** (dimension 2), not absolute velocities:

$$
\Delta u = \begin{bmatrix} 
\Delta v \\ 
\Delta \omega 
\end{bmatrix}
$$

This formulation forces the cost function to penalize *changes* in velocity (acceleration/jerk), promoting smoothness.

## 2. Differential Drive Dynamics

The continuous kinematic model is:

$$
\begin{align}
\dot{x} &= v \cos \theta \\
\dot{y} &= v \sin \theta \\
\dot{\sin \theta} &= \cos \theta \cdot \omega \\
\dot{\cos \theta} &= -\sin \theta \cdot \omega
\end{align}
$$

### Linearization
The MPC linearizes this model around the **current robot velocity** ($u_{ref}$).
*Crucially*, if the robot is stopped ($v \approx 0$), the model would lose steerability (changing $\theta$ wouldn't affect $x, y$). To prevent this, we enforce a minimum linearization velocity:
$$ |v_{ref}| = \max(|v_{measured}|, 0.1) $$

The discrete linearized matrices $A_d$ (4x4) and $B_d$ (4x2) are derived using Euler forward integration:

$$
x_{k+1} \approx A_d x_k + B_d u_k
$$

Where terms like $\Delta x \approx v_{ref} \cos(\theta_{ref}) \Delta t$ appear in $A_d$ and terms like $\Delta x \approx \cos(\theta_{ref}) \Delta t \Delta v$ appear in $B_d$.

### Augmented Dynamics
To optimize increments $\Delta u$, we augment the system:

$$
\begin{bmatrix} x_{k+1} \\ u_k \end{bmatrix} = 
\begin{bmatrix} A_d & B_d \\ 0 & I \end{bmatrix} 
\begin{bmatrix} x_k \\ u_{k-1} \end{bmatrix} + 
\begin{bmatrix} B_d \\ I \end{bmatrix} \Delta u_k
$$

This is the standard form $X_{k+1} = A_{aug} X_k + B_{aug} \Delta U_k$.

## 3. Optimization Problem (OSQP)

We solve the following Quadratic Program (QP) at 10Hz:

$$
\min_{\Delta U} \sum_{k=0}^{N_p} \| x_k - x_{ref,k} \|^2_Q + \sum_{k=0}^{N_c} \| \Delta u_k \|^2_{R_d}
$$

### Constraints
1.  **Dynamics**: Enforced via the prediction matrices $S_x, S_u$.
2.  **Control Limits (Box)**:
    - Acceleration bounds: $\Delta u_{min} \le \Delta u \le \Delta u_{max}$
    - Velocity bounds: Implemented dynamically by clipping the acceleration bounds based on current velocity.

### Implicit Geometric Constraint
$$ \sin^2(\theta) + \cos^2(\theta) = 1 $$
**Usage Note**: This quadratic equality constraint is **NOT enforced** by the OSQP solver (which only handles linear constraints).
- **Consequence**: The state vector can technically drift off the unit circle.
- **Mitigation**: The MPC re-plans every 100ms. The predicted horizon is short enough that drift is negligible. The reference trajectory itself is on the unit circle, creating a "soft" constraint via the $Q$ matrix.

## 4. Implementation Details

- **Upper Triangular P**: OSQP requires the cost matrix $P$ to be upper triangular. The code explicitly filters the Eigen matrix to remove the lower triangular part before passing it to the C API.
- **Reference Generation**:
    - The controller receives a path with timestamps.
    - `get_reference_trajectory_horizon` interpolates this path to find exactly where the robot *should* be at $t, t+\Delta t, t+2\Delta t...$
    - This allows for "Time Constrained" behavior: if the robot is late, the reference is ahead, creating a larger error term that drives higher velocities.
