# Non-Linear MPC Formulation for Differential Drive Robots

This document presents the mathematical foundation of the `time_constrained_mpc` controller. The system employs a **Successive Linearization Model Predictive Control (MPC)** strategy, formulated as a Quadratic Program (QP) and solved via **OSQP**.

## 1. System Dynamics

### 1.1 Continuous Kinematic Model
The robot is modeled as a standard unicycle (differential drive) system. The state vector is defined as $\mathbf{x} = [x, y, \theta]^\top$ and the control input as $\mathbf{u} = [v, \omega]^\top$.

$$
\dot{x} = v \cos(\theta) \\
\dot{y} = v \sin(\theta) \\
\dot{\theta} = \omega
$$

### 1.2 Sin/Cos State Representation
To avoid singularities associated with Euler angles (specifically the wrapping at $\pm \pi$), the orientation $\theta$ is embedded using its sine and cosine components. This maps the state space to $\mathbb{R}^4$:

$$
\mathbf{x}_{sc} = \begin{bmatrix} x \\ y \\ \sin(\theta) \\ \cos(\theta) \end{bmatrix}
$$

The derivatives are given by the chain rule:

$$
\dot{x} = v \cos(\theta) \\
\dot{y} = v \sin(\theta) \\
\dot{\sin(\theta)} = \cos(\theta) \cdot \omega \\
\dot{\cos(\theta)} = -\sin(\theta) \cdot \omega
$$

*Note: The geometric constraint $\sin^2(\theta) + \cos^2(\theta) = 1$ is implicitly maintained by the cost function tracking a reference trajectory that satisfies it, but is not strictly enforced as a hard constraint in the QP to maintain convexity.*

## 2. Linearization and Discretization

The non-linear dynamics $\dot{\mathbf{x}}_{sc} = f(\mathbf{x}_{sc}, \mathbf{u})$ are linearized around a reference operating point. We use the **current measured velocity** $\mathbf{u}_{ref} = [v_{ref}, \omega_{ref}]^\top$ and the reference trajectory orientation for linearization.

### 2.1 Jacobian Derivation
The Jacobian matrices $A_c = \frac{\partial f}{\partial \mathbf{x}}$ and $B_c = \frac{\partial f}{\partial \mathbf{u}}$ are:

$$
A_c = \begin{bmatrix} 
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & \omega_{ref} \\
0 & 0 & -\omega_{ref} & 0
\end{bmatrix}, \quad
B_c = \begin{bmatrix} 
\cos(\theta_{ref}) & 0 \\
\sin(\theta_{ref}) & 0 \\
0 & \cos(\theta_{ref}) \\
0 & -\sin(\theta_{ref})
\end{bmatrix}
$$

### 2.2 Discretization (Euler Forward)
For a sampling time $T_s$, the discrete-time matrices $A_d \approx I + A_c T_s$ and $B_d \approx B_c T_s$ are:

$$
A_d = \begin{bmatrix} 
1 & 0 & 0 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix} + T_s \cdot \text{CouplingTerms}^*
$$

*\*Note: In the implementation, we approximate the coupling of linear velocity to position using the reference orientation components.*

**Singularity Avoidance at Low Speed**:
When $v_{meas} \approx 0$, the Jacobian terms relating $\theta$ to $x,y$ vanish, rendering the system uncontrollable in the solver's view. To preserve rank and steerability, we impose a lower bound on the linearization velocity:
$$ v_{ref}^* = \text{sgn}(v_{ref}) \cdot \max(|v_{ref}|, 0.01 \text{ m/s}) $$

## 3. Augmented Formulation (Velocity Increments)

To penalize control smoothness (jerk/acceleration minimization) rather than absolute control effort, the system is augmented to include the previous control input $\mathbf{u}_{k-1}$ as part of the state.

### 3.1 Augmented State Vector
$$
\mathbf{\xi}_k = \begin{bmatrix} \mathbf{x}_k \\ \mathbf{u}_{k-1} \end{bmatrix} \in \mathbb{R}^6
$$

### 3.2 Optimization Variable
The solver optimizes the **control increments**:
$$
\Delta \mathbf{u}_k = \mathbf{u}_k - \mathbf{u}_{k-1}
$$

### 3.3 Augmented State-Space Model
The dynamics for $\mathbf{\xi}_{k+1}$ become:

$$
\begin{bmatrix} \mathbf{x}_{k+1} \\ \mathbf{u}_k \end{bmatrix} = 
\underbrace{\begin{bmatrix} A_d & B_d \\ 0_{2\times4} & I_{2\times2} \end{bmatrix}}_{\mathcal{A}}
\begin{bmatrix} \mathbf{x}_k \\ \mathbf{u}_{k-1} \end{bmatrix} + 
\underbrace{\begin{bmatrix} B_d \\ I_{2\times2} \end{bmatrix}}_{\mathcal{B}}
\Delta \mathbf{u}_k
$$

## 4. Optimization Problem (QP)

We solve the following Finite Horizon Optimal Control problem at each step $k$:

$$
\min_{\Delta \mathbf{U}} \quad \sum_{i=0}^{N_p} \| \mathbf{C}\mathbf{\xi}_{k+i} - \mathbf{r}_{k+i} \|^2_Q + \sum_{j=0}^{N_c-1} \| \Delta \mathbf{u}_{k+j} \|^2_{R_d}
$$

**Subject to:**
1.  **System Dynamics**: $\mathbf{\xi}_{k+i+1} = \mathcal{A}\mathbf{\xi}_{k+i} + \mathcal{B}\Delta \mathbf{u}_{k+i}$
2.  **Input Constraints** (Velocity & Acceleration):
    $$
    \mathbf{u}_{min} \leq \mathbf{u}_{k-1} + \sum_{j=0}^{i} \Delta \mathbf{u}_{k+j} \leq \mathbf{u}_{max}
    $$
    $$
    \Delta \mathbf{u}_{min} \leq \Delta \mathbf{u}_{k+i} \leq \Delta \mathbf{u}_{max}
    $$

### 4.1 Solver Implementation (OSQP)
*   **Hessian ($P$) Construction**: The dense Hessian $P = S_u^\top \bar{Q} S_u + \bar{R}$ is computed using Eigen.
*   **Triangular Filtering**: OSQP requires only the upper triangular part of $P$. The implementation explicitly filters these entries to ensure numerical stability.
*   **Constraint Handling**: Box constraints on $\Delta \mathbf{u}$ are dynamically adjusted based on the current accumulated velocity $\mathbf{u}_{prev}$ to enforce absolute velocity limits effectively:
    $$ \Delta u_{max}^{step} = \min(a_{max} \cdot T_s, \quad v_{max} - u_{prev}) $$

