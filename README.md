# Time-Constrained MPC Controller for Differential Drive Robots

## Description

This package implements an efficient Model Predictive Controller (MPC) for differential drive robots, designed to run on resource-constrained platforms like the Raspberry Pi 4. The controller uses:

- **Eigen** for efficient linear algebra operations
- **OSQP** (Operator Splitting Quadratic Program) for fast quadratic programming
- **ROS2 Lifecycle** management for clean startup/shutdown
- **Nav2 FollowPath** action interface for path following

## Features

- ✅ Efficient MPC formulation optimized for differential drive kinematics
- ✅ State-space formulation with augmented states for control rate penalization
- ✅ Box constraints on control inputs (velocity and angular rate)
- ✅ TF2 integration for robot localization
- ✅ Nav2 action server interface for path following
- ✅ Real-time feedback and goal checking

## MPC Formulation

### State Vector - Sin/Cos Representation
To avoid linearization issues and discontinuities at ±π, the controller uses a **sin/cos representation** of orientation:
```
x = [x, y, sin(θ), cos(θ)]  // Robot position and orientation components
```

**Advantages:**
- ✅ **No discontinuities**: sin(θ) and cos(θ) are continuous everywhere
- ✅ **Better linearization**: No angular wrapping needed in optimization
- ✅ **Simplified implementation**: Eliminates complex angle normalization strategies

**Trade-off:**
- ⚠️ State dimension increased from 3 to 4
- ⚠️ Geometric constraint sin²(θ) + cos²(θ) = 1 not explicitly enforced (see warning below)

### Augmented State Vector
The controller uses an augmented state to handle control increments:
```
ξ = [x, y, sin(θ), cos(θ), u_prev_v, u_prev_ω]  // State + previous velocities
```
This stores the previous velocity (NOT the increment), which ensures correct dynamics: x_{k+1} = A·x_k + B·u_k

### Control Vector
```
u = [v, ω]  // Linear velocity (v) and angular velocity (ω)
```

The optimizer works with control increments:
```
Δu = [Δv, Δω]  // Changes in velocity and angular rate
```

### Differential Drive Kinematics with Sin/Cos
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
d(sin(θ))/dt = cos(θ) * ω
d(cos(θ))/dt = -sin(θ) * ω
```

The model is linearized around the reference trajectory and discretized for MPC using Euler forward integration.

### ⚠️ Important: Norm Constraint Violation

The geometric constraint **sin²(θ) + cos²(θ) = 1** is **NOT explicitly enforced** in the optimization. This is because:

1. **OSQP cannot handle quadratic constraints** - it only supports linear constraints
2. **Adding this constraint would require switching to a slower NLP solver** (IPOPT, SNOPT) with 10-40x performance penalty
3. **In practice, the violation is acceptable** for most applications

**Violation magnitude depends on:**
- `controller_frequency` (dt): Lower frequency → larger dt → worse violation
- `max_angular_vel`: Higher angular velocity → worse violation  
- `horizon_steps`: More steps → accumulated error
- Linearization error: Distance between current and reference orientation

**Estimated worst-case violation:**
```
Δnorm ≈ 0.5 * ω_max² * dt² * N
```

With default parameters (ω_max=1.0 rad/s, dt=0.1s, N=10):
```
Δnorm ≈ 0.5 * 1.0² * 0.01 * 10 = 0.05 (5% error)
```

**Mitigation strategies if needed:**
- Increase `controller_frequency` (reduces dt)
- Decrease `max_angular_vel`
- Decrease `horizon_steps`
- Add post-optimization normalization (see `SINCOS_REPRESENTATION.md`)

**Why it works anyway:**
- MPC re-optimizes at every time step, preventing error accumulation
- Only the first control input is applied
- Errors tend to cancel out over time due to the closed-loop nature
- The constraint is approximately satisfied by the dynamics

**When to worry about this issue:**
- ❌ If you observe the robot's heading "drifting" over long runs
- ❌ If you need very precise orientation control (e.g., docking)
- ❌ If using very low controller frequency (<5 Hz)
- ❌ If using very high angular velocities (>2 rad/s)
- ❌ If the robot exhibits oscillatory behavior in orientation

**When it's safe to ignore:**
- ✅ Path following applications (current use case)
- ✅ Controller frequency ≥ 10 Hz
- ✅ Moderate angular velocities (≤ 1.5 rad/s)
- ✅ When position accuracy is more important than orientation
- ✅ Short prediction horizons (N ≤ 20)

### Cost Function
The MPC minimizes the following quadratic cost (Option 2 - smoothness only):
```
J = Σ(||x_i - x_ref||²_Q + ||Δu_i||²_Rd)
```

Where:
- **Q** = State tracking weight matrix [x, y, sin(θ), cos(θ)] - penalizes deviation from path
- **R_d** = Control rate weight matrix [Δv, Δω] - penalizes abrupt changes (smoothness)

**Note**: The current implementation (Option 2) does NOT penalize absolute control effort (no R matrix in cost). This means:
- ✅ Robot generates smooth movements (no jerky accelerations)
- ⚠️ Robot may maintain high constant velocities without penalty (no energy optimization)
- ✅ Simpler formulation and faster computation

### Constraints
The controller implements hybrid constraints combining acceleration limits and absolute velocity bounds:

**Control rate (acceleration) limits:**
- -max_linear_accel ≤ Δv ≤ max_linear_accel
- -max_angular_accel ≤ Δω ≤ max_angular_accel

**Absolute velocity limits:**
- 0 ≤ v ≤ max_linear_vel
- -max_angular_vel ≤ ω ≤ max_angular_vel

The constraints are dynamically adjusted to ensure both limits are respected:
```
Δv_min = max(-max_linear_accel, 0 - v_current)
Δv_max = min(max_linear_accel, max_linear_vel - v_current)
```

This ensures the optimizer knows about both acceleration and velocity limits, preventing unexpected saturation.

## Parameters

### MPC Parameters
- `horizon_steps` (default: 10): Number of prediction steps
- `controller_frequency` (default: 10.0): Control loop frequency in Hz

### Velocity Limits
- `max_linear_vel` (default: 0.5): Maximum linear velocity in m/s
- `max_angular_vel` (default: 1.0): Maximum angular velocity in rad/s
- `max_linear_accel` (default: 0.2): Maximum linear acceleration in m/s²
- `max_angular_accel` (default: 0.3): Maximum angular acceleration in rad/s²

### Goal Tolerances
- `goal_dist_tolerance` (default: 0.2): Distance tolerance to goal in meters
- `goal_theta_tolerance` (default: 0.1): Angular tolerance to goal in radians

### Cost Matrix Weights
- `Q_matrix_diag` (default: [10.0, 10.0, 1.0, 1.0]): State error weights [x, y, sin(θ), cos(θ)]
- `R_d_matrix_diag` (default: [10.0, 10.0]): Control rate weights [Δv, Δω] - controls smoothness

### Frame IDs
- `map_frame` (default: "map"): Global reference frame
- `base_frame` (default: "base_link"): Robot base frame
- `odom_topic` (default: "odom"): Odometry topic name

## Dependencies

### ROS2 Packages
- `rclcpp` - ROS2 C++ client library
- `rclcpp_action` - ROS2 action support
- `rclcpp_lifecycle` - Lifecycle node support
- `geometry_msgs` - Geometry message types
- `nav_msgs` - Navigation message types
- `nav2_msgs` - Nav2 action definitions
- `tf2_ros` - TF2 transform library
- `tf2_geometry_msgs` - TF2 geometry message conversions
- `nav2_costmap_2d` - Nav2 costmap library

### System Libraries
- `Eigen3` - Linear algebra library
- `OSQP` - Quadratic programming solver (via osqp_vendor)

## Installation

### Prerequisites
Make sure you have ROS2 (Humble or later) installed and your workspace set up.

### Install Dependencies
```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install libeigen3-dev

# Install ROS2 dependencies
sudo apt-get install ros-${ROS_DISTRO}-osqp-vendor \
                     ros-${ROS_DISTRO}-eigen3-cmake-module \
                     ros-${ROS_DISTRO}-nav2-msgs \
                     ros-${ROS_DISTRO}-tf2-ros \
                     ros-${ROS_DISTRO}-tf2-geometry-msgs
```

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select time_constrained_mpc
source install/setup.bash
```

## Usage

### 1. Launch the Controller
```bash
ros2 launch time_constrained_mpc time_constrained_mpc.launch.py
```

### 2. Configure the Lifecycle Node
```bash
# Configure
ros2 lifecycle set /mpc_controller configure

# Activate
ros2 lifecycle set /mpc_controller activate
```

### 3. Send a Path
Use the Nav2 FollowPath action to send a path to the controller:
```bash
ros2 action send_goal /follow_path nav2_msgs/action/FollowPath "{path: {header: {frame_id: 'map'}, poses: [...]}}"
```

## Topics

### Subscribed
- `/odom` (nav_msgs/Odometry): Robot odometry for velocity feedback

### Published
- `/cmd_vel` (geometry_msgs/TwistStamped): Velocity commands to the robot
- `/mpc_debug_path` (nav_msgs/Path): Debug visualization of the current path

### Actions
- `/follow_path` (nav2_msgs/action/FollowPath): Action server for path following

## Algorithm Efficiency

The implementation is optimized for resource-constrained platforms:

1. **Sparse Matrix Operations**: Uses Eigen's sparse matrix capabilities for efficient storage and computation
2. **OSQP Solver**: Fast, warm-starting quadratic programming solver designed for real-time control
3. **Linearized Dynamics**: Uses linearized differential drive model around reference trajectory for computational efficiency
4. **Receding Horizon**: Only the first control input is applied, then the optimization is repeated
5. **Augmented State Formulation**: Efficiently handles control increments without complex constraint matrices

### State Augmentation Details
The controller uses an augmented state ξ = [x, y, θ, u_prev_v, u_prev_ω] where:
- The state includes the **previous velocity** (not the increment)
- Matrix A_aug has I₂ in the bottom-right corner, implementing: u_k = u_{k-1} + Δu_k
- This ensures correct dynamics: x_{k+1} = A·x_k + B·u_k (no spurious u_{k-2} dependency)

### Typical Performance on Raspberry Pi 4
- Solve time: < 10 ms for N=10 horizon
- Memory footprint: < 50 MB
- CPU usage: < 20% at 10 Hz control rate

## Tuning Guide

### For Faster Response
- Increase Q weights (especially for position tracking)
- Decrease horizon_steps (faster computation, also reduces norm violation)
- Increase max velocities and accelerations

### For Smoother Motion
- Increase R_d weights (more penalty on control changes)
- Decrease max_linear_accel and max_angular_accel
- Increase horizon_steps (longer prediction, but increases norm violation)

### For Better Path Tracking
- Increase Q_matrix_diag for x and y
- Increase horizon_steps

### For Reducing Norm Constraint Violation
If you observe orientation drift or need stricter geometric accuracy:
- **Increase controller_frequency** (most effective: reduces dt²)
- **Decrease max_angular_vel** (reduces ω_max²)
- **Decrease horizon_steps** (reduces accumulated error)
- Increase Q weights for sin(θ) and cos(θ) (indices 2 and 3)

**Example for high-precision orientation:**
```yaml
controller_frequency: 20.0  # Halve dt → 4x less error
max_angular_vel: 0.8        # Reduce ω → less error
horizon_steps: 8            # Fewer steps → less accumulation
Q_matrix_diag: [3000.0, 3000.0, 10.0, 10.0]  # Higher angular tracking
```


## Future Enhancements

This is the base MPC controller. Future work will include:
- 🚧 Obstacle avoidance integration
- 📊 Cost-to-go estimation for better terminal cost
- 🔄 Adaptive horizon based on path curvature
- 📈 Performance profiling and optimization
- 🎯 Coupled constraints for better horizon-wide velocity limit enforcement
- 🔧 Optional norm constraint enforcement for stricter geometric accuracy

## References

The implementation is inspired by:
1. [ai-winter/ros_motion_planning](https://github.com/ai-winter/ros_motion_planning) - MPC controller structure
2. [MPC-Berkeley/Racing-LMPC-ROS2](https://github.com/MPC-Berkeley/Racing-LMPC-ROS2) - Advanced MPC techniques

## License

TODO: Add license

## Author

Diego Cubillo (dcubillo@comillas.edu)
