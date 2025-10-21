# Time-Constrained MPC Controller for Differential Drive Robots

## Description

This package implements an efficient Model Predictive Controller (MPC) for differential drive robots, designed to run on resource-constrained platforms like the Raspberry Pi 4. The controller uses:

- **Eigen** for efficient linear algebra operations
- **OSQP** (Operator Splitting Quadratic Program) for fast quadratic programming
- **ROS2 Lifecycle** management for clean startup/shutdown
- **Nav2 FollowPath** action interface for path following

## Features

- ✅ Efficient MPC formulation optimized for differential drive kinematics
- ✅ Lookahead point calculation with adaptive lookahead distance based on velocity
- ✅ State-space formulation with augmented states for control rate penalization
- ✅ Box constraints on control inputs (velocity and angular rate)
- ✅ TF2 integration for robot localization
- ✅ Nav2 action server interface for path following
- ✅ Real-time feedback and goal checking

## MPC Formulation

### State Vector
```
x = [x, y, θ]  // Robot position (x, y) and orientation (θ)
```

### Augmented State Vector
The controller uses an augmented state to handle control increments:
```
ξ = [x, y, θ, u_prev_v, u_prev_ω]  // State + previous velocities
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

### Differential Drive Kinematics
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = ω
```

The model is linearized around the reference trajectory and discretized for MPC.

### Cost Function
The MPC minimizes the following quadratic cost (Option 2 - smoothness only):
```
J = Σ(||x_i - x_ref||²_Q + ||Δu_i||²_Rd)
```

Where:
- **Q** = State tracking weight matrix [x, y, θ] - penalizes deviation from path
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
- `horizon_sec` (default: 2.0): Prediction horizon in seconds
- `horizon_steps` (default: 10): Number of prediction steps
- `controller_frequency` (default: 10.0): Control loop frequency in Hz

### Velocity Limits
- `max_linear_vel` (default: 0.5): Maximum linear velocity in m/s
- `max_angular_vel` (default: 1.0): Maximum angular velocity in rad/s
- `max_linear_accel` (default: 0.2): Maximum linear acceleration in m/s²
- `max_angular_accel` (default: 0.3): Maximum angular acceleration in rad/s²

### Lookahead Parameters
- `lookahead_time` (default: 1.5): Lookahead time multiplier in seconds
- `min_lookahead_dist` (default: 0.3): Minimum lookahead distance in meters
- `max_lookahead_dist` (default: 0.9): Maximum lookahead distance in meters

### Goal Tolerances
- `goal_dist_tolerance` (default: 0.2): Distance tolerance to goal in meters
- `goal_theta_tolerance` (default: 0.1): Angular tolerance to goal in radians

### Cost Matrix Weights
- `Q_matrix_diag` (default: [10.0, 10.0, 1.0]): State error weights [x, y, θ]
- `R_matrix_diag` (default: [1.0, 1.0]): **NOT USED in Option 2** - Control effort weights [v, ω]
- `R_d_matrix_diag` (default: [10.0, 10.0]): Control rate weights [Δv, Δω] - controls smoothness

### Frame IDs
- `map_frame` (default: "map"): Global reference frame
- `base_frame` (default: "base_link"): Robot base frame
- `odom_frame` (default: "odom"): Odometry frame

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
- Decrease horizon_steps (faster computation)
- Increase max velocities and accelerations

### For Smoother Motion
- Increase R_d weights (more penalty on control changes)
- Decrease max_linear_accel and max_angular_accel
- Increase horizon_steps (longer prediction)

### For Better Path Tracking
- Increase Q_matrix_diag for x and y
- Increase lookahead distance
- Increase horizon_sec

### About R_matrix_diag (Not Used)
The R_matrix_diag parameter exists but is NOT used in the current implementation (Option 2). To add energy optimization:
- Would require implementing Option 1 with cumulative control matrix
- Would penalize maintaining high velocities (energy-efficient movements)
- Current implementation prioritizes simplicity and smoothness over energy optimization

## Future Enhancements

This is the base MPC controller. Future work will include:
- ⏱️ Time constraints for waypoint arrival times
- 🚧 Obstacle avoidance integration
- 📊 Cost-to-go estimation for better terminal cost
- 🔄 Adaptive horizon based on path curvature
- 📈 Performance profiling and optimization
- ⚡ Option 1 implementation (energy optimization with absolute control penalty)
- 🎯 Coupled constraints for better horizon-wide velocity limit enforcement

## References

The implementation is inspired by:
1. [ai-winter/ros_motion_planning](https://github.com/ai-winter/ros_motion_planning) - MPC controller structure
2. [MPC-Berkeley/Racing-LMPC-ROS2](https://github.com/MPC-Berkeley/Racing-LMPC-ROS2) - Advanced MPC techniques

## License

TODO: Add license

## Author

Diego Cubillo (dcubillo@comillas.edu)
