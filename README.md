# Time-Constrained MPC Controller

This package implements a Model Predictive Controller (MPC) for differential drive robots, optimized for **temporal trajectory tracking** on resource-constrained platforms (Raspberry Pi 4). It ensures the robot reaches specific waypoints at specific times.

## Features

- **Temporal Tracking**: Follows a trajectory $(x, y, \theta, t)$, adjusting speed to meet timing constraints.
- **Sin/Cos state**: Uses $\sin(\theta)$ and $\cos(\theta)$ in the state vector to avoid singularities and discontinuities.
- **Efficient Solver**: Uses **OSQP** (1.0+) with sparse matrices for sub-10ms solve times.
- **Augmented State**: Penalizes control **increments** (smoothness) rather than absolute values.
- **Upper Triangular Filtering**: Handles OSQP's matrix requirements automatically.

## Installation

### Dependencies
1. **ROS 2** (Humble/Jazzy)
2. **Eigen3**
3. **OSQP** (System library)

```bash
# 1. Install system dependencies
sudo apt update
sudo apt install libeigen3-dev libosqp-dev has-osqp

# If libosqp-dev is not available (e.g. old Ubuntu), install from source:
# git clone --recursive https://github.com/osqp/osqp
# cd osqp/build && cmake .. && sudo make install
# sudo ldconfig

# 2. Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select time_constrained_mpc
source install/setup.bash
```

## Usage

### 1. Launch Node
```bash
ros2 launch time_constrained_mpc mpc_example.launch.py
```

### 2. Send Path with Timestamps
Use the `nav2_msgs/action/FollowPath` interface. The controller interpolates the path based on the current time to find the exact reference $(x_{ref}, y_{ref}, \theta_{ref})$ for the horizon.

## Parameters (`mpc_params.yaml`)

- `controller_frequency`: 10.0 Hz (default)
- `prediction_horizon_steps`: 10
- `control_horizon_steps`: 10
- `max_linear_vel`: 0.5 m/s
- `max_angular_vel`: 1.0 rad/s
- `Q_matrix_diag`: [10, 10, 1, 1] (Weights for x, y, sin, cos)
- `R_d_matrix_diag`: [10, 10] (Weights for $\Delta v, \Delta \omega$)

See `SINCOS_REPRESENTATION.md` for mathematical details.
