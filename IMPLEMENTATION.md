# Implementation Summary

## What Has Been Implemented

This is a complete, efficient Model Predictive Controller (MPC) for differential drive robots, optimized to run on resource-constrained platforms like the Raspberry Pi 4.

### Core Features Implemented ✅

1. **Efficient MPC Solver**
   - Uses Eigen for sparse matrix operations
   - OSQP quadratic programming solver (fast and warm-starting)
   - Linearized differential drive kinematics
   - Receding horizon control
   - State-space formulation with augmented states

2. **Path Following**
   - Adaptive lookahead point calculation based on velocity
   - Smooth path tracking with position and orientation control
   - Goal detection with configurable tolerances

3. **ROS2 Integration**
   - Lifecycle node for clean startup/shutdown
   - Nav2 FollowPath action server
   - TF2 integration for robot localization
   - Odometry subscription for velocity feedback
   - Velocity command publishing

4. **Configuration**
   - YAML-based parameter configuration
   - Runtime parameter updates
   - Tunable MPC weights (Q, R, R_d)
   - Velocity and acceleration limits

### File Structure

```
time_constrained_mpc/
├── CMakeLists.txt              # Build configuration with Eigen and OSQP
├── package.xml                 # ROS2 package dependencies
├── README.md                   # Comprehensive documentation
├── BUILD.md                    # Build and run instructions
├── config/
│   └── mpc_params.yaml        # Default parameters
├── include/
│   └── time_constrained_mpc/
│       └── time_constrained_mpc.hpp  # Header with class definition
├── src/
│   ├── time_constrained_mpc.cpp      # Main implementation
│   └── main.cpp                      # Entry point
├── launch/
│   └── time_constrained_mpc.launch.py  # Lifecycle launch file
└── scripts/
    └── send_example_path.py    # Example path sender
```

### Key Implementation Details

#### MPC Formulation

**State Vector:** `x = [x, y, θ]`
- Position (x, y) in global frame
- Orientation (θ)

**Control Vector:** `u = [v, ω]`
- Linear velocity (v)
- Angular velocity (ω)

**Augmented State:** `x_aug = [x, y, θ, Δu_v, Δu_ω]`
- Includes control increments for rate penalization

**Cost Function:**
```
J = Σ(||x_i - x_ref||²_Q + ||Δu_i||²_R + ||Δ²u_i||²_Rd)
```

**Constraints:**
- Box constraints on velocities
- Control rate limits for smooth motion

#### Differential Drive Kinematics

Linearized around reference trajectory:
```cpp
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = ω
```

Discretized using forward Euler:
```cpp
A_d = I + [0  0  -v*sin(θ)*dt]
          [0  0   v*cos(θ)*dt]
          [0  0   0           ]

B_d = [cos(θ)*dt  0  ]
      [sin(θ)*dt  0  ]
      [0          dt ]
```

#### OSQP Solver Integration

The QP problem is formulated as:
```
minimize    (1/2) * x^T * P * x + q^T * x
subject to  l <= A * x <= u
```

Where:
- `P` = Hessian matrix (from quadratic cost)
- `q` = Gradient vector
- `A` = Constraint matrix
- `l, u` = Lower and upper bounds

#### Performance Optimizations

1. **Sparse Matrix Storage**: Only non-zero elements stored
2. **Warm Starting**: Previous solution used as initial guess
3. **Limited Iterations**: Max 4000 iterations for real-time performance
4. **CSC Format**: Compressed Sparse Column for OSQP
5. **Efficient Eigen Operations**: No dynamic allocations in hot path

### What's NOT Implemented Yet (Future Work)

The following features are planned but not yet implemented:

1. **Time Constraints** ⏱️
   - Waypoint arrival time constraints
   - Time-optimal path following
   - Temporal coordination with other robots

2. **Obstacle Avoidance** 🚧
   - Dynamic obstacle detection
   - Collision avoidance constraints
   - Costmap integration

3. **Advanced Features** 📊
   - Adaptive horizon based on curvature
   - Terminal cost-to-go estimation
   - Learning-based model corrections
   - Multi-objective optimization

4. **Additional Constraints** 🔒
   - Acceleration limits
   - Jerk limits
   - State constraints (e.g., forbidden zones)

### Tuning Guide

#### For Your Specific Robot

1. **Measure your robot's capabilities**:
   - Maximum linear velocity
   - Maximum angular velocity
   - Acceleration limits

2. **Update parameters** in `config/mpc_params.yaml`:
   ```yaml
   max_linear_vel: <your_max_vel>
   max_angular_vel: <your_max_angular_vel>
   ```

3. **Tune MPC weights**:
   - Start with default values
   - Increase Q for tighter tracking
   - Increase R_d for smoother motion
   - Decrease R for more aggressive control

4. **Adjust horizon**:
   - Longer horizon = better but slower
   - Start with 10 steps, adjust based on performance

### Performance Benchmarks

Expected performance on Raspberry Pi 4:

| Metric | Value |
|--------|-------|
| Solve Time | < 10 ms (N=10) |
| CPU Usage | < 20% @ 10 Hz |
| Memory | < 50 MB |
| Control Frequency | 10-20 Hz |

### Testing Checklist

- [ ] Build successfully
- [ ] Node starts and configures
- [ ] TF transforms available
- [ ] Odometry being received
- [ ] Can send a simple path
- [ ] Robot follows the path
- [ ] Stops at goal
- [ ] Can handle path cancellation
- [ ] Parameters update at runtime
- [ ] Visualizes correctly in RViz2

### Integration with Nav2

This controller can be integrated into the Nav2 stack:

1. **As a Controller Plugin**: Implement the Nav2 controller interface
2. **As a Standalone Node**: Use with Nav2 planner output
3. **With Behavior Trees**: Integrate into BT for complex behaviors

### Known Limitations

1. **Assumes differential drive**: Won't work for other robot types
2. **Requires good odometry**: Relies on accurate velocity feedback
3. **No obstacle avoidance**: Only follows the given path
4. **Static parameters**: Most require node restart to change
5. **Single path at a time**: Can't handle multiple concurrent paths

### References and Inspirations

This implementation was inspired by:

1. **ai-winter/ros_motion_planning**
   - MPC structure and formulation
   - Cost function design
   - https://github.com/ai-winter/ros_motion_planning

2. **MPC-Berkeley/Racing-LMPC-ROS2**
   - Advanced MPC techniques
   - OSQP integration patterns
   - https://github.com/MPC-Berkeley/Racing-LMPC-ROS2

3. **Classical MPC Theory**
   - "Model Predictive Control" by Camacho & Bordons
   - "Predictive Control for Linear and Hybrid Systems" by Borrelli et al.

### Development Notes

The code follows these principles:

- **Efficiency First**: Optimized for embedded systems
- **Clean Architecture**: Separation of concerns
- **ROS2 Best Practices**: Lifecycle nodes, actions, parameters
- **Eigen Patterns**: Efficient linear algebra
- **Readability**: Well-commented and documented

### Contributing

Future contributors should focus on:

1. Adding time constraint formulations
2. Integrating costmap-based obstacle avoidance
3. Performance profiling and optimization
4. Adding more robot kinematic models
5. Creating comprehensive test suites

### License

TODO: Add license information

### Contact

For questions or issues:
- Author: Diego Cubillo
- Email: dcubillo@comillas.edu
- Repository: time_constrained_mpc
