# Build and Run Instructions

## Prerequisites

### System Dependencies

```bash
# Update package list
sudo apt-get update

# Install Eigen3
sudo apt-get install libeigen3-dev

# Install OSQP and other ROS2 dependencies
sudo apt-get install \
    ros-${ROS_DISTRO}-osqp-vendor \
    ros-${ROS_DISTRO}-eigen3-cmake-module \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-nav2-costmap-2d
```

Replace `${ROS_DISTRO}` with your ROS2 distribution (e.g., `humble`, `iron`, `jazzy`).

## Build

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build the package
colcon build --packages-select time_constrained_mpc --symlink-install

# Source the workspace
source install/setup.bash
```

### Build with Specific Options

```bash
# Build in release mode for better performance
colcon build --packages-select time_constrained_mpc \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build with verbose output for debugging
colcon build --packages-select time_constrained_mpc \
    --symlink-install \
    --event-handlers console_cohesion+
```

## Run

### Option 1: Using Launch File (Recommended)

The launch file automatically configures and activates the lifecycle node:

```bash
ros2 launch time_constrained_mpc time_constrained_mpc.launch.py
```

### Option 2: Manual Lifecycle Management

```bash
# Terminal 1: Start the node
ros2 run time_constrained_mpc time_constrained_mpc \
    --ros-args --params-file $(ros2 pkg prefix time_constrained_mpc)/share/time_constrained_mpc/config/mpc_params.yaml

# Terminal 2: Configure the node
ros2 lifecycle set /mpc_controller configure

# Terminal 3: Activate the node
ros2 lifecycle set /mpc_controller activate
```

## Send Test Path

### Using the Example Script

```bash
# Make the script executable
chmod +x src/time_constrained_mpc/scripts/send_example_path.py

# Run the example (sends a straight line path)
python3 src/time_constrained_mpc/scripts/send_example_path.py
```

### Using ROS2 CLI

Send a simple path using the command line:

```bash
ros2 action send_goal /follow_path nav2_msgs/action/FollowPath "
path:
  header:
    frame_id: 'map'
  poses:
  - header:
      frame_id: 'map'
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - header:
      frame_id: 'map'
    pose:
      position: {x: 1.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - header:
      frame_id: 'map'
    pose:
      position: {x: 2.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

## Debugging

### Check Node Status

```bash
# List lifecycle nodes
ros2 lifecycle nodes

# Get current state
ros2 lifecycle get /mpc_controller

# List available transitions
ros2 lifecycle list /mpc_controller
```

### Monitor Topics

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor debug path
ros2 topic echo /mpc_debug_path

# Check odometry
ros2 topic echo /odom
```

### View Active Parameters

```bash
# List all parameters
ros2 param list /mpc_controller

# Get specific parameter
ros2 param get /mpc_controller max_linear_vel

# Set parameter at runtime
ros2 param set /mpc_controller max_linear_vel 0.3
```

### Visualize in RViz2

```bash
# Launch RViz2
rviz2

# Add the following displays:
# - Path (/mpc_debug_path)
# - TF
# - Odometry (/odom)
```

## Troubleshooting

### Build Errors

#### "osqp not found"
```bash
# Install osqp_vendor
sudo apt-get install ros-${ROS_DISTRO}-osqp-vendor
```

#### "Eigen3 not found"
```bash
# Install Eigen3 and the cmake module
sudo apt-get install libeigen3-dev ros-${ROS_DISTRO}-eigen3-cmake-module
```

#### "tf2_geometry_msgs not found"
```bash
sudo apt-get install ros-${ROS_DISTRO}-tf2-geometry-msgs
```

### Runtime Errors

#### "No transform available"
Make sure your robot is publishing:
- TF transforms from `map` → `odom` → `base_link`
- Or configure the correct frame IDs in the parameters

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Echo a specific transform
ros2 run tf2_ros tf2_echo map base_link
```

#### "No odometry received"
Check that odometry is being published:

```bash
# List topics
ros2 topic list | grep odom

# Echo odometry
ros2 topic echo /odom
```

#### "MPC solver failed"
- Check that the lookahead point is reasonable
- Reduce horizon_steps if computation is too slow
- Increase solver tolerance in the code if needed

### Performance Issues on Raspberry Pi

If the controller is too slow:

1. **Reduce horizon steps**:
   ```bash
   ros2 param set /mpc_controller horizon_steps 5
   ```

2. **Build in Release mode**:
   ```bash
   colcon build --packages-select time_constrained_mpc \
       --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. **Lower controller frequency**:
   ```bash
   ros2 param set /mpc_controller controller_frequency 5.0
   ```

4. **Use CPU governor for performance**:
   ```bash
   sudo cpufreq-set -g performance
   ```

## Next Steps

After getting the basic controller working:

1. **Tune the MPC weights** for your robot
2. **Adjust velocity limits** based on your robot's capabilities
3. **Test with different path types** (straight, curved, circular)
4. **Integrate with Nav2** for full autonomous navigation
5. **Add obstacle avoidance** (future work)
6. **Implement time constraints** (future work)

## Uninstall

```bash
# Remove the package
cd ~/ros2_ws
colcon build --packages-select time_constrained_mpc --cmake-target uninstall

# Or manually remove
rm -rf install/time_constrained_mpc
rm -rf build/time_constrained_mpc
```
