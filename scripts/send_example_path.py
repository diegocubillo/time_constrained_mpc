#!/usr/bin/env python3
"""
Example script to send a simple path to the MPC controller.
This demonstrates how to use the FollowPath action.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class PathFollowerExample(Node):
    def __init__(self):
        super().__init__('path_follower_example')
        self._action_client = ActionClient(self, FollowPath, '/follow_path')

    def send_circular_path(self, radius=2.0, num_points=20):
        """Send a circular path to the controller"""

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create path
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        # Generate circular path
        for i in range(num_points):
            angle = 2.0 * math.pi * i / num_points

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = radius * math.cos(angle)
            pose.pose.position.y = radius * math.sin(angle)
            pose.pose.position.z = 0.0

            # Set orientation tangent to the circle
            yaw = angle + math.pi / 2
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)

            path.poses.append(pose)

        # Create goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        self.get_logger().info(f'Sending path with {len(path.poses)} poses')

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_line_path(self, length=3.0, num_points=15, ms_increment=100):
        """Send a straight line path to the controller"""

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create path
        path = Path()
        path.header.frame_id = 'map'
        current_time = self.get_clock().now()
        path.header.stamp = current_time.to_msg()

        # Generate straight line path
        for i in range(num_points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            iteration_time = current_time + rclpy.duration.Duration(
                nanoseconds=ms_increment * 1e6 * i)
            pose.header.stamp = iteration_time.to_msg()

            # Position along x-axis
            pose.pose.position.x = length * i / (num_points - 1)
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0

            # Orientation along x-axis
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        # Create goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        self.get_logger().info(
            f'Sending line path with {len(path.poses)} poses'
            )

        velocity = length / ((num_points - 1) * (ms_increment / 1000.0))
        self.get_logger().info(
            f'Velocity between points: {velocity:.2f} m/s'
            )

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_sine_path(
            self,
            amplitude=1.0,
            wavelength=2.0,
            max_velocity=0.3,
            point_spacing=0.1):
        """
        Send a sine wave path to the controller
        
        Args:
            amplitude: Amplitude of the sine wave in meters
            wavelength: Wavelength of the sine wave in meters
            max_velocity: Maximum velocity in m/s for temporal constraints
            point_spacing: Distance between consecutive points in meters
        """

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Calculate length for 3 complete periods
        length = 3.0 * wavelength
        
        # Calculate number of points based on spacing
        num_points = int(length / point_spacing) + 1

        # Create path
        path = Path()
        path.header.frame_id = 'map'
        current_time = self.get_clock().now()
        path.header.stamp = current_time.to_msg()

        self.get_logger().info(
            f'Generating sine wave: {num_points} points, '
            f'{length:.2f}m length (3 periods)')

        # Pre-calculate positions to compute arc length
        positions = []
        for i in range(num_points):
            x = -point_spacing * i
            y = amplitude * math.sin(2 * math.pi * x / wavelength)
            positions.append((x, y))

        # Generate sine wave path (propagating towards negative x)
        accumulated_time = 0.0
        for i in range(num_points):
            x, y = positions[i]

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            
            # Calculate timestamp based on actual arc length and velocity
            pose_time = current_time + rclpy.duration.Duration(
                seconds=accumulated_time)
            pose.header.stamp = pose_time.to_msg()
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orientation tangent to the sine wave
            # For negative x progression: dx = -1.0
            dx = -1.0
            slope = (amplitude * (2 * math.pi / wavelength) *
                  math.cos(2 * math.pi * x / wavelength))
            # The y-component of the tangent vector is slope * dx
            dy = slope * dx
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)

            path.poses.append(pose)
            
            # Calculate actual distance to next point for time increment
            if i < num_points - 1:
                x_next, y_next = positions[i + 1]
                segment_length = math.sqrt(
                    (x_next - x)**2 + (y_next - y)**2)
                # Time increment based on actual segment length
                accumulated_time += segment_length / max_velocity

        # Calculate total path statistics
        total_distance = (num_points - 1) * point_spacing
        total_time = accumulated_time
        avg_velocity = total_distance / total_time if total_time > 0 else 0.0

        self.get_logger().info(
            f'Sine wave path: {len(path.poses)} poses, '
            f'Distance: {total_distance:.2f}m, '
            f'Time: {total_time:.2f}s, '
            f'Avg velocity: {avg_velocity:.2f}m/s')

        # Override the last pose orientation to face positive x (yaw = 0)
        # This tests the goal orientation preservation feature
        path.poses[-1].pose.orientation.x = 0.0
        path.poses[-1].pose.orientation.y = 0.0
        path.poses[-1].pose.orientation.z = 0.0
        path.poses[-1].pose.orientation.w = 1.0

        # Create goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Path following completed!')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance to goal: {feedback.distance_to_goal:.2f}m, '
            f'Speed: {feedback.speed:.2f}m/s')


def main(args=None):
    rclpy.init(args=args)

    node = PathFollowerExample()

    # Choose which path to send:
    # node.send_circular_path(radius=2.0, num_points=20)
    # node.send_line_path(length=8.0, num_points=16, ms_increment=1500)
    node.send_sine_path(
        amplitude=1.0,
        wavelength=2.0,
        max_velocity=0.3,
        point_spacing=0.1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
