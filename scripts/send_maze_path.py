#!/usr/bin/env python3
"""
Example script to send a maze path to the MPC controller.
This demonstrates how to use the FollowPath action with time constraints.
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

    def send_maze_path(self, num_points=50, point_spacing=0.5,
                       max_velocity=0.3):
        """
        Send a maze path to the controller

        Args:
            num_points: Number of points to send from the generated
                maze (default: 50)
            point_spacing: Distance between consecutive points in
                meters (default: 0.5)
            max_velocity: Maximum velocity in m/s for temporal
                constraints (default: 0.3)
        """

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Generate full maze (200 points)
        full_maze_points = self._generate_maze_waypoints(
            total_points=200, spacing=point_spacing)

        # Select only the first num_points
        maze_points = full_maze_points[:num_points]

        self.get_logger().info(
            f'Generated maze with {len(full_maze_points)} total points, '
            f'using first {len(maze_points)} points')

        # Create path with temporal constraints
        path = Path()
        path.header.frame_id = 'map'
        current_time = self.get_clock().now()
        path.header.stamp = current_time.to_msg()

        # Calculate time increment based on spacing and velocity
        time_increment_seconds = point_spacing / max_velocity

        self.get_logger().info(
            f'Point spacing: {point_spacing:.2f}m, '
            f'Max velocity: {max_velocity:.2f}m/s, '
            f'Time between points: {time_increment_seconds:.3f}s')

        # Generate path with timestamps
        accumulated_time = 0.0
        for i, (x, y, theta) in enumerate(maze_points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'

            # Calculate timestamp for this point
            pose_time = current_time + rclpy.duration.Duration(
                seconds=accumulated_time)
            pose.header.stamp = pose_time.to_msg()

            # Position
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orientation from theta
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)

            path.poses.append(pose)

            # Accumulate time for next point
            accumulated_time += time_increment_seconds

        # Calculate total path statistics
        total_distance = (len(maze_points) - 1) * point_spacing
        total_time = accumulated_time
        avg_velocity = total_distance / total_time if total_time > 0 else 0.0

        self.get_logger().info(
            f'Sending maze path: {len(path.poses)} poses, '
            f'Total distance: {total_distance:.2f}m, '
            f'Total time: {total_time:.2f}s, '
            f'Avg velocity: {avg_velocity:.2f}m/s')

        # Create goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(
            self.goal_response_callback)

    def _generate_maze_waypoints(self, total_points=200, spacing=0.5):
        """
        Generate maze waypoints with compact spiral pattern

        Returns:
            List of tuples (x, y, theta) representing waypoints
        """
        waypoints = []
        x, y = 0.0, 0.0
        direction = 0  # 0: East, 1: North, 2: West, 3: South

        # Compact spiral pattern: short segments that decrease
        # Start with segments of 4 points and create a spiral
        segment_lengths = [4, 4, 5, 5, 6, 6, 7, 7, 4, 4, 5, 5,
                           3, 3, 4, 4, 5, 5, 3, 3]
        segment_idx = 0
        points_in_segment = 0

        for i in range(total_points):
            # Calculate theta based on direction
            theta = direction * math.pi / 2.0  # 0, π/2, π, 3π/2

            waypoints.append((x, y, theta))

            # Move to next point
            if i < total_points - 1:  # Don't move after last point
                points_in_segment += 1

                # Check if we need to turn
                seg_len = segment_lengths[
                    segment_idx % len(segment_lengths)]
                if points_in_segment >= seg_len:
                    # Always turn left for spiral pattern
                    direction = (direction + 1) % 4

                    segment_idx += 1
                    points_in_segment = 0

                # Move in current direction
                if direction == 0:  # East
                    x += spacing
                elif direction == 1:  # North
                    y += spacing
                elif direction == 2:  # West
                    x -= spacing
                elif direction == 3:  # South
                    y -= spacing

        return waypoints

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

    # Send maze path with:
    # - 50 points from the generated 200-point maze
    # - 0.5m spacing between points
    # - 0.3 m/s maximum velocity
    node.send_maze_path(num_points=50, point_spacing=0.5,
                        max_velocity=0.3)

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
