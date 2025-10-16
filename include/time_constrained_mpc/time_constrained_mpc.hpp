#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "bondcpp/bond.hpp"
#include "osqp/osqp.h"

using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<nav2_msgs::action::FollowPath>;


namespace mpc_controller
{

class MPCController : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MPCController();
  ~MPCController() override;

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Action handling (FollowPath)
  void handle_goal(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void handle_cancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  // Main control loop (timer callback)
  void control_loop();

  // Robot state acquisition
  geometry_msgs::msg::PoseStamped get_robot_pose();
  geometry_msgs::msg::Twist get_robot_velocity();

  // Path processing
  geometry_msgs::msg::PoseStamped calculate_lookahead_point();
  
  // MPC solver
  geometry_msgs::msg::Twist solve_mpc(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &vel,
    const nav_msgs::msg::Path &path);

  // Command publication
  void publish_velocity_command(const geometry_msgs::msg::Twist &cmd);

  // Path publication for debugging
  void publish_debug_path();

  // Action feedback
  void update_feedback(const geometry_msgs::msg::PoseStamped &pose);

  // Goal check & reset
  bool goal_reached(const geometry_msgs::msg::PoseStamped &pose, const nav_msgs::msg::Path &path);
  void reset_state();

  // Bond management
  void create_bond();
  void destroy_bond();
  void bond_timeout_callback();

private:
  // Helper methods for MPC
  Eigen::Vector2d differential_drive_model(const Eigen::Vector3d &state, 
                                           const Eigen::Vector2d &control, 
                                           double dt);
  void build_mpc_matrices(const Eigen::Vector3d &current_state,
                         const Eigen::Vector3d &desired_state,
                         const Eigen::Vector2d &u_ref,
                         Eigen::SparseMatrix<double> &P,
                         Eigen::VectorXd &q,
                         Eigen::SparseMatrix<double> &A,
                         Eigen::VectorXd &l,
                         Eigen::VectorXd &u);

  // ROS2 components
  std::shared_ptr<rclcpp::TimerBase> timer_path_pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_control_loop_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<nav2_msgs::action::FollowPath>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleFollowPath> current_goal_handle_;
  
  // Bond for heartbeat monitoring
  std::unique_ptr<bond::Bond> bond_;
  std::string bond_id_;
  bool bond_timeout_detected_{false};
  
  // State variables
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Odometry current_odom_;
  bool initialized_{false};
  bool has_odom_{false};
  Eigen::Vector2d du_prev_{Eigen::Vector2d::Zero()};

  // MPC parameters
  double max_linear_vel_;
  double max_angular_vel_;
  double horizon_sec_;
  int horizon_steps_;
  double d_t_;  // Control time step
  double lookahead_time_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double goal_dist_tolerance_;
  double goal_theta_tolerance_;
  bool use_stamped_cmd_vel_;  // Use TwistStamped (true) or Twist (false)
  
  // MPC weight matrices
  Eigen::Matrix3d Q_;  // State error weight
  Eigen::Matrix2d R_;  // Control weight
  Eigen::Matrix2d R_d_;  // Control rate weight
  
  // Frame IDs
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
};

}  // namespace mpc_controller