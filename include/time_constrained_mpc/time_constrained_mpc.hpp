#pragma once

#include <memory>
#include <vector>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

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

  // Action feedback
  void update_feedback(const geometry_msgs::msg::PoseStamped &pose);

  // Goal check & reset
  bool goal_reached(const geometry_msgs::msg::PoseStamped &pose, const nav_msgs::msg::Path &path);
  void reset_state();

private:
  std::shared_ptr<rclcpp::TimerBase> timer_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  // std::shared_ptr<tf2_ros::Buffer> tf_;
  // std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  bool initialized_{false};

  // MPC parameters
  double max_linear_vel_;
  double max_angular_vel_;
  double horizon_sec_;
  int horizon_steps_;
};

}  // namespace mpc_controller