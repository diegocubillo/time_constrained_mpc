#include "time_constrained_mpc/time_constrained_mpc.hpp"

namespace mpc_controller
{

MPCController::MPCController()
: rclcpp_lifecycle::LifecycleNode("mpc_controller")
{
}

MPCController::~MPCController() = default;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_configure(const rclcpp_lifecycle::State & state)
{
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_debug_path", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("mpc_cmd_vel", 10);

  max_linear_vel_ = this->declare_parameter<double>("max_linear_vel", 0.5);
  max_angular_vel_ = this->declare_parameter<double>("max_angular_vel", 1.0);
  horizon_sec_ = this->declare_parameter<double>("horizon_sec", 2.0);
  horizon_steps_ = this->declare_parameter<int>("horizon_steps", 10);

  initialized_ = true;

  RCLCPP_INFO(get_logger(), "MPCController on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_activate(const rclcpp_lifecycle::State & state)
{
  path_pub_->on_activate();
  cmd_vel_pub_->on_activate();

  // Start main control loop timer
  timer_path_pub_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MPCController::control_loop, this)
  );

  RCLCPP_INFO(get_logger(), "MPCController on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_deactivate(const rclcpp_lifecycle::State & state)
{
  path_pub_->on_deactivate();
  cmd_vel_pub_->on_deactivate();

  if (timer_path_pub_) {
    timer_path_pub_->cancel();
  }

  RCLCPP_INFO(get_logger(), "MPCController on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_cleanup(const rclcpp_lifecycle::State & state)
{
  path_pub_.reset();
  cmd_vel_pub_.reset();
  timer_path_pub_.reset();
  RCLCPP_INFO(get_logger(), "MPCController on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_shutdown(const rclcpp_lifecycle::State & state)
{
  path_pub_.reset();
  cmd_vel_pub_.reset();
  timer_path_pub_.reset();
  RCLCPP_INFO(get_logger(), "MPCController on_shutdown() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ----- ACTION INTERFACE STUBS -----
void MPCController::handle_goal(const std::shared_ptr<GoalHandleFollowPath> /*goal_handle*/)
{
  // TODO: Implement action goal handling, set global_plan_, reset state, etc.
}

void MPCController::handle_cancel(const std::shared_ptr<GoalHandleFollowPath> /*goal_handle*/)
{
  // TODO: Handle cancellation of the action, stop robot, reset state
}

// ----- CONTROL LOOP -----
void MPCController::control_loop()
{
  if (!initialized_ || global_plan_.poses.empty()) {
    return;
  }

  // Get current robot pose and velocity
  auto pose = get_robot_pose();
  auto velocity = get_robot_velocity();

  // Calculate lookahead point
  auto lookahead = calculate_lookahead_point();

  // Solve MPC
  auto cmd = solve_mpc(pose, velocity, global_plan_);

  // Publish command
  publish_velocity_command(cmd);

  // Publish debug path (optional)
  path_pub_->publish(global_plan_);

  // Update feedback to action (stub)
  update_feedback(pose);

  // Check goal reached
  if (goal_reached(pose, global_plan_)) {
    reset_state();
  }
}

// ----- ROBOT STATE -----
geometry_msgs::msg::PoseStamped MPCController::get_robot_pose()
{
  geometry_msgs::msg::PoseStamped pose;
  // TODO: Implement retrieval of robot pose, e.g., from tf or topic
  return pose;
}

geometry_msgs::msg::Twist MPCController::get_robot_velocity()
{
  geometry_msgs::msg::Twist vel;
  // TODO: Implement retrieval of robot velocity, e.g., from topic
  return vel;
}

// ----- PATH PROCESSING -----
geometry_msgs::msg::PoseStamped MPCController::calculate_lookahead_point()
{
  geometry_msgs::msg::PoseStamped lookahead;
  // TODO: Implement lookahead logic based on global_plan_, robot pose, horizon, etc.
  return lookahead;
}

// ----- MPC SOLVER -----
geometry_msgs::msg::Twist MPCController::solve_mpc(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &vel,
    const nav_msgs::msg::Path &path)
{
  geometry_msgs::msg::Twist cmd;
  // TODO: Implement MPC QP solver
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  return cmd;
}

// ----- COMMAND PUBLICATION -----
void MPCController::publish_velocity_command(const geometry_msgs::msg::Twist &cmd)
{
  geometry_msgs::msg::TwistStamped cmd_stamped;
  cmd_stamped.header.stamp = this->now();
  cmd_stamped.header.frame_id = "base_link";
  cmd_stamped.twist = cmd;
  cmd_vel_pub_->publish(cmd_stamped);
}

// ----- ACTION FEEDBACK -----
void MPCController::update_feedback(const geometry_msgs::msg::PoseStamped &pose)
{
  // TODO: Publish action feedback (progress, error, etc.)
}

// ----- GOAL CHECK & RESET -----
bool MPCController::goal_reached(const geometry_msgs::msg::PoseStamped &pose, const nav_msgs::msg::Path &path)
{
  // TODO: Implement goal reached logic
  return false;
}

void MPCController::reset_state()
{
  // TODO: Reset internal state, stop robot if needed
}

}  // namespace mpc_controller