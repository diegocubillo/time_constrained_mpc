#include "time_constrained_mpc/time_constrained_mpc.hpp"
#include <tf2/utils.h>
#include <cmath>

namespace mpc_controller
{

MPCController::MPCController()
: rclcpp_lifecycle::LifecycleNode("mpc_controller")
{
  // Initialize bond ID to match what Nav2 lifecycle manager expects
  bond_id_ = get_name();
}

MPCController::~MPCController() = default;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_configure(const rclcpp_lifecycle::State & state)
{
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_debug_path", 10);
  
  // Determine which type of cmd_vel to use
  use_stamped_cmd_vel_ = this->declare_parameter<bool>("use_stamped_cmd_vel", false);
  
  // Create the appropriate publisher based on the parameter
  if (use_stamped_cmd_vel_) {
    cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    RCLCPP_INFO(get_logger(), "Using TwistStamped for cmd_vel");
  } else {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    RCLCPP_INFO(get_logger(), "Using Twist for cmd_vel");
  }

  // Parameters
  max_linear_vel_ = this->declare_parameter<double>("max_linear_vel", 0.5);
  max_angular_vel_ = this->declare_parameter<double>("max_angular_vel", 1.0);
  max_linear_accel_ = this->declare_parameter<double>("max_linear_accel", 0.2);
  max_angular_accel_ = this->declare_parameter<double>("max_angular_accel", 0.3);
  horizon_sec_ = this->declare_parameter<double>("horizon_sec", 2.0);
  horizon_steps_ = this->declare_parameter<int>("horizon_steps", 10);
  
  lookahead_time_ = this->declare_parameter<double>("lookahead_time", 1.5);
  min_lookahead_dist_ = this->declare_parameter<double>("min_lookahead_dist", 0.3);
  max_lookahead_dist_ = this->declare_parameter<double>("max_lookahead_dist", 0.9);
  goal_dist_tolerance_ = this->declare_parameter<double>("goal_dist_tolerance", 0.2);
  goal_theta_tolerance_ = this->declare_parameter<double>("goal_theta_tolerance", 0.1);
  
  // Frame IDs
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  
  // Create bond
  create_bond();
  
  // Control time step
  double controller_frequency = this->declare_parameter<double>("controller_frequency", 10.0);
  d_t_ = 1.0 / controller_frequency;
  
  // MPC weight matrices Q[x, y, theta], R[v, w], R_d[dv, dw]
  std::vector<double> q_diag = this->declare_parameter<std::vector<double>>(
    "Q_matrix_diag", {10.0, 10.0, 1.0});
  std::vector<double> r_diag = this->declare_parameter<std::vector<double>>(
    "R_matrix_diag", {1.0, 1.0});
  std::vector<double> rd_diag = this->declare_parameter<std::vector<double>>(
    "R_d_matrix_diag", {10.0, 10.0});
  
  Q_ = Eigen::Matrix3d::Zero();
  R_ = Eigen::Matrix2d::Zero();
  R_d_ = Eigen::Matrix2d::Zero();
  
  for (size_t i = 0; i < 3; ++i) {
    Q_(i, i) = q_diag[i];
  }
  for (size_t i = 0; i < 2; ++i) {
    R_(i, i) = r_diag[i];
    R_d_(i, i) = rd_diag[i];
  }
  
  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Odometry subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      current_odom_ = *msg;
      has_odom_ = true;
    });

  // Action server for FollowPath
  action_server_ = rclcpp_action::create_server<nav2_msgs::action::FollowPath>(
    this,
    "follow_path",
    [this](const rclcpp_action::GoalUUID & uuid, 
           std::shared_ptr<const nav2_msgs::action::FollowPath::Goal> goal) {
      (void)uuid;
      (void)goal;
      
      // Only accept goals when the node is active (initialized)
      if (!initialized_) {
        RCLCPP_WARN(get_logger(), 
          "Rejecting goal - controller is not active. Please activate the node first.");
        return rclcpp_action::GoalResponse::REJECT;
      }
      
      RCLCPP_INFO(get_logger(), "Received goal request - accepting");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
      RCLCPP_INFO(get_logger(), "Received cancel request");
      handle_cancel(goal_handle);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
      handle_goal(goal_handle);
    });

  // Note: initialized_ will be set to true in on_activate()
  initialized_ = false;

  RCLCPP_INFO(get_logger(), "MPCController on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_activate(const rclcpp_lifecycle::State & state)
{
  path_pub_->on_activate();
  
  // Activate the appropriate cmd_vel publisher
  if (use_stamped_cmd_vel_) {
    cmd_vel_stamped_pub_->on_activate();
  } else {
    cmd_vel_pub_->on_activate();
  }

  // Bond should already be started from on_configure
  if (bond_) {
    RCLCPP_INFO(get_logger(), "Bond is active with ID: %s", bond_id_.c_str());
  }

  // Start main control loop timer
  timer_control_loop_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MPCController::control_loop, this)
  );

  timer_path_pub_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MPCController::publish_debug_path, this)
  );

  // Enable the controller - now it can accept goals
  initialized_ = true;

  RCLCPP_INFO(get_logger(), "MPCController on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_deactivate(const rclcpp_lifecycle::State & state)
{
  path_pub_->on_deactivate();
  
  // Deactivate the appropriate cmd_vel publisher
  if (use_stamped_cmd_vel_) {
    cmd_vel_stamped_pub_->on_deactivate();
  } else {
    cmd_vel_pub_->on_deactivate();
  }

  if (timer_path_pub_) {
    timer_path_pub_->cancel();
  }

  // Stop bond
  if (bond_) {
    // Bond will be automatically destroyed, no explicit shutdown needed
    RCLCPP_INFO(get_logger(), "Bond stopped");
  }

  // Disable the controller - stop accepting goals
  initialized_ = false;
  
  // Stop the robot
  reset_state();

  RCLCPP_INFO(get_logger(), "MPCController on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_cleanup(const rclcpp_lifecycle::State & state)
{
  path_pub_.reset();
  cmd_vel_stamped_pub_.reset();
  cmd_vel_pub_.reset();
  timer_path_pub_.reset();
  
  // Destroy bond
  destroy_bond();
  
  RCLCPP_INFO(get_logger(), "MPCController on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_shutdown(const rclcpp_lifecycle::State & state)
{
  path_pub_.reset();
  cmd_vel_stamped_pub_.reset();
  cmd_vel_pub_.reset();
  timer_path_pub_.reset();
  
  // Destroy bond
  destroy_bond();
  
  RCLCPP_INFO(get_logger(), "MPCController on_shutdown() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ----- ACTION INTERFACE -----
void MPCController::handle_goal(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  global_plan_ = goal->path;
  current_goal_handle_ = goal_handle;
  du_prev_ = Eigen::Vector2d::Zero();
  
  RCLCPP_INFO(get_logger(), "Received new path with %zu poses", global_plan_.poses.size());
  
  // Execute in a separate thread
  std::thread{[this, goal_handle]() {
    // The control loop will handle following the path
    // This just marks the goal as executing
    RCLCPP_INFO(get_logger(), "Executing path following");
  }}.detach();
}

void MPCController::handle_cancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Canceling goal");
  
  // Stop the robot
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  publish_velocity_command(stop_cmd);
  
  // Clear the path
  global_plan_.poses.clear();
  current_goal_handle_.reset();
  
  // Mark goal as canceled
  auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();
  goal_handle->canceled(result);
}

// ----- CONTROL LOOP -----
void MPCController::control_loop()
{
  if (!initialized_ || global_plan_.poses.empty()) {
    return;
  }

  // Check bond status
  if (bond_timeout_detected_) {
    RCLCPP_ERROR(get_logger(), "Bond timeout detected! Stopping robot for safety.");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    publish_velocity_command(stop_cmd);
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
  pose.header.frame_id = base_frame_;
  pose.header.stamp = this->now();
  
  try {
    // Get transform from map to base_link
    auto transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero);
    
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    pose.header.frame_id = map_frame_;
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not get robot pose: %s", ex.what());
  }
  
  return pose;
}

geometry_msgs::msg::Twist MPCController::get_robot_velocity()
{
  geometry_msgs::msg::Twist vel;
  
  if (has_odom_) {
    vel = current_odom_.twist.twist;
  }
  // TODO add fallback using tf if no odom received
  return vel;
}

// ----- PATH PROCESSING -----
geometry_msgs::msg::PoseStamped MPCController::calculate_lookahead_point()
{
  geometry_msgs::msg::PoseStamped lookahead;
  
  if (global_plan_.poses.empty()) {
    return lookahead;
  }
  
  auto robot_pose = get_robot_pose();
  auto velocity = get_robot_velocity();
  
  // Calculate lookahead distance based on velocity
  double vt = std::hypot(velocity.linear.x, velocity.linear.y);
  double lookahead_dist = std::clamp(
    vt * lookahead_time_,
    min_lookahead_dist_,
    max_lookahead_dist_
  );
  
  // Find the closest point on the path to the robot
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  // Find the lookahead point on the path
  double accumulated_dist = 0.0;
  for (size_t i = closest_idx; i < global_plan_.poses.size() - 1; ++i) {
    double dx = global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x;
    double dy = global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y;
    double segment_dist = std::hypot(dx, dy);
    
    if (accumulated_dist + segment_dist >= lookahead_dist) {
      // Interpolate between points i and i+1
      double ratio = (lookahead_dist - accumulated_dist) / segment_dist;
      lookahead.pose.position.x = global_plan_.poses[i].pose.position.x + ratio * dx;
      lookahead.pose.position.y = global_plan_.poses[i].pose.position.y + ratio * dy;
      lookahead.pose.orientation = global_plan_.poses[i + 1].pose.orientation;
      lookahead.header = global_plan_.header;
      return lookahead;
    }
    
    accumulated_dist += segment_dist;
  }
  
  // If we didn't find a point, return the last point
  if (!global_plan_.poses.empty()) {
    lookahead = global_plan_.poses.back();
  }
  
  return lookahead;
}

// ----- MPC SOLVER -----
geometry_msgs::msg::Twist MPCController::solve_mpc(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &vel,
    const nav_msgs::msg::Path &path)
{
  geometry_msgs::msg::Twist cmd;
  
  if (path.poses.empty()) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }
  
  // Get lookahead point as desired state
  auto lookahead = calculate_lookahead_point();
  if (lookahead.header.frame_id.empty()) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }
  
  // Extract current state [x, y, theta]
  double current_x = pose.pose.position.x;
  double current_y = pose.pose.position.y;
  double current_theta = tf2::getYaw(pose.pose.orientation);
  Eigen::Vector3d current_state(current_x, current_y, current_theta);
  
  // Extract desired state [x, y, theta]
  double desired_x = lookahead.pose.position.x;
  double desired_y = lookahead.pose.position.y;
  double desired_theta = tf2::getYaw(lookahead.pose.orientation);
  Eigen::Vector3d desired_state(desired_x, desired_y, desired_theta);
  
  // Reference control (based on current velocity)
  double vt = vel.linear.x;
  double wt = vel.angular.z;
  Eigen::Vector2d u_ref(vt, wt);
  
  // Build MPC optimization problem
  Eigen::SparseMatrix<double> P, A;
  Eigen::VectorXd q, l, u;
  
  build_mpc_matrices(current_state, desired_state, u_ref, P, q, A, l, u);
  
  // Solve using OSQP
  OSQPWorkspace* work = nullptr;
  OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  
  if (settings) {
    osqp_set_default_settings(settings);
    settings->verbose = false;
    settings->warm_start = true;
    settings->max_iter = 4000;
    settings->eps_abs = 1.0e-4;
    settings->eps_rel = 1.0e-4;
  }
  
  // Convert Eigen sparse matrices to OSQP format
  std::vector<c_float> P_data, q_data, A_data, l_data, u_data;
  std::vector<c_int> P_indices, P_indptr, A_indices, A_indptr;
  
  // P matrix (upper triangular)
  for (int k = 0; k < P.outerSize(); ++k) {
    P_indptr.push_back(P_data.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(P, k); it; ++it) {
      if (it.row() <= it.col()) {  // Upper triangular only
        P_data.push_back(it.value());
        P_indices.push_back(it.row());
      }
    }
  }
  P_indptr.push_back(P_data.size());
  
  // A matrix
  for (int k = 0; k < A.outerSize(); ++k) {
    A_indptr.push_back(A_data.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      A_data.push_back(it.value());
      A_indices.push_back(it.row());
    }
  }
  A_indptr.push_back(A_data.size());
  
  // Convert q, l, u vectors
  for (int i = 0; i < q.size(); ++i) {
    q_data.push_back(q(i));
  }
  for (int i = 0; i < l.size(); ++i) {
    l_data.push_back(l(i));
    u_data.push_back(u(i));
  }
  
  // Setup OSQP problem
  if (data) {
    data->n = P.cols();
    data->m = A.rows();
    data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(), 
                        P_indices.data(), P_indptr.data());
    data->q = q_data.data();
    data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(), 
                        A_indices.data(), A_indptr.data());
    data->l = l_data.data();
    data->u = u_data.data();
  }
  
  // Solve
  c_int exitflag = osqp_setup(&work, data, settings);
  
  if (exitflag == 0 && work) {
    osqp_solve(work);
    
    if (work->solution && work->info->status_val > 0) {
      // Extract first control input (MPC receding horizon)
      double u_v = work->solution->x[0] + du_prev_(0) + u_ref(0);
      double u_w = work->solution->x[1] + du_prev_(1) + u_ref(1);
      
      // Update previous control increment
      du_prev_(0) = work->solution->x[0];
      du_prev_(1) = work->solution->x[1];
      
      // Saturate controls as safety measure
      // (Should not be necessary if constraints are properly set, but kept as failsafe)
      cmd.linear.x = std::clamp(u_v, 0.0, max_linear_vel_);
      cmd.angular.z = std::clamp(u_w, -max_angular_vel_, max_angular_vel_);
    } else {
      RCLCPP_WARN(get_logger(), "MPC solver failed with status: %d", 
                  work->info ? work->info->status_val : -1);
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
    
    // Cleanup
    osqp_cleanup(work);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to setup OSQP solver");
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }
  
  if (data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    c_free(data);
  }
  if (settings) c_free(settings);
  
  return cmd;
}

// ----- COMMAND PUBLICATION -----
void MPCController::publish_velocity_command(const geometry_msgs::msg::Twist &cmd)
{
  if (use_stamped_cmd_vel_) {
    // Publish TwistStamped
    geometry_msgs::msg::TwistStamped cmd_stamped;
    cmd_stamped.header.stamp = this->now();
    cmd_stamped.header.frame_id = "base_link";
    cmd_stamped.twist = cmd;
    cmd_vel_stamped_pub_->publish(cmd_stamped);
  } else {
    // Publish Twist
    cmd_vel_pub_->publish(cmd);
  }
}

// ----- PATH PUBLICATION -----
void MPCController::publish_debug_path()
{
  if (!global_plan_.poses.empty()) {
    path_pub_->publish(global_plan_);
  }
}

// ----- ACTION FEEDBACK -----
void MPCController::update_feedback(const geometry_msgs::msg::PoseStamped &pose)
{
  if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
    return;
  }
  
  auto feedback = std::make_shared<nav2_msgs::action::FollowPath::Feedback>();
  feedback->speed = get_robot_velocity().linear.x;
  feedback->distance_to_goal = std::hypot(
    global_plan_.poses.back().pose.position.x - pose.pose.position.x,
    global_plan_.poses.back().pose.position.y - pose.pose.position.y);
  
  current_goal_handle_->publish_feedback(feedback);
}

// ----- GOAL CHECK & RESET -----
bool MPCController::goal_reached(const geometry_msgs::msg::PoseStamped &pose, const nav_msgs::msg::Path &path)
{
  if (path.poses.empty()) {
    return false;
  }
  
  const auto& goal = path.poses.back();
  
  // Check distance to goal
  double dx = goal.pose.position.x - pose.pose.position.x;
  double dy = goal.pose.position.y - pose.pose.position.y;
  double dist = std::hypot(dx, dy);
  
  // Check orientation difference
  double goal_theta = tf2::getYaw(goal.pose.orientation);
  double current_theta = tf2::getYaw(pose.pose.orientation);
  double theta_error = std::abs(std::atan2(std::sin(goal_theta - current_theta), 
                                           std::cos(goal_theta - current_theta)));
  
  return (dist < goal_dist_tolerance_) && (theta_error < goal_theta_tolerance_);
}

void MPCController::reset_state()
{
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();
    current_goal_handle_->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal reached!");
  }
  
  global_plan_.poses.clear();
  du_prev_ = Eigen::Vector2d::Zero();
  current_goal_handle_.reset();
  
  // Stop the robot
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  publish_velocity_command(stop_cmd);
}

// ----- MPC HELPER FUNCTIONS -----
Eigen::Vector2d MPCController::differential_drive_model(
    const Eigen::Vector3d &state, 
    const Eigen::Vector2d &control, 
    double dt)
{
  // Differential drive kinematics:
  // dx/dt = v * cos(theta)
  // dy/dt = v * sin(theta)
  // dtheta/dt = omega
  
  double theta = state(2);
  double v = control(0);
  double omega = control(1);
  
  Eigen::Vector2d state_dot;
  state_dot(0) = v * std::cos(theta);  // dx
  state_dot(1) = v * std::sin(theta);  // dy
  
  return state_dot;
}

void MPCController::build_mpc_matrices(
    const Eigen::Vector3d &current_state,
    const Eigen::Vector3d &desired_state,
    const Eigen::Vector2d &u_ref,
    Eigen::SparseMatrix<double> &P,
    Eigen::VectorXd &q,
    Eigen::SparseMatrix<double> &A,
    Eigen::VectorXd &l,
    Eigen::VectorXd &u)
{
  const int nx = 3;  // state dimension [x, y, theta]
  const int nu = 2;  // control dimension [v, omega]
  const int N = horizon_steps_;
  
  // Augmented state: [x, y, theta, du_v, du_omega]
  const int dim_x = nx;
  const int dim_u = nu;
  const int dim_aug = dim_x + dim_u;  // 5
  
  // State error
  Eigen::Vector3d e = current_state - desired_state;
  // Normalize angle error
  e(2) = std::atan2(std::sin(e(2)), std::cos(e(2)));
  
  // Linearized dynamics around reference trajectory
  // State matrix A (3x3)
  Eigen::Matrix3d A_d = Eigen::Matrix3d::Identity();
  A_d(0, 2) = -u_ref(0) * std::sin(desired_state(2)) * d_t_;
  A_d(1, 2) = u_ref(0) * std::cos(desired_state(2)) * d_t_;
  
  // Control matrix B (3x2)
  Eigen::MatrixXd B_d = Eigen::MatrixXd::Zero(dim_x, dim_u);
  B_d(0, 0) = std::cos(desired_state(2)) * d_t_;
  B_d(1, 0) = std::sin(desired_state(2)) * d_t_;
  B_d(2, 1) = d_t_;
  
  // Augmented system matrices
  // The augmented state is ξ_k = [x_k; u_{k-1}]
  // where u_{k-1} is the PREVIOUS VELOCITY (not increment)
  // 
  // Dynamics:
  //   x_{k+1} = A_d·x_k + B_d·u_{k-1} + B_d·Δu_k = A_d·x_k + B_d·u_k  ✓
  //   u_k = u_{k-1} + Δu_k  ✓
  //
  // This ensures the robot dynamics are physically correct:
  //   x_{k+1} only depends on current velocity u_k, not on u_{k-2}
  Eigen::MatrixXd A_aug = Eigen::MatrixXd::Zero(dim_aug, dim_aug);
  A_aug.topLeftCorner(dim_x, dim_x) = A_d;
  A_aug.topRightCorner(dim_x, dim_u) = B_d;
  A_aug.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();  // Store u_{k-1}
  
  Eigen::MatrixXd B_aug = Eigen::MatrixXd::Zero(dim_aug, dim_u);
  B_aug.topLeftCorner(dim_x, dim_u) = B_d;
  B_aug.bottomLeftCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();
  
  // Output matrix C (3x5)
  Eigen::MatrixXd C_aug = Eigen::MatrixXd::Zero(dim_x, dim_aug);
  C_aug.topLeftCorner(dim_x, dim_x) = Eigen::Matrix3d::Identity();
  
  // Build prediction matrices
  Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * N, dim_aug);
  Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * N, dim_u * N);
  
  Eigen::MatrixXd A_pow = Eigen::MatrixXd::Identity(dim_aug, dim_aug);
  for (int i = 0; i < N; ++i) {
    A_pow = A_pow * A_aug;
    S_x.block(dim_x * i, 0, dim_x, dim_aug) = C_aug * A_pow;
    
    for (int j = 0; j <= i; ++j) {
      Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(dim_aug, dim_aug);
      for (int k = 0; k < i - j; ++k) {
        temp = temp * A_aug;
      }
      S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C_aug * temp * B_aug;
    }
  }
  
  // Build cost matrices
  // Q_bar: Penalizes trajectory tracking error
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(dim_x * N, dim_x * N);
  for (int i = 0; i < N; ++i) {
    Q_bar.block(dim_x * i, dim_x * i, dim_x, dim_x) = Q_;
  }
  
  // R_d_bar: Penalizes control rate changes (smoothness)
  // NOTE: We only use R_d (not R) because we don't penalize absolute control effort
  // This is "Option 2": only smooth movements, no energy optimization
  Eigen::MatrixXd R_d_bar = Eigen::MatrixXd::Zero(dim_u * N, dim_u * N);
  for (int i = 0; i < N; ++i) {
    R_d_bar.block(dim_u * i, dim_u * i, dim_u, dim_u) = R_d_;
  }
  
  // QP problem: min 0.5 * x^T * P * x + q^T * x
  // subject to: l <= A*x <= u
  
  // Augmented state vector: ξ_0 = [e; u_{-1}]
  // where e = x_current - x_desired (state error)
  // and u_{-1} = u_ref + du_prev (previous velocity, not increment)
  Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(dim_aug);
  x_aug.head(dim_x) = e;
  x_aug.tail(dim_u) = u_ref + du_prev_;  // Previous velocity (absolute)
  
  // P matrix (Hessian)
  // Cost: ||x_i - x_ref||²_Q + ||Δu_i||²_{R_d}
  Eigen::MatrixXd P_dense = S_u.transpose() * Q_bar * S_u + R_d_bar;
  P = P_dense.sparseView();
  
  // q vector (gradient)
  q = S_u.transpose() * Q_bar * S_x * x_aug;
  
  // Constraints: control limits
  // We use box constraints: l <= Δu <= u
  // OSQP needs: l <= A*Δu <= u, where A is identity for box constraints
  const int n_constraints = dim_u * N;  // One constraint per control variable
  A.resize(n_constraints, dim_u * N);
  l.resize(n_constraints);
  u.resize(n_constraints);
  
  // Build constraint matrix (identity for box constraints)
  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < dim_u * N; ++i) {
    triplets.push_back(Eigen::Triplet<double>(i, i, 1.0));
  }
  A.setFromTriplets(triplets.begin(), triplets.end());
  
  // Calculate current accumulated velocities
  // v_current = v_ref + du_prev
  double v_current = u_ref(0) + du_prev_(0);
  double w_current = u_ref(1) + du_prev_(1);
  
  // Set constraint bounds for each step in the horizon
  for (int i = 0; i < N; ++i) {
    // Linear velocity constraints (Δv)
    // We want: 0 <= v_current + Δv_i <= v_max
    // Therefore: -v_current <= Δv_i <= v_max - v_current
    // But also: -max_linear_accel_ <= Δv_i <= max_linear_accel_ (acceleration limits)
    // Final bounds are the intersection of both constraints
    double delta_v_min = std::max(-max_linear_accel_, 0.0 - v_current);
    double delta_v_max = std::min(max_linear_accel_, max_linear_vel_ - v_current);
    
    l(dim_u * i) = delta_v_min;
    u(dim_u * i) = delta_v_max;
    
    // Angular velocity constraints (Δω)
    // We want: -ω_max <= w_current + Δω_i <= ω_max
    // Therefore: -ω_max - w_current <= Δω_i <= ω_max - w_current
    // But also: -max_angular_accel_ <= Δω_i <= max_angular_accel_ (angular acceleration limits)
    double delta_w_min = std::max(-max_angular_accel_, -max_angular_vel_ - w_current);
    double delta_w_max = std::min(max_angular_accel_, max_angular_vel_ - w_current);
    
    l(dim_u * i + 1) = delta_w_min;
    u(dim_u * i + 1) = delta_w_max;
    
    // Note: For simplicity, we assume the same velocity limits apply throughout
    // the horizon. A more sophisticated approach would accumulate the deltas
    // to predict v_i = v_current + sum(Δv_j for j=0..i-1)
    // However, this would make the constraints coupled and non-box constraints.
  }
}

// ----- BOND MANAGEMENT -----
void MPCController::create_bond()
{
  try {
    // Create bond with the topic name that Nav2 lifecycle manager expects
    // Nav2 expects the bond to be on topic "bond" with ID matching the node name
    bond_ = std::make_unique<bond::Bond>(
      std::string("bond"),  // topic namespace - Nav2 default
      bond_id_,             // bond ID should be the node name
      shared_from_this(),   // lifecycle node shared pointer
      [this]() { 
        RCLCPP_INFO(get_logger(), "Bond broken callback triggered");
        bond_timeout_callback(); 
      },  // broken callback
      [this]() { 
        RCLCPP_INFO(get_logger(), "Bond formed successfully");
      }  // formed callback
    );
    
    // Start the bond immediately - Nav2 expects this
    bond_->start();
    
    RCLCPP_INFO(get_logger(), "Bond created and started with ID: %s on topic: bond", bond_id_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to create bond: %s", e.what());
  }
}

void MPCController::destroy_bond()
{
  if (bond_) {
    bond_.reset();
    RCLCPP_INFO(get_logger(), "Bond destroyed");
  }
}

void MPCController::bond_timeout_callback()
{
  RCLCPP_ERROR(get_logger(), "Bond connection broken! Communication with lifecycle manager lost.");
  bond_timeout_detected_ = true;
  
  // Stop the robot for safety
  if (initialized_) {
    RCLCPP_WARN(get_logger(), "Stopping robot due to bond failure");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    publish_velocity_command(stop_cmd);
    
    // Cancel current goal if active
    reset_state();
  }
}

}  // namespace mpc_controller