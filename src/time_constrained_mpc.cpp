#include "time_constrained_mpc/time_constrained_mpc.hpp"
#include <algorithm>
#include <cmath>
#include <tf2/utils.h>
#include <unistd.h>

namespace mpc_controller {

MPCController::MPCController(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("mpc_controller", options) {
  // Initialize bond ID to match Nav2 lifecycle manager expectations
  bond_id_ = get_name();

  // Initialize logger instance
  mpc_logger_ = std::make_unique<MPCLogger>();

  // Declare parameters
  this->declare_parameter<bool>("use_stamped_cmd_vel", false);
  this->declare_parameter<bool>("debug_mpc", false);
  this->declare_parameter<double>("max_linear_vel", 0.5);
  this->declare_parameter<double>("max_angular_vel", 1.0);
  this->declare_parameter<double>("max_linear_accel", 0.2);
  this->declare_parameter<double>("max_angular_accel", 0.3);
  this->declare_parameter<int>("prediction_horizon_steps", 10);
  this->declare_parameter<int>("control_horizon_steps", 10);
  this->declare_parameter<double>("goal_dist_tolerance", 0.2);
  this->declare_parameter<double>("goal_theta_tolerance", 0.1);
  this->declare_parameter<double>("goal_approach_radius", 0.3);
  this->declare_parameter<double>("goal_approach_vel", 0.08);
  this->declare_parameter<double>("goal_approach_extension", -1.0);
  this->declare_parameter<double>("max_spatial_error", 2.0);
  this->declare_parameter<double>("max_temporal_error", 5.0);
  this->declare_parameter<double>("progress_search_window", 1.0);
  this->declare_parameter<double>("path_smoothing_window", 0.5);
  this->declare_parameter<double>("cusp_angle_threshold", 3.0 * M_PI / 4.0);
  this->declare_parameter<double>("max_reference_lead", 0.8);
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<bool>("use_bond", true);
  this->declare_parameter<double>("controller_frequency", 10.0);
  this->declare_parameter<std::vector<double>>("Q_matrix_diag",
                                               {10.0, 10.0, 1.0, 1.0});
  this->declare_parameter<std::vector<double>>("R_d_matrix_diag", {10.0, 10.0});
}

MPCController::~MPCController() {
  if (debug_mpc_ && mpc_logger_) {
    mpc_logger_->close();
  }
  RCLCPP_INFO(get_logger(), "MPCController destroyed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("mpc_global_path", 10);

  // Determine which type of cmd_vel to use
  this->get_parameter("use_stamped_cmd_vel", use_stamped_cmd_vel_);

  // Create the appropriate publisher based on the parameter
  if (use_stamped_cmd_vel_) {
    cmd_vel_stamped_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    RCLCPP_INFO(get_logger(), "Using TwistStamped for cmd_vel");
  } else {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    RCLCPP_INFO(get_logger(), "Using Twist for cmd_vel");
  }

  // Choose if debug topics are created based on parameter
  this->get_parameter("debug_mpc", debug_mpc_);

  if (debug_mpc_) {
    predicted_path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("mpc_predicted_path", 10);
    debug_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mpc_debug_pose", 10);
  }

  // Parameters
  this->get_parameter("max_linear_vel", max_linear_vel_);
  this->get_parameter("max_angular_vel", max_angular_vel_);
  this->get_parameter("max_linear_accel", max_linear_accel_);
  this->get_parameter("max_angular_accel", max_angular_accel_);
  this->get_parameter("prediction_horizon_steps", prediction_horizon_steps_);
  this->get_parameter("control_horizon_steps", control_horizon_steps_);

  // Validate horizons
  if (control_horizon_steps_ > prediction_horizon_steps_) {
    RCLCPP_WARN(get_logger(),
                "Control horizon (%d) cannot be larger than prediction horizon "
                "(%d). Clamping control horizon.",
                control_horizon_steps_, prediction_horizon_steps_);
    control_horizon_steps_ = prediction_horizon_steps_;
  }

  this->get_parameter("goal_dist_tolerance", goal_dist_tolerance_);
  this->get_parameter("goal_theta_tolerance", goal_theta_tolerance_);
  this->get_parameter("goal_approach_radius", goal_approach_radius_);
  this->get_parameter("goal_approach_vel", goal_approach_vel_);
  this->get_parameter("goal_approach_extension", goal_approach_extension_);
  this->get_parameter("max_spatial_error", max_spatial_error_);
  this->get_parameter("max_temporal_error", max_temporal_error_);
  this->get_parameter("progress_search_window", progress_search_window_);
  this->get_parameter("path_smoothing_window", path_smoothing_window_);
  this->get_parameter("cusp_angle_threshold", cusp_angle_threshold_);
  this->get_parameter("max_reference_lead", max_reference_lead_);

  // Frame IDs
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("base_frame", base_frame_);

  // Create bond
  this->get_parameter("use_bond", use_bond_);
  if (use_bond_) {
    create_bond();
  }

  // Control time step
  double controller_frequency;
  this->get_parameter("controller_frequency", controller_frequency);
  d_t_ = 1.0 / controller_frequency;

  // Resolve the terminal-reference extension (carrot beyond the goal). A negative
  // value means "auto": use N*v_app*dt, the smallest extension for which the goal
  // never falls inside the clamped tail of the prediction horizon, so the robot
  // crosses the goal at the approach speed with (essentially) no anticipatory
  // braking. Smaller positive values leave some braking; larger values none.
  if (goal_approach_extension_ < 0.0) {
    goal_approach_extension_ =
        prediction_horizon_steps_ * goal_approach_vel_ * d_t_;
  }

  // MPC weight matrices Q[x, y, s_theta, c_theta], R_d[dv, dw]
  std::vector<double> q_diag;
  this->get_parameter("Q_matrix_diag", q_diag);
  std::vector<double> rd_diag;
  this->get_parameter("R_d_matrix_diag", rd_diag);

  Q_ = Eigen::Matrix4d::Zero();
  R_d_ = Eigen::Matrix2d::Zero();

  for (size_t i = 0; i < 4; ++i) {
    Q_(i, i) = q_diag[i];
  }
  for (size_t i = 0; i < 2; ++i) {
    R_d_(i, i) = rd_diag[i];
  }

  // this->declare_parameter("qos_overrides./tf.subscription.reliability",
  // "best_effort");
  // this->set_parameter(rclcpp::Parameter("qos_overrides./tf.subscription.reliability",
  // "best_effort"));

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  // Action server for FollowPath
  action_server_ = rclcpp_action::create_server<nav2_msgs::action::FollowPath>(
      this, "follow_path",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const nav2_msgs::action::FollowPath::Goal> goal) {
        (void)uuid;
        (void)goal;

        // Only accept goals when the node is active (initialized)
        if (!initialized_) {
          RCLCPP_WARN(get_logger(), "Rejecting goal - controller is not "
                                    "active. Please activate the node first.");
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

  // Create timers but keep them paused (will be activated in on_activate)
  timer_control_loop_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&MPCController::control_loop, this));
  timer_control_loop_->cancel();

  timer_path_pub_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MPCController::publish_debug_path, this));
  timer_path_pub_->cancel();

  // Note: initialized_ will be set to true in on_activate()
  initialized_ = false;

  // Calculate horizon seconds (just for the logger)
  double prediction_horizon_sec = prediction_horizon_steps_ * d_t_;
  double control_horizon_sec = control_horizon_steps_ * d_t_;

  // Log all configuration parameters
  RCLCPP_INFO(get_logger(), "MPCController on_configure() is called.");
  RCLCPP_INFO(get_logger(), "=== MPC Configuration Parameters ===");
  RCLCPP_INFO(get_logger(), "Controller frequency: %.1f Hz", 1.0 / d_t_);
  RCLCPP_INFO(get_logger(), "Prediction Horizon: %d steps (%.2f sec)",
              prediction_horizon_steps_, prediction_horizon_sec);
  RCLCPP_INFO(get_logger(), "Control Horizon: %d steps (%.2f sec)",
              control_horizon_steps_, control_horizon_sec);
  RCLCPP_INFO(get_logger(), "Control time step: %.3f sec", d_t_);
  RCLCPP_INFO(get_logger(), "=== Velocity Limits ===");
  RCLCPP_INFO(get_logger(), "Max linear velocity: %.2f m/s", max_linear_vel_);
  RCLCPP_INFO(get_logger(), "Max angular velocity: %.2f rad/s",
              max_angular_vel_);
  RCLCPP_INFO(get_logger(), "Max linear acceleration: %.2f m/s²",
              max_linear_accel_);
  RCLCPP_INFO(get_logger(), "Max angular acceleration: %.2f rad/s²",
              max_angular_accel_);

  // Convert accelerations from SI units (m/s², rad/s²) to velocity increments
  // per time step This is done once here to avoid repeated calculations in the
  // control loop
  max_linear_accel_ *= d_t_;  // Now in m/s per time step
  max_angular_accel_ *= d_t_; // Now in rad/s per time step

  RCLCPP_INFO(get_logger(), "=== Goal Tolerances ===");
  RCLCPP_INFO(get_logger(), "Distance tolerance: %.2f m", goal_dist_tolerance_);
  RCLCPP_INFO(get_logger(), "Theta tolerance: %.2f rad", goal_theta_tolerance_);
  RCLCPP_INFO(get_logger(), "Goal approach radius: %.2f m",
              goal_approach_radius_);
  RCLCPP_INFO(get_logger(), "Goal approach speed: %.2f m/s",
              goal_approach_vel_);
  RCLCPP_INFO(get_logger(), "Goal approach extension: %.2f m (beyond goal)",
              goal_approach_extension_);
  RCLCPP_INFO(get_logger(), "Max spatial error: %.2f m", max_spatial_error_);
  RCLCPP_INFO(get_logger(), "Max temporal error: %.2f s", max_temporal_error_);
  RCLCPP_INFO(get_logger(), "Progress search window: %.2f m",
              progress_search_window_);
  RCLCPP_INFO(get_logger(), "=== Path Processing ===");
  RCLCPP_INFO(get_logger(), "Path smoothing window: %.2f m",
              path_smoothing_window_);
  RCLCPP_INFO(get_logger(), "Cusp angle threshold: %.2f rad (%.0f deg)",
              cusp_angle_threshold_, cusp_angle_threshold_ * 180.0 / M_PI);
  RCLCPP_INFO(get_logger(), "Max reference lead: %.2f m", max_reference_lead_);
  RCLCPP_INFO(get_logger(), "=== MPC Cost Weights ===");
  RCLCPP_INFO(
      get_logger(),
      "Q (state tracking) [x, y, sin(θ), cos(θ)]: [%.1f, %.1f, %.1f, %.1f]",
      Q_(0, 0), Q_(1, 1), Q_(2, 2), Q_(3, 3));
  RCLCPP_INFO(get_logger(), "R_d (control rate) [Δv, Δω]: [%.1f, %.1f]",
              R_d_(0, 0), R_d_(1, 1));
  RCLCPP_INFO(get_logger(), "=== Frame IDs ===");
  RCLCPP_INFO(get_logger(), "Map frame: %s", map_frame_.c_str());
  RCLCPP_INFO(get_logger(), "Base frame: %s", base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "====================================");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  path_pub_->on_activate();
  if (debug_mpc_) {
    predicted_path_pub_->on_activate();
    debug_pose_pub_->on_activate();
  }

  // Activate the appropriate cmd_vel publisher
  if (use_stamped_cmd_vel_) {
    cmd_vel_stamped_pub_->on_activate();
  } else {
    cmd_vel_pub_->on_activate();
  }

  if (use_bond_ && bond_) {
    try {
      bond_timeout_detected_ = false;
      bond_->start();
      RCLCPP_INFO(get_logger(), "Bond is active with ID: %s", bond_id_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to activate bond: %s", e.what());
    }
  } else {
    RCLCPP_INFO(get_logger(), "Bond management is disabled");
  }

  // Start main control loop timer
  if (timer_control_loop_) {
    timer_control_loop_->reset();
  }

  if (timer_path_pub_) {
    timer_path_pub_->reset();
  }

  // Enable the controller - now it can accept goals
  initialized_ = true;

  // Open log file if debug is enabled
  if (debug_mpc_) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    std::string home_dir = std::getenv("HOME");
    std::string log_file = home_dir + "/.ros/log/mpc_data_" + ss.str() + "_" +
                           std::to_string(getpid()) + ".csv";

    if (mpc_logger_->open(log_file)) {
      RCLCPP_INFO(get_logger(), "MPC Logging started: %s", log_file.c_str());
      mpc_logger_->write_header(prediction_horizon_steps_);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open MPC log file: %s",
                   log_file.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "MPCController on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  path_pub_->on_deactivate();
  if (debug_mpc_) {
    predicted_path_pub_->on_deactivate();
    debug_pose_pub_->on_deactivate();
  }

  // Deactivate the appropriate cmd_vel publisher
  if (use_stamped_cmd_vel_) {
    cmd_vel_stamped_pub_->on_deactivate();
  } else {
    cmd_vel_pub_->on_deactivate();
  }

  if (timer_control_loop_) {
    timer_control_loop_->cancel();
  }
  if (timer_path_pub_) {
    timer_path_pub_->cancel();
  }

  // Stop bond
  if (use_bond_ && bond_) {
    bond_->breakBond();
    RCLCPP_INFO(get_logger(), "Bond stopped");
  }

  // Disable the controller - stop accepting goals
  initialized_ = false;

  // Stop the robot
  reset_state(false);

  // Close log file
  if (debug_mpc_ && mpc_logger_) {
    mpc_logger_->close();
    RCLCPP_INFO(get_logger(), "MPC Logging stopped");
  }

  RCLCPP_INFO(get_logger(), "MPCController on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  path_pub_.reset();
  cmd_vel_stamped_pub_.reset();
  cmd_vel_pub_.reset();
  timer_path_pub_.reset();
  timer_control_loop_.reset();
  action_server_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  if (debug_mpc_) {
    predicted_path_pub_.reset();
    debug_pose_pub_.reset();
  }

  // Destroy bond
  destroy_bond();

  RCLCPP_INFO(get_logger(), "MPCController on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MPCController::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "MPCController on_shutdown() is called.");

  // Abort goal if active and stop the robot while publishers and action server
  // are still alive
  reset_state(false);

  // Destroy bond
  destroy_bond();

  // Reset and destroy all ROS2 components
  path_pub_.reset();
  cmd_vel_stamped_pub_.reset();
  cmd_vel_pub_.reset();
  timer_path_pub_.reset();
  timer_control_loop_.reset();
  action_server_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  if (debug_mpc_) {
    predicted_path_pub_.reset();
    debug_pose_pub_.reset();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

// ----- ACTION INTERFACE -----
void MPCController::handle_goal(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto original_path = goal->path;

  // Abort previous goal if active (server-side preemption)
  // Note: we use abort() instead of canceled() because the goal is in EXECUTING
  // state. canceled() requires the goal to be in CANCELING state first, which
  // would cause an invalid state machine transition and crash.
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();
    current_goal_handle_->abort(result);
    RCLCPP_INFO(get_logger(), "Previous goal aborted (preempted by new goal)");
  }

  // Store new goal
  current_goal_handle_ = goal_handle;

  // Check if the last pose of the original path has a valid explicit
  // orientation. A default-constructed quaternion in ROS2 is (0,0,0,0) which is
  // not normalized. If the quaternion is normalized (x²+y²+z²+w² ≈ 1), the goal
  // has an explicit orientation.
  has_goal_orientation_ = false;
  if (!original_path.poses.empty()) {
    const auto &goal_quat = original_path.poses.back().pose.orientation;
    double norm_sq = goal_quat.x * goal_quat.x + goal_quat.y * goal_quat.y +
                     goal_quat.z * goal_quat.z + goal_quat.w * goal_quat.w;
    if (std::abs(norm_sq - 1.0) < 0.1 && norm_sq > 0.01) {
      has_goal_orientation_ = true;
      goal_orientation_ = goal_quat;
      double goal_yaw = tf2::getYaw(goal_quat);
      RCLCPP_INFO(get_logger(),
                  "Goal has explicit orientation: yaw = %.2f rad (%.1f deg)",
                  goal_yaw, goal_yaw * 180.0 / M_PI);
    } else {
      // Unspecified final orientation (invalid quaternion).
      // To avoid turning towards 0.0 radians at the end of the path during MPC
      // tracking, we propagate the orientation of the second-to-last pose to
      // the last pose.
      if (original_path.poses.size() >= 2) {
        original_path.poses.back().pose.orientation =
            original_path.poses[original_path.poses.size() - 2]
                .pose.orientation;
        RCLCPP_INFO(get_logger(),
                    "Goal has no explicit orientation constraint. Using "
                    "second-to-last pose orientation for path smoothing.");
      }
    }
  }

  // Start the incremental MPC from the previous control input u_prev_ (the last
  // velocity the MPC commanded, or zero after a reset). This ensures smooth
  // transitions if the robot is already moving. We clamp in place to the
  // configured limits for safety (a no-op unless the limits changed)
  u_prev_(0) = std::clamp(u_prev_(0), 0.0, max_linear_vel_);
  u_prev_(1) = std::clamp(u_prev_(1), -max_angular_vel_, max_angular_vel_);

  // Record when path execution starts
  path_start_time_ = this->now();

  RCLCPP_INFO(get_logger(), "Received new path with %zu poses",
              original_path.poses.size());

  // Split the plan at direction reversals (cusps) BEFORE smoothing: an
  // averaging smoother cannot represent a cusp (the tangent flips 180 deg
  // between consecutive poses) and would collapse the reversal into geometry
  // the forward-only MPC cannot track. Each monotone segment is processed
  // independently and the reversal is executed as a timed in-place rotation
  // between segments, scheduled into a dwell window the executor synthesizes
  // symmetrically around the vertex stamp by borrowing time from the two
  // adjacent plan intervals (plus any wait the plan already contains). See
  // split_path_at_cusps() and the handover in control_loop().
  auto raw_segments = split_path_at_cusps(original_path);

  plan_segments_.clear();
  plan_segments_.reserve(raw_segments.size());
  for (const auto &raw_segment : raw_segments) {
    // Interpolate to increase point density. Plan timestamps are preserved at
    // the original waypoints and interpolated linearly in between.
    auto interpolated_path = interpolate_path(raw_segment, 0.1); // 10cm spacing

    // Smooth sharp (non-cusp) corners. The moving average is index-preserving
    // and carries each pose's timestamp through untouched.
    auto smoothed_path = smooth_path(interpolated_path, path_smoothing_window_);

    // Resample to uniform spatial spacing, interpolating timestamps locally
    // between neighbouring smoothed poses so the plan's local time structure
    // (per-waypoint speeds, waits) survives smoothing.
    plan_segments_.push_back(resample_and_retime_path(smoothed_path, 0.1));
  }

  current_segment_idx_ = 0;
  global_plan_ = plan_segments_.front();

  if (plan_segments_.size() > 1) {
    RCLCPP_INFO(get_logger(),
                "Plan split into %zu monotone segments (%zu reversals)",
                plan_segments_.size(), plan_segments_.size() - 1);
  }

  // Restart the monotonic progress tracker for the new plan (see
  // calculate_temporal_error). Must be reset together with global_plan_ so a
  // stale index from the previous, possibly longer, path is never reused.
  progress_idx_ = 0;

  control_phase_ = ControlPhase::INITIAL_ROTATION; // Reset control phase

  // Validate that all timestamps are in the future. The span covers the whole
  // plan: first pose of the first segment to last pose of the last segment.
  if (!global_plan_.poses.empty() && !plan_segments_.back().poses.empty()) {
    auto first_time = rclcpp::Time(global_plan_.poses.front().header.stamp);
    auto last_time =
        rclcpp::Time(plan_segments_.back().poses.back().header.stamp);
    RCLCPP_INFO(get_logger(),
                "Path temporal span: %.2f to %.2f seconds from now",
                (first_time - path_start_time_).seconds(),
                (last_time - path_start_time_).seconds());
  }

  // Execute in a separate thread
  std::thread{[this, goal_handle]() {
    // The control loop will handle following the path
    // This just marks the goal as executing
    RCLCPP_INFO(get_logger(), "Executing path following");
  }}.detach();
}

void MPCController::handle_cancel(
    const std::shared_ptr<GoalHandleFollowPath> /*goal_handle*/) {
  RCLCPP_INFO(get_logger(),
              "Cancel request accepted, deferring to control loop");
}

// ----- CONTROL LOOP -----
void MPCController::control_loop() {
  // Handle cancellations safely in the execution thread
  if (current_goal_handle_ && current_goal_handle_->is_canceling()) {
    auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();
    current_goal_handle_->canceled(result);
    RCLCPP_INFO(get_logger(), "Goal canceled.");

    // Call reset_state(false) to stop robot and clear internal variables.
    // Since we already marked it as canceled, it is no longer active,
    // so reset_state won't call abort().
    reset_state(false);
    return;
  }

  if (!initialized_ || global_plan_.poses.empty()) {
    return;
  }

  // Check bond status
  if (use_bond_ && bond_timeout_detected_) {
    RCLCPP_ERROR(get_logger(),
                 "Bond timeout detected! Stopping robot for safety.");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    publish_velocity_command(stop_cmd);
    return;
  }

  // Get current robot pose and current time. The "current velocity" used by the
  // MPC is the previous control input u_prev_ (no measured velocity is read).
  auto pose = get_robot_pose();
  rclcpp::Time current_time = this->now();

  // Calculate temporal error
  double temporal_error = calculate_temporal_error(pose, current_time);

  switch (control_phase_) {

  // ===== INACTIVE =====
  // No path to follow. In practice control_loop() returns earlier when the plan
  // is empty, so this is only a defensive no-op that keeps the switch total.
  case ControlPhase::INACTIVE:
    return;

  // ===== PHASE 1: INITIAL ROTATION =====
  // Rotate in place to align with the first point of the path
  case ControlPhase::INITIAL_ROTATION: {
    double target_yaw =
        tf2::getYaw(global_plan_.poses.front().pose.orientation);
    double current_yaw = tf2::getYaw(pose.pose.orientation);
    double angle_error = target_yaw - current_yaw;

    // Normalize angle error to [-pi, pi]
    while (angle_error > M_PI)
      angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI)
      angle_error += 2.0 * M_PI;

    if (std::abs(angle_error) > goal_theta_tolerance_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Phase INITIAL_ROTATION: aligning with path (error: %.2f rad)",
          angle_error);

      geometry_msgs::msg::Twist rotate_cmd;
      rotate_cmd.linear.x = 0.0;
      double direction = (angle_error > 0) ? 1.0 : -1.0;
      rotate_cmd.angular.z = direction * (max_angular_vel_ * 0.8);

      publish_velocity_command(rotate_cmd);
      log_control_step(pose, rotate_cmd, current_time,
                       ControlPhase::INITIAL_ROTATION, 0.0, temporal_error, {});
      update_feedback(pose, temporal_error);
      return;
    }

    // Initial rotation done → transition to PATH_FOLLOWING
    RCLCPP_INFO(get_logger(),
                "Initial rotation completed. Switching to PATH_FOLLOWING.");
    control_phase_ = ControlPhase::PATH_FOLLOWING;
    // Fall through to PATH_FOLLOWING immediately
    [[fallthrough]];
  }

  // ===== PHASE 2: PATH FOLLOWING (MPC) =====
  // MPC tracks the path. Goal is reached when position + time are met.
  // Orientation is NOT checked here — it will be handled by FINAL_ROTATION.
  case ControlPhase::PATH_FOLLOWING: {
    // Get reference trajectory for the MPC horizon based on current time
    auto reference_trajectory = get_reference_trajectory_horizon(
        current_time, prediction_horizon_steps_, d_t_);

    // Solve MPC with temporal references
    double solve_time_ms = 0.0;
    auto cmd = solve_mpc(pose, reference_trajectory, solve_time_ms);

    // Publish command
    publish_velocity_command(cmd);

    // Log data.
    log_control_step(pose, cmd, current_time, ControlPhase::PATH_FOLLOWING,
                     solve_time_ms, temporal_error, last_predicted_states_);

    // Publish debug path
    if (path_pub_) {
      path_pub_->publish(global_plan_);
    }

    // Update feedback
    update_feedback(pose, temporal_error);
    if (!initialized_ || global_plan_.poses.empty()) {
      return;
    }

    // Intermediate segment (direction reversal ahead): hand over to the next
    // segment instead of the terminal goal phases. The temporal reference
    // saturates at the segment's last pose (the cusp vertex, held at its
    // arrival stamp), so the MPC brings the robot to rest on the vertex; we
    // capture once it is inside the goal tolerance and essentially stopped.
    if (current_segment_idx_ + 1 < plan_segments_.size()) {
      const auto &seg_end = global_plan_.poses.back();
      double dsx = seg_end.pose.position.x - pose.pose.position.x;
      double dsy = seg_end.pose.position.y - pose.pose.position.y;
      double dist_to_end = std::hypot(dsx, dsy);
      if (dist_to_end < goal_dist_tolerance_ &&
          std::abs(u_prev_(0)) < goal_approach_vel_) {
        current_segment_idx_++;
        global_plan_ = plan_segments_[current_segment_idx_];
        // Reset the monotonic progress tracker together with the plan, as in
        // handle_goal().
        progress_idx_ = 0;
        // Rotate in place toward the new segment's departure direction. While
        // the dwell window lasts, the new segment's reference clamps at its
        // first pose (departure stamp still in the future), so after aligning
        // the robot simply holds until the plan says to leave; any rotation
        // overrun beyond the dwell shows up as temporal error the MPC
        // recovers downstream.
        control_phase_ = ControlPhase::INITIAL_ROTATION;
        RCLCPP_INFO(get_logger(),
                    "Cusp vertex reached (%.3f m off, v=%.2f m/s). Starting "
                    "segment %zu/%zu with an in-place rotation.",
                    dist_to_end, u_prev_(0), current_segment_idx_ + 1,
                    plan_segments_.size());
      }
      return;
    }

    // Hand over to the dedicated terminal phase once the robot is on time and
    // within the capture radius of the goal. The temporal gate keeps the
    // time-constrained schedule honoured before the final precise approach; the
    // GOAL_APPROACH phase then closes the last centimetres.
    {
      const auto &goal = global_plan_.poses.back();
      double dgx = goal.pose.position.x - pose.pose.position.x;
      double dgy = goal.pose.position.y - pose.pose.position.y;
      double dist_to_goal = std::hypot(dgx, dgy);
      bool time_reached = current_time >= rclcpp::Time(goal.header.stamp);
      if (time_reached && dist_to_goal < goal_approach_radius_) {
        RCLCPP_INFO(get_logger(),
                    "Within goal approach radius (%.3f m). Switching to "
                    "GOAL_APPROACH.",
                    dist_to_goal);
        // Anchor the terminal reference to the world at this instant: record the
        // entry time and the robot's along-track offset from the goal. The
        // reference will advance from here at the approach speed on a fixed
        // world schedule (see get_approach_reference_horizon).
        double gth = goal_tangent_yaw();
        goal_approach_start_time_ = current_time;
        goal_approach_start_along_ =
            std::cos(gth) * (pose.pose.position.x - goal.pose.position.x) +
            std::sin(gth) * (pose.pose.position.y - goal.pose.position.y);
        control_phase_ = ControlPhase::GOAL_APPROACH;
      }
    }
    return;
  }

  // ===== PHASE 2b: GOAL APPROACH (terminal MPC) =====
  // Close the last centimetres to the goal precisely. We reuse the MPC but feed
  // it a terminal reference (get_approach_reference_horizon) that advances along
  // the goal tangent at the approach speed and clamps beyond the goal by
  // goal_approach_extension. The phase ends when the robot crosses the plane
  // perpendicular to the tangent at the goal (along-track residual s <= 0), with
  // a settle fallback so it never hangs.
  case ControlPhase::GOAL_APPROACH: {
    const auto &goal = global_plan_.poses.back();
    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;
    double gtheta =
        goal_tangent_yaw();       // path direction of travel into the goal
    double tx = std::cos(gtheta); // unit tangent at the goal
    double ty = std::sin(gtheta);

    // Signed along-track distance still to go: positive before the goal, <= 0
    // once the robot has crossed the goal's perpendicular plane.
    double s =
        tx * (gx - pose.pose.position.x) + ty * (gy - pose.pose.position.y);

    // Finish on plane crossing, or on a settle fallback (a hair short and
    // essentially stopped) so we can never hang if the robot stalls.
    bool crossed = s <= 0.0;
    bool settled =
        (s < goal_dist_tolerance_) && (u_prev_(0) < 0.25 * goal_approach_vel_);
    if (crossed || settled) {
      RCLCPP_INFO(get_logger(),
                  "Goal approach finished (along-track residual %.3f m, %s).",
                  s, crossed ? "crossed goal plane" : "settled");
      // Zero the linear velocity so it is not carried into the next phase.
      geometry_msgs::msg::Twist stop_cmd;
      publish_velocity_command(stop_cmd);
      if (has_goal_orientation_) {
        control_phase_ = ControlPhase::FINAL_ROTATION;
      } else {
        reset_state(true);
      }
      update_feedback(pose, temporal_error);
      return;
    }

    // Otherwise keep driving in with the terminal reference.
    auto reference_trajectory = get_approach_reference_horizon(current_time);
    double solve_time_ms = 0.0;
    auto cmd = solve_mpc(pose, reference_trajectory, solve_time_ms);
    publish_velocity_command(cmd);

    log_control_step(pose, cmd, current_time, ControlPhase::GOAL_APPROACH,
                     solve_time_ms, temporal_error, last_predicted_states_);

    if (path_pub_) {
      path_pub_->publish(global_plan_);
    }

    update_feedback(pose, temporal_error);
    return;
  }

  // ===== PHASE 3: FINAL ROTATION =====
  // Rotate in place to align with the goal orientation.
  // This only runs when has_goal_orientation_ is true.
  case ControlPhase::FINAL_ROTATION: {
    double target_yaw = tf2::getYaw(goal_orientation_);
    double current_yaw = tf2::getYaw(pose.pose.orientation);
    double angle_error = target_yaw - current_yaw;

    // Normalize angle error to [-pi, pi]
    while (angle_error > M_PI)
      angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI)
      angle_error += 2.0 * M_PI;

    if (std::abs(angle_error) > goal_theta_tolerance_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Phase FINAL_ROTATION: aligning with goal "
                           "orientation (error: %.2f rad)",
                           angle_error);

      geometry_msgs::msg::Twist rotate_cmd;
      rotate_cmd.linear.x = 0.0;
      double direction = (angle_error > 0) ? 1.0 : -1.0;
      rotate_cmd.angular.z = direction * (max_angular_vel_ * 0.8);

      publish_velocity_command(rotate_cmd);
      log_control_step(pose, rotate_cmd, current_time,
                       ControlPhase::FINAL_ROTATION, 0.0, temporal_error, {});
      update_feedback(pose, temporal_error);
      return;
    }

    // Final rotation done — goal fully achieved
    RCLCPP_INFO(get_logger(),
                "Final rotation completed. Goal orientation reached.");
    reset_state(true);
    return;
  }

  } // end switch
}

// ----- ROBOT STATE -----
geometry_msgs::msg::PoseStamped MPCController::get_robot_pose() {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = base_frame_;
  pose.header.stamp = this->now();

  try {
    // Get transform from map to base_link
    auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_,
                                                 tf2::TimePointZero);

    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    pose.header.frame_id = map_frame_;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not get robot pose: %s", ex.what());
  }

  return pose;
}

// ----- CUSP SPLITTING -----
std::vector<nav_msgs::msg::Path>
MPCController::split_path_at_cusps(const nav_msgs::msg::Path &original_path) {
  std::vector<nav_msgs::msg::Path> segments;

  const auto &poses = original_path.poses;
  if (poses.size() < 3 || cusp_angle_threshold_ >= M_PI) {
    segments.push_back(original_path);
    return segments;
  }

  // Steps shorter than this are wait actions (repeated vertices with
  // advancing stamps) and do not define a direction of motion.
  constexpr double kMotionEps = 1e-3; // m

  auto set_yaw = [](geometry_msgs::msg::PoseStamped &p, double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    p.pose.orientation = tf2::toMsg(q);
  };

  // Materialize poses[first..last] (inclusive).
  auto make_segment = [&](size_t first, size_t last) {
    nav_msgs::msg::Path seg;
    seg.header = original_path.header;
    seg.poses.assign(poses.begin() + first, poses.begin() + last + 1);
    return seg;
  };

  size_t seg_start = 0;         // first pose of the segment being built
  size_t last_motion_start = 0; // pose the most recent motion step left from
  size_t last_motion_end = 0;   // pose the most recent motion step arrived at
  double prev_dir = 0.0;        // direction of that motion step
  bool have_dir = false;
  bool entry_patch = false; // current segment starts at a cusp departure
  double entry_dir = 0.0;   // outgoing direction at that departure
  bool entry_stamp_patch = false; // departure stamp was delayed by a borrow
  rclcpp::Time entry_stamp;       // the delayed departure stamp

  // Walk the raw waypoints comparing directions of consecutive motion steps.
  // At a cusp, the incoming segment ends at the pose where the previous
  // motion ARRIVED (arrival stamp) and the outgoing segment starts at the
  // pose the new motion LEAVES from (departure stamp). Wait poses in between
  // belong to neither segment; together with the symmetric borrow applied
  // below they form the dwell window the control loop spends rotating in
  // place. Boundary orientations around the cusp are overwritten with the
  // motion direction so that (a) the alignment rotation into the next segment
  // has a well-defined target even if the planner left waypoint orientations
  // unset, and (b) the reference the MPC holds while dwelling on the vertex
  // matches the heading the robot arrives with.
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    if (std::hypot(dx, dy) < kMotionEps) {
      continue; // wait step
    }
    double dir = std::atan2(dy, dx);

    if (have_dir) {
      double diff = std::abs(std::remainder(dir - prev_dir, 2.0 * M_PI));
      if (diff > cusp_angle_threshold_) {
        rclcpp::Time t_arr(poses[last_motion_end].header.stamp); // arrival
        rclcpp::Time t_dep(poses[i].header.stamp);               // departure
        double dwell = (t_dep - t_arr).seconds();
        double rotation_time = diff / (0.8 * max_angular_vel_);

        // The planner distributes timestamps homogeneously and models no
        // rotation kinematics, so in general dwell < rotation_time. The
        // executor synthesizes the missing dwell itself: it borrows
        // deficit/2 from EACH of the two plan intervals adjacent to the
        // reversal vertex (arrive early by `borrow`, depart late by
        // `borrow`), spreading the stop symmetrically around the vertex
        // stamp so the arrival and departure speeds stay equal. Every other
        // waypoint stamp is untouched: the speed-up is confined to the cells
        // the plan already assigns the robot around the vertex, so it cannot
        // interfere with the collision guarantees of the original plan. The
        // borrow is capped so the compressed reference speed never exceeds
        // max_linear_vel_; any remaining deficit is borrowed at runtime and
        // repaid downstream by the temporal-error dynamics.
        double deficit = std::max(0.0, rotation_time - dwell);
        double borrow = 0.0;
        if (deficit > 0.0 && max_linear_vel_ > 0.0) {
          rclcpp::Time t_in_start(poses[last_motion_start].header.stamp);
          if (entry_stamp_patch && last_motion_start == seg_start) {
            // Consecutive cusps share this interval: the previous cusp
            // already delayed this segment's departure stamp.
            t_in_start = entry_stamp;
          }
          double d_in =
              std::hypot(poses[last_motion_end].pose.position.x -
                             poses[last_motion_start].pose.position.x,
                         poses[last_motion_end].pose.position.y -
                             poses[last_motion_start].pose.position.y);
          double margin_in =
              (t_arr - t_in_start).seconds() - d_in / max_linear_vel_;

          rclcpp::Time t_out_end(poses[i + 1].header.stamp);
          double d_out = std::hypot(
              poses[i + 1].pose.position.x - poses[i].pose.position.x,
              poses[i + 1].pose.position.y - poses[i].pose.position.y);
          double margin_out =
              (t_out_end - t_dep).seconds() - d_out / max_linear_vel_;

          borrow =
              std::max(0.0, std::min({deficit / 2.0, margin_in, margin_out}));
        }

        // Close the incoming segment at its arrival pose on the cusp vertex,
        // shifted `borrow` seconds early.
        auto seg = make_segment(seg_start, last_motion_end);
        if (entry_patch) {
          set_yaw(seg.poses.front(), entry_dir);
        }
        if (entry_stamp_patch) {
          seg.poses.front().header.stamp =
              static_cast<builtin_interfaces::msg::Time>(entry_stamp);
        }
        set_yaw(seg.poses.back(), prev_dir);
        if (borrow > 0.0) {
          seg.poses.back().header.stamp =
              static_cast<builtin_interfaces::msg::Time>(
                  t_arr - rclcpp::Duration::from_seconds(borrow));
        }
        segments.push_back(std::move(seg));

        double window = dwell + 2.0 * borrow;
        RCLCPP_INFO(get_logger(),
                    "Cusp at (%.2f, %.2f): %.0f deg reversal, plan dwell "
                    "%.2f s, rotation needs ~%.2f s, borrowed %.2f s/side "
                    "from adjacent intervals -> window %.2f s (%s)",
                    poses[i].pose.position.x, poses[i].pose.position.y,
                    diff * 180.0 / M_PI, dwell, rotation_time, borrow, window,
                    window + 1e-9 >= rotation_time
                        ? "feasible"
                        : "still short; remainder repaid at runtime");

        seg_start = i;
        entry_patch = true;
        entry_dir = dir;
        entry_stamp_patch = borrow > 0.0;
        if (entry_stamp_patch) {
          entry_stamp = t_dep + rclcpp::Duration::from_seconds(borrow);
        }
      }
    }

    prev_dir = dir;
    have_dir = true;
    last_motion_start = i;
    last_motion_end = i + 1;
  }

  // Final segment up to the end of the plan (its last pose keeps the goal
  // orientation resolved in handle_goal()).
  auto seg = make_segment(seg_start, poses.size() - 1);
  if (entry_patch) {
    set_yaw(seg.poses.front(), entry_dir);
  }
  if (entry_stamp_patch) {
    seg.poses.front().header.stamp =
        static_cast<builtin_interfaces::msg::Time>(entry_stamp);
  }
  segments.push_back(std::move(seg));

  return segments;
}

// ----- PATH INTERPOLATION -----
nav_msgs::msg::Path
MPCController::interpolate_path(const nav_msgs::msg::Path &original_path,
                                double target_spacing) {
  nav_msgs::msg::Path interpolated_path;
  interpolated_path.header = original_path.header;

  if (original_path.poses.size() < 2) {
    // Not enough points to interpolate, return original
    return original_path;
  }

  RCLCPP_INFO(get_logger(), "Interpolating path with target spacing: %.2fm",
              target_spacing);

  // Always include the first point
  interpolated_path.poses.push_back(original_path.poses[0]);

  // Interpolate between consecutive poses
  for (size_t i = 0; i < original_path.poses.size() - 1; ++i) {
    const auto &pose0 = original_path.poses[i];
    const auto &pose1 = original_path.poses[i + 1];

    // Calculate distance between poses
    double dx = pose1.pose.position.x - pose0.pose.position.x;
    double dy = pose1.pose.position.y - pose0.pose.position.y;
    double segment_dist = std::hypot(dx, dy);

    // Calculate time difference
    rclcpp::Time t0(pose0.header.stamp);
    rclcpp::Time t1(pose1.header.stamp);
    double dt_total = (t1 - t0).seconds();

    // Calculate number of intermediate points needed
    int num_interpolated =
        static_cast<int>(std::floor(segment_dist / target_spacing));

    // Add interpolated points
    for (int j = 1; j <= num_interpolated; ++j) {
      double ratio = static_cast<double>(j) / (num_interpolated + 1);

      geometry_msgs::msg::PoseStamped interp_pose;
      interp_pose.header.frame_id = pose0.header.frame_id;

      // Interpolate position
      interp_pose.pose.position.x = pose0.pose.position.x + ratio * dx;
      interp_pose.pose.position.y = pose0.pose.position.y + ratio * dy;
      interp_pose.pose.position.z =
          pose0.pose.position.z +
          ratio * (pose1.pose.position.z - pose0.pose.position.z);

      // Interpolate timestamp
      rclcpp::Time interp_time =
          t0 + rclcpp::Duration::from_seconds(ratio * dt_total);
      interp_pose.header.stamp =
          static_cast<builtin_interfaces::msg::Time>(interp_time);

      // Interpolate orientation (simple approach: use direction of motion)
      double theta = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      interp_pose.pose.orientation = tf2::toMsg(q);

      interpolated_path.poses.push_back(interp_pose);
    }

    // Add the next original point (unless it's the last one, handled below)
    if (i < original_path.poses.size() - 2) {
      interpolated_path.poses.push_back(pose1);
    }
  }

  // Always include the last point
  interpolated_path.poses.push_back(original_path.poses.back());

  RCLCPP_INFO(get_logger(), "Path interpolated: %zu -> %zu poses",
              original_path.poses.size(), interpolated_path.poses.size());

  return interpolated_path;
}

// ----- PATH SMOOTHING -----
nav_msgs::msg::Path
MPCController::smooth_path(const nav_msgs::msg::Path &original_path,
                           double smoothing_window) {
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = original_path.header;

  // Special case: no smoothing requested
  if (smoothing_window == 0.0) {
    RCLCPP_INFO(get_logger(), "Path smoothing disabled (window = 0.0m)");
    return original_path;
  }

  if (original_path.poses.size() < 3) {
    // Not enough points to smooth, return original
    return original_path;
  }

  RCLCPP_INFO(get_logger(), "Smoothing path with %zu poses (window: %.2fm)",
              original_path.poses.size(), smoothing_window);

  // Convert smoothing window from meters to number of points
  // Calculate average spacing between points
  double total_dist = 0.0;
  for (size_t i = 0; i < original_path.poses.size() - 1; ++i) {
    double dx = original_path.poses[i + 1].pose.position.x -
                original_path.poses[i].pose.position.x;
    double dy = original_path.poses[i + 1].pose.position.y -
                original_path.poses[i].pose.position.y;
    total_dist += std::hypot(dx, dy);
  }
  double avg_spacing = total_dist / (original_path.poses.size() - 1);
  int window_points =
      std::max(2, static_cast<int>(smoothing_window / avg_spacing));

  RCLCPP_INFO(get_logger(), "Average spacing: %.3fm, window points: %d",
              avg_spacing, window_points);

  // Apply moving average filter
  for (size_t i = 0; i < original_path.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped smoothed_pose;
    smoothed_pose.header = original_path.poses[i].header;

    // Keep first and last points unchanged
    if (i == 0 || i == original_path.poses.size() - 1) {
      smoothed_pose = original_path.poses[i];
      smoothed_path.poses.push_back(smoothed_pose);
      continue;
    }

    // Calculate window bounds
    int start_idx = std::max(0, static_cast<int>(i) - window_points);
    int end_idx = std::min(static_cast<int>(original_path.poses.size()) - 1,
                           static_cast<int>(i) + window_points);

    // Average position
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;
    for (int j = start_idx; j <= end_idx; ++j) {
      sum_x += original_path.poses[j].pose.position.x;
      sum_y += original_path.poses[j].pose.position.y;
      count++;
    }

    smoothed_pose.pose.position.x = sum_x / count;
    smoothed_pose.pose.position.y = sum_y / count;
    smoothed_pose.pose.position.z = original_path.poses[i].pose.position.z;

    // Recompute orientation based on smoothed positions
    if (i < original_path.poses.size() - 1) {
      double dx = smoothed_path.poses.size() > 0
                      ? (smoothed_pose.pose.position.x -
                         smoothed_path.poses.back().pose.position.x)
                      : (original_path.poses[i + 1].pose.position.x -
                         smoothed_pose.pose.position.x);
      double dy = smoothed_path.poses.size() > 0
                      ? (smoothed_pose.pose.position.y -
                         smoothed_path.poses.back().pose.position.y)
                      : (original_path.poses[i + 1].pose.position.y -
                         smoothed_pose.pose.position.y);
      double theta = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      smoothed_pose.pose.orientation = tf2::toMsg(q);
    } else {
      smoothed_pose.pose.orientation = original_path.poses[i].pose.orientation;
    }

    smoothed_path.poses.push_back(smoothed_pose);
  }

  RCLCPP_INFO(get_logger(), "Path smoothed: %zu -> %zu poses",
              original_path.poses.size(), smoothed_path.poses.size());

  return smoothed_path;
}

// ----- PATH RESAMPLING AND RETIMING -----
nav_msgs::msg::Path MPCController::resample_and_retime_path(
    const nav_msgs::msg::Path &smoothed_path, double spacing) {
  if (smoothed_path.poses.size() < 2) {
    return smoothed_path;
  }

  RCLCPP_INFO(get_logger(), "Resampling and retiming path with spacing: %.2fm",
              spacing);

  // 1. Calculate total length of smoothed_path (geometry)
  std::vector<double> smooth_dists;
  smooth_dists.push_back(0.0);
  for (size_t i = 0; i < smoothed_path.poses.size() - 1; ++i) {
    double dx = smoothed_path.poses[i + 1].pose.position.x -
                smoothed_path.poses[i].pose.position.x;
    double dy = smoothed_path.poses[i + 1].pose.position.y -
                smoothed_path.poses[i].pose.position.y;
    double d = std::hypot(dx, dy);
    smooth_dists.push_back(smooth_dists.back() + d);
  }
  double total_smooth_dist = smooth_dists.back();

  // 2. Resample smoothed path at fixed spacing
  nav_msgs::msg::Path final_path;
  final_path.header = smoothed_path.header;

  // Add start point exactly (smooth_path keeps it untouched, so it already
  // carries the plan's first timestamp)
  final_path.poses.push_back(smoothed_path.poses.front());

  double current_dist = spacing;
  size_t current_idx = 0; // Index in smoothed_path

  while (current_dist < total_smooth_dist) {
    // Find segment in smoothed_path
    while (current_idx < smooth_dists.size() - 1 &&
           smooth_dists[current_idx + 1] < current_dist) {
      current_idx++;
    }

    if (current_idx >= smoothed_path.poses.size() - 1)
      break;

    // Interpolate geometry
    double seg_start_dist = smooth_dists[current_idx];
    double seg_end_dist = smooth_dists[current_idx + 1];
    double seg_len = seg_end_dist - seg_start_dist;

    double ratio = 0.0;
    if (seg_len > 1e-6) {
      ratio = (current_dist - seg_start_dist) / seg_len;
    }

    const auto &p1 = smoothed_path.poses[current_idx];
    const auto &p2 = smoothed_path.poses[current_idx + 1];

    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header = smoothed_path.header;
    new_pose.pose.position.x =
        p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
    new_pose.pose.position.y =
        p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
    new_pose.pose.position.z = p1.pose.position.z; // Keep Z

    // Calculate Orientation (Tangent)
    double tangent_theta = std::atan2(p2.pose.position.y - p1.pose.position.y,
                                      p2.pose.position.x - p1.pose.position.x);
    tf2::Quaternion q;
    q.setRPY(0, 0, tangent_theta);
    new_pose.pose.orientation = tf2::toMsg(q);

    // Map distance to time LOCALLY: smooth_path() is index-preserving and
    // carries every pose's plan timestamp through, so the stamps of the two
    // bracketing smoothed poses bound this sample's time. Interpolating
    // within the bracket preserves the plan's local time structure
    // (per-waypoint speeds, waits) instead of smearing it over the whole path
    // with a global distance normalisation -- which also broke down at
    // reversals and waits, where time advances at zero arc length. Where
    // smoothing shortens the geometry (corner cutting), the same plan time
    // spans a slightly shorter arc and the reference naturally slows down at
    // the corner.
    rclcpp::Time t1(p1.header.stamp);
    rclcpp::Time t2(p2.header.stamp);
    new_pose.header.stamp = static_cast<builtin_interfaces::msg::Time>(
        t1 + rclcpp::Duration::from_seconds(ratio * (t2 - t1).seconds()));

    final_path.poses.push_back(new_pose);

    current_dist += spacing;
  }

  // Add end point exactly (smooth_path keeps it untouched, so it already
  // carries the plan's exact end timestamp)
  final_path.poses.push_back(smoothed_path.poses.back());

  RCLCPP_INFO(get_logger(), "Resampled path: %zu -> %zu poses",
              smoothed_path.poses.size(), final_path.poses.size());

  return final_path;
}

// ----- TEMPORAL REFERENCE CALCULATION -----
geometry_msgs::msg::PoseStamped
MPCController::get_temporal_reference(const rclcpp::Time &requested_time) {
  geometry_msgs::msg::PoseStamped reference;

  if (global_plan_.poses.empty()) {
    return reference;
  }

  // Anti-windup on the temporal reference: bound its spatial lead over the
  // robot's actual progress on the plan (progress_idx_, maintained by
  // calculate_temporal_error every cycle). The MPC linearizes its input map
  // around the reference orientation, which is only valid near the reference;
  // if the robot falls behind (e.g. an in-place cusp rotation that had to
  // borrow schedule time), an unbounded reference keeps racing ahead, drags
  // the linearization out of its validity region (observed as saturated
  // pirouetting) and invites corner cutting. Clamping the lookup time to the
  // stamp of the pose max_reference_lead_ metres ahead of the matched
  // progress keeps the tracking problem well-posed; the schedule lag is still
  // measured against the ORIGINAL stamps by calculate_temporal_error, so the
  // reported temporal error stays honest and the clamp releases itself as the
  // robot advances and catches up.
  rclcpp::Time target_time = requested_time;
  if (max_reference_lead_ > 0.0) {
    const auto &poses = global_plan_.poses;
    size_t lead_idx = progress_idx_;
    double arc = 0.0;
    while (lead_idx + 1 < poses.size() && arc < max_reference_lead_) {
      double dx = poses[lead_idx + 1].pose.position.x -
                  poses[lead_idx].pose.position.x;
      double dy = poses[lead_idx + 1].pose.position.y -
                  poses[lead_idx].pose.position.y;
      arc += std::hypot(dx, dy);
      ++lead_idx;
    }
    rclcpp::Time lead_time(poses[lead_idx].header.stamp);
    if (target_time > lead_time) {
      target_time = lead_time;
    }
  }

  // If target time is before the first pose, return first pose
  rclcpp::Time first_time(global_plan_.poses.front().header.stamp);
  if (target_time <= first_time) {
    reference = global_plan_.poses.front();
    reference.header.stamp = target_time;
    return reference;
  }

  // If target time is after the last pose, return last pose
  rclcpp::Time last_time(global_plan_.poses.back().header.stamp);
  if (target_time >= last_time) {
    reference = global_plan_.poses.back();
    reference.header.stamp = target_time;
    return reference;
  }

  // Find the two poses that bracket the target time
  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i) {
    rclcpp::Time t0(global_plan_.poses[i].header.stamp);
    rclcpp::Time t1(global_plan_.poses[i + 1].header.stamp);

    if (target_time >= t0 && target_time <= t1) {
      // Interpolate between poses i and i+1
      double dt_total = (t1 - t0).seconds();
      double dt_elapsed = (target_time - t0).seconds();

      if (dt_total < 1e-6) {
        // Timestamps are too close, just return first pose
        reference = global_plan_.poses[i];
        reference.header.stamp = target_time;
        return reference;
      }

      double ratio = dt_elapsed / dt_total;

      // Linear interpolation of position
      reference.pose.position.x =
          global_plan_.poses[i].pose.position.x +
          ratio * (global_plan_.poses[i + 1].pose.position.x -
                   global_plan_.poses[i].pose.position.x);
      reference.pose.position.y =
          global_plan_.poses[i].pose.position.y +
          ratio * (global_plan_.poses[i + 1].pose.position.y -
                   global_plan_.poses[i].pose.position.y);

      // Angular interpolation: properly handle ±180° crossing
      double theta_0 = tf2::getYaw(global_plan_.poses[i].pose.orientation);
      double theta_1 = tf2::getYaw(global_plan_.poses[i + 1].pose.orientation);

      // Calculate angular difference (take shortest path)
      double angular_diff = theta_1 - theta_0;
      // Adjust if difference is larger than π
      if (angular_diff > M_PI)
        angular_diff -= 2.0 * M_PI;
      else if (angular_diff < -M_PI)
        angular_diff += 2.0 * M_PI;

      // Interpolate along the shortest path
      double theta_interp = theta_0 + ratio * angular_diff;

      // Convert back to quaternion
      tf2::Quaternion q;
      q.setRPY(0, 0, theta_interp);
      reference.pose.orientation = tf2::toMsg(q);

      reference.header.stamp = target_time;
      reference.header.frame_id = global_plan_.header.frame_id;

      return reference;
    }
  }

  // Fallback: return last pose
  reference = global_plan_.poses.back();
  reference.header.stamp = target_time;
  return reference;
}

std::vector<Eigen::Vector4d> MPCController::get_reference_trajectory_horizon(
    const rclcpp::Time &current_time, int N, double dt) {
  std::vector<Eigen::Vector4d> references;
  references.reserve(N);

  for (int i = 0; i < N; ++i) {
    // Calculate target time for this step in the horizon
    rclcpp::Time target_time =
        current_time + rclcpp::Duration::from_seconds(i * dt);

    // Get the pose at that time
    auto pose = get_temporal_reference(target_time);

    // Publish the first reference for debugging
    if (i == 0 && debug_mpc_ && debug_pose_pub_) {
      debug_pose_pub_->publish(pose);
    }

    // Convert to Eigen vector [x, y, sin(theta), cos(theta)]
    double theta = tf2::getYaw(pose.pose.orientation);
    Eigen::Vector4d ref;
    ref(0) = pose.pose.position.x;
    ref(1) = pose.pose.position.y;
    ref(2) = std::sin(theta);
    ref(3) = std::cos(theta);

    references.push_back(ref);
  }

  return references;
}

double MPCController::goal_tangent_yaw() {
  const auto &poses = global_plan_.poses;
  const auto &goal = poses.back();
  // Walk back from the goal to the first point that is far enough to define a
  // reliable direction of travel into the goal (robust to duplicated end
  // samples). This is the geometric path tangent, which may differ from the
  // goal orientation when an explicit final orientation was requested.
  for (int i = static_cast<int>(poses.size()) - 2; i >= 0; --i) {
    double dx = goal.pose.position.x - poses[i].pose.position.x;
    double dy = goal.pose.position.y - poses[i].pose.position.y;
    if (std::hypot(dx, dy) > 1e-3) {
      return std::atan2(dy, dx);
    }
  }
  // Degenerate path (all points coincident): fall back to the goal orientation.
  return tf2::getYaw(goal.pose.orientation);
}

std::vector<Eigen::Vector4d>
MPCController::get_approach_reference_horizon(const rclcpp::Time &current_time) {
  // Terminal (goal-approach) reference.
  //
  // We reuse the trajectory-tracking MPC but feed it a purpose-built reference
  // for the final centimetres: a point on the goal tangent that advances at the
  // approach speed on a fixed WORLD schedule. Its along-track offset from the
  // goal is a_head(t) = a_entry + v_app * (t - t_entry), where a_entry and
  // t_entry were captured when the phase began, and it CLAMPS at
  // goal_approach_extension beyond the goal.
  //
  // Anchoring the reference to the world (rather than recomputing it from the
  // current robot pose each cycle) is what makes the tracker hold the approach
  // speed: if the robot falls behind, the reference keeps moving and the growing
  // error pulls the command back up to v_app, exactly as in path following. Were
  // the reference re-anchored to the robot every cycle, the model's linearisation
  // bias would instead settle the speed below v_app.
  //
  // The extension controls the braking near the goal. With extension 0 the head
  // stops on the goal, so the horizon tail piles up on the target and the MPC
  // decelerates to rest on it (no overshoot, but a slow final crawl). With
  // extension >= N*v_app*dt the goal never falls inside the clamped tail, so the
  // reference keeps pulling through the real goal and the robot crosses it at the
  // approach speed with essentially no braking (the phase then stops it on the
  // goal-plane crossing). Intermediate values give a terminal speed between 0 and
  // v_app. Every sample lies on the tangent line through the goal, which also
  // drives the cross-track error to zero. The speed is shaped purely by the
  // reference schedule, so the solver keeps coordinating v and omega optimally
  // under the usual limits (no velocity is hard-fixed).
  std::vector<Eigen::Vector4d> references;
  references.reserve(prediction_horizon_steps_);

  const auto &goal = global_plan_.poses.back();
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;
  const double gtheta =
      goal_tangent_yaw();                // path direction of travel into goal
  const double c_ref = std::cos(gtheta); // unit tangent at the goal
  const double s_ref = std::sin(gtheta);

  // Along-track offset of the reference head now (advanced from the entry anchor
  // at the approach speed), clamped beyond the goal by the extension.
  const double elapsed = (current_time - goal_approach_start_time_).seconds();
  const double a_head = goal_approach_start_along_ + goal_approach_vel_ * elapsed;

  for (int i = 0; i < prediction_horizon_steps_; ++i) {
    // Reference for horizon step i, i.e. the head projected dt*i into the future.
    double a_i = a_head + goal_approach_vel_ * i * d_t_;
    if (a_i > goal_approach_extension_) {
      a_i = goal_approach_extension_;
    }
    Eigen::Vector4d ref;
    ref(0) = gx + c_ref * a_i; // on the tangent line through the goal
    ref(1) = gy + s_ref * a_i;
    ref(2) = s_ref; // heading aligned with the path tangent
    ref(3) = c_ref;
    references.push_back(ref);
  }

  return references;
}

double MPCController::calculate_temporal_error(
    geometry_msgs::msg::PoseStamped current_pose, rclcpp::Time current_time) {
  if (global_plan_.poses.empty()) {
    return 0.0;
  }

  const auto &poses = global_plan_.poses;
  const size_t N = poses.size();

  // Guard against a stale index left over from a previous (possibly longer)
  // plan, in case this runs before the reset in handle_goal/reset_state.
  if (progress_idx_ >= N) {
    progress_idx_ = N - 1;
  }

  auto dist_to = [&](size_t i) {
    double dx = poses[i].pose.position.x - current_pose.pose.position.x;
    double dy = poses[i].pose.position.y - current_pose.pose.position.y;
    return std::hypot(dx, dy);
  };

  // Locate the robot on the plan with a MONOTONIC, bounded forward-window
  // argmin: scan from the last progress index forward, accumulating arc length,
  // and keep the closest pose within progress_search_window_ metres. The index
  // never moves backward (so a past self-crossing branch cannot be matched ->
  // no spurious timeout) and the search span is capped (so a future branch the
  // path revisits cannot be jumped onto -> no overstated progress). This
  // assumes the plan does not fold back on itself within the search window; at
  // the ~0.1 m resample spacing and typical loop sizes that holds comfortably.
  size_t best_idx = progress_idx_;
  double min_dist = dist_to(progress_idx_);
  double arc = 0.0;
  for (size_t i = progress_idx_; i + 1 < N && arc < progress_search_window_;
       ++i) {
    double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    arc += std::hypot(dx, dy);
    double d = dist_to(i + 1);
    if (d < min_dist) {
      min_dist = d;
      best_idx = i + 1;
    }
  }
  progress_idx_ = best_idx;

  // Get the timestamp of the matched pose
  rclcpp::Time trajectory_time(poses[progress_idx_].header.stamp);

  // Calculate temporal error: positive = ahead of schedule, negative = behind
  double temporal_error = (trajectory_time - current_time).seconds();

  // Log temporal tracking information
  if (std::abs(temporal_error) > 1.0) {
    if (temporal_error > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Temporal tracking: %.2f s ahead of schedule",
                           temporal_error);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Temporal tracking: %.2f s behind schedule",
                           std::abs(temporal_error));
    }
  }
  RCLCPP_DEBUG(get_logger(), "Temporal error: %.3f s", temporal_error);

  return temporal_error;
}

// ----- MPC SOLVER -----
geometry_msgs::msg::Twist MPCController::solve_mpc(
    const geometry_msgs::msg::PoseStamped &pose,
    const std::vector<Eigen::Vector4d> &reference_trajectory,
    double &solve_time_ms) {
  geometry_msgs::msg::Twist cmd;
  solve_time_ms = 0.0;

  // 1. Build QP matrices
  // --------------------------------------------------------
  // State Vector:   x = [x, y, sin(theta), cos(theta)]^T
  // Control Vector: u = [v, omega]^T
  // Augmented State: xi = [x^T, u_{k-1}^T]^T
  // --------------------------------------------------------

  Eigen::Vector4d current_state;
  current_state << pose.pose.position.x, pose.pose.position.y,
      std::sin(tf2::getYaw(pose.pose.orientation)),
      std::cos(tf2::getYaw(pose.pose.orientation));

  // Linearization Point
  // We linearize the non-linear dynamics around the previous control input
  // u_prev_ (= u_{k-1}), NOT an odometry measurement. This is the controller's
  // notion of the robot's current velocity, so it relies solely on external
  // POSITION feedback (tf/mocap) and the linearization stays consistent with
  // the incremental (delta-u) formulation, whose augmented state also
  // propagates u_{k-1}. Using the command instead of a measurement removes
  // sensitivity to low-speed odometry artifacts (noise, encoder quantization,
  // actuator deadband) that otherwise corrupt the orientation->position
  // coupling term A_d(0,3)=A_d(1,2)=v*dt and degrade low-speed tracking. NOTE:
  // we copy into a local u_ref so the singularity avoidance below does not
  // mutate the persistent u_prev_ integrator state.
  Eigen::Vector2d u_ref = u_prev_;

  // Singularity Avoidance:
  // When v approx 0, the Jacobian terms coupling orientation to position
  // vanish. We impose a minimum linearization velocity (|v| >= 0.01 m/s) to
  // ensure the solver acknowledges that orientation changes affect the spatial
  // state.
  constexpr double MIN_LINEARIZATION_VEL = 0.01;
  if (std::abs(u_ref(0)) < MIN_LINEARIZATION_VEL) {
    u_ref(0) = (u_ref(0) >= 0) ? MIN_LINEARIZATION_VEL : -MIN_LINEARIZATION_VEL;
  }
  if (reference_trajectory.size() > 1) {
    // Estimate from trajectory... for now assume 0 or last command
    // Ideally we would have u_ref in the trajectory.
  }

  // Sparse matrices for OSQP
  Eigen::SparseMatrix<double> P_eigen;
  Eigen::VectorXd q_eigen;
  Eigen::SparseMatrix<double> A_eigen;
  Eigen::VectorXd l_eigen;
  Eigen::VectorXd u_eigen;

  build_mpc_matrices(current_state, reference_trajectory, u_ref, P_eigen,
                     q_eigen, A_eigen, l_eigen, u_eigen);

  // 2. Convert to OSQP format
  // Note: Eigen stores in CCS (Compressed Column Storage) which is compatible
  // with CSC IF we make sure it is compressed. .makeCompressed() does that.

  P_eigen.makeCompressed();
  A_eigen.makeCompressed();

  // Extract data arrays
  // In OSQP 1.0, we use OSQPInt and OSQPFloat
  // cast from Eigen int/double to OSQP types

  OSQPInt n_vars = q_eigen.size();
  OSQPInt m_constraints = l_eigen.size();

  std::vector<OSQPFloat> q_data(q_eigen.data(),
                                q_eigen.data() + q_eigen.size());
  std::vector<OSQPFloat> l_data(l_eigen.data(),
                                l_eigen.data() + l_eigen.size());
  std::vector<OSQPFloat> u_data(u_eigen.data(),
                                u_eigen.data() + u_eigen.size());

  // P matrix - FILTER FOR UPPER TRIANGULAR ONLY
  std::vector<OSQPFloat> P_val;
  std::vector<OSQPInt> P_row_idx;
  std::vector<OSQPInt> P_col_ptr;
  P_col_ptr.reserve(P_eigen.outerSize() + 1);
  P_col_ptr.push_back(0); // Start with 0

  for (int k = 0; k < P_eigen.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(P_eigen, k); it; ++it) {
      if (it.row() <= it.col()) { // Keep only upper triangular part
        P_val.push_back(static_cast<OSQPFloat>(it.value()));
        P_row_idx.push_back(static_cast<OSQPInt>(it.row()));
      }
    }
    P_col_ptr.push_back(static_cast<OSQPInt>(P_val.size()));
  }

  // A matrix
  std::vector<OSQPFloat> A_val(A_eigen.valuePtr(),
                               A_eigen.valuePtr() + A_eigen.nonZeros());
  std::vector<OSQPInt> A_row_idx(A_eigen.innerIndexPtr(),
                                 A_eigen.innerIndexPtr() + A_eigen.nonZeros());
  std::vector<OSQPInt> A_col_ptr(A_eigen.outerIndexPtr(),
                                 A_eigen.outerIndexPtr() + A_eigen.outerSize() +
                                     1);

  // 3. Setup OSQP
  OSQPSolver *solver = nullptr;
  OSQPSettings *settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));

  if (settings) {
    osqp_set_default_settings(settings);
    settings->verbose = 0; // Disable printing
    settings->alpha = 1.0; // ADMM alpha
    // settings->eps_abs = 1e-3;
    // settings->eps_rel = 1e-3;
    // settings->max_iter = 4000;
  }

  // Create CSC matrices using OSQP helper or manual struct population
  OSQPCscMatrix P_mat;
  OSQPCscMatrix_set_data(&P_mat, n_vars, n_vars, P_val.size(), P_val.data(),
                         P_row_idx.data(), P_col_ptr.data());

  OSQPCscMatrix A_mat;
  OSQPCscMatrix_set_data(&A_mat, m_constraints, n_vars, A_val.size(),
                         A_val.data(), A_row_idx.data(), A_col_ptr.data());

  OSQPInt exitflag =
      osqp_setup(&solver, &P_mat, q_data.data(), &A_mat, l_data.data(),
                 u_data.data(), m_constraints, n_vars, settings);

  if (exitflag == 0) {
    // 4. Solve
    auto start_solve = std::chrono::steady_clock::now();
    osqp_solve(solver);
    auto end_solve = std::chrono::steady_clock::now();
    solve_time_ms =
        std::chrono::duration<double, std::milli>(end_solve - start_solve)
            .count();

    // 5. Extract solution
    if (solver->info->status_val == OSQP_SOLVED ||
        solver->info->status_val == OSQP_SOLVED_INACCURATE) {
      // Extract optimal control increments
      double delta_v = solver->solution->x[0];
      double delta_w = solver->solution->x[1];

      // Update persistent execution state (u_{k-1})
      // But first, save u_{k-1} for prediction start
      double v_last_cmd = u_prev_(0);
      double w_last_cmd = u_prev_(1);

      // Current control: u_k = u_{k-1} + \Delta u_k
      double u_v = v_last_cmd + delta_v;
      double u_w = w_last_cmd + delta_w;

      // Update state for next step
      u_prev_(0) = u_v;
      u_prev_(1) = u_w;

      // Saturate commands for safety
      cmd.linear.x = std::clamp(u_v, 0.0, max_linear_vel_);
      cmd.angular.z = std::clamp(u_w, -max_angular_vel_, max_angular_vel_);

      // === FORWARD SIMULATION (Prediction) ===
      // Reconstruct the predicted trajectory from the optimal control
      // increments. This is used for both logging and visual debugging.
      last_predicted_states_.clear();
      last_predicted_states_.reserve(prediction_horizon_steps_);

      // Simulation variables
      double x_sim = current_state(0);
      double y_sim = current_state(1);
      double s_sim = current_state(2);
      double c_sim = current_state(3);
      double v_sim = v_last_cmd;
      double w_sim = w_last_cmd;

      for (int k = 0; k < prediction_horizon_steps_; ++k) {
        // Apply control increment for step k
        if (k < control_horizon_steps_) {
          v_sim += solver->solution->x[2 * k];
          w_sim += solver->solution->x[2 * k + 1];

          // Saturate predicted velocity to match robot constraints.
          // This prevents the visualization from showing "backward" paths
          // (negative velocity) when the real robot is clamped to 0.
          v_sim = std::clamp(v_sim, 0.0, max_linear_vel_);
          w_sim = std::clamp(w_sim, -max_angular_vel_, max_angular_vel_);
        }

        // Integrate Dynamics (Euler Forward)
        x_sim += v_sim * c_sim * d_t_;
        y_sim += v_sim * s_sim * d_t_;
        double s_next = s_sim + c_sim * w_sim * d_t_;
        double c_next = c_sim - s_sim * w_sim * d_t_;

        // Normalize orientation vector to prevent numerical drift
        double norm = std::hypot(s_next, c_next);
        if (norm > 1e-6) {
          s_sim = s_next / norm;
          c_sim = c_next / norm;
        } else {
          s_sim = s_next;
          c_sim = c_next;
        }

        // Store
        last_predicted_states_.push_back(
            Eigen::Vector4d(x_sim, y_sim, s_sim, c_sim));
      }

      // Visual Debugging
      if (debug_mpc_) {
        nav_msgs::msg::Path predicted_path;
        predicted_path.header.frame_id = map_frame_;
        predicted_path.header.stamp = this->now();
        predicted_path.poses.reserve(prediction_horizon_steps_ + 1);
        predicted_path.poses.push_back(pose); // Add start pose

        for (const auto &state : last_predicted_states_) {
          geometry_msgs::msg::PoseStamped p;
          p.header = predicted_path.header;
          p.pose.position.x = state(0);
          p.pose.position.y = state(1);
          p.pose.position.z = 0.0;

          tf2::Quaternion q;
          q.setRPY(0, 0, std::atan2(state(2), state(3)));
          p.pose.orientation = tf2::toMsg(q);

          predicted_path.poses.push_back(p);
        }

        if (predicted_path_pub_) {
          predicted_path_pub_->publish(predicted_path);
        }
      }

    } else {
      RCLCPP_WARN(get_logger(), "MPC solver failed with status: %lld",
                  solver->info ? (long long)solver->info->status_val : -1);
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
  }

  // Cleanup
  if (solver)
    osqp_cleanup(solver);
  if (settings)
    free(settings);

  return cmd;
}

// ----- COMMAND PUBLICATION -----
void MPCController::publish_velocity_command(
    const geometry_msgs::msg::Twist &cmd) {
  if (use_stamped_cmd_vel_) {
    if (cmd_vel_stamped_pub_) {
      // Publish TwistStamped
      geometry_msgs::msg::TwistStamped cmd_stamped;
      cmd_stamped.header.stamp = this->now();
      cmd_stamped.header.frame_id = "base_link";
      cmd_stamped.twist = cmd;
      cmd_vel_stamped_pub_->publish(cmd_stamped);
    }
  } else {
    if (cmd_vel_pub_) {
      // Publish Twist
      cmd_vel_pub_->publish(cmd);
    }
  }
}

// ----- PATH PUBLICATION -----
void MPCController::publish_debug_path() {
  if (!global_plan_.poses.empty() && path_pub_) {
    path_pub_->publish(global_plan_);
  }
}

// ----- ACTION FEEDBACK -----
void MPCController::update_feedback(const geometry_msgs::msg::PoseStamped &pose,
                                    double temporal_error) {
  if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
    return;
  }

  auto feedback = std::make_shared<nav2_msgs::action::FollowPath::Feedback>();
  // Report the last commanded linear velocity (the MPC's u_prev_); no odometry.
  feedback->speed = u_prev_(0);

  // Calculate distance to temporal reference instead of final goal
  rclcpp::Time current_time = this->now();
  auto ref_pose = get_temporal_reference(current_time);
  double dx = ref_pose.pose.position.x - pose.pose.position.x;
  double dy = ref_pose.pose.position.y - pose.pose.position.y;
  double distance_to_ref = std::hypot(dx, dy);
  feedback->distance_to_goal = distance_to_ref;

  current_goal_handle_->publish_feedback(feedback);

  // --- SAFETY ABORT CHECKS ---

  // 1. Spatial tracking error check
  if (max_spatial_error_ > 0.0 && distance_to_ref > max_spatial_error_) {
    RCLCPP_ERROR(get_logger(),
                 "Safety Abort: Spatial tracking error (%.2f m) exceeded "
                 "maximum limit (%.2f m)!",
                 distance_to_ref, max_spatial_error_);
    reset_state(false);
    return;
  }

  // 2. Temporal delay check. Skipped during the terminal phases (GOAL_APPROACH
  // and FINAL_ROTATION), whose small deliberate slow-down is not a tracking
  // failure and must not trigger a temporal abort.
  if (max_temporal_error_ > 0.0 && !global_plan_.poses.empty() &&
      control_phase_ != ControlPhase::GOAL_APPROACH &&
      control_phase_ != ControlPhase::FINAL_ROTATION) {
    // The temporal_error returned by calculate_temporal_error() is:
    // (trajectory_time - current_time) If we are lagging behind,
    // trajectory_time < current_time, making temporal_error negative.
    // Therefore, the tracking delay is exactly -temporal_error.
    double delay = -temporal_error;

    if (delay > max_temporal_error_) {
      RCLCPP_ERROR(get_logger(),
                   "Safety Abort: Temporal delay (%.2f s) exceeded maximum "
                   "limit (%.2f s)!",
                   delay, max_temporal_error_);
      reset_state(false);
      return;
    }
  }
}

// ----- LOGGING -----
void MPCController::log_control_step(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &cmd, const rclcpp::Time &current_time,
    ControlPhase phase, double solve_time_ms, double temporal_error,
    const std::vector<Eigen::Vector4d> &predicted_states) {
  if (!debug_mpc_ || !mpc_logger_ || global_plan_.poses.empty()) {
    return;
  }

  // Reference the controller is (or would be) tracking at this instant, and the
  // spatial error against it. During the rotation phases the robot does not
  // translate, so this is the standing distance to the reference.
  auto ref_pose = get_temporal_reference(current_time);
  double dx = ref_pose.pose.position.x - pose.pose.position.x;
  double dy = ref_pose.pose.position.y - pose.pose.position.y;
  double spatial_error = std::hypot(dx, dy);

  mpc_logger_->log(current_time.seconds(), static_cast<int>(phase), pose,
                   ref_pose, spatial_error, temporal_error, cmd, solve_time_ms,
                   predicted_states);
}

// ----- RESET -----
void MPCController::reset_state(bool success) {
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();
    if (success) {
      current_goal_handle_->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal reached!");
    } else {
      current_goal_handle_->abort(result);
      RCLCPP_INFO(get_logger(), "Goal aborted!");
    }
  }

  global_plan_.poses.clear();
  plan_segments_.clear();
  current_segment_idx_ = 0;
  u_prev_ = Eigen::Vector2d::Zero();
  progress_idx_ = 0;
  current_goal_handle_.reset();
  control_phase_ = ControlPhase::INACTIVE; // no path to follow anymore
  has_goal_orientation_ = false;

  // Stop the robot
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  publish_velocity_command(stop_cmd);
}

// ----- MPC HELPER FUNCTIONS -----

Eigen::Vector2d
MPCController::differential_drive_model(const Eigen::Vector4d &state,
                                        const Eigen::Vector2d &control,
                                        double /*dt*/) {
  // Differential drive kinematics with sin/cos representation:
  // dx/dt = v * cos(theta) = v * c_theta
  // dy/dt = v * sin(theta) = v * s_theta

  double s_theta = state(2); // sin(theta)
  double c_theta = state(3); // cos(theta)
  double v = control(0);

  Eigen::Vector2d state_dot;
  state_dot(0) = v * c_theta; // dx
  state_dot(1) = v * s_theta; // dy

  return state_dot;
}

void MPCController::build_mpc_matrices(
    const Eigen::Vector4d &current_state,
    const std::vector<Eigen::Vector4d> &reference_trajectory,
    const Eigen::Vector2d &u_ref, Eigen::SparseMatrix<double> &P,
    Eigen::VectorXd &q, Eigen::SparseMatrix<double> &A, Eigen::VectorXd &l,
    Eigen::VectorXd &u) {
  const int nx = 4; // state dimension [x, y, sin(theta), cos(theta)]
  const int nu = 2; // control dimension [v, omega]
  const int Np = prediction_horizon_steps_;
  const int Nc = control_horizon_steps_;

  // Augmented state: [x, y, sin(theta), cos(theta), u_v, u_omega]
  const int dim_x = nx;
  const int dim_u = nu;
  const int dim_aug = dim_x + dim_u; // 6

  // Linearize the input map around the CURRENT state orientation: v advances
  // the robot along ITS heading and omega rotates ITS orientation vector.
  // Near the reference this coincides with linearizing around the reference
  // orientation (the previous behavior, s,c ~ s_ref,c_ref), so the
  // well-tracked regime is unchanged. Far from the reference, however, the
  // reference orientation flips the sign of the orientation->position channel
  // (e.g. robot at yaw ~95 deg with cos<0 while the reference tangent has
  // cos>0): the dominant position weight then drags omega to saturation and
  // the robot pirouettes instead of driving back to the path.
  double s_ref = current_state(2); // sin(theta)
  double c_ref = current_state(3); // cos(theta)

  // ====================================================
  // Linearized dynamics with sin/cos representation
  // State matrix A_d (4x4) - Discretized using Euler forward:
  Eigen::Matrix4d A_d = Eigen::Matrix4d::Identity();
  A_d(0, 3) = u_ref(0) * d_t_; // dx depends on c_theta
  A_d(1, 2) = u_ref(0) * d_t_; // dy depends on s_theta

  // Control matrix B_d (4x2) - Maps [v, omega] to state derivatives
  Eigen::MatrixXd B_d = Eigen::MatrixXd::Zero(dim_x, dim_u);
  B_d(0, 0) = c_ref * d_t_;  // dx/dv = c_theta * dt
  B_d(1, 0) = s_ref * d_t_;  // dy/dv = s_theta * dt
  B_d(2, 1) = c_ref * d_t_;  // ds_theta/domega = c_theta * dt
  B_d(3, 1) = -s_ref * d_t_; // dc_theta/domega = -s_theta * dt

  // Augmented system matrices
  // Dynamics:
  //   x_{k+1} = A_d·x_k + B_d·u_{k-1} + B_d·Δu_k
  //   u_k = u_{k-1} + Δu_k
  Eigen::MatrixXd A_aug = Eigen::MatrixXd::Zero(dim_aug, dim_aug);
  A_aug.topLeftCorner(dim_x, dim_x) = A_d;
  A_aug.topRightCorner(dim_x, dim_u) = B_d;
  A_aug.bottomRightCorner(dim_u, dim_u) =
      Eigen::Matrix2d::Identity(); // Store u_{k-1}

  Eigen::MatrixXd B_aug = Eigen::MatrixXd::Zero(dim_aug, dim_u);
  B_aug.topLeftCorner(dim_x, dim_u) = B_d;
  B_aug.bottomLeftCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

  // Output matrix C (4x6)
  Eigen::MatrixXd C_aug = Eigen::MatrixXd::Zero(dim_x, dim_aug);
  C_aug.topLeftCorner(dim_x, dim_x) = Eigen::Matrix4d::Identity();

  // Build prediction matrices
  // S_x predicts state over Np steps
  Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * Np, dim_aug);
  // S_u maps Nc control moves to Np state predictions
  Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * Np, dim_u * Nc);

  Eigen::MatrixXd A_pow = Eigen::MatrixXd::Identity(dim_aug, dim_aug);
  for (int i = 0; i < Np; ++i) {
    A_pow = A_pow * A_aug;
    S_x.block(dim_x * i, 0, dim_x, dim_aug) = C_aug * A_pow;

    // For S_u: sum over j=0 to min(i, Nc-1)
    // The sum is \sum A^(i-j) * B * du_j
    for (int j = 0; j <= i; ++j) {
      if (j >= Nc)
        break; // Don't optimize inputs beyond control horizon

      Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(dim_aug, dim_aug);
      // Construct A^(i-j)
      for (int k = 0; k < i - j; ++k) {
        temp = temp * A_aug;
      }
      S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C_aug * temp * B_aug;
    }
  }

  // Build cost matrices
  // Q_bar: Penalizes trajectory tracking error (size Np)
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(dim_x * Np, dim_x * Np);
  for (int i = 0; i < Np; ++i) {
    Q_bar.block(dim_x * i, dim_x * i, dim_x, dim_x) = Q_;
  }

  // R_d_bar: Penalizes control rate changes (size Nc)
  Eigen::MatrixXd R_d_bar = Eigen::MatrixXd::Zero(dim_u * Nc, dim_u * Nc);
  for (int i = 0; i < Nc; ++i) {
    R_d_bar.block(dim_u * i, dim_u * i, dim_u, dim_u) = R_d_;
  }

  // Build reference vector for the entire prediction horizon
  Eigen::VectorXd x_ref_vec(dim_x * Np);
  for (int i = 0; i < Np && i < static_cast<int>(reference_trajectory.size());
       ++i) {
    x_ref_vec.segment(dim_x * i, dim_x) = reference_trajectory[i];
  }
  // If reference_trajectory is shorter than Np, repeat the last reference
  for (int i = reference_trajectory.size(); i < Np; ++i) {
    x_ref_vec.segment(dim_x * i, dim_x) = reference_trajectory.back();
  }

  // Augmented state vector (initial state)
  Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(dim_aug);
  x_aug.head(dim_x) = current_state;
  x_aug.tail(dim_u) = u_prev_; // Previous velocity command (u_{k-1})

  // P matrix (Hessian)
  // Cost: ||x_i - x_ref||²_Q + ||Δu_i||²_{R_d}
  Eigen::MatrixXd P_dense = S_u.transpose() * Q_bar * S_u + R_d_bar;
  P = P_dense.sparseView();

  // q vector (gradient)
  Eigen::VectorXd x_predicted = S_x * x_aug;
  Eigen::VectorXd error_vec = x_predicted - x_ref_vec;
  q = S_u.transpose() * Q_bar * error_vec;

  // Constraints: control limits for Nc steps
  const int n_constraints = dim_u * Nc;
  A.resize(n_constraints, dim_u * Nc);
  l.resize(n_constraints);
  u.resize(n_constraints);

  // Build constraint matrix (identity)
  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < dim_u * Nc; ++i) {
    triplets.push_back(Eigen::Triplet<double>(i, i, 1.0));
  }
  A.setFromTriplets(triplets.begin(), triplets.end());

  // Current State for Constraint Generation
  // u_prev_ stores the absolute command from the previous step.
  // Constraints are applied to delta_u:
  //      u_min <= u_prev + delta_u <= u_max
  //  =>  u_min - u_prev <= delta_u <= u_max - u_prev
  double v_current = u_prev_(0);
  double w_current = u_prev_(1);

  // Set constraint bounds for each step in the control horizon
  for (int i = 0; i < Nc; ++i) {
    // Linear velocity constraints
    double delta_v_min = std::max(-max_linear_accel_, 0.0 - v_current);
    double delta_v_max =
        std::min(max_linear_accel_, max_linear_vel_ - v_current);

    // Angular velocity constraints
    double delta_w_min =
        std::max(-max_angular_accel_, -max_angular_vel_ - w_current);
    double delta_w_max =
        std::min(max_angular_accel_, max_angular_vel_ - w_current);

    if (delta_v_min > delta_v_max)
      delta_v_min = delta_v_max;
    if (delta_w_min > delta_w_max)
      delta_w_min = delta_w_max;

    l(dim_u * i) = delta_v_min;
    u(dim_u * i) = delta_v_max;

    l(dim_u * i + 1) = delta_w_min;
    u(dim_u * i + 1) = delta_w_max;
  }
}

// ----- BOND MANAGEMENT -----
void MPCController::create_bond() {
  try {
    // Create bond with the topic name that Nav2 lifecycle manager expects
    // Nav2 expects the bond to be on topic "bond" with ID matching the node
    // name
    bond_ = std::make_unique<bond::Bond>(
        std::string("bond"), // topic namespace - Nav2 default
        bond_id_,            // bond ID should be the node name
        shared_from_this(),  // lifecycle node shared pointer
        [this]() {
          RCLCPP_INFO(get_logger(), "Bond broken callback triggered");
          bond_timeout_callback();
        }, // broken callback
        [this]() {
          RCLCPP_INFO(get_logger(), "Bond formed successfully");
        } // formed callback
    );

    bond_->setHeartbeatPeriod(1.0);
    // Config large connection timeout
    bond_->setConnectTimeout(10.0);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to create bond: %s", e.what());
  }
}

void MPCController::destroy_bond() {
  if (bond_) {
    bond_.reset();
    RCLCPP_INFO(get_logger(), "Bond destroyed");
  }
}

void MPCController::bond_timeout_callback() {
  RCLCPP_ERROR(
      get_logger(),
      "Bond connection broken! Communication with lifecycle manager lost.");
  bond_timeout_detected_ = true;

  // Stop the robot for safety
  if (initialized_) {
    RCLCPP_WARN(get_logger(), "Stopping robot due to bond failure");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    publish_velocity_command(stop_cmd);

    // Cancel current goal if active
    reset_state(false);
  }
}

} // namespace mpc_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mpc_controller::MPCController)