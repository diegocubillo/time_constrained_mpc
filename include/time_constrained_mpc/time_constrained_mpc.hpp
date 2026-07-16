#pragma once

#include "bondcpp/bond.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "osqp/osqp.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "time_constrained_mpc/mpc_logger.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <string>
#include <vector>

using GoalHandleFollowPath =
    rclcpp_action::ServerGoalHandle<nav2_msgs::action::FollowPath>;

namespace mpc_controller {

class MPCController : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit MPCController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MPCController() override;

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state) override;

  // Action handling (FollowPath)
  void handle_goal(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void handle_cancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  // Main control loop (timer callback)
  void control_loop();

  // Robot state acquisition
  geometry_msgs::msg::PoseStamped get_robot_pose();

  // Path processing
  nav_msgs::msg::Path interpolate_path(const nav_msgs::msg::Path &original_path,
                                       double target_spacing = 0.1);
  nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path &original_path,
                                  double smoothing_window = 0.5);
  nav_msgs::msg::Path
  resample_and_retime_path(const nav_msgs::msg::Path &interpolated_path,
                           const nav_msgs::msg::Path &smoothed_path,
                           double spacing = 0.1);

  // Temporal reference calculation
  geometry_msgs::msg::PoseStamped
  get_temporal_reference(const rclcpp::Time &target_time);
  std::vector<Eigen::Vector4d>
  get_reference_trajectory_horizon(const rclcpp::Time &current_time, int N,
                                   double dt);
  // Terminal reference for the GOAL_APPROACH phase: a world-anchored point that
  // advances along the goal tangent at the approach speed (parameterised by wall
  // time from phase entry) and clamps beyond the goal by goal_approach_extension.
  std::vector<Eigen::Vector4d>
  get_approach_reference_horizon(const rclcpp::Time &current_time);
  // Yaw of the path's direction of travel into the goal, taken from the last
  // distinct path segment (NOT the goal orientation, which may differ).
  double goal_tangent_yaw();
  double calculate_temporal_error(geometry_msgs::msg::PoseStamped current_pose,
                                  rclcpp::Time current_time);

  // MPC solver
  geometry_msgs::msg::Twist
  solve_mpc(const geometry_msgs::msg::PoseStamped &pose,
            const std::vector<Eigen::Vector4d> &reference_trajectory,
            double &solve_time_ms);

  // Command publication
  void publish_velocity_command(const geometry_msgs::msg::Twist &cmd);

  // Path publication for debugging
  void publish_debug_path();

  // Action feedback
  void update_feedback(const geometry_msgs::msg::PoseStamped &pose,
                       double temporal_error);

  // Reset internal state and finish the current goal (success or abort).
  void reset_state(bool success);

  // Bond management
  void create_bond();
  void destroy_bond();
  void bond_timeout_callback();

private:
  // Control phase state machine.
  // The numeric values are part of the MPC log format (column "state", written
  // by MPCLogger): INACTIVE 0, INITIAL_ROTATION 1, PATH_FOLLOWING 2,
  // GOAL_APPROACH 3, FINAL_ROTATION 4. Keep them stable so the analysis tools
  // stay consistent.
  enum class ControlPhase {
    INACTIVE = 0,         // Not following any path (idle / mission complete)
    INITIAL_ROTATION = 1, // Rotate in place to align with path start
    PATH_FOLLOWING = 2,   // MPC tracks the path (position + time only)
    GOAL_APPROACH = 3, // MPC closes the last centimetres with a slow terminal
                       // reference; finishes on goal-plane crossing
    FINAL_ROTATION = 4 // Rotate in place to align with goal orientation
  };

  // Helper methods for MPC
  Eigen::Vector2d differential_drive_model(const Eigen::Vector4d &state,
                                           const Eigen::Vector2d &control,
                                           double dt);
  void
  build_mpc_matrices(const Eigen::Vector4d &current_state,
                     const std::vector<Eigen::Vector4d> &reference_trajectory,
                     const Eigen::Vector2d &u_ref,
                     Eigen::SparseMatrix<double> &P, Eigen::VectorXd &q,
                     Eigen::SparseMatrix<double> &A, Eigen::VectorXd &l,
                     Eigen::VectorXd &u);

  // Write one log row for the current control step: the active phase (numeric
  // state), the reference the controller is tracking and the commanded
  // velocity. Called from every active phase so the log captures the full
  // timeline (INITIAL_ROTATION / PATH_FOLLOWING / FINAL_ROTATION), not just
  // path following.
  void log_control_step(const geometry_msgs::msg::PoseStamped &pose,
                        const geometry_msgs::msg::Twist &cmd,
                        const rclcpp::Time &current_time, ControlPhase phase,
                        double solve_time_ms, double temporal_error,
                        const std::vector<Eigen::Vector4d> &predicted_states);

  // ROS2 components
  std::shared_ptr<rclcpp::TimerBase> timer_path_pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_control_loop_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      predicted_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::PoseStamped>::SharedPtr debug_pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<nav2_msgs::action::FollowPath>::SharedPtr
      action_server_;
  std::shared_ptr<GoalHandleFollowPath> current_goal_handle_;

  // Bond for heartbeat monitoring
  std::unique_ptr<bond::Bond> bond_;
  std::string bond_id_;
  bool bond_timeout_detected_{false};

  // State variables
  nav_msgs::msg::Path global_plan_;
  rclcpp::Time path_start_time_; // Time when path execution started
  // World anchor for the terminal approach reference, captured when the
  // GOAL_APPROACH phase begins: the wall time at entry and the robot's
  // along-track offset from the goal at that instant. The terminal reference is
  // parameterised from these (fixed in the world, not re-anchored to the robot
  // each cycle), so it keeps advancing at the approach speed and the tracker
  // holds that speed instead of settling below it.
  rclcpp::Time goal_approach_start_time_;
  double goal_approach_start_along_{0.0};
  bool initialized_{false};
  ControlPhase control_phase_{ControlPhase::INACTIVE};
  bool has_goal_orientation_{
      false}; // Whether the goal has an explicit orientation
  geometry_msgs::msg::Quaternion goal_orientation_; // Stored goal orientation
  Eigen::Vector2d u_prev_{Eigen::Vector2d::Zero()};

  // Monotonic along-path progress index used by calculate_temporal_error() to
  // locate where the robot actually is on the plan. It only ever advances (via a
  // bounded forward-window argmin), so paths that cross themselves cannot make
  // the schedule-lag estimate jump to a past branch (spurious timeout) or leap
  // onto a future branch (overstated progress). Reset to 0 on every new goal.
  size_t progress_idx_{0};

  // MPC parameters
  double max_linear_vel_;
  double max_angular_vel_;
  double max_linear_accel_;  // Max linear accel (m/s² from config, converted to
                             // m/s per timestep in on_configure)
  double max_angular_accel_; // Max angular accel (rad/s² from config, converted
                             // to rad/s per timestep in on_configure)
  int prediction_horizon_steps_;
  int control_horizon_steps_;
  double d_t_; // Control time step

  double goal_dist_tolerance_;
  double goal_theta_tolerance_;
  double
      goal_approach_radius_; // Distance to goal at which PATH_FOLLOWING hands
                             // over to the GOAL_APPROACH terminal phase
  double goal_approach_vel_; // Capped linear speed during GOAL_APPROACH (m/s)
  double goal_approach_extension_; // Distance the terminal reference aims BEYOND
                                   // the goal along the tangent (carrot). Larger
                                   // => less anticipatory braking; the robot
                                   // crosses the goal at higher speed. A negative
                                   // value is resolved in on_configure to
                                   // N*v_app*dt (the no-braking threshold).
  double max_spatial_error_;
  double max_temporal_error_;
  double progress_search_window_; // Forward look-ahead (m) for the monotonic
                                  // progress-index argmin in
                                  // calculate_temporal_error()
  bool use_stamped_cmd_vel_;     // Use TwistStamped (true) or Twist (false)
  double path_smoothing_window_; // Smoothing window in meters
  bool debug_mpc_;               // Enable MPC debugging output
  bool use_bond_;                // Enable bond usage

  // MPC weight matrices
  Eigen::Matrix4d Q_;   // State error weight [x, y, s_theta, c_theta]
  Eigen::Matrix2d R_d_; // Control rate weight

  // Frame IDs
  std::string map_frame_;
  std::string base_frame_;

  // Logger
  std::unique_ptr<MPCLogger> mpc_logger_;
  std::vector<Eigen::Vector4d> last_predicted_states_;
};

} // namespace mpc_controller