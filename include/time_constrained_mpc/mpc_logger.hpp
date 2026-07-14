#ifndef TIME_CONSTRAINED_MPC__MPC_LOGGER_HPP_
#define TIME_CONSTRAINED_MPC__MPC_LOGGER_HPP_

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <string>
#include <tf2/utils.h>
#include <vector>

namespace mpc_controller {

class MPCLogger {
public:
  MPCLogger() = default;
  ~MPCLogger() { close(); }

  bool open(const std::string &filename) {
    close();

    // Ensure directory exists
    std::filesystem::path path(filename);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }

    file_.open(filename);
    if (!file_.is_open()) {
      return false;
    }

    // Write header.
    // "state" is the numeric ControlPhase (INACTIVE 0, INITIAL_ROTATION 1,
    // PATH_FOLLOWING 2, GOAL_APPROACH 3, FINAL_ROTATION 4). error_spatial is the
    // distance to the time-based reference; error_temporal is the schedule lag
    // from calculate_temporal_error (positive = ahead of schedule, negative =
    // behind), measured against a monotonic along-path progress index so it is
    // robust to self-crossing paths.
    file_ << "timestamp,state,"
          << "ref_x,ref_y,ref_theta,"
          << "robot_x,robot_y,robot_theta,"
          << "error_spatial,error_temporal,"
          << "cmd_v,cmd_w,"
          << "solve_time_ms";

    // Add columns for predicted states (sin/cos)
    // We don't know N yet, but we can just write a generic header or assume a
    // max N Better: The user of this class should ensure consistency. For the
    // header, we'll just add a prefix note or handle it dynamically if we want
    // to be strict. Let's just add a flexible number of columns for prediction.
    // Actually, let's just write "pred_sin_0,pred_cos_0,..." for a reasonable
    // max N or just append them. For simplicity in this helper, we'll assume
    // the caller handles the header or we just dump the vector. Let's make the
    // header dynamic based on the first log call? No, header must be first.
    // Let's assume a max N of 50 for the header, or just write a fixed number
    // if N is constant. Since N is a parameter, we can pass it to open().

    return true;
  }

  void write_header(int horizon_steps) {
    if (!file_.is_open())
      return;

    horizon_steps_ = horizon_steps;
    for (int i = 0; i < horizon_steps; ++i) {
      file_ << ",pred_sin_" << i << ",pred_cos_" << i;
    }
    file_ << "\n";
  }

  void log(double timestamp, int control_state,
           const geometry_msgs::msg::PoseStamped &current_pose,
           const geometry_msgs::msg::PoseStamped &ref_pose,
           double error_spatial, double error_temporal,
           const geometry_msgs::msg::Twist &cmd, double solve_time_ms,
           const std::vector<Eigen::Vector4d> &predicted_states) {
    if (!file_.is_open())
      return;

    double current_theta = tf2::getYaw(current_pose.pose.orientation);
    double ref_theta = tf2::getYaw(ref_pose.pose.orientation);

    // control_state is streamed as an int (unaffected by std::fixed), so it is
    // written as e.g. "2" rather than "2.000000".
    file_ << std::fixed << std::setprecision(6) << timestamp << ","
          << control_state << "," << ref_pose.pose.position.x << ","
          << ref_pose.pose.position.y << "," << ref_theta << ","
          << current_pose.pose.position.x << "," << current_pose.pose.position.y
          << "," << current_theta << "," << error_spatial << ","
          << error_temporal << "," << cmd.linear.x << "," << cmd.angular.z
          << "," << solve_time_ms;

    // Always write exactly horizon_steps_ prediction pairs so every row has the
    // same number of columns as the header, even when there is no MPC solution
    // for this step (rotation phases pass an empty prediction vector).
    for (int i = 0; i < horizon_steps_; ++i) {
      if (i < static_cast<int>(predicted_states.size())) {
        // state is [x, y, sin, cos]
        file_ << "," << predicted_states[i](2) << "," << predicted_states[i](3);
      } else {
        file_ << ",0,0";
      }
    }

    file_ << "\n";
  }

  void close() {
    if (file_.is_open()) {
      file_.close();
    }
  }

private:
  std::ofstream file_;
  int horizon_steps_{
      0}; // number of prediction pairs per row (set in write_header)
};

} // namespace mpc_controller

#endif // TIME_CONSTRAINED_MPC__MPC_LOGGER_HPP_
