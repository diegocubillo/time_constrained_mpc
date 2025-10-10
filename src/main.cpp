#include "time_constrained_mpc/time_constrained_mpc.hpp"
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<mpc_controller::MPCController>();
  
  rclcpp::spin(node->get_node_base_interface());
  
  rclcpp::shutdown();
  return 0;
}
