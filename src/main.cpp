#include <rclcpp/rclcpp.hpp>
#include "cart_detect_sim/cart_detect_sim_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cart_detect_sim::CartDetectSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
