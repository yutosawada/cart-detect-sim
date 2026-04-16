#pragma once

#include <map>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <cart_detect/srv/get_carts.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cart_detect_sim/srv/set_cart.hpp"
#include "cart_detect_sim/srv/remove_cart.hpp"
#include <sim_world_msgs/msg/sim_cart_array.hpp>

namespace cart_detect_sim
{

struct SimCart
{
  int32_t id;
  uint8_t type;
  double x;
  double y;
  double yaw;
};

class CartDetectSimNode : public rclcpp::Node
{
public:
  explicit CartDetectSimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- detect service (cart_detect::srv::GetCarts) ---
  void onDetect(
    const cart_detect::srv::GetCarts::Request::SharedPtr req,
    cart_detect::srv::GetCarts::Response::SharedPtr res);

  // --- cartdetect/enable_detection service (SetBool) ---
  void onEnableDetection(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  // --- sim/set_cart service ---
  void onSetCart(
    const cart_detect_sim::srv::SetCart::Request::SharedPtr req,
    cart_detect_sim::srv::SetCart::Response::SharedPtr res);

  // --- sim/remove_cart service ---
  void onRemoveCart(
    const cart_detect_sim::srv::RemoveCart::Request::SharedPtr req,
    cart_detect_sim::srv::RemoveCart::Response::SharedPtr res);

  void loadInitialCarts();

  // --- sim_world integration (optional) ---
  void onSimWorldCarts(const sim_world_msgs::msg::SimCartArray::SharedPtr msg);

  rclcpp::Service<cart_detect::srv::GetCarts>::SharedPtr srv_detect_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_detection_;
  rclcpp::Service<cart_detect_sim::srv::SetCart>::SharedPtr srv_set_cart_;
  rclcpp::Service<cart_detect_sim::srv::RemoveCart>::SharedPtr srv_remove_cart_;
  rclcpp::Subscription<sim_world_msgs::msg::SimCartArray>::SharedPtr sub_sim_world_carts_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mtx_;
  std::map<int32_t, SimCart> carts_;
  bool detection_enabled_{false};
  double max_detection_distance_;
  std::string map_frame_id_;
  bool use_sim_world_{false};
};

}  // namespace cart_detect_sim
