#include "cart_detect_sim/cart_detect_sim_node.h"

#include <cmath>
#include <algorithm>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace cart_detect_sim
{

CartDetectSimNode::CartDetectSimNode(const rclcpp::NodeOptions & options)
: Node("cart_detect_sim", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  max_detection_distance_ = declare_parameter<double>("max_detection_distance", 5.0);
  map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");

  // Declare initial_carts as a list of cart parameter names
  // e.g. initial_carts: ["cart_30", "cart_31"]
  // cart_30.id: 30, cart_30.type: 1, cart_30.x: 5.0, cart_30.y: 2.0, cart_30.yaw: 0.0
  declare_parameter<std::vector<std::string>>("initial_carts", std::vector<std::string>{});

  // --- Services ---

  // detect — same name as production cart_detect
  srv_detect_ = create_service<cart_detect::srv::GetCarts>(
    "detect",
    std::bind(&CartDetectSimNode::onDetect, this,
              std::placeholders::_1, std::placeholders::_2));

  // cartdetect/enable_detection — matches commander's client name
  srv_enable_detection_ = create_service<std_srvs::srv::SetBool>(
    "cartdetect/enable_detection",
    std::bind(&CartDetectSimNode::onEnableDetection, this,
              std::placeholders::_1, std::placeholders::_2));

  // sim management services
  srv_set_cart_ = create_service<cart_detect_sim::srv::SetCart>(
    "sim/set_cart",
    std::bind(&CartDetectSimNode::onSetCart, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_remove_cart_ = create_service<cart_detect_sim::srv::RemoveCart>(
    "sim/remove_cart",
    std::bind(&CartDetectSimNode::onRemoveCart, this,
              std::placeholders::_1, std::placeholders::_2));

  loadInitialCarts();

  RCLCPP_INFO(get_logger(),
    "cart_detect_sim ready: %zu carts loaded, detection=%s, max_distance=%.1fm",
    carts_.size(), detection_enabled_ ? "ON" : "OFF", max_detection_distance_);
}

void CartDetectSimNode::loadInitialCarts()
{
  std::vector<std::string> cart_names;
  get_parameter("initial_carts", cart_names);

  for (const auto & name : cart_names) {
    SimCart cart;
    cart.id   = declare_parameter<int>(name + ".id", 0);
    cart.type = static_cast<uint8_t>(declare_parameter<int>(name + ".type", 1));
    cart.x    = declare_parameter<double>(name + ".x", 0.0);
    cart.y    = declare_parameter<double>(name + ".y", 0.0);
    cart.yaw  = declare_parameter<double>(name + ".yaw", 0.0);

    carts_[cart.id] = cart;
    RCLCPP_INFO(get_logger(),
      "  loaded cart id=%d type=%d pos=(%.2f, %.2f, %.2f rad)",
      cart.id, cart.type, cart.x, cart.y, cart.yaw);
  }
}

void CartDetectSimNode::onDetect(
  const cart_detect::srv::GetCarts::Request::SharedPtr req,
  cart_detect::srv::GetCarts::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (!detection_enabled_) {
    RCLCPP_DEBUG(get_logger(), "[detect] detection disabled, returning empty");
    return;
  }

  if (carts_.empty()) {
    RCLCPP_DEBUG(get_logger(), "[detect] no carts registered");
    return;
  }

  // Get robot position in map frame via TF
  const std::string & source_frame = req->source_frame;
  geometry_msgs::msg::TransformStamped robot_tf;
  bool have_robot_tf = false;

  try {
    robot_tf = tf_buffer_.lookupTransform(
      map_frame_id_, source_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.5));
    have_robot_tf = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(),
      "[detect] cannot look up %s -> %s: %s",
      map_frame_id_.c_str(), source_frame.c_str(), ex.what());
  }

  struct CartWithDistance {
    cart_detect::msg::CartTransform ct;
    double distance;
  };
  std::vector<CartWithDistance> results;

  for (const auto & [id, cart] : carts_) {
    // Filter by ID range (same as production: 30-99)
    if (id < cart_detect::srv::GetCarts::Request::CART_ID_MIN ||
        id > cart_detect::srv::GetCarts::Request::CART_ID_MAX) {
      continue;
    }

    // Compute relative transform: source_frame -> cart
    double dx = cart.x;
    double dy = cart.y;
    double dist = 0.0;

    if (have_robot_tf) {
      const double rx = robot_tf.transform.translation.x;
      const double ry = robot_tf.transform.translation.y;
      dx = cart.x - rx;
      dy = cart.y - ry;
      dist = std::sqrt(dx * dx + dy * dy);

      // Distance filter
      if (dist > max_detection_distance_) {
        continue;
      }

      // Transform delta into source_frame coordinates
      // Extract robot yaw from quaternion
      const auto & q = robot_tf.transform.rotation;
      const double siny = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const double robot_yaw = std::atan2(siny, cosy);

      const double cos_r = std::cos(-robot_yaw);
      const double sin_r = std::sin(-robot_yaw);
      const double local_x = dx * cos_r - dy * sin_r;
      const double local_y = dx * sin_r + dy * cos_r;
      dx = local_x;
      dy = local_y;
    }

    cart_detect::msg::CartTransform ct;
    ct.id = id;
    ct.type = cart.type;
    ct.transform.translation.x = dx;
    ct.transform.translation.y = dy;
    ct.transform.translation.z = 0.0;

    // Cart orientation relative to robot
    double rel_yaw = cart.yaw;
    if (have_robot_tf) {
      const auto & q = robot_tf.transform.rotation;
      const double siny = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      rel_yaw = cart.yaw - std::atan2(siny, cosy);
    }
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, rel_yaw);
    ct.transform.rotation.x = quat.x();
    ct.transform.rotation.y = quat.y();
    ct.transform.rotation.z = quat.z();
    ct.transform.rotation.w = quat.w();

    results.push_back({ct, dist});
  }

  // Sort by distance (closest first)
  std::sort(results.begin(), results.end(),
    [](const CartWithDistance & a, const CartWithDistance & b) {
      return a.distance < b.distance;
    });

  for (const auto & r : results) {
    res->transforms.push_back(r.ct);
  }

  RCLCPP_INFO(get_logger(),
    "[detect] source_frame=%s, returning %zu cart(s)",
    source_frame.c_str(), res->transforms.size());
}

void CartDetectSimNode::onEnableDetection(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  detection_enabled_ = req->data;
  res->success = true;
  res->message = detection_enabled_ ? "Detection enabled" : "Detection disabled";
  RCLCPP_INFO(get_logger(), "[enable_detection] %s", res->message.c_str());
}

void CartDetectSimNode::onSetCart(
  const cart_detect_sim::srv::SetCart::Request::SharedPtr req,
  cart_detect_sim::srv::SetCart::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(mtx_);

  SimCart cart;
  cart.id   = req->id;
  cart.type = req->type;
  cart.x    = req->x;
  cart.y    = req->y;
  cart.yaw  = req->yaw;

  const bool exists = carts_.count(req->id) > 0;
  carts_[req->id] = cart;
  res->success = true;

  RCLCPP_INFO(get_logger(),
    "[set_cart] %s cart id=%d type=%d pos=(%.2f, %.2f, %.2f rad)",
    exists ? "updated" : "added", cart.id, cart.type, cart.x, cart.y, cart.yaw);
}

void CartDetectSimNode::onRemoveCart(
  const cart_detect_sim::srv::RemoveCart::Request::SharedPtr req,
  cart_detect_sim::srv::RemoveCart::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(mtx_);

  auto it = carts_.find(req->id);
  if (it != carts_.end()) {
    carts_.erase(it);
    res->success = true;
    RCLCPP_INFO(get_logger(), "[remove_cart] removed cart id=%d", req->id);
  } else {
    res->success = false;
    RCLCPP_WARN(get_logger(), "[remove_cart] cart id=%d not found", req->id);
  }
}

}  // namespace cart_detect_sim
