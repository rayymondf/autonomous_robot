#include <chrono>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
  : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  latest_path_ = *msg;
  has_path_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = *msg;
  has_odom_ = true;
}

void ControlNode::timerCallback() {
  if (!has_path_ || !has_odom_) return;
  auto cmd = control_.computeVelocity(latest_path_, latest_odom_);
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
