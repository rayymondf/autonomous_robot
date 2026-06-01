#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

void CostmapNode::publishMessage() {
  auto msg = std_msgs::msg::String();
  msg.data = "Hello, ROS 2!";
  string_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  auto grid = costmap_.processScan(msg);
  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
