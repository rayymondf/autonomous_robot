#include <chrono>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  has_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = *msg;
  has_odom_ = true;
}

void MapMemoryNode::timerCallback() {
  if (has_costmap_ && has_odom_) {
    map_memory_.updateMap(latest_costmap_, latest_odom_);
  }
  // always publish so the planner has an initial map before robot moves
  map_pub_->publish(map_memory_.getMap());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
