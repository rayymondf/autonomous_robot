#include <chrono>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
  : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.updateMap(msg);
  replan();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  planner_.updateGoal(msg);
  replan();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  planner_.updatePose(msg);
}

void PlannerNode::timerCallback() {
  if (!planner_.checkGoalReached()) return;
  // publish empty path so the control node stops
  nav_msgs::msg::Path empty;
  empty.header.frame_id = "sim_world";
  empty.header.stamp = this->now();
  path_pub_->publish(empty);
}

void PlannerNode::replan() {
  auto path = planner_.planPath();
  if (path.has_value()) {
    path_pub_->publish(path.value());
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
