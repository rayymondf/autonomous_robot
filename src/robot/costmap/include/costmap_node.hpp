#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  robot::CostmapCore costmap_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishMessage();
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif
