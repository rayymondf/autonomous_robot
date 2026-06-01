#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot {

class ControlCore {
public:
  explicit ControlCore(const rclcpp::Logger& logger);

  geometry_msgs::msg::Twist computeVelocity(
    const nav_msgs::msg::Path& path,
    const nav_msgs::msg::Odometry& odom);

private:
  rclcpp::Logger logger_;

  static constexpr double LOOKAHEAD_DISTANCE = 1.0;
  static constexpr double GOAL_TOLERANCE = 0.1;
  static constexpr double LINEAR_SPEED = 0.5;
};

} // namespace robot

#endif
