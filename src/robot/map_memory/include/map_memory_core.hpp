#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <optional>

namespace robot {

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger& logger);

  void updateMap(
    const nav_msgs::msg::OccupancyGrid& costmap,
    const nav_msgs::msg::Odometry& odom);

  const nav_msgs::msg::OccupancyGrid& getMap() const;

private:
  rclcpp::Logger logger_;
  nav_msgs::msg::OccupancyGrid global_map_;
  std::optional<nav_msgs::msg::Odometry> last_odom_;

  static constexpr double RESOLUTION = 0.1;
  static constexpr int MAP_SIZE = 300;
  static constexpr double MOVEMENT_THRESHOLD = 1.5;

  void initializeMap();
};

} // namespace robot

#endif
