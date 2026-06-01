#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>
#include <utility>

namespace robot {

class CostmapCore {
public:
  explicit CostmapCore(const rclcpp::Logger& logger);

  nav_msgs::msg::OccupancyGrid processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
  rclcpp::Logger logger_;

  nav_msgs::msg::OccupancyGrid costmap_;
  std::vector<std::pair<int, int>> obstacles_;

  static constexpr double resolution_ = 0.1;
  static constexpr int width_ = 200;
  static constexpr int height_ = 200;
  static constexpr double origin_x_ = -10.0;
  static constexpr double origin_y_ = -10.0;
  static constexpr double inflation_radius_ = 0.9;

  void initializeCostmap();
  void convertToGrid(double range, double angle, int& grid_x, int& grid_y);
  void markObstacle(int grid_x, int grid_y);
  void inflateObstacles();
};

} // namespace robot

#endif
