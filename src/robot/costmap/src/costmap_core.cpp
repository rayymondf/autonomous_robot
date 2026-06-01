#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger)
{
  // initialize map metadata
  costmap_.header.frame_id = "lidar_link";
  costmap_.info.resolution = resolution_;
  costmap_.info.width = width_;
  costmap_.info.height = height_;
  costmap_.info.origin.position.x = origin_x_;
  costmap_.info.origin.position.y = origin_y_;
  costmap_.info.origin.position.z = 0.0;
  costmap_.info.origin.orientation.w = 1.0;
  costmap_.data.resize(width_ * height_, 0);
}

void CostmapCore::initializeCostmap()
{
  std::fill(costmap_.data.begin(), costmap_.data.end(), 0);
  obstacles_.clear();
}

void CostmapCore::convertToGrid(double range, double angle, int& grid_x, int& grid_y)
{
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  grid_x = static_cast<int>((x - origin_x_) / resolution_);
  grid_y = static_cast<int>((y - origin_y_) / resolution_);
}

void CostmapCore::markObstacle(int grid_x, int grid_y)
{
  if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
    int index = grid_y * width_ + grid_x;
    costmap_.data[index] = 100;
    obstacles_.push_back({grid_x, grid_y});
  }
}

void CostmapCore::inflateObstacles()
{
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

  for (const auto& obs : obstacles_) {
    int ox = obs.first;
    int oy = obs.second;
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        int nx = ox + dx;
        int ny = oy + dy;
        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
          double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
          if (dist <= inflation_radius_) {
            int cost = static_cast<int>(100.0 * (1.0 - (dist / inflation_radius_)));
            int index = ny * width_ + nx;
            // keep highest cost value
            costmap_.data[index] = std::max(static_cast<int>(costmap_.data[index]), cost);
          }
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::processScan(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // reset map
  costmap_.header.stamp = scan->header.stamp;
  costmap_.header.frame_id = scan->header.frame_id;
  initializeCostmap();

  // mark obstacles from scan
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (std::isinf(range) || std::isnan(range) ||
        range < scan->range_min || range > scan->range_max) {
      continue;
    }
    double angle = scan->angle_min + i * scan->angle_increment;
    int grid_x, grid_y;
    convertToGrid(range, angle, grid_x, grid_y);
    markObstacle(grid_x, grid_y);
  }

  // apply inflation
  inflateObstacles();

  return costmap_;
}

} // namespace robot
