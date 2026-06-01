#include "map_memory_core.hpp"
#include <cmath>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {
  initializeMap();
}

void MapMemoryCore::initializeMap() {
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = RESOLUTION;
  global_map_.info.width = MAP_SIZE;
  global_map_.info.height = MAP_SIZE;
  global_map_.info.origin.position.x = -(MAP_SIZE * RESOLUTION) / 2.0;
  global_map_.info.origin.position.y = -(MAP_SIZE * RESOLUTION) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  // -1 = unknown
  global_map_.data.assign(MAP_SIZE * MAP_SIZE, -1);
}

void MapMemoryCore::updateMap(
  const nav_msgs::msg::OccupancyGrid& costmap,
  const nav_msgs::msg::Odometry& odom)
{
  double rx = odom.pose.pose.position.x;
  double ry = odom.pose.pose.position.y;

  // only merge when robot has moved past the threshold (always merge on first call)
  if (last_odom_.has_value()) {
    double lx = last_odom_->pose.pose.position.x;
    double ly = last_odom_->pose.pose.position.y;
    double dist = std::sqrt((rx - lx) * (rx - lx) + (ry - ly) * (ry - ly));
    if (dist < MOVEMENT_THRESHOLD) return;
  }

  last_odom_ = odom;
  global_map_.header.stamp = odom.header.stamp;

  // extract yaw from quaternion
  const auto& q = odom.pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  int cw = static_cast<int>(costmap.info.width);
  int ch = static_cast<int>(costmap.info.height);

  for (int ly = 0; ly < ch; ++ly) {
    for (int lx = 0; lx < cw; ++lx) {
      int8_t val = costmap.data[ly * cw + lx];
      if (val <= 0) continue;

      // local position in lidar frame (costmap is centered at lidar)
      double local_x = costmap.info.origin.position.x + (lx + 0.5) * costmap.info.resolution;
      double local_y = costmap.info.origin.position.y + (ly + 0.5) * costmap.info.resolution;

      // rotate into world frame and translate by robot position
      double world_x = rx + local_x * std::cos(yaw) - local_y * std::sin(yaw);
      double world_y = ry + local_x * std::sin(yaw) + local_y * std::cos(yaw);

      int gx = static_cast<int>((world_x - global_map_.info.origin.position.x) / RESOLUTION);
      int gy = static_cast<int>((world_y - global_map_.info.origin.position.y) / RESOLUTION);

      if (gx >= 0 && gx < MAP_SIZE && gy >= 0 && gy < MAP_SIZE) {
        int idx = gy * MAP_SIZE + gx;
        // new sensor data takes priority; keep highest cost seen
        global_map_.data[idx] = std::max(global_map_.data[idx], val);
      }
    }
  }
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getMap() const {
  return global_map_;
}

} // namespace robot
