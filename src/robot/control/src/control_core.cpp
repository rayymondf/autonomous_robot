#include "control_core.hpp"
#include <cmath>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const nav_msgs::msg::Path& path,
  const nav_msgs::msg::Odometry& odom)
{
  geometry_msgs::msg::Twist cmd;
  if (path.poses.empty()) return cmd;

  double rx = odom.pose.pose.position.x;
  double ry = odom.pose.pose.position.y;
  const auto& q = odom.pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // stop if within tolerance of final goal
  const auto& goal_pos = path.poses.back().pose.position;
  double dist_to_goal = std::sqrt(
    (rx - goal_pos.x) * (rx - goal_pos.x) + (ry - goal_pos.y) * (ry - goal_pos.y));
  if (dist_to_goal < GOAL_TOLERANCE) return cmd;

  // find first path point at or beyond the lookahead distance
  size_t lookahead_idx = path.poses.size() - 1;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - rx;
    double dy = path.poses[i].pose.position.y - ry;
    if (std::sqrt(dx * dx + dy * dy) >= LOOKAHEAD_DISTANCE) {
      lookahead_idx = i;
      break;
    }
  }

  double lx = path.poses[lookahead_idx].pose.position.x;
  double ly = path.poses[lookahead_idx].pose.position.y;

  // transform lookahead point into robot frame
  double dx = lx - rx;
  double dy = ly - ry;
  double local_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
  double local_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);

  double ld_sq = local_x * local_x + local_y * local_y;
  if (ld_sq < 1e-6) return cmd;

  // pure pursuit: kappa = 2 * lateral_error / lookahead^2
  double curvature = 2.0 * local_y / ld_sq;

  cmd.linear.x = LINEAR_SPEED;
  cmd.angular.z = LINEAR_SPEED * curvature;

  return cmd;
}

} // namespace robot
