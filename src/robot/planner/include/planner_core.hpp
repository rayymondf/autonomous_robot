#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <vector>
#include <unordered_map>
#include <queue>
#include <optional>

namespace robot {

struct CellIndex {
  int x, y;
  CellIndex() : x(0), y(0) {}
  CellIndex(int x, int y) : x(x), y(y) {}
  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

struct CellIndexHash {
  size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 16);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(const CellIndex& idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger& logger);

  void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updateGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void updatePose(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool checkGoalReached();
  std::optional<nav_msgs::msg::Path> planPath();

private:
  rclcpp::Logger logger_;

  enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
  State state_;

  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;
  bool goal_received_{false};

  double check_bound_ratio_;
  int cost_threshold_;

  double heuristic(const CellIndex& a, const CellIndex& b);
  std::vector<CellIndex> getNeighbors(const CellIndex& current);
  bool isValid(const CellIndex& idx, double yaw);
  CellIndex worldToGrid(double wx, double wy);
  void gridToWorld(const CellIndex& idx, double& wx, double& wy);
};

} // namespace robot

#endif
