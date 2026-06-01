#include "planner_core.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : logger_(logger), state_(State::WAITING_FOR_GOAL) {}

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
}

void PlannerCore::updateGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

void PlannerCore::updatePose(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

bool PlannerCore::checkGoalReached()
{
  if (!goal_received_) return false;
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  // goal tolerance; reset so timer fires only once
  if (std::sqrt(dx * dx + dy * dy) < 0.5) {
    goal_received_ = false;
    return true;
  }
  return false;
}

std::optional<nav_msgs::msg::Path> PlannerCore::planPath()
{
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan path: Missing map or goal!");
    return std::nullopt;
  }

  // initialize path message
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = current_map_.header.frame_id;

  // convert world to grid
  CellIndex start_idx = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex goal_idx = worldToGrid(goal_.point.x, goal_.point.y);

  // calculate start yaw
  auto& q = robot_pose_.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double current_yaw = std::atan2(siny_cosp, cosy_cosp);

  // a* initialization
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  bool found_path = false;
  check_bound_ratio_ = 1.0;
  cost_threshold_ = 10;

  // search with adaptive thresholds, retry up to limit
  int max_retries = 5;
  int retry_count = 0;

  while (!found_path && retry_count < max_retries) {
    // validate start position; fall back to center cell check if footprint fails
    bool start_valid = isValid(start_idx, current_yaw);
    if (!start_valid) {
      int idx = start_idx.y * current_map_.info.width + start_idx.x;
      if (idx >= 0 && idx < static_cast<int>(current_map_.data.size())) {
        if (current_map_.data[idx] < cost_threshold_) {
          start_valid = true;
        }
      }
    }

    if (!start_valid) {
      cost_threshold_ += 10;
      retry_count++;
      continue;
    }

    // check goal cell directly — footprint orientation at goal is unknown
    {
      bool goal_in_bounds = goal_idx.x >= 0 &&
                            goal_idx.x < static_cast<int>(current_map_.info.width) &&
                            goal_idx.y >= 0 &&
                            goal_idx.y < static_cast<int>(current_map_.info.height);
      bool goal_passable = goal_in_bounds &&
                           current_map_.data[goal_idx.y * current_map_.info.width + goal_idx.x] < cost_threshold_;
      if (!goal_passable) {
        cost_threshold_ += 10;
        retry_count++;
        continue;
      }
    }

    // clear search data
    while (!open_set.empty()) open_set.pop();
    g_score.clear();
    came_from.clear();
    closed_set.clear();

    g_score[start_idx] = 0.0;
    open_set.emplace(start_idx, heuristic(start_idx, goal_idx));

    while (!open_set.empty()) {
      AStarNode current = open_set.top();
      open_set.pop();

      // skip already-expanded nodes
      if (closed_set.count(current.index)) continue;
      closed_set.insert(current.index);

      if (current.index == goal_idx) {
        found_path = true;
        break;
      }

      std::vector<CellIndex> neighbors = getNeighbors(current.index);
      for (const auto& neighbor : neighbors) {
        // traversal cost
        double dist_cost = std::sqrt(
          std::pow(neighbor.x - current.index.x, 2) +
          std::pow(neighbor.y - current.index.y, 2));

        // obstacle penalty factor
        int n_idx = neighbor.y * current_map_.info.width + neighbor.x;
        int8_t cell_cost = current_map_.data[n_idx];
        double cost_penalty = 0.0;
        if (cell_cost > 0) {
          cost_penalty = static_cast<double>(cell_cost) * 0.05;
        }

        double tentative_g = g_score[current.index] + dist_cost + cost_penalty;

        if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
          came_from[neighbor] = current.index;
          g_score[neighbor] = tentative_g;
          double f = tentative_g + heuristic(neighbor, goal_idx);
          open_set.emplace(neighbor, f);
        }
      }
    }

    if (!found_path) {
      // increase threshold and retry
      cost_threshold_ += 10;
      retry_count++;
    }
  }

  if (found_path) {
    // extract path from search result
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    CellIndex current = goal_idx;
    while (current != start_idx) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      double wx, wy;
      gridToWorld(current, wx, wy);
      p.pose.position.x = wx;
      p.pose.position.y = wy;
      p.pose.orientation.w = 1.0;
      poses.push_back(p);
      current = came_from[current];
    }
    // include robot start point
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose = robot_pose_;
    poses.push_back(p);

    std::reverse(poses.begin(), poses.end());
    path.poses = poses;
    return path;
  } else {
    RCLCPP_WARN(logger_, "No path found to goal!");
    return std::nullopt;
  }
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b)
{
  // euclidean distance
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& current)
{
  std::vector<CellIndex> neighbors;
  // 8-connected search directions
  const int dirs[8][2] = {
    {0, 1}, {1, 1}, {1, 0}, {1, -1},
    {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
  };

  for (int i = 0; i < 8; ++i) {
    int dx = dirs[i][0];
    int dy = dirs[i][1];
    CellIndex neighbor(current.x + dx, current.y + dy);
    double move_yaw = std::atan2(dy, dx);
    if (!isValid(neighbor, move_yaw)) continue;
    neighbors.push_back(neighbor);
  }
  return neighbors;
}

bool PlannerCore::isValid(const CellIndex& idx, double yaw)
{
  if (idx.x < 0 || idx.x >= static_cast<int>(current_map_.info.width) ||
      idx.y < 0 || idx.y >= static_cast<int>(current_map_.info.height)) {
    return false;
  }

  // check robot footprint for collisions
  double ROBOT_LENGTH = 0.6;
  double ROBOT_WIDTH  = 0.6;
  double STEP = current_map_.info.resolution;
  if (STEP <= 0.001) STEP = 0.1;

  double rx, ry;
  gridToWorld(idx, rx, ry);

  for (double back = 0.0; back <= ROBOT_LENGTH + STEP / 2.0; back += STEP) {
    for (double side = -ROBOT_WIDTH / 2.0; side <= ROBOT_WIDTH / 2.0 + STEP / 2.0; side += STEP) {
      double lx = back - ROBOT_LENGTH / 2.0;
      double ly = side;
      double wx = rx + std::cos(yaw) * lx - std::sin(yaw) * ly;
      double wy = ry + std::sin(yaw) * lx + std::cos(yaw) * ly;

      CellIndex pt_idx = worldToGrid(wx, wy);
      if (pt_idx.x < 0 || pt_idx.x >= static_cast<int>(current_map_.info.width) ||
          pt_idx.y < 0 || pt_idx.y >= static_cast<int>(current_map_.info.height)) {
        return false;
      }

      int map_idx = pt_idx.y * current_map_.info.width + pt_idx.x;
      if (current_map_.data[map_idx] >= cost_threshold_) return false;
    }
  }

  return true;
}

CellIndex PlannerCore::worldToGrid(double wx, double wy)
{
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;
  int gx = static_cast<int>((wx - ox) / res);
  int gy = static_cast<int>((wy - oy) / res);
  return CellIndex(gx, gy);
}

void PlannerCore::gridToWorld(const CellIndex& idx, double& wx, double& wy)
{
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;
  // world coordinate of cell center
  wx = ox + (idx.x + 0.5) * res;
  wy = oy + (idx.y + 0.5) * res;
}

} // namespace robot
