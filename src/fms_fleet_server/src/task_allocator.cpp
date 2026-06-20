#include "fms_fleet_server/task_allocator.hpp"

#include <cmath>
#include <limits>

namespace fms_fleet_server {

std::optional<std::pair<std::string, double>> select_best_robot(
    const std::map<std::string, fms_msgs::msg::RobotStatus>& robots,
    const fms_msgs::msg::TaskAssignment& request,
    double soc_weight)
{
  std::string best_robot;
  double best_score = std::numeric_limits<double>::max();

  for (const auto& [robot_id, status] : robots) {
    if (status.state != fms_msgs::msg::RobotStatus::STATE_IDLE) {
      continue;
    }
    const double dx = request.pick_pose.position.x - status.pose.position.x;
    const double dy = request.pick_pose.position.y - status.pose.position.y;
    const double distance_to_pick = std::sqrt(dx * dx + dy * dy);
    const double score = distance_to_pick + (100.0 - status.battery_soc) * soc_weight;

    if (score < best_score) {
      best_score = score;
      best_robot = robot_id;
    }
  }

  if (best_robot.empty()) {
    return std::nullopt;
  }
  return std::make_pair(best_robot, best_score);
}

}  // namespace fms_fleet_server
