#pragma once

#include <fms_msgs/msg/robot_status.hpp>
#include <fms_msgs/msg/task_assignment.hpp>

#include <map>
#include <optional>
#include <string>
#include <utility>

namespace fms_fleet_server {

// Pure scoring logic used by FleetServerNode::select_robot (Step 3.3). Kept
// free of rclcpp::Node/Mongo/RabbitMQ/gRPC so it can be unit-tested without
// constructing the full fleet server (see fms_fleet_server/test/test_task_allocator.cpp).
//
// Scores every IDLE robot in `robots` by distance-to-pick + (100 - SOC) *
// soc_weight and returns the lowest-scoring robot_id, or std::nullopt if no
// robot is currently IDLE.
std::optional<std::pair<std::string, double>> select_best_robot(
    const std::map<std::string, fms_msgs::msg::RobotStatus>& robots,
    const fms_msgs::msg::TaskAssignment& request,
    double soc_weight);

}  // namespace fms_fleet_server
