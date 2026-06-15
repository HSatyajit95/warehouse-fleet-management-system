#pragma once

#include <fms_msgs/msg/robot_status.hpp>
#include <string>

namespace fms_fleet_server {

inline std::string robot_state_to_string(uint8_t state)
{
  switch (state) {
    case fms_msgs::msg::RobotStatus::STATE_IDLE:       return "IDLE";
    case fms_msgs::msg::RobotStatus::STATE_ASSIGNED:   return "ASSIGNED";
    case fms_msgs::msg::RobotStatus::STATE_NAVIGATING: return "NAVIGATING";
    case fms_msgs::msg::RobotStatus::STATE_EXECUTING:  return "EXECUTING";
    case fms_msgs::msg::RobotStatus::STATE_REPORTING:  return "REPORTING";
    case fms_msgs::msg::RobotStatus::STATE_RECOVERING: return "RECOVERING";
    case fms_msgs::msg::RobotStatus::STATE_CHARGING:   return "CHARGING";
    default:                                           return "UNKNOWN";
  }
}

}  // namespace fms_fleet_server
