#pragma once

#include <fms_msgs/msg/task_assignment.hpp>
#include <fms_msgs/msg/task_completion.hpp>
#include <string>

namespace fms_fleet_server {

inline std::string task_type_to_string(uint8_t type)
{
  switch (type) {
    case fms_msgs::msg::TaskAssignment::TASK_PICK:   return "PICK";
    case fms_msgs::msg::TaskAssignment::TASK_DROP:   return "DROP";
    case fms_msgs::msg::TaskAssignment::TASK_CHARGE: return "CHARGE";
    default:                                         return "UNKNOWN";
  }
}

inline std::string task_result_to_string(uint8_t result)
{
  switch (result) {
    case fms_msgs::msg::TaskCompletion::RESULT_SUCCESS:   return "COMPLETED";
    case fms_msgs::msg::TaskCompletion::RESULT_FAILED:    return "FAILED";
    case fms_msgs::msg::TaskCompletion::RESULT_CANCELLED: return "CANCELLED";
    default:                                              return "UNKNOWN";
  }
}

}  // namespace fms_fleet_server
