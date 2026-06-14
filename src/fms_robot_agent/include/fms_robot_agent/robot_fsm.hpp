#pragma once
#include <cstdint>
#include <string>

namespace fms_robot_agent {

enum class RobotState : uint8_t {
  IDLE       = 0,
  ASSIGNED   = 1,
  NAVIGATING = 2,
  EXECUTING  = 3,
  REPORTING  = 4,
  RECOVERING = 5,
  CHARGING   = 6,
};

enum class FsmEvent {
  TASK_RECEIVED,
  NAV_STARTED,
  NAV_SUCCEEDED,
  NAV_FAILED,
  EXECUTION_DONE,
  REPORT_SENT,
  RECOVERY_DONE,
  RECOVERY_FAILED,
  LOW_BATTERY,
  BATTERY_FULL,
  FAULT_INJECTED,
};

class RobotFSM {
public:
  RobotFSM() = default;

  RobotState state() const { return state_; }
  uint8_t    state_id() const { return static_cast<uint8_t>(state_); }
  const char* state_name() const;

  // Returns true if the transition was valid, false if ignored.
  bool process(FsmEvent event);

private:
  RobotState state_{RobotState::IDLE};
};

}  // namespace fms_robot_agent
