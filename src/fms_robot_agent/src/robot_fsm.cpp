#include "fms_robot_agent/robot_fsm.hpp"
#include <stdexcept>

namespace fms_robot_agent {

const char* RobotFSM::state_name() const {
  switch (state_) {
    case RobotState::IDLE:       return "IDLE";
    case RobotState::ASSIGNED:   return "ASSIGNED";
    case RobotState::NAVIGATING: return "NAVIGATING";
    case RobotState::EXECUTING:  return "EXECUTING";
    case RobotState::REPORTING:  return "REPORTING";
    case RobotState::RECOVERING: return "RECOVERING";
    case RobotState::CHARGING:   return "CHARGING";
  }
  return "UNKNOWN";
}

bool RobotFSM::process(FsmEvent event) {
  const RobotState prev = state_;

  switch (state_) {

    case RobotState::IDLE:
      if (event == FsmEvent::TASK_RECEIVED) { state_ = RobotState::ASSIGNED; break; }
      if (event == FsmEvent::LOW_BATTERY)   { state_ = RobotState::CHARGING; break; }
      return false;

    case RobotState::ASSIGNED:
      if (event == FsmEvent::NAV_STARTED)  { state_ = RobotState::NAVIGATING; break; }
      if (event == FsmEvent::LOW_BATTERY)  { state_ = RobotState::CHARGING;   break; }
      return false;

    case RobotState::NAVIGATING:
      if (event == FsmEvent::NAV_STARTED)    { /* self-loop: 2nd segment nav */ break; }
      if (event == FsmEvent::NAV_SUCCEEDED)  { state_ = RobotState::EXECUTING;  break; }
      if (event == FsmEvent::NAV_FAILED)     { state_ = RobotState::RECOVERING; break; }
      if (event == FsmEvent::FAULT_INJECTED) { state_ = RobotState::RECOVERING; break; }
      if (event == FsmEvent::LOW_BATTERY)    { state_ = RobotState::CHARGING;   break; }
      return false;

    case RobotState::EXECUTING:
      if (event == FsmEvent::EXECUTION_DONE) { state_ = RobotState::NAVIGATING; break; }
      if (event == FsmEvent::REPORT_SENT)    { state_ = RobotState::IDLE;       break; }
      if (event == FsmEvent::LOW_BATTERY)    { state_ = RobotState::CHARGING;   break; }
      return false;

    case RobotState::REPORTING:
      if (event == FsmEvent::REPORT_SENT) { state_ = RobotState::IDLE; break; }
      return false;

    case RobotState::RECOVERING:
      if (event == FsmEvent::RECOVERY_DONE)   { state_ = RobotState::ASSIGNED; break; }
      if (event == FsmEvent::RECOVERY_FAILED) { state_ = RobotState::IDLE;     break; }
      if (event == FsmEvent::LOW_BATTERY)     { state_ = RobotState::CHARGING; break; }
      return false;

    case RobotState::CHARGING:
      if (event == FsmEvent::BATTERY_FULL) { state_ = RobotState::IDLE; break; }
      return false;
  }

  return state_ != prev;
}

}  // namespace fms_robot_agent
