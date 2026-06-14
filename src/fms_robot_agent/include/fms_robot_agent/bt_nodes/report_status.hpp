#pragma once
#include <behaviortree_cpp_v3/action_node.h>

namespace fms_robot_agent { class RobotAgentNode; }

namespace fms_robot_agent::bt {

// Publishes TaskCompletion and transitions FSM REPORTING → IDLE.
// Synchronous — executes entirely in tick().
class ReportStatus : public BT::SyncActionNode {
public:
  ReportStatus(const std::string& name, const BT::NodeConfiguration& config,
               RobotAgentNode* node);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  RobotAgentNode* node_;
};

}  // namespace fms_robot_agent::bt
