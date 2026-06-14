#pragma once
#include <behaviortree_cpp_v3/action_node.h>

namespace fms_robot_agent { class RobotAgentNode; }

namespace fms_robot_agent::bt {

// Blocks until battery SOC >= FULL_THRESHOLD.
// Signals the agent node that charging is active so the battery model charges.
class ChargeBattery : public BT::StatefulActionNode {
public:
  ChargeBattery(const std::string& name, const BT::NodeConfiguration& config,
                RobotAgentNode* node);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  RobotAgentNode* node_;
};

}  // namespace fms_robot_agent::bt
