#pragma once
#include <behaviortree_cpp_v3/condition_node.h>

namespace fms_robot_agent { class RobotAgentNode; }

namespace fms_robot_agent::bt {

// Returns SUCCESS when battery SOC >= threshold, FAILURE when low.
class BatteryOK : public BT::ConditionNode {
public:
  BatteryOK(const std::string& name, const BT::NodeConfiguration& config,
            RobotAgentNode* node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  RobotAgentNode* node_;
};

}  // namespace fms_robot_agent::bt
