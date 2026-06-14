#include "fms_robot_agent/bt_nodes/battery_ok.hpp"
#include "fms_robot_agent/robot_agent_node.hpp"

namespace fms_robot_agent::bt {

BatteryOK::BatteryOK(const std::string& name,
                     const BT::NodeConfiguration& config,
                     RobotAgentNode* node)
: BT::ConditionNode(name, config), node_(node) {}

BT::PortsList BatteryOK::providedPorts() {
  return { BT::InputPort<double>("threshold", 20.0, "Low battery threshold (%)") };
}

BT::NodeStatus BatteryOK::tick() {
  double threshold = getInput<double>("threshold").value_or(BatteryModel::LOW_THRESHOLD);
  return (node_->battery_.soc() >= threshold)
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
}

}  // namespace fms_robot_agent::bt
