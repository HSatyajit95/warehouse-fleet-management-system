#include "fms_robot_agent/bt_nodes/charge_battery.hpp"
#include "fms_robot_agent/robot_agent_node.hpp"

namespace fms_robot_agent::bt {

ChargeBattery::ChargeBattery(const std::string& name,
                             const BT::NodeConfiguration& config,
                             RobotAgentNode* node)
: BT::StatefulActionNode(name, config), node_(node) {}

BT::NodeStatus ChargeBattery::onStart() {
  node_->is_charging_.store(true);
  node_->is_moving_.store(false);

  {
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::LOW_BATTERY);
  }

  RCLCPP_INFO(node_->get_logger(), "[ChargeBattery] Charging... SOC=%.1f%%",
              node_->battery_.soc());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ChargeBattery::onRunning() {
  if (node_->battery_.is_full()) {
    node_->is_charging_.store(false);
    {
      std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
      node_->fsm_.process(FsmEvent::BATTERY_FULL);
    }
    RCLCPP_INFO(node_->get_logger(), "[ChargeBattery] Full. SOC=%.1f%%",
                node_->battery_.soc());
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void ChargeBattery::onHalted() {
  node_->is_charging_.store(false);
  RCLCPP_DEBUG(node_->get_logger(), "[ChargeBattery] Halted.");
}

}  // namespace fms_robot_agent::bt
