#include "fms_robot_agent/bt_nodes/execute_pick_drop.hpp"
#include "fms_robot_agent/robot_agent_node.hpp"

namespace fms_robot_agent::bt {

ExecutePickDrop::ExecutePickDrop(const std::string& name,
                                 const BT::NodeConfiguration& config,
                                 RobotAgentNode* node)
: BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList ExecutePickDrop::providedPorts() {
  return {
    BT::InputPort<std::string>("operation",    "pick or drop"),
    BT::InputPort<double>     ("duration_secs", 3.0, "Simulated operation time"),
  };
}

BT::NodeStatus ExecutePickDrop::onStart() {
  auto op        = getInput<std::string>("operation").value_or("pick");
  duration_secs_ = getInput<double>("duration_secs").value_or(3.0);
  start_time_    = node_->get_clock()->now();

  RCLCPP_INFO(node_->get_logger(), "[ExecutePickDrop] Starting %s (%.1fs)...",
              op.c_str(), duration_secs_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecutePickDrop::onRunning() {
  auto elapsed = (node_->get_clock()->now() - start_time_).seconds();
  if (elapsed < duration_secs_) {
    return BT::NodeStatus::RUNNING;
  }

  auto op = getInput<std::string>("operation").value_or("pick");
  RCLCPP_INFO(node_->get_logger(), "[ExecutePickDrop] %s complete.", op.c_str());

  if (op == "pick") {
    // Pick done → robot needs to navigate to drop location next.
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::EXECUTION_DONE);  // EXECUTING → NAVIGATING
  }
  // For "drop": FSM stays in EXECUTING; ReportStatus will fire REPORT_SENT → IDLE.
  return BT::NodeStatus::SUCCESS;
}

void ExecutePickDrop::onHalted() {
  RCLCPP_DEBUG(node_->get_logger(), "[ExecutePickDrop] Halted.");
}

}  // namespace fms_robot_agent::bt
