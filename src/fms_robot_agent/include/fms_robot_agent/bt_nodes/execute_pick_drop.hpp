#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/time.hpp>

namespace fms_robot_agent { class RobotAgentNode; }

namespace fms_robot_agent::bt {

// Simulates a pick or drop operation with a configurable timed delay.
class ExecutePickDrop : public BT::StatefulActionNode {
public:
  ExecutePickDrop(const std::string& name, const BT::NodeConfiguration& config,
                  RobotAgentNode* node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  RobotAgentNode* node_;
  rclcpp::Time    start_time_;
  double          duration_secs_{3.0};
};

}  // namespace fms_robot_agent::bt
