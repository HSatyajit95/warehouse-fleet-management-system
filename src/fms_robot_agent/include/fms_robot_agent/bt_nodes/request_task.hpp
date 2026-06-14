#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <fms_msgs/msg/task_assignment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace fms_robot_agent { class RobotAgentNode; }

namespace fms_robot_agent::bt {

// Blocks until a TaskAssignment is received on /robot_N/task_assignment.
// On success: writes pick_pose and drop_pose to the blackboard.
// Handles battery-interrupted task re-queuing transparently.
class RequestTask : public BT::StatefulActionNode {
public:
  RequestTask(const std::string& name, const BT::NodeConfiguration& config,
              RobotAgentNode* node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  RobotAgentNode* node_;
};

}  // namespace fms_robot_agent::bt
