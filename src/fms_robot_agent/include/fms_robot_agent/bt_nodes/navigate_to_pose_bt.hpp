#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <memory>
#include <mutex>

namespace fms_robot_agent { class RobotAgentNode; }

namespace fms_robot_agent::bt {

// Wraps the Nav2 NavigateToPose action client.
// Reads the goal pose from the blackboard key specified by the "goal" InputPort.
class NavigateToPoseBT : public BT::StatefulActionNode {
public:
  using NavAction   = nav2_msgs::action::NavigateToPose;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<NavAction>;

  NavigateToPoseBT(const std::string& name, const BT::NodeConfiguration& config,
                   RobotAgentNode* node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  struct NavState {
    bool response_received{false};   // goal_response_callback has fired
    bool goal_accepted{false};
    bool result_received{false};
    rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};
    GoalHandle::SharedPtr goal_handle;
  };

  RobotAgentNode*           node_;
  std::shared_ptr<NavState> nav_state_;
  std::mutex                mutex_;
};

}  // namespace fms_robot_agent::bt
