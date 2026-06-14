#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <mutex>

namespace fms_robot_agent { class RobotAgentNode; }

namespace fms_robot_agent::bt {

// Runs Nav2 recovery behaviors: spin then backup.
// Returns SUCCESS when both complete. FAILURE after max_attempts exhausted.
class RequestRecovery : public BT::StatefulActionNode {
public:
  using SpinAction   = nav2_msgs::action::Spin;
  using BackupAction = nav2_msgs::action::BackUp;

  RequestRecovery(const std::string& name, const BT::NodeConfiguration& config,
                  RobotAgentNode* node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  enum class RecoveryPhase { SPIN, BACKUP, DONE, FAILED };

  struct RecovState {
    bool result_received{false};
    bool result_success{false};
  };

  void start_spin();
  void start_backup();

  RobotAgentNode*              node_;
  RecoveryPhase                phase_{RecoveryPhase::SPIN};
  int                          attempt_{0};
  std::shared_ptr<RecovState>  spin_state_;
  std::shared_ptr<RecovState>  backup_state_;
  std::mutex                   mutex_;
};

}  // namespace fms_robot_agent::bt
