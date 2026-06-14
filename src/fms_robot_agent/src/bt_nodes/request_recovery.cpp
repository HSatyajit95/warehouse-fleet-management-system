#include "fms_robot_agent/bt_nodes/request_recovery.hpp"
#include "fms_robot_agent/robot_agent_node.hpp"

namespace fms_robot_agent::bt {

RequestRecovery::RequestRecovery(const std::string& name,
                                 const BT::NodeConfiguration& config,
                                 RobotAgentNode* node)
: BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList RequestRecovery::providedPorts() {
  return { BT::InputPort<int>("max_attempts", 3, "Max recovery attempts") };
}

BT::NodeStatus RequestRecovery::onStart() {
  attempt_ = 0;
  phase_   = RecoveryPhase::SPIN;
  RCLCPP_INFO(node_->get_logger(), "[RequestRecovery] Starting recovery...");
  {
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::NAV_FAILED);  // ensure RECOVERING state
  }
  start_spin();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RequestRecovery::onRunning() {
  int max_attempts = getInput<int>("max_attempts").value_or(node_->recovery_max_attempts_);

  if (phase_ == RecoveryPhase::SPIN) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!spin_state_) return BT::NodeStatus::RUNNING;
    if (!spin_state_->result_received) return BT::NodeStatus::RUNNING;

    if (spin_state_->result_success) {
      RCLCPP_DEBUG(node_->get_logger(), "[RequestRecovery] Spin done, starting backup.");
      phase_ = RecoveryPhase::BACKUP;
      start_backup();
    } else {
      RCLCPP_WARN(node_->get_logger(), "[RequestRecovery] Spin failed.");
      attempt_++;
      if (attempt_ >= max_attempts) {
        phase_ = RecoveryPhase::FAILED;
      } else {
        phase_ = RecoveryPhase::SPIN;
        start_spin();
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == RecoveryPhase::BACKUP) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!backup_state_) return BT::NodeStatus::RUNNING;
    if (!backup_state_->result_received) return BT::NodeStatus::RUNNING;

    if (backup_state_->result_success) {
      phase_ = RecoveryPhase::DONE;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[RequestRecovery] Backup failed.");
      attempt_++;
      if (attempt_ >= max_attempts) {
        phase_ = RecoveryPhase::FAILED;
      } else {
        phase_ = RecoveryPhase::SPIN;
        start_spin();
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == RecoveryPhase::DONE) {
    RCLCPP_INFO(node_->get_logger(), "[RequestRecovery] Recovery succeeded.");
    {
      std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
      node_->fsm_.process(FsmEvent::RECOVERY_DONE);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // FAILED
  RCLCPP_ERROR(node_->get_logger(), "[RequestRecovery] Recovery failed after %d attempts.", attempt_);
  {
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::RECOVERY_FAILED);
  }
  return BT::NodeStatus::FAILURE;
}

void RequestRecovery::onHalted() {
  RCLCPP_DEBUG(node_->get_logger(), "[RequestRecovery] Halted.");
  std::lock_guard<std::mutex> lk(mutex_);
  spin_state_.reset();
  backup_state_.reset();
}

void RequestRecovery::start_spin() {
  if (!node_->spin_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), "[RequestRecovery] Spin action server not ready.");
    std::lock_guard<std::mutex> lk(mutex_);
    spin_state_ = std::make_shared<RecovState>();
    spin_state_->result_received = true;
    spin_state_->result_success  = false;
    return;
  }

  auto s = std::make_shared<RecovState>();
  auto s_weak = std::weak_ptr<RecovState>(s);
  auto m = &mutex_;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    spin_state_ = s;
  }

  SpinAction::Goal goal;
  goal.target_yaw = 6.28f;   // full 360°
  goal.time_allowance.sec = 10;

  auto opts = rclcpp_action::Client<SpinAction>::SendGoalOptions{};
  opts.result_callback = [s_weak, m](const auto& result) {
    std::lock_guard<std::mutex> lk(*m);
    if (auto st = s_weak.lock()) {
      st->result_received = true;
      st->result_success  = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    }
  };

  node_->spin_client_->async_send_goal(goal, opts);
  RCLCPP_DEBUG(node_->get_logger(), "[RequestRecovery] Spin goal sent.");
}

void RequestRecovery::start_backup() {
  if (!node_->backup_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), "[RequestRecovery] BackUp action server not ready.");
    std::lock_guard<std::mutex> lk(mutex_);
    backup_state_ = std::make_shared<RecovState>();
    backup_state_->result_received = true;
    backup_state_->result_success  = false;
    return;
  }

  auto s = std::make_shared<RecovState>();
  auto s_weak = std::weak_ptr<RecovState>(s);
  auto m = &mutex_;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    backup_state_ = s;
  }

  BackupAction::Goal goal;
  goal.target.x = 0.3f;   // back up 0.3 m
  goal.speed    = 0.15f;
  goal.time_allowance.sec = 8;

  auto opts = rclcpp_action::Client<BackupAction>::SendGoalOptions{};
  opts.result_callback = [s_weak, m](const auto& result) {
    std::lock_guard<std::mutex> lk(*m);
    if (auto st = s_weak.lock()) {
      st->result_received = true;
      st->result_success  = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    }
  };

  node_->backup_client_->async_send_goal(goal, opts);
  RCLCPP_DEBUG(node_->get_logger(), "[RequestRecovery] BackUp goal sent.");
}

}  // namespace fms_robot_agent::bt
