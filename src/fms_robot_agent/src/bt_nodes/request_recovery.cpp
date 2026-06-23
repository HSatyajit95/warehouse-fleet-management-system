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
    // start_spin()/start_backup() below lock mutex_ themselves — read the
    // shared state and release lk first, or calling them while still
    // holding lk self-deadlocks this thread on the non-recursive
    // mutex_ (which also freezes battery_timer_/status_timer_, since
    // they share this BT tick's callback group/thread).
    bool received, success;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!spin_state_) return BT::NodeStatus::RUNNING;
      received = spin_state_->result_received;
      success  = spin_state_->result_success;
    }
    if (!received) return BT::NodeStatus::RUNNING;

    if (success) {
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
    bool received, success;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!backup_state_) return BT::NodeStatus::RUNNING;
      received = backup_state_->result_received;
      success  = backup_state_->result_success;
    }
    if (!received) return BT::NodeStatus::RUNNING;

    if (success) {
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
  // action_server_is_ready() (not wait_for_action_server()) — this runs
  // inside the BT tick on cb_group_timer_, which shares a 2-thread
  // MultiThreadedExecutor with the action-result callback group. A
  // blocking wait here can starve that thread (and with it
  // battery_timer_/status_timer_, also on cb_group_timer_) well past its
  // nominal timeout. The behavior_server's spin/backup servers are
  // long-lived Nav2 lifecycle nodes brought up once at startup, so an
  // instant readiness check is sufficient here.
  if (!node_->spin_client_->action_server_is_ready()) {
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
  // See start_spin()'s comment — non-blocking check for the same reason.
  if (!node_->backup_client_->action_server_is_ready()) {
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
