#include "fms_robot_agent/bt_nodes/navigate_to_pose_bt.hpp"
#include "fms_robot_agent/robot_agent_node.hpp"

namespace fms_robot_agent::bt {

NavigateToPoseBT::NavigateToPoseBT(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   RobotAgentNode* node)
: BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList NavigateToPoseBT::providedPorts() {
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
}

BT::NodeStatus NavigateToPoseBT::onStart() {
  auto goal_opt = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_opt) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToPose:%s] No goal port set.", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // action_server_is_ready() (not wait_for_action_server()) — onStart()
  // runs inside the BT tick on cb_group_timer_, which shares a 2-thread
  // MultiThreadedExecutor with the action-result callback group; a
  // blocking wait here risks starving that thread (and battery_timer_/
  // status_timer_ along with it) well past its nominal timeout. See the
  // same fix in request_recovery.cpp for the confirmed case of this.
  if (!node_->nav_client_->action_server_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "[NavigateToPose:%s] Action server not ready.", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  nav_state_ = std::make_shared<NavState>();
  auto state_weak = std::weak_ptr<NavState>(nav_state_);
  auto mutex_raw  = &mutex_;

  auto goal_msg = NavAction::Goal{};
  goal_msg.pose = goal_opt.value();
  goal_msg.pose.header.stamp = node_->get_clock()->now();

  auto opts = rclcpp_action::Client<NavAction>::SendGoalOptions{};
  opts.goal_response_callback = [state_weak, mutex_raw](auto handle) {
    std::lock_guard<std::mutex> lk(*mutex_raw);
    if (auto s = state_weak.lock()) {
      s->response_received = true;
      s->goal_accepted     = (handle != nullptr);
      s->goal_handle       = handle;
    }
  };
  opts.result_callback = [state_weak, mutex_raw](const auto& result) {
    std::lock_guard<std::mutex> lk(*mutex_raw);
    if (auto s = state_weak.lock()) {
      s->result_received = true;
      s->result_code     = result.code;
    }
  };

  node_->nav_client_->async_send_goal(goal_msg, opts);

  {
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::NAV_STARTED);
  }
  node_->is_moving_.store(true);

  RCLCPP_INFO(node_->get_logger(), "[NavigateToPose:%s] Goal sent → (%.1f, %.1f)",
              name().c_str(),
              goal_msg.pose.pose.position.x,
              goal_msg.pose.pose.position.y);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseBT::onRunning() {
  // Check fault injection
  if (node_->fault_injected_.exchange(false)) {
    RCLCPP_WARN(node_->get_logger(), "[NavigateToPose:%s] Fault injected — cancelling goal.",
                name().c_str());
    onHalted();
    {
      std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
      node_->fsm_.process(FsmEvent::FAULT_INJECTED);
    }
    return BT::NodeStatus::FAILURE;
  }

  // Read shared state under the lock, then release before touching FSM.
  bool response_received, goal_accepted, result_received;
  rclcpp_action::ResultCode result_code;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!nav_state_) return BT::NodeStatus::FAILURE;
    response_received = nav_state_->response_received;
    goal_accepted     = nav_state_->goal_accepted;
    result_received   = nav_state_->result_received;
    result_code       = nav_state_->result_code;
  }

  if (!response_received) {
    return BT::NodeStatus::RUNNING;  // waiting for goal response
  }

  if (!goal_accepted) {
    RCLCPP_ERROR(node_->get_logger(), "[NavigateToPose:%s] Goal rejected.", name().c_str());
    node_->is_moving_.store(false);
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::NAV_FAILED);
    return BT::NodeStatus::FAILURE;
  }

  if (!result_received) {
    return BT::NodeStatus::RUNNING;  // navigating
  }

  node_->is_moving_.store(false);

  if (result_code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToPose:%s] Succeeded.", name().c_str());
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::NAV_SUCCEEDED);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(node_->get_logger(), "[NavigateToPose:%s] Failed (code=%d).",
              name().c_str(), static_cast<int>(result_code));
  std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
  node_->fsm_.process(FsmEvent::NAV_FAILED);
  return BT::NodeStatus::FAILURE;
}

void NavigateToPoseBT::onHalted() {
  node_->is_moving_.store(false);
  std::lock_guard<std::mutex> lk(mutex_);
  if (nav_state_ && nav_state_->goal_handle && !nav_state_->result_received) {
    node_->nav_client_->async_cancel_goal(nav_state_->goal_handle);
    RCLCPP_DEBUG(node_->get_logger(), "[NavigateToPose:%s] Cancel sent.", name().c_str());
  }
  nav_state_.reset();
}

}  // namespace fms_robot_agent::bt
