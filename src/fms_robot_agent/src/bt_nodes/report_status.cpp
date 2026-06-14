#include "fms_robot_agent/bt_nodes/report_status.hpp"
#include "fms_robot_agent/robot_agent_node.hpp"

namespace fms_robot_agent::bt {

ReportStatus::ReportStatus(const std::string& name,
                           const BT::NodeConfiguration& config,
                           RobotAgentNode* node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus ReportStatus::tick() {
  fms_msgs::msg::TaskCompletion msg;
  msg.header.stamp = node_->get_clock()->now();

  {
    std::lock_guard<std::mutex> lk(node_->task_mutex_);
    if (node_->current_task_.has_value()) {
      msg.task_id  = node_->current_task_->task_id;
      msg.robot_id = node_->robot_name_;
    }
    node_->current_task_.reset();
  }

  msg.result = fms_msgs::msg::TaskCompletion::RESULT_SUCCESS;
  msg.duration_secs = static_cast<float>(
      (node_->get_clock()->now() - node_->task_start_time_).seconds());

  if (node_->completion_pub_ && node_->completion_pub_->is_activated()) {
    node_->completion_pub_->publish(msg);
  }

  {
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::REPORT_SENT);  // EXECUTING → IDLE (or REPORTING → IDLE)
  }

  RCLCPP_INFO(node_->get_logger(), "[ReportStatus] Task %s completed in %.1fs.",
              msg.task_id.c_str(), msg.duration_secs);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace fms_robot_agent::bt
