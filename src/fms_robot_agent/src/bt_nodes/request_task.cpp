#include "fms_robot_agent/bt_nodes/request_task.hpp"
#include "fms_robot_agent/robot_agent_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace fms_robot_agent::bt {

RequestTask::RequestTask(const std::string& name,
                         const BT::NodeConfiguration& config,
                         RobotAgentNode* node)
: BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList RequestTask::providedPorts() {
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pick_pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("drop_pose"),
  };
}

BT::NodeStatus RequestTask::onStart() {
  RCLCPP_DEBUG(node_->get_logger(), "[RequestTask] Waiting for task...");
  return onRunning();
}

BT::NodeStatus RequestTask::onRunning() {
  std::optional<fms_msgs::msg::TaskAssignment> task;

  {
    std::lock_guard<std::mutex> lk(node_->task_mutex_);
    // Interrupted task (from battery preemption) has priority.
    if (node_->interrupted_task_.has_value()) {
      task = std::move(node_->interrupted_task_);
      node_->interrupted_task_.reset();
      RCLCPP_INFO(node_->get_logger(), "[RequestTask] Resuming interrupted task %s",
                  task->task_id.c_str());
    } else if (node_->pending_task_.has_value()) {
      task = std::move(node_->pending_task_);
      node_->pending_task_.reset();
    }
  }

  if (!task.has_value()) {
    return BT::NodeStatus::RUNNING;
  }

  {
    std::lock_guard<std::mutex> lk(node_->task_mutex_);
    node_->current_task_ = task;
  }
  node_->task_start_time_ = node_->get_clock()->now();

  // Write goal poses to blackboard so NavigateToPoseBT instances can read them.
  geometry_msgs::msg::PoseStamped pick_ps, drop_ps;
  pick_ps.header.frame_id = "map";
  pick_ps.header.stamp    = node_->get_clock()->now();
  pick_ps.pose             = task->pick_pose;

  drop_ps.header.frame_id = "map";
  drop_ps.header.stamp    = node_->get_clock()->now();
  drop_ps.pose             = task->drop_pose;

  setOutput("pick_pose", pick_ps);
  setOutput("drop_pose", drop_ps);

  {
    std::lock_guard<std::mutex> lk(node_->fsm_mutex_);
    node_->fsm_.process(FsmEvent::TASK_RECEIVED);
  }

  RCLCPP_INFO(node_->get_logger(),
              "[RequestTask] Task %s accepted: pick(%.1f,%.1f) drop(%.1f,%.1f)",
              task->task_id.c_str(),
              task->pick_pose.position.x, task->pick_pose.position.y,
              task->drop_pose.position.x, task->drop_pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

void RequestTask::onHalted() {
  RCLCPP_DEBUG(node_->get_logger(), "[RequestTask] Halted while waiting for task.");
}

}  // namespace fms_robot_agent::bt
