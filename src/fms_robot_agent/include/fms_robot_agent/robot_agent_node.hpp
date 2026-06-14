#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <fms_msgs/msg/robot_status.hpp>
#include <fms_msgs/msg/task_assignment.hpp>
#include <fms_msgs/msg/task_completion.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "fms_robot_agent/robot_fsm.hpp"
#include "fms_robot_agent/battery_model.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace fms_robot_agent {

class RobotAgentNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using NavAction    = nav2_msgs::action::NavigateToPose;
  using SpinAction   = nav2_msgs::action::Spin;
  using BackupAction = nav2_msgs::action::BackUp;

  explicit RobotAgentNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{});

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State&) override;

  // ── Public state (accessed directly by BT nodes) ──────────────────────────

  std::string robot_name_;

  // FSM
  RobotFSM   fsm_;
  std::mutex fsm_mutex_;

  // Tasks  (all guarded by task_mutex_)
  std::optional<fms_msgs::msg::TaskAssignment> pending_task_;
  std::optional<fms_msgs::msg::TaskAssignment> current_task_;
  std::optional<fms_msgs::msg::TaskAssignment> interrupted_task_;
  std::mutex task_mutex_;

  // Battery (battery_ updated from battery_timer_, is_moving_ set by BT nodes)
  BatteryModel        battery_;
  std::atomic<bool>   is_moving_{false};
  std::atomic<bool>   is_charging_{false};

  // Action clients (created once; all BT node instances share them)
  rclcpp_action::Client<NavAction>::SharedPtr    nav_client_;
  rclcpp_action::Client<SpinAction>::SharedPtr   spin_client_;
  rclcpp_action::Client<BackupAction>::SharedPtr backup_client_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<fms_msgs::msg::TaskCompletion>::SharedPtr
      completion_pub_;

  // Config
  double pick_duration_secs_{3.0};
  double drop_duration_secs_{2.0};
  int    recovery_max_attempts_{3};

  // Task timing (set by RequestTask BT node)
  rclcpp::Time task_start_time_;

  // Fault injection flag (set by service, read by BT tick)
  std::atomic<bool> fault_injected_{false};

  // Charger pose (populated from params in on_configure)
  geometry_msgs::msg::PoseStamped charger_pose_;

private:
  // Callbacks
  void bt_tick_cb();
  void battery_update_cb();
  void status_publish_cb();
  void task_assignment_cb(const fms_msgs::msg::TaskAssignment::SharedPtr msg);

  bool fault_service_cb(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  void register_bt_nodes();

  // ROS interfaces
  rclcpp::Subscription<fms_msgs::msg::TaskAssignment>::SharedPtr task_sub_;
  rclcpp_lifecycle::LifecyclePublisher<fms_msgs::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr fault_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr bt_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_action_;

  // BT
  BT::BehaviorTreeFactory   factory_;
  std::unique_ptr<BT::Tree> bt_tree_;
  BT::Blackboard::Ptr       blackboard_;

  // Battery update bookkeeping
  rclcpp::Time last_battery_update_;
};

}  // namespace fms_robot_agent
