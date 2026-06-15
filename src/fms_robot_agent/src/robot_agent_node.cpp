#include "fms_robot_agent/robot_agent_node.hpp"
#include "fms_robot_agent/bt_nodes/request_task.hpp"
#include "fms_robot_agent/bt_nodes/navigate_to_pose_bt.hpp"
#include "fms_robot_agent/bt_nodes/execute_pick_drop.hpp"
#include "fms_robot_agent/bt_nodes/report_status.hpp"
#include "fms_robot_agent/bt_nodes/request_recovery.hpp"
#include "fms_robot_agent/bt_nodes/battery_ok.hpp"
#include "fms_robot_agent/bt_nodes/charge_battery.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <chrono>
#include <stdexcept>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

namespace fms_robot_agent {

RobotAgentNode::RobotAgentNode(const rclcpp::NodeOptions& opts)
: rclcpp_lifecycle::LifecycleNode("robot_agent", opts)
{
  declare_parameter("robot_name",           "robot_1");
  declare_parameter("pick_duration_secs",   3.0);
  declare_parameter("drop_duration_secs",   2.0);
  declare_parameter("recovery_max_attempts", 3);
  declare_parameter("initial_battery_soc",  100.0);
  declare_parameter("low_battery_threshold", 20.0);
  // charger_pose as [x, y, yaw_deg]
  declare_parameter("charger_pose", std::vector<double>{-20.0, 0.0, 180.0});
  declare_parameter("bt_xml_path", std::string{""});
}

// ── on_configure ──────────────────────────────────────────────────────────────

CallbackReturn RobotAgentNode::on_configure(const rclcpp_lifecycle::State&) {
  robot_name_             = get_parameter("robot_name").as_string();
  pick_duration_secs_     = get_parameter("pick_duration_secs").as_double();
  drop_duration_secs_     = get_parameter("drop_duration_secs").as_double();
  recovery_max_attempts_  = get_parameter("recovery_max_attempts").as_int();
  double initial_soc      = get_parameter("initial_battery_soc").as_double();
  battery_.set_soc(initial_soc);

  auto cp = get_parameter("charger_pose").as_double_array();
  charger_pose_.header.frame_id = "map";
  charger_pose_.pose.position.x = cp.size() > 0 ? cp[0] : -20.0;
  charger_pose_.pose.position.y = cp.size() > 1 ? cp[1] :   0.0;
  charger_pose_.pose.position.z = 0.0;
  double yaw_deg = cp.size() > 2 ? cp[2] : 180.0;
  double yaw_rad = yaw_deg * M_PI / 180.0;
  charger_pose_.pose.orientation.z = std::sin(yaw_rad / 2.0);
  charger_pose_.pose.orientation.w = std::cos(yaw_rad / 2.0);

  // Callback groups
  cb_group_timer_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_action_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Action clients
  nav_client_ = rclcpp_action::create_client<NavAction>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "/" + robot_name_ + "/navigate_to_pose", cb_group_action_);

  spin_client_ = rclcpp_action::create_client<SpinAction>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "/" + robot_name_ + "/spin", cb_group_action_);

  backup_client_ = rclcpp_action::create_client<BackupAction>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "/" + robot_name_ + "/backup", cb_group_action_);

  // Publishers (created in configure, activated in activate)
  status_pub_ = create_publisher<fms_msgs::msg::RobotStatus>(
      "/" + robot_name_ + "/robot_status", rclcpp::QoS(10));
  completion_pub_ = create_publisher<fms_msgs::msg::TaskCompletion>(
      "/" + robot_name_ + "/task_completion", rclcpp::QoS(10));

  // Subscription (no lifecycle needed — tasks can arrive before activation)
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_timer_;
  task_sub_ = create_subscription<fms_msgs::msg::TaskAssignment>(
      "/" + robot_name_ + "/task_assignment", rclcpp::QoS(10),
      [this](const fms_msgs::msg::TaskAssignment::SharedPtr msg) {
        task_assignment_cb(msg);
      },
      sub_opts);

  // Fault injection service
  fault_srv_ = create_service<std_srvs::srv::SetBool>(
      "/" + robot_name_ + "/inject_fault",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        fault_service_cb(req, res);
      },
      rmw_qos_profile_services_default,
      cb_group_timer_);

  // Build BT
  register_bt_nodes();

  std::string bt_xml = get_parameter("bt_xml_path").as_string();
  if (bt_xml.empty()) {
    bt_xml = ament_index_cpp::get_package_share_directory("fms_robot_agent")
             + "/bt_xml/robot_agent.xml";
  }

  blackboard_ = BT::Blackboard::create();
  blackboard_->set("robot_name",   robot_name_);
  blackboard_->set("charger_pose", charger_pose_);
  blackboard_->set("battery_soc",  battery_.soc());
  blackboard_->set("pick_duration",  pick_duration_secs_);
  blackboard_->set("drop_duration",  drop_duration_secs_);

  try {
    bt_tree_ = std::make_unique<BT::Tree>(
        factory_.createTreeFromFile(bt_xml, blackboard_));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load BT XML: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  last_battery_update_ = get_clock()->now();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "[%s] Configured. SOC=%.0f%%", robot_name_.c_str(), battery_.soc());
  return CallbackReturn::SUCCESS;
}

// ── on_activate ──────────────────────────────────────────────────────────────

CallbackReturn RobotAgentNode::on_activate(const rclcpp_lifecycle::State&) {
  status_pub_->on_activate();
  completion_pub_->on_activate();

  task_start_time_ = get_clock()->now();

  bt_timer_ = create_wall_timer(50ms,
      [this]() { bt_tick_cb(); }, cb_group_timer_);

  battery_timer_ = create_wall_timer(100ms,
      [this]() { battery_update_cb(); }, cb_group_timer_);

  status_timer_ = create_wall_timer(500ms,
      [this]() { status_publish_cb(); }, cb_group_timer_);

  RCLCPP_INFO(get_logger(), "[%s] Active — BT running.", robot_name_.c_str());
  return CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────────

CallbackReturn RobotAgentNode::on_deactivate(const rclcpp_lifecycle::State&) {
  bt_timer_.reset();
  battery_timer_.reset();
  status_timer_.reset();
  status_pub_->on_deactivate();
  completion_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated.", robot_name_.c_str());
  return CallbackReturn::SUCCESS;
}

// ── on_cleanup ────────────────────────────────────────────────────────────────

CallbackReturn RobotAgentNode::on_cleanup(const rclcpp_lifecycle::State&) {
  bt_tree_.reset();
  task_sub_.reset();
  nav_client_.reset();
  spin_client_.reset();
  backup_client_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up.", robot_name_.c_str());
  return CallbackReturn::SUCCESS;
}

// ── on_shutdown ───────────────────────────────────────────────────────────────

CallbackReturn RobotAgentNode::on_shutdown(const rclcpp_lifecycle::State&) {
  bt_tree_.reset();
  return CallbackReturn::SUCCESS;
}

// ── Private callbacks ─────────────────────────────────────────────────────────

void RobotAgentNode::bt_tick_cb() {
  if (!bt_tree_) return;

  // Update battery_soc on the blackboard each tick for BatteryOK to read.
  blackboard_->set("battery_soc", battery_.soc());

  try {
    bt_tree_->tickRoot();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "[BT tick] Exception: %s", e.what());
  }
}

void RobotAgentNode::battery_update_cb() {
  auto now = get_clock()->now();
  double dt = (now - last_battery_update_).seconds();
  last_battery_update_ = now;

  battery_.update(dt, is_moving_.load(), is_charging_.load());

  // Save interrupted task when battery goes low mid-task
  if (battery_.is_low()) {
    std::lock_guard<std::mutex> lk(task_mutex_);
    if (current_task_.has_value() && !interrupted_task_.has_value()) {
      interrupted_task_ = std::move(current_task_);
      current_task_.reset();
      RCLCPP_WARN(get_logger(), "[Battery] Low (%.1f%%) — task %s interrupted.",
                  battery_.soc(), interrupted_task_->task_id.c_str());
    }
  }
}

void RobotAgentNode::status_publish_cb() {
  if (!status_pub_ || !status_pub_->is_activated()) return;

  fms_msgs::msg::RobotStatus msg;
  msg.header.stamp = get_clock()->now();
  msg.robot_id     = robot_name_;
  msg.battery_soc  = static_cast<float>(battery_.soc());

  {
    std::lock_guard<std::mutex> lk(fsm_mutex_);
    msg.state = fsm_.state_id();
  }
  {
    std::lock_guard<std::mutex> lk(task_mutex_);
    if (current_task_.has_value()) {
      msg.current_task_id = current_task_->task_id;
    }
  }
  msg.status_message = fsm_.state_name();

  try {
    auto tf = tf_buffer_->lookupTransform(
        "map", robot_name_ + "/base_footprint", tf2::TimePointZero);
    msg.pose.position.x = tf.transform.translation.x;
    msg.pose.position.y = tf.transform.translation.y;
    msg.pose.position.z = tf.transform.translation.z;
    msg.pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[%s] Could not look up map -> %s/base_footprint: %s",
        robot_name_.c_str(), robot_name_.c_str(), e.what());
  }

  status_pub_->publish(msg);
}

void RobotAgentNode::task_assignment_cb(
    const fms_msgs::msg::TaskAssignment::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(task_mutex_);
  if (pending_task_.has_value()) {
    RCLCPP_WARN(get_logger(), "[%s] Dropping queued task %s (new task %s arrived).",
                robot_name_.c_str(),
                pending_task_->task_id.c_str(),
                msg->task_id.c_str());
  }
  pending_task_ = *msg;
  RCLCPP_INFO(get_logger(), "[%s] Task %s received.", robot_name_.c_str(), msg->task_id.c_str());
}

bool RobotAgentNode::fault_service_cb(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    fault_injected_.store(true);
    RCLCPP_WARN(get_logger(), "[%s] Fault injected via service.", robot_name_.c_str());
    res->success = true;
    res->message = "Fault injected";
  } else {
    res->success = true;
    res->message = "No fault (data=false)";
  }
  return true;
}

// ── BT node registration ──────────────────────────────────────────────────────

void RobotAgentNode::register_bt_nodes() {
  auto* self = this;

  factory_.registerBuilder<bt::RequestTask>("RequestTask",
      [self](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<bt::RequestTask>(name, config, self);
      });

  factory_.registerBuilder<bt::NavigateToPoseBT>("NavigateToPose",
      [self](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<bt::NavigateToPoseBT>(name, config, self);
      });

  factory_.registerBuilder<bt::ExecutePickDrop>("ExecutePickDrop",
      [self](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<bt::ExecutePickDrop>(name, config, self);
      });

  factory_.registerBuilder<bt::ReportStatus>("ReportStatus",
      [self](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<bt::ReportStatus>(name, config, self);
      });

  factory_.registerBuilder<bt::RequestRecovery>("RequestRecovery",
      [self](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<bt::RequestRecovery>(name, config, self);
      });

  factory_.registerBuilder<bt::BatteryOK>("BatteryOK",
      [self](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<bt::BatteryOK>(name, config, self);
      });

  factory_.registerBuilder<bt::ChargeBattery>("ChargeBattery",
      [self](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<bt::ChargeBattery>(name, config, self);
      });
}

}  // namespace fms_robot_agent
