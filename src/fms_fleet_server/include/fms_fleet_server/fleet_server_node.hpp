#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fms_msgs/msg/robot_status.hpp>
#include <fms_msgs/msg/task_assignment.hpp>
#include <fms_msgs/msg/task_completion.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "fms_fleet_server/mongo_store.hpp"
#include "fms_fleet_server/rabbitmq_client.hpp"
#include "fms_fleet_server/fleet_grpc_server.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fms_fleet_server {

class FleetServerNode : public rclcpp::Node {
public:
  explicit FleetServerNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{});
  ~FleetServerNode() override;

  // ── Public API for FleetGrpcServer (Phase 3.5) ────────────────────────────

  struct SubmitResult {
    bool accepted;
    std::string task_id;
    std::string robot_id;
    std::string message;
  };

  // Score IDLE robots against `request` and, if one is available, dispatch
  // the task to it (same allocator used by /fleet/task_request and
  // tasks.incoming).
  SubmitResult submit_task(fms_msgs::msg::TaskAssignment request);

  // Snapshot of the latest known status for every tracked robot.
  std::vector<fms_msgs::msg::RobotStatus> get_fleet_status();

  // Look up a task's lifecycle record from the `tasks` collection.
  std::optional<TaskRecord> get_task_status(const std::string& task_id);

  struct CommandResult {
    bool success;
    std::string message;
  };

  // Forward `command` to `robot_id`. Only "inject_fault" is currently
  // supported (calls that robot's /<robot_id>/inject_fault SetBool
  // service with `value`). Blocks for up to 2s waiting for the robot's
  // response.
  CommandResult send_robot_command(const std::string& robot_id, const std::string& command, bool value);

private:
  void robot_status_cb(const fms_msgs::msg::RobotStatus::SharedPtr msg);
  void task_completion_cb(const fms_msgs::msg::TaskCompletion::SharedPtr msg);
  void task_request_cb(const fms_msgs::msg::TaskAssignment::SharedPtr msg);
  void print_summary_cb();

  // Score IDLE robots against `request` and return the best candidate's
  // robot_id and score, or std::nullopt if no robot is currently IDLE.
  std::optional<std::pair<std::string, double>> select_robot(
      const fms_msgs::msg::TaskAssignment& request);

  // Stamp/route `assignment` to `assignment.robot_id`'s task_assignment topic
  // and record it as "ASSIGNED" in the `tasks` collection. Generates
  // `assignment.task_id` in place if it was empty.
  void dispatch_assignment(fms_msgs::msg::TaskAssignment& assignment);

  // Consumer loop for the "tasks.incoming" RabbitMQ queue, run on its own thread.
  void rabbitmq_consume_loop();

  int num_robots_{4};
  double soc_weight_{0.5};

  std::vector<rclcpp::Subscription<fms_msgs::msg::RobotStatus>::SharedPtr> status_subs_;
  std::vector<rclcpp::Subscription<fms_msgs::msg::TaskCompletion>::SharedPtr> completion_subs_;
  rclcpp::Subscription<fms_msgs::msg::TaskAssignment>::SharedPtr task_request_sub_;
  std::vector<rclcpp::Publisher<fms_msgs::msg::TaskAssignment>::SharedPtr> task_assignment_pubs_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> fault_clients_;
  rclcpp::TimerBase::SharedPtr summary_timer_;

  std::mutex fleet_mutex_;
  std::map<std::string, fms_msgs::msg::RobotStatus> latest_status_;
  // robot_id -> index into task_assignment_pubs_
  std::map<std::string, size_t> robot_index_;

  std::unique_ptr<MongoStore> mongo_store_;

  std::unique_ptr<RabbitMqClient> rabbitmq_;
  std::thread rabbitmq_thread_;
  std::atomic<bool> rabbitmq_running_{false};

  std::unique_ptr<FleetGrpcServer> grpc_server_;
  std::thread grpc_thread_;
};

}  // namespace fms_fleet_server
