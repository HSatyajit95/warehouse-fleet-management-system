#pragma once

#include <fms_msgs/msg/robot_status.hpp>
#include <fms_msgs/msg/task_assignment.hpp>
#include <fms_msgs/msg/task_completion.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace fms_fleet_server {

// Subset of a `tasks` collection document needed to answer GetTaskStatus.
struct TaskRecord {
  std::string task_id;
  std::string robot_id;
  std::string status;
  int32_t     result{0};
  double      duration_secs{0.0};
  std::string error_message;
};

// Wraps a MongoDB connection and the `robots` / `telemetry` / `tasks`
// collections used by the fleet server. One instance per process.
class MongoStore {
public:
  MongoStore(const std::string& uri, const std::string& db_name);

  // Upsert the robot's latest known status into the `robots` collection.
  void upsert_robot(const fms_msgs::msg::RobotStatus& status);

  // Append a snapshot of the robot's status to the `telemetry` collection.
  void insert_telemetry(const fms_msgs::msg::RobotStatus& status);

  // Insert a new `tasks` doc with status "ASSIGNED" for a freshly allocated task.
  void insert_task(const fms_msgs::msg::TaskAssignment& task);

  // Update the matching `tasks` doc (by task_id) with its final result.
  void update_task_completion(const fms_msgs::msg::TaskCompletion& completion);

  // Look up a `tasks` doc by task_id. Returns std::nullopt if not found.
  std::optional<TaskRecord> find_task(const std::string& task_id);

private:
  static mongocxx::instance& instance();

  // Order matters: instance() must run before client_ is constructed.
  mongocxx::instance& instance_;
  mongocxx::client client_;
  mongocxx::database db_;

  // mongocxx::client is not thread-safe; the ROS executor thread, the
  // RabbitMQ consumer thread, and the gRPC server thread all call into
  // this store concurrently, so every operation is serialized.
  std::mutex mutex_;
};

}  // namespace fms_fleet_server
