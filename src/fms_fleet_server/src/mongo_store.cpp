#include "fms_fleet_server/mongo_store.hpp"
#include "fms_fleet_server/robot_status_utils.hpp"
#include "fms_fleet_server/task_utils.hpp"

#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/options/update.hpp>

using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_document;
using bsoncxx::builder::stream::close_document;

namespace fms_fleet_server {

mongocxx::instance& MongoStore::instance()
{
  // mongocxx::instance must be created exactly once per process and must
  // outlive all clients.
  static mongocxx::instance inst{};
  return inst;
}

MongoStore::MongoStore(const std::string& uri, const std::string& db_name)
: instance_(instance())
, client_(mongocxx::uri{uri})
, db_(client_[db_name])
{
}

namespace {

document robot_status_to_doc(const fms_msgs::msg::RobotStatus& status)
{
  document doc{};
  doc << "robot_id"        << status.robot_id
      << "state"            << static_cast<int32_t>(status.state)
      << "state_str"        << robot_state_to_string(status.state)
      << "battery_soc"      << status.battery_soc
      << "current_task_id"  << status.current_task_id
      << "status_message"   << status.status_message
      << "pose" << open_document
          << "position" << open_document
              << "x" << status.pose.position.x
              << "y" << status.pose.position.y
              << "z" << status.pose.position.z
          << close_document
          << "orientation" << open_document
              << "x" << status.pose.orientation.x
              << "y" << status.pose.orientation.y
              << "z" << status.pose.orientation.z
              << "w" << status.pose.orientation.w
          << close_document
      << close_document
      << "stamp_sec"  << static_cast<int64_t>(status.header.stamp.sec)
      << "stamp_nanosec" << static_cast<int64_t>(status.header.stamp.nanosec);
  return doc;
}

}  // namespace

void MongoStore::upsert_robot(const fms_msgs::msg::RobotStatus& status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto robots = db_["robots"];
  auto filter = document{} << "robot_id" << status.robot_id << finalize;
  auto update = document{} << "$set" << bsoncxx::types::b_document{robot_status_to_doc(status).view()} << finalize;

  mongocxx::options::update opts{};
  opts.upsert(true);
  robots.update_one(filter.view(), update.view(), opts);
}

void MongoStore::insert_telemetry(const fms_msgs::msg::RobotStatus& status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto telemetry = db_["telemetry"];
  telemetry.insert_one(robot_status_to_doc(status).view());
}

void MongoStore::insert_task(const fms_msgs::msg::TaskAssignment& task)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto tasks = db_["tasks"];
  document doc{};
  doc << "task_id"        << task.task_id
      << "robot_id"        << task.robot_id
      << "task_type"       << static_cast<int32_t>(task.task_type)
      << "task_type_str"   << task_type_to_string(task.task_type)
      << "pick_pose" << open_document
          << "x" << task.pick_pose.position.x
          << "y" << task.pick_pose.position.y
          << "z" << task.pick_pose.position.z
      << close_document
      << "drop_pose" << open_document
          << "x" << task.drop_pose.position.x
          << "y" << task.drop_pose.position.y
          << "z" << task.drop_pose.position.z
      << close_document
      << "payload_id"      << task.payload_id
      << "priority"        << static_cast<int32_t>(task.priority)
      << "deadline_secs"   << task.deadline_secs
      << "status"          << "ASSIGNED"
      << "assigned_stamp_sec"     << static_cast<int64_t>(task.header.stamp.sec)
      << "assigned_stamp_nanosec" << static_cast<int64_t>(task.header.stamp.nanosec);
  tasks.insert_one(doc.view());
}

void MongoStore::update_task_completion(const fms_msgs::msg::TaskCompletion& completion)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto tasks = db_["tasks"];
  auto filter = document{} << "task_id" << completion.task_id << finalize;
  auto update = document{} << "$set" << open_document
      << "status"          << task_result_to_string(completion.result)
      << "result"          << static_cast<int32_t>(completion.result)
      << "duration_secs"   << completion.duration_secs
      << "error_message"   << completion.error_message
      << "completed_stamp_sec"     << static_cast<int64_t>(completion.header.stamp.sec)
      << "completed_stamp_nanosec" << static_cast<int64_t>(completion.header.stamp.nanosec)
      << close_document
      << finalize;
  tasks.update_one(filter.view(), update.view());
}

std::optional<TaskRecord> MongoStore::find_task(const std::string& task_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto tasks = db_["tasks"];
  auto filter = document{} << "task_id" << task_id << finalize;
  auto found = tasks.find_one(filter.view());
  if (!found) {
    return std::nullopt;
  }

  auto view = found->view();
  TaskRecord rec;
  rec.task_id  = task_id;
  if (view["robot_id"])     rec.robot_id      = std::string(view["robot_id"].get_string().value);
  if (view["status"])       rec.status        = std::string(view["status"].get_string().value);
  if (view["result"])       rec.result        = view["result"].get_int32().value;
  if (view["duration_secs"]) rec.duration_secs = view["duration_secs"].get_double().value;
  if (view["error_message"]) rec.error_message = std::string(view["error_message"].get_string().value);
  return rec;
}

}  // namespace fms_fleet_server
