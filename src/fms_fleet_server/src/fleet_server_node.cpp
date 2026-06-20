#include "fms_fleet_server/fleet_server_node.hpp"
#include "fms_fleet_server/robot_status_utils.hpp"
#include "fms_fleet_server/task_allocator.hpp"
#include "fms_fleet_server/task_utils.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace std::chrono_literals;

namespace fms_fleet_server {

FleetServerNode::FleetServerNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("fleet_server", opts)
{
  num_robots_ = declare_parameter<int>("num_robots", 4);
  soc_weight_  = declare_parameter<double>("soc_weight", 0.5);
  const std::string mongo_uri = declare_parameter<std::string>("mongo_uri", "mongodb://localhost:27017");
  const std::string mongo_db  = declare_parameter<std::string>("mongo_db", "fms");

  mongo_store_ = std::make_unique<MongoStore>(mongo_uri, mongo_db);

  for (int i = 1; i <= num_robots_; ++i) {
    const std::string robot_name = "robot_" + std::to_string(i);
    robot_index_[robot_name] = static_cast<size_t>(i - 1);

    status_subs_.push_back(create_subscription<fms_msgs::msg::RobotStatus>(
        "/" + robot_name + "/robot_status", rclcpp::QoS(10),
        [this](const fms_msgs::msg::RobotStatus::SharedPtr msg) {
          robot_status_cb(msg);
        }));

    completion_subs_.push_back(create_subscription<fms_msgs::msg::TaskCompletion>(
        "/" + robot_name + "/task_completion", rclcpp::QoS(10),
        [this](const fms_msgs::msg::TaskCompletion::SharedPtr msg) {
          task_completion_cb(msg);
        }));

    task_assignment_pubs_.push_back(create_publisher<fms_msgs::msg::TaskAssignment>(
        "/" + robot_name + "/task_assignment", rclcpp::QoS(10)));

    fault_clients_[robot_name] = create_client<std_srvs::srv::SetBool>(
        "/" + robot_name + "/inject_fault");
  }

  task_request_sub_ = create_subscription<fms_msgs::msg::TaskAssignment>(
      "/fleet/task_request", rclcpp::QoS(10),
      [this](const fms_msgs::msg::TaskAssignment::SharedPtr msg) {
        task_request_cb(msg);
      });

  summary_timer_ = create_wall_timer(2s, std::bind(&FleetServerNode::print_summary_cb, this));

  const std::string rabbitmq_host = declare_parameter<std::string>("rabbitmq_host", "localhost");
  const int rabbitmq_port          = declare_parameter<int>("rabbitmq_port", 5672);
  const std::string rabbitmq_user  = declare_parameter<std::string>("rabbitmq_user", "guest");
  const std::string rabbitmq_pass  = declare_parameter<std::string>("rabbitmq_password", "guest");

  rabbitmq_ = std::make_unique<RabbitMqClient>(rabbitmq_host, rabbitmq_port, rabbitmq_user, rabbitmq_pass);
  rabbitmq_->setup_topology();
  rabbitmq_running_ = true;
  rabbitmq_thread_ = std::thread(&FleetServerNode::rabbitmq_consume_loop, this);

  const int grpc_port = declare_parameter<int>("grpc_port", 50051);
  const std::string grpc_address = "0.0.0.0:" + std::to_string(grpc_port);
  grpc_server_ = std::make_unique<FleetGrpcServer>(this, grpc_address);
  grpc_thread_ = std::thread(&FleetGrpcServer::run, grpc_server_.get());

  RCLCPP_INFO(get_logger(), "fleet_server started, tracking %d robot(s), gRPC on %s",
              num_robots_, grpc_address.c_str());
}

FleetServerNode::~FleetServerNode()
{
  rabbitmq_running_ = false;
  if (rabbitmq_thread_.joinable()) {
    rabbitmq_thread_.join();
  }

  if (grpc_server_) {
    grpc_server_->shutdown();
  }
  if (grpc_thread_.joinable()) {
    grpc_thread_.join();
  }
}

FleetServerNode::SubmitResult FleetServerNode::submit_task(fms_msgs::msg::TaskAssignment request)
{
  auto best = select_robot(request);
  if (!best) {
    return SubmitResult{false, "", "", "No IDLE robot available"};
  }
  const auto& [best_robot, best_score] = *best;

  request.robot_id = best_robot;
  dispatch_assignment(request);

  RCLCPP_INFO(get_logger(), "grpc SubmitTask: assigned task %s to %s (score=%.2f)",
              request.task_id.c_str(), best_robot.c_str(), best_score);

  return SubmitResult{true, request.task_id, best_robot, "Assigned"};
}

std::vector<fms_msgs::msg::RobotStatus> FleetServerNode::get_fleet_status()
{
  std::lock_guard<std::mutex> lock(fleet_mutex_);
  std::vector<fms_msgs::msg::RobotStatus> result;
  result.reserve(latest_status_.size());
  for (const auto& [robot_id, status] : latest_status_) {
    result.push_back(status);
  }
  return result;
}

std::optional<TaskRecord> FleetServerNode::get_task_status(const std::string& task_id)
{
  return mongo_store_->find_task(task_id);
}

FleetServerNode::CommandResult FleetServerNode::send_robot_command(
    const std::string& robot_id, const std::string& command, bool value)
{
  if (command != "inject_fault") {
    return CommandResult{false, "unsupported command: " + command};
  }

  auto it = fault_clients_.find(robot_id);
  if (it == fault_clients_.end()) {
    return CommandResult{false, "unknown robot_id: " + robot_id};
  }

  auto client = it->second;
  if (!client->service_is_ready()) {
    return CommandResult{false, robot_id + "'s inject_fault service is not available"};
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = value;

  auto future = client->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    return CommandResult{false, robot_id + " did not respond to inject_fault within 2s"};
  }

  auto response = future.get();
  return CommandResult{response->success, response->message};
}

void FleetServerNode::robot_status_cb(const fms_msgs::msg::RobotStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(fleet_mutex_);
    latest_status_[msg->robot_id] = *msg;
  }

  mongo_store_->upsert_robot(*msg);
  mongo_store_->insert_telemetry(*msg);
}

void FleetServerNode::task_completion_cb(const fms_msgs::msg::TaskCompletion::SharedPtr msg)
{
  const char* result_str =
      msg->result == fms_msgs::msg::TaskCompletion::RESULT_SUCCESS   ? "SUCCESS" :
      msg->result == fms_msgs::msg::TaskCompletion::RESULT_FAILED    ? "FAILED" :
                                                                         "CANCELLED";
  RCLCPP_INFO(get_logger(), "task_completion: robot=%s task=%s result=%s duration=%.2fs",
              msg->robot_id.c_str(), msg->task_id.c_str(), result_str, msg->duration_secs);

  mongo_store_->update_task_completion(*msg);

  if (msg->result != fms_msgs::msg::TaskCompletion::RESULT_SUCCESS) {
    nlohmann::json doc;
    doc["task_id"]        = msg->task_id;
    doc["robot_id"]       = msg->robot_id;
    doc["result"]         = task_result_to_string(msg->result);
    doc["duration_secs"]  = msg->duration_secs;
    doc["error_message"]  = msg->error_message;
    rabbitmq_->publish_failed(doc.dump());
  }
}

std::optional<std::pair<std::string, double>> FleetServerNode::select_robot(
    const fms_msgs::msg::TaskAssignment& request)
{
  std::lock_guard<std::mutex> lock(fleet_mutex_);
  return select_best_robot(latest_status_, request, soc_weight_);
}

void FleetServerNode::dispatch_assignment(fms_msgs::msg::TaskAssignment& assignment)
{
  if (assignment.task_id.empty()) {
    assignment.task_id = "task_" + std::to_string(now().nanoseconds());
  }
  assignment.header.stamp = now();

  task_assignment_pubs_[robot_index_.at(assignment.robot_id)]->publish(assignment);
  mongo_store_->insert_task(assignment);
}

void FleetServerNode::task_request_cb(const fms_msgs::msg::TaskAssignment::SharedPtr msg)
{
  auto best = select_robot(*msg);
  if (!best) {
    RCLCPP_WARN(get_logger(), "task_request: no IDLE robot available, dropping task %s",
                msg->task_id.c_str());
    return;
  }
  const auto& [best_robot, best_score] = *best;

  fms_msgs::msg::TaskAssignment assignment = *msg;
  assignment.robot_id = best_robot;
  dispatch_assignment(assignment);

  RCLCPP_INFO(get_logger(), "task_request: assigned task %s to %s (score=%.2f)",
              assignment.task_id.c_str(), best_robot.c_str(), best_score);
}

void FleetServerNode::rabbitmq_consume_loop()
{
  while (rabbitmq_running_) {
    RabbitMqClient::Message message;
    bool got = false;
    try {
      got = rabbitmq_->consume_incoming(message, 1);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "rabbitmq consume error: %s", e.what());
      continue;
    }
    if (!got) {
      continue;
    }

    fms_msgs::msg::TaskAssignment request;
    try {
      auto json = nlohmann::json::parse(message.body);
      request.task_id   = json.value("task_id", "");
      request.task_type = json.value("task_type", 0);
      request.pick_pose.position.x = json.at("pick").at("x").get<double>();
      request.pick_pose.position.y = json.at("pick").at("y").get<double>();
      request.drop_pose.position.x = json.at("drop").at("x").get<double>();
      request.drop_pose.position.y = json.at("drop").at("y").get<double>();
      request.pick_pose.orientation.w = 1.0;
      request.drop_pose.orientation.w = 1.0;
      request.priority      = json.value("priority", 1);
      request.deadline_secs = json.value("deadline_secs", 300.0f);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "tasks.incoming: invalid message (%s), rejecting", e.what());
      rabbitmq_->reject(message.delivery_tag, false);
      continue;
    }

    auto best = select_robot(request);
    if (!best) {
      RCLCPP_WARN(get_logger(), "tasks.incoming: no IDLE robot available for task %s, "
                  "routing to tasks.failed", request.task_id.c_str());
      rabbitmq_->reject(message.delivery_tag, false);
      continue;
    }
    const auto& [best_robot, best_score] = *best;

    request.robot_id = best_robot;
    dispatch_assignment(request);
    rabbitmq_->ack(message.delivery_tag);

    RCLCPP_INFO(get_logger(), "tasks.incoming: assigned task %s to %s (score=%.2f)",
                request.task_id.c_str(), best_robot.c_str(), best_score);
  }
}

void FleetServerNode::print_summary_cb()
{
  std::lock_guard<std::mutex> lock(fleet_mutex_);

  if (latest_status_.empty()) {
    RCLCPP_INFO(get_logger(), "fleet summary: no robot status received yet");
    return;
  }

  std::ostringstream oss;
  oss << "fleet summary:\n";
  oss << std::left << std::setw(10) << "robot" << std::setw(12) << "state"
      << std::setw(8) << "SOC%" << std::setw(16) << "task" << "\n";
  for (const auto& [robot_id, status] : latest_status_) {
    oss << std::left << std::setw(10) << robot_id
        << std::setw(12) << robot_state_to_string(status.state)
        << std::setw(8) << std::fixed << std::setprecision(1) << status.battery_soc
        << std::setw(16) << (status.current_task_id.empty() ? "-" : status.current_task_id)
        << "\n";
  }
  RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
}

}  // namespace fms_fleet_server
