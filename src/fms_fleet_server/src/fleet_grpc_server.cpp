#include "fms_fleet_server/fleet_grpc_server.hpp"
#include "fms_fleet_server/fleet_server_node.hpp"
#include "fms_fleet_server/robot_status_utils.hpp"

#include <grpcpp/grpcpp.h>

namespace fms_fleet_server {

FleetGrpcServer::FleetGrpcServer(FleetServerNode* node, const std::string& address)
: node_(node), address_(address)
{
}

void FleetGrpcServer::run()
{
  grpc::ServerBuilder builder;
  builder.AddListeningPort(address_, grpc::InsecureServerCredentials());
  builder.RegisterService(this);
  server_ = builder.BuildAndStart();
  server_->Wait();
}

void FleetGrpcServer::shutdown()
{
  if (server_) {
    server_->Shutdown();
  }
}

grpc::Status FleetGrpcServer::SubmitTask(grpc::ServerContext* /*context*/,
                                          const SubmitTaskRequest* request,
                                          SubmitTaskResponse* response)
{
  fms_msgs::msg::TaskAssignment task;
  task.task_type = static_cast<uint8_t>(request->task_type());
  task.pick_pose.position.x = request->pick_pose().x();
  task.pick_pose.position.y = request->pick_pose().y();
  task.pick_pose.orientation.w = 1.0;
  task.drop_pose.position.x = request->drop_pose().x();
  task.drop_pose.position.y = request->drop_pose().y();
  task.drop_pose.orientation.w = 1.0;
  task.payload_id = request->payload_id();
  task.priority = static_cast<uint8_t>(request->priority());
  task.deadline_secs = static_cast<float>(request->deadline_secs());

  auto result = node_->submit_task(std::move(task));

  response->set_accepted(result.accepted);
  response->set_task_id(result.task_id);
  response->set_robot_id(result.robot_id);
  response->set_message(result.message);
  return grpc::Status::OK;
}

grpc::Status FleetGrpcServer::GetFleetStatus(grpc::ServerContext* /*context*/,
                                              const GetFleetStatusRequest* /*request*/,
                                              GetFleetStatusResponse* response)
{
  for (const auto& status : node_->get_fleet_status()) {
    auto* info = response->add_robots();
    info->set_robot_id(status.robot_id);
    info->set_state(status.state);
    info->set_state_str(robot_state_to_string(status.state));
    info->set_battery_soc(status.battery_soc);
    info->set_current_task_id(status.current_task_id);
    info->mutable_pose()->set_x(status.pose.position.x);
    info->mutable_pose()->set_y(status.pose.position.y);
  }
  return grpc::Status::OK;
}

grpc::Status FleetGrpcServer::GetTaskStatus(grpc::ServerContext* /*context*/,
                                             const GetTaskStatusRequest* request,
                                             GetTaskStatusResponse* response)
{
  auto task = node_->get_task_status(request->task_id());
  if (!task) {
    response->set_found(false);
    response->set_task_id(request->task_id());
    return grpc::Status::OK;
  }

  response->set_found(true);
  response->set_task_id(task->task_id);
  response->set_robot_id(task->robot_id);
  response->set_status(task->status);
  response->set_result(task->result);
  response->set_duration_secs(task->duration_secs);
  response->set_error_message(task->error_message);
  return grpc::Status::OK;
}

grpc::Status FleetGrpcServer::SendRobotCommand(grpc::ServerContext* /*context*/,
                                                const SendRobotCommandRequest* request,
                                                SendRobotCommandResponse* response)
{
  auto result = node_->send_robot_command(request->robot_id(), request->command(), request->value());
  response->set_success(result.success);
  response->set_message(result.message);
  return grpc::Status::OK;
}

}  // namespace fms_fleet_server
