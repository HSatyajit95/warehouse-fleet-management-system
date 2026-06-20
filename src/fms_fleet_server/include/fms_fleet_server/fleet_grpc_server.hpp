#pragma once

#include "fleet.grpc.pb.h"
#include <grpcpp/grpcpp.h>

#include <memory>
#include <string>

namespace fms_fleet_server {

class FleetServerNode;

// gRPC service implementation for Phase 3.5. Thin adapter that converts
// proto requests/responses to/from FleetServerNode's public API
// (submit_task / get_fleet_status / get_task_status).
class FleetGrpcServer final : public FleetService::Service {
public:
  FleetGrpcServer(FleetServerNode* node, const std::string& address);

  // Blocks until shutdown() is called from another thread.
  void run();
  void shutdown();

  grpc::Status SubmitTask(grpc::ServerContext* context,
                           const SubmitTaskRequest* request,
                           SubmitTaskResponse* response) override;

  grpc::Status GetFleetStatus(grpc::ServerContext* context,
                               const GetFleetStatusRequest* request,
                               GetFleetStatusResponse* response) override;

  grpc::Status GetTaskStatus(grpc::ServerContext* context,
                              const GetTaskStatusRequest* request,
                              GetTaskStatusResponse* response) override;

  grpc::Status SendRobotCommand(grpc::ServerContext* context,
                                 const SendRobotCommandRequest* request,
                                 SendRobotCommandResponse* response) override;

private:
  FleetServerNode* node_;
  std::string address_;
  std::unique_ptr<grpc::Server> server_;
};

}  // namespace fms_fleet_server
