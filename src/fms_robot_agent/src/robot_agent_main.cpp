#include "fms_robot_agent/robot_agent_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<fms_robot_agent::RobotAgentNode>();

  // Use MultiThreadedExecutor: timer callbacks (BT, battery, status) run in
  // cb_group_timer_, action response callbacks run in cb_group_action_.
  // Two threads are sufficient: one per callback group.
  rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions{}, 2 /*threads*/);
  exec.add_node(node->get_node_base_interface());

  // Auto-configure and activate for convenience in Phase 2 (no external manager).
  node->configure();
  node->activate();

  exec.spin();

  // Graceful shutdown: deactivate first so lifecycle publishers are still valid,
  // then cleanup and shutdown before rclcpp context is destroyed.
  if (rclcpp::ok()) {
    node->deactivate();
    node->cleanup();
    node->shutdown();
  }

  rclcpp::shutdown();
  return 0;
}
