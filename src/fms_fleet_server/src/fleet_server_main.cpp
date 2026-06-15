#include <rclcpp/rclcpp.hpp>
#include "fms_fleet_server/fleet_server_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fms_fleet_server::FleetServerNode>());
  rclcpp::shutdown();
  return 0;
}
