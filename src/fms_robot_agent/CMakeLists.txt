cmake_minimum_required(VERSION 3.16)
project(fms_robot_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fms_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPS
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  std_msgs
  std_srvs
  fms_msgs
  tf2_ros
)

add_library(fms_agent_lib SHARED
  src/robot_fsm.cpp
  src/battery_model.cpp
  src/bt_nodes/request_task.cpp
  src/bt_nodes/navigate_to_pose_bt.cpp
  src/bt_nodes/execute_pick_drop.cpp
  src/bt_nodes/report_status.cpp
  src/bt_nodes/request_recovery.cpp
  src/bt_nodes/battery_ok.cpp
  src/bt_nodes/charge_battery.cpp
  src/robot_agent_node.cpp
)

target_include_directories(fms_agent_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fms_agent_lib ${DEPS})

add_executable(robot_agent_node src/robot_agent_main.cpp)
target_link_libraries(robot_agent_node fms_agent_lib)
ament_target_dependencies(robot_agent_node rclcpp rclcpp_lifecycle)

install(TARGETS fms_agent_lib robot_agent_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config bt_xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/task_injector.py
  scripts/task_request.py
  scripts/task_request_amqp.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_fsm test/test_robot_fsm.cpp)
  target_link_libraries(test_robot_fsm fms_agent_lib)

  ament_add_gtest(test_battery_model test/test_battery_model.cpp)
  target_link_libraries(test_battery_model fms_agent_lib)
endif()

ament_package()
