cmake_minimum_required(VERSION 3.8)
project(fms_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  behavior_trees
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/warehouse_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
