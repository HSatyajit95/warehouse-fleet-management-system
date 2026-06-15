cmake_minimum_required(VERSION 3.16)
project(fms_fleet_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fms_msgs REQUIRED)
find_package(mongocxx REQUIRED)
find_package(bsoncxx REQUIRED)
find_package(nlohmann_json REQUIRED)

find_path(RABBITMQ_INCLUDE_DIR amqp.h)
find_library(RABBITMQ_LIBRARY rabbitmq)
if(NOT RABBITMQ_INCLUDE_DIR OR NOT RABBITMQ_LIBRARY)
  message(FATAL_ERROR "librabbitmq (rabbitmq-c) not found. Install librabbitmq-dev.")
endif()

# ── Protobuf / gRPC (Phase 3.5) ──────────────────────────────────────────────
find_package(Protobuf REQUIRED)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)
find_library(GRPCPP_LIBRARY grpc++ REQUIRED)
find_library(GRPC_LIBRARY grpc REQUIRED)
find_library(GPR_LIBRARY gpr REQUIRED)

set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/proto/fleet.proto)
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto-gen)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_SRCS ${PROTO_GEN_DIR}/fleet.pb.cc)
set(PROTO_HDRS ${PROTO_GEN_DIR}/fleet.pb.h)
set(GRPC_SRCS  ${PROTO_GEN_DIR}/fleet.grpc.pb.cc)
set(GRPC_HDRS  ${PROTO_GEN_DIR}/fleet.grpc.pb.h)

add_custom_command(
  OUTPUT ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS --grpc_out=${PROTO_GEN_DIR}
       --cpp_out=${PROTO_GEN_DIR}
       --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
       -I ${CMAKE_CURRENT_SOURCE_DIR}/proto
       ${PROTO_FILE}
  DEPENDS ${PROTO_FILE}
)

set(DEPS
  rclcpp
  geometry_msgs
  std_msgs
  fms_msgs
)

add_library(fms_fleet_lib SHARED
  src/fleet_server_node.cpp
  src/mongo_store.cpp
  src/rabbitmq_client.cpp
  src/fleet_grpc_server.cpp
  ${PROTO_SRCS}
  ${GRPC_SRCS}
)

target_include_directories(fms_fleet_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RABBITMQ_INCLUDE_DIR}
  ${PROTO_GEN_DIR}
)

ament_target_dependencies(fms_fleet_lib ${DEPS})
target_link_libraries(fms_fleet_lib mongo::mongocxx_shared mongo::bsoncxx_shared
  nlohmann_json::nlohmann_json ${RABBITMQ_LIBRARY}
  ${GRPCPP_LIBRARY} ${GRPC_LIBRARY} ${GPR_LIBRARY} ${Protobuf_LIBRARIES})

add_executable(fleet_server_node src/fleet_server_main.cpp)
target_link_libraries(fleet_server_node fms_fleet_lib)
ament_target_dependencies(fleet_server_node rclcpp)

install(TARGETS fms_fleet_lib fleet_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
