cmake_minimum_required(VERSION 3.8)
project(swarm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConnectedTeammateList.msg"
  "msg/GlobalExtrinsic.msg"
  "msg/GlobalExtrinsicStatus.msg"
  "msg/ObserveTeammate.msg"
  "msg/QuadStatePub.msg"
  "msg/SpatialTemporalOffset.msg"
  "msg/SpatialTemporalOffsetStatus.msg"
  "msg/TeammateInfo.msg"
  "msg/TeamStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_include_directories(
  "include"
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c"
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
