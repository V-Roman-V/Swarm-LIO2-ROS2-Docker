cmake_minimum_required(VERSION 3.5)
project(swarm_lio)

set(CMAKE_BUILD_TYPE "Release")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(OpenMP REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(swarm_msgs REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  include
)

rosidl_generate_interfaces(swarm_lio_interfaces
  "msg/Pose6D.msg"
  "msg/States.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(swarm_lio
        src/laserMapping.cpp
        include/ikd-Tree/ikd_Tree.cpp
        src/preprocess.cpp
        src/MultiUAV.cpp
        )

ament_target_dependencies(swarm_lio
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  swarm_msgs
)

target_link_libraries(swarm_lio
  ${PCL_LIBRARIES}
  TBB::tbb
  gtsam
  Python3::Python
  OpenMP::OpenMP_CXX
)

target_include_directories(swarm_lio PRIVATE
  ${Python3_INCLUDE_DIRS}
  ${swarm_msgs_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../../swarm_msgs/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS swarm_lio
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_target_interfaces(swarm_lio
  swarm_lio_interfaces
  "rosidl_typesupport_cpp"
)

ament_export_dependencies(
  geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2 tf2_ros pcl_ros pcl_conversions
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY mesh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
