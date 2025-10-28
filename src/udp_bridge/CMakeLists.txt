cmake_minimum_required(VERSION 3.8)
project(udp_bridge)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread)
find_package(fmt REQUIRED)

include_directories(
        SYSTEM
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${fmt_INCLUDE_DIRS}
)

add_executable(udp_online
        src/udp_online.cpp
        )
target_link_libraries(udp_online
        Eigen3::Eigen
        Boost::filesystem
        Boost::thread
        fmt::fmt
)
ament_target_dependencies(udp_online
        rclcpp
        std_msgs
        sensor_msgs
        swarm_msgs
        ament_index_cpp
)

install(TARGETS udp_online
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_package()
