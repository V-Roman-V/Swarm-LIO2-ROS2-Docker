cmake_minimum_required(VERSION 2.8.3)
project(swarm_msgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        message_generation
        )
add_message_files(
        DIRECTORY msg
        FILES
        GlobalExtrinsic.msg
        GlobalExtrinsicStatus.msg
        QuadStatePub.msg
        ObserveTeammate.msg
        TeamStatus.msg
        TeammateInfo.msg
        SpatialTemporalOffset.msg
        SpatialTemporalOffsetStatus.msg
        ConnectedTeammateList.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime std_msgs geometry_msgs nav_msgs
)


include_directories(
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)
