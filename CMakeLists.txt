cmake_minimum_required(VERSION 3.0.2)
project(uav_search)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  airsim_ros_pkgs
)

catkin_package(CATKIN_DEPENDS
    roscpp
    geometry_msgs
    airsim_ros_pkgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(airsim_bridge_node src/airsim_bridge_node.cpp)

target_link_libraries(airsim_bridge_node
  ${catkin_LIBRARIES}
)
