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

# add all launch files to share directory
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## EXECUTABLES
# airsim bridge to connect twist_teleop_keyboard
add_executable(airsim_bridge_node src/airsim_bridge_node.cpp)
target_link_libraries(airsim_bridge_node
  ${catkin_LIBRARIES}
)

# custom drone teleop
add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop
  ${catkin_LIBRARIES}
)