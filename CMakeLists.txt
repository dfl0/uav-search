cmake_minimum_required(VERSION 3.0.2)
project(uav_search)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf geometry_msgs nav_msgs visualization_msgs
  depth_image_proc
  airsim_ros_pkgs
)

find_package(octomap REQUIRED)

catkin_package(CATKIN_DEPENDS
    roscpp
    tf
    geometry_msgs
    nav_msgs
    visualization_msgs
    depth_image_proc
    airsim_ros_pkgs
    DEPENDS octomap
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# INSTALL
# add python scripts
catkin_install_python(PROGRAMS
  scripts/yolo_node.py
  scripts/static_image_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add all launch files to share directory
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# EXECUTABLES
add_executable(airsim_bridge_node src/airsim_bridge_node.cpp)
target_link_libraries(airsim_bridge_node
  ${catkin_LIBRARIES}
)

add_executable(coordinator_node src/coordinator_node.cpp)
target_link_libraries(coordinator_node
    ${catkin_LIBRARIES}
)

add_executable(drone_controller_node src/drone_controller_node.cpp)
target_link_libraries(drone_controller_node
    ${catkin_LIBRARIES}
)