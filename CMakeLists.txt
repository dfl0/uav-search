cmake_minimum_required(VERSION 3.0.2)
project(uav_search)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf geometry_msgs
  depth_image_proc
  airsim_ros_pkgs
)

find_package(octomap REQUIRED)

catkin_package(CATKIN_DEPENDS
    roscpp
    tf
    geometry_msgs
    depth_image_proc
    airsim_ros_pkgs

    DEPENDS octomap
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

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
