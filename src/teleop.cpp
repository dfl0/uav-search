#include <ros/ros.h>
#include <airsim_ros_pkgs/VelCmd.h>

#include <iostream>

int main(int argc, char** argv){
    // init ros
    ros::init(argc, argv, "drone_teleop");
    ros::NodeHandle n;

    // topics
    std::cout << "Hello world!\n";
}