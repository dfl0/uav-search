#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <airsim_ros_pkgs/VelCmd.h>

class AirSimBridge {
public:
    AirSimBridge() {
        ros::NodeHandle nh_private("~");
        std::string drone_name;
        nh_private.param<std::string>("drone_name", drone_name, "drone_1");

        sub_ = nh_.subscribe("/cmd_vel", 1, &AirSimBridge::velocityCallback, this);

        std::string airsim_topic = "/airsim_node/" + drone_name + "/vel_cmd_body_frame";
        pub_ = nh_.advertise<airsim_ros_pkgs::VelCmd>(airsim_topic, 1);

        ROS_INFO("Bridge initialized. Relaying /cmd_vel to %s", airsim_topic.c_str());
    }

    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        ROS_INFO("Relaying velocity: x=%0.2f, y=%0.2f", msg->linear.x, msg->linear.y);
        airsim_ros_pkgs::VelCmd vel_cmd;
        vel_cmd.twist = *msg;  // types match exactly
        pub_.publish(vel_cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_bridge_node");
    AirSimBridge bridge;
    ros::spin();
    return 0;
}
