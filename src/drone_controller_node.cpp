#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <airsim_ros_pkgs/VelCmd.h>

class DroneController {
public:
    DroneController() {
        std::string drone_name;
        if (!pnh_.getParam("drone_name", drone_name)) {
            ROS_FATAL("No drone_name param found");
            ros::shutdown();
            return;
        }

        goal_sub_ = nh_.subscribe(
            "/" + drone_name + "/goal",
            1,
            &DroneController::goalCallback,
            this
        );

        odom_sub_ = nh_.subscribe(
            "/airsim_node/" + drone_name + "/odom_local_ned",
            1,
            &DroneController::odomCallback,
            this
        );

        vel_pub_ = nh_.advertise<airsim_ros_pkgs::VelCmd>(
            "/airsim_node/" + drone_name + "/vel_cmd_world_frame",
            1
        );
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};

    tf::TransformListener tf_listener_;

    geometry_msgs::PoseStamped goal_odom_;
    std::string odom_frame_id_;
    bool has_goal_ = false;

    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher vel_pub_;

    double x_ = 0;
    double y_ = 0;
    double z_ = 0;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (odom_frame_id_.empty())
            return;

        try {
            tf_listener_.transformPose(odom_frame_id_, *msg, goal_odom_);
            has_goal_ = true;
        }
        catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Goal TF failed: %s", ex.what());
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (odom_frame_id_.empty())
            odom_frame_id_ = msg->header.frame_id;

        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;

        airsim_ros_pkgs::VelCmd cmd;

        if (!has_goal_) {
            vel_pub_.publish(cmd);
            return;
        }

        double dx = goal_odom_.pose.position.x - x_;
        double dy = goal_odom_.pose.position.y - y_;
        double dz = goal_odom_.pose.position.z - z_;

        double dist_sq = dx*dx + dy*dy;

        if (dist_sq < 1.0) {
            has_goal_ = false;
            vel_pub_.publish(cmd);
            return;
        }

        double kxy  = 0.4;
        double kz   = 0.6;
        double kyaw = 1.0;

        double desired_yaw = std::atan2(dy, dx);
        double current_yaw = tf::getYaw(msg->pose.pose.orientation);
        double yaw_err = std::atan2(std::sin(desired_yaw - current_yaw), std::cos(desired_yaw - current_yaw));

        cmd.twist.linear.x = kxy*dx;
        cmd.twist.linear.y = kxy*dy;
        cmd.twist.linear.z = kz*dz;
        cmd.twist.angular.z = kyaw*yaw_err;

        vel_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_controller_node");
    DroneController node;
    ros::spin();
    return 0;
}
