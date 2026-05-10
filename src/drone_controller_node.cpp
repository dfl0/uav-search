#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <airsim_ros_pkgs/VelCmd.h>

static constexpr double CRUISE_SPEED        = 4.0; // m/s
static constexpr double FINAL_SPEED         = 1.0; // m/s near goal
static constexpr double SLOW_DIST           = 4.0; // begin ramp-down (m)
static constexpr double LOOKAHEAD_DIST      = 3.0; // lookahead on path (m)
static constexpr double WAYPOINT_ACCEPT_RAD = 1.0; // advance anchor (m)
static constexpr double FINAL_ACCEPT_RAD    = 1.5; // path complete (m)
static constexpr double KZ                  = 2.5; // altitude P-gain
static constexpr double KYAW                = 3.0; // yaw P-gain
static constexpr double MAX_VZ              = 2.0; // m/s
static constexpr double MAX_YAW_RATE        = 1.5; // rad/s

class DroneController {
public:
    DroneController() {

        std::string drone_name;

        if (!pnh_.getParam("drone_name", drone_name)) {
            ROS_FATAL("No drone_name param found");
            ros::shutdown();
            return;
        }

        path_sub_ = nh_.subscribe("/" + drone_name + "/path", 1,
                                  &DroneController::pathCallback, this);

        odom_sub_ = nh_.subscribe("/airsim_node/" + drone_name + "/odom_local_ned", 1,
                                  &DroneController::odomCallback, this);

        vel_pub_ = nh_.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/" + drone_name + "/vel_cmd_world_frame", 1);

        ROS_INFO("Drone controller initialized");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};

    tf::TransformListener tf_listener_;

    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher vel_pub_;

    std::vector<geometry_msgs::PoseStamped> current_path_;

    int current_waypoint_ = 0;

    bool has_path_ = false;

    std::string odom_frame_id_;

    double x_ = 0;
    double y_ = 0;
    double z_ = 0;

    void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
        if (odom_frame_id_.empty()) { // ← add this
            ROS_WARN_THROTTLE(1.0, "Path received before first odom — ignoring");
            return;
        }

        current_path_.clear();
        has_path_ = false;

        if (msg->poses.empty()) {
            ROS_WARN("Received empty path");
            return;
        }

        for (const auto &pose : msg->poses) {
            geometry_msgs::PoseStamped transformed;

            try {
                tf_listener_.transformPose(odom_frame_id_, pose, transformed);
                current_path_.push_back(transformed);
            } catch (tf::TransformException &ex) {
                ROS_WARN_THROTTLE(1.0, "Path TF failed: %s", ex.what());
                return;
            }
        }

        current_waypoint_ = 0;
        has_path_ = true;

        ROS_INFO("Received path with %lu waypoints", current_path_.size());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        if (odom_frame_id_.empty())
            odom_frame_id_ = msg->header.frame_id;

        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;

        airsim_ros_pkgs::VelCmd cmd;

        if (!has_path_) {
            vel_pub_.publish(cmd);
            return;
        }

        advanceWaypoints();

        const auto &final_wp = current_path_.back().pose.position;
        double fdx = final_wp.x - x_;
        double fdy = final_wp.y - y_;
        double fdz = final_wp.z - z_;
        double dist_to_final = std::sqrt(fdx * fdx + fdy * fdy + fdz * fdz);

        if (current_waypoint_ >= (int)current_path_.size() - 1 &&
            dist_to_final < FINAL_ACCEPT_RAD) {
            has_path_ = false;
            ROS_INFO("Path complete");
            vel_pub_.publish(cmd);
            return;
        }

        auto target = getLookaheadPoint();

        double tx = target.x - x_;
        double ty = target.y - y_;
        double tz = target.z - z_;
        double dist_to_target = std::sqrt(tx * tx + ty * ty + tz * tz);

        double speed = CRUISE_SPEED;
        if (dist_to_final < SLOW_DIST)
            speed = FINAL_SPEED + (CRUISE_SPEED - FINAL_SPEED) * (dist_to_final / SLOW_DIST);

        double vx = 0.0, vy = 0.0;
        if (dist_to_target > 0.01) {
            double horiz = std::sqrt(tx * tx + ty * ty);
            double horiz_frac = (dist_to_target > 0.01) ? horiz / dist_to_target : 1.0;
            vx = (tx / dist_to_target) * speed;
            vy = (ty / dist_to_target) * speed;
        }

        double vz = std::max(-MAX_VZ, std::min(MAX_VZ, KZ * tz));

        double desired_yaw = std::atan2(ty, tx);
        double current_yaw = tf::getYaw(msg->pose.pose.orientation);
        double yaw_err = std::atan2(std::sin(desired_yaw - current_yaw),
                                    std::cos(desired_yaw - current_yaw));

        cmd.twist.linear.x = std::max(-CRUISE_SPEED, std::min(CRUISE_SPEED, vx));
        cmd.twist.linear.y = std::max(-CRUISE_SPEED, std::min(CRUISE_SPEED, vy));
        cmd.twist.linear.z = vz;
        cmd.twist.angular.z = std::max(-MAX_YAW_RATE, std::min(MAX_YAW_RATE, KYAW * yaw_err));

        vel_pub_.publish(cmd);
    }

    void advanceWaypoints() {
        while (current_waypoint_ < (int)current_path_.size() - 1) {
            const auto &wp = current_path_[current_waypoint_].pose.position;

            double dx = wp.x - x_;
            double dy = wp.y - y_;
            double dz = wp.z - z_;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < WAYPOINT_ACCEPT_RAD) {
                current_waypoint_++;
                continue;
            }

            const auto &next = current_path_[current_waypoint_ + 1].pose.position;
            double path_dx = next.x - wp.x;
            double path_dy = next.y - wp.y;
            double path_dz = next.z - wp.z;
            double dot = path_dx * (-dx) + path_dy * (-dy) + path_dz * (-dz);
            if (dot > 0.0) {
                current_waypoint_++;
                continue;
            }

            break;
        }
    }

    geometry_msgs::Point getLookaheadPoint() {
        double remaining = LOOKAHEAD_DIST;

        for (int i = current_waypoint_; i < (int)current_path_.size() - 1; i++) {
            const auto &p1 = current_path_[i].pose.position;
            const auto &p2 = current_path_[i + 1].pose.position;

            double sdx = p2.x - p1.x;
            double sdy = p2.y - p1.y;
            double sdz = p2.z - p1.z;
            double seg_len = std::sqrt(sdx * sdx + sdy * sdy + sdz * sdz);

            if (seg_len < 1e-6)
                continue;

            if (remaining <= seg_len) {
                double t = remaining / seg_len;
                geometry_msgs::Point pt;
                pt.x = p1.x + t * sdx;
                pt.y = p1.y + t * sdy;
                pt.z = p1.z + t * sdz;
                return pt;
            }

            remaining -= seg_len;
        }

        return current_path_.back().pose.position;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_controller_node");
    DroneController node;
    ros::spin();
    return 0;
}
