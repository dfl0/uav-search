#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/bind.hpp>
#include <queue>
#include <vector>
#include <unordered_map>

class Coordinator {
public:
    Coordinator() {
        map_sub_ = nh_.subscribe(
            "/projected_map",
            1,
            &Coordinator::mapCallback,
            this
        );

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/frontier_markers",
            1
        );

        ROS_INFO("Initializing drones");

        std::vector<std::string> drone_names;

        if (!pnh_.getParam("drones", drone_names)) {
            ROS_FATAL("No drones param found");
            ros::shutdown();
            return;
        }

        for (const auto& name : drone_names) {

            ROS_INFO("Registering drone %s", name.c_str());

            drones_[name] = DroneState();
            drones_[name].name = name;

            odom_subs_[name] = nh_.subscribe<nav_msgs::Odometry>(
                "/airsim_node/" + name + "/odom_local_ned",
                1,
                boost::bind(&Coordinator::odomCallback, this, _1, name)
            );

            ROS_INFO("Subscribed to %s", ("/airsim_node/" + name + "/odom_local_ned").c_str());

            goal_pubs_[name] = nh_.advertise<geometry_msgs::PoseStamped>(
                "/" + name + "/goal",
                1,
                true
            );
        }
    }

private:
    struct DroneState {
        std::string name;

        bool active = false;
        bool has_goal = false;
        int assigned_frontier = -1;
        ros::Time last_assigned_time;

        double x;
        double y;
        double z;
    };

    struct FrontierCluster {
        std::vector<std::pair<int,int>> cells;

        double centroid_x = 0.0;
        double centroid_y = 0.0;

        bool assigned = false;

        double world_x = 0.0;
        double world_y = 0.0;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};

    tf::TransformListener tf_listener_;

    std::unordered_map<std::string, DroneState> drones_;

    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;

    std::unordered_map<std::string, ros::Subscriber> odom_subs_;
    std::unordered_map<std::string, ros::Publisher> goal_pubs_;
    std::unordered_map<std::string, FrontierCluster> last_assigned_frontiers_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                      const std::string& drone_name) {
        auto& drone = drones_[drone_name];

        geometry_msgs::PoseStamped pose_in;
        pose_in.header = msg->header;
        pose_in.pose   = msg->pose.pose;

        geometry_msgs::PoseStamped pose_map;
        try {
            tf_listener_.transformPose("map", pose_in, pose_map);

            drone.x = pose_map.pose.position.x;
            drone.y = pose_map.pose.position.y;
            drone.z = pose_map.pose.position.z;

            drone.active = true;
        }
        catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "%s", ex.what());
            return;
        }

        if (drone.has_goal) {
            auto& f = last_assigned_frontiers_[drone.name];

            double dx = drone.x - f.world_x;
            double dy = drone.y - f.world_y;

            ROS_INFO_THROTTLE(
                1.0,
                "%s: drone=(%.2f %.2f) goal=(%.2f %.2f)",
                drone.name.c_str(), drone.x, drone.y, f.world_x, f.world_y
            );

            if (dx*dx + dy*dy < 1.0) {
                drone.has_goal = false;
                drone.assigned_frontier = -1;

                ROS_INFO("%s reached goal", drone.name.c_str());
            }
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        std::vector<FrontierCluster> clusters = detectFrontiers(*msg);
        assignFrontiers(clusters);
        publishMarkers(clusters, *msg);
    }

    std::vector<FrontierCluster> detectFrontiers(const nav_msgs::OccupancyGrid& map) {
        // float resolution = map.info.resolution;

        int width = map.info.width;
        int height = map.info.height;

        // std::vector<std::pair<int,int>> frontier_cells;
        std::vector<bool> frontier_mask(
            width * height,
            false
        );

        // find all frontier cells
        int n_frontier_cells = 0;
        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                if (isFrontierCell(x, y, map)) {
                    frontier_mask[y*width + x] = true;
                    n_frontier_cells++;
                }
            }
        }

        ROS_INFO_THROTTLE(1.0, "Detected %d frontier cells", n_frontier_cells);

        // cluster frontier cells

        std::vector<bool> visited(
            width * height,
            false
        );

        std::vector<FrontierCluster> clusters;

        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                int idx = y*width + x;
                if (!frontier_mask[idx])
                    continue;
                if (visited[idx])
                    continue;

                FrontierCluster cluster;

                growCluster(
                    x, y,
                    frontier_mask,
                    visited,
                    map,
                    cluster
                );

                if (cluster.cells.size() < 5 && clusters.size() > 0)
                    continue;

                computeCentroid(cluster);

                cluster.world_x = map.info.origin.position.x + (cluster.centroid_x + 0.5)*map.info.resolution;
                cluster.world_y = map.info.origin.position.y + (cluster.centroid_y + 0.5)*map.info.resolution;

                clusters.push_back(cluster);
            }
        }

        ROS_INFO_THROTTLE(1.0, "Detected %lu frontier clusters", clusters.size());

        return clusters;
    }

    bool isFrontierCell(int x, int y, const nav_msgs::OccupancyGrid& map) {
        int width = map.info.width;
        int height = map.info.height;

        const auto& data = map.data;

        int idx = y*width + x;

        if (data[idx] != 0)
            return false;

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {

                if (dx == 0 && dy == 0)
                    continue;

                int nx = x + dx;
                int ny = y + dy;

                if (nx < 0 || ny < 0 ||
                    nx >= width || ny >= height)
                    continue;

                int nidx = ny*width + nx;

                if (data[nidx] == -1)
                    return true;
            }
        }

        return false;
    }

    void growCluster(int start_x, int start_y,
                     const std::vector<bool>& frontier_mask,
                     std::vector<bool>& visited,
                     const nav_msgs::OccupancyGrid& map,
                     FrontierCluster& cluster) {
        int width  = map.info.width;
        int height = map.info.height;

        std::queue<std::pair<int,int>> q;

        q.push({start_x, start_y});

        visited[start_y * width + start_x] = true;

        while (!q.empty()) {
            auto current = q.front();
            q.pop();

            int x = current.first;
            int y = current.second;

            cluster.cells.push_back(current);

            // 8-connected BFS
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {

                    if (dx == 0 && dy == 0)
                        continue;

                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || ny < 0 ||
                        nx >= width || ny >= height)
                        continue;

                    int nidx = ny * width + nx;

                    if (!frontier_mask[nidx])
                        continue;

                    if (visited[nidx])
                        continue;

                    visited[nidx] = true;

                    q.push({nx, ny});
                }
            }
        }
    }

    void computeCentroid(FrontierCluster& cluster) {
        double sum_x = 0.0;
        double sum_y = 0.0;

        for (const auto& cell : cluster.cells) {
            sum_x += cell.first;
            sum_y += cell.second;
        }

        cluster.centroid_x = sum_x/cluster.cells.size();
        cluster.centroid_y = sum_y/cluster.cells.size();
    }

    void publishMarkers(const std::vector<FrontierCluster>& clusters, const nav_msgs::OccupancyGrid& map) {
        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker points;

        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(clear);

        points.header.frame_id = map.header.frame_id;
        points.header.stamp = ros::Time::now();
        points.ns = "frontier_cells";
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.action = visualization_msgs::Marker::ADD;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.color.r = 1.0;
        points.color.a = 0.6;

        int marker_id = 1;

        for (const auto& cluster : clusters) {

            // frontier cells
            for (const auto& cell : cluster.cells) {
                geometry_msgs::Point p;

                p.x = map.info.origin.position.x + (cell.first + 0.5)*map.info.resolution;
                p.y = map.info.origin.position.y + (cell.second + 0.5)*map.info.resolution;
                p.z = 0.0;

                points.points.push_back(p);
            }

            // centroid
            visualization_msgs::Marker centroid;

            centroid.header.frame_id = map.header.frame_id;
            centroid.header.stamp = ros::Time::now();
            centroid.ns = "frontier_centroids";
            centroid.id = marker_id++;
            centroid.type = visualization_msgs::Marker::CYLINDER;
            centroid.action = visualization_msgs::Marker::ADD;

            centroid.pose.position.x = map.info.origin.position.x + (cluster.centroid_x + 0.5)*map.info.resolution;
            centroid.pose.position.y = map.info.origin.position.y + (cluster.centroid_y + 0.5)*map.info.resolution;
            centroid.pose.position.z = 0.0;
            centroid.pose.orientation.w = 1.0;

            // scale based on cluster size
            double size = 1.0 + 0.02*cluster.cells.size();

            centroid.scale.x = size;
            centroid.scale.y = size;
            centroid.scale.z = 0.01;

            centroid.color.r = 0.0;
            centroid.color.g = 1.0;
            centroid.color.b = 0.0;
            centroid.color.a = 0.6;

            ma.markers.push_back(centroid);
        }

        ma.markers.push_back(points);

        for (const auto& [name, drone] : drones_) {

            if (!drone.active)
                continue;

            // drone sphere

            visualization_msgs::Marker drone_marker;

            drone_marker.header.frame_id = "map";
            drone_marker.header.stamp = ros::Time::now();

            drone_marker.ns = "drones";
            drone_marker.id = marker_id++;

            drone_marker.type = visualization_msgs::Marker::SPHERE;
            drone_marker.action = visualization_msgs::Marker::ADD;

            drone_marker.pose.position.x = drone.x;
            drone_marker.pose.position.y = drone.y;
            drone_marker.pose.position.z = 0.3;

            drone_marker.pose.orientation.w = 1.0;

            drone_marker.scale.x = 0.5;
            drone_marker.scale.y = 0.5;
            drone_marker.scale.z = 0.5;

            drone_marker.color.r = 0.0;
            drone_marker.color.g = 0.0;
            drone_marker.color.b = 1.0;
            drone_marker.color.a = 1.0;

            ma.markers.push_back(drone_marker);

            // GOAL LINE

            if (drone.has_goal) {
                auto& f = last_assigned_frontiers_[name];

                visualization_msgs::Marker line;

                line.header.frame_id = "map";
                line.header.stamp = ros::Time::now();

                line.ns = "goal_lines";
                line.id = marker_id++;

                line.type = visualization_msgs::Marker::LINE_STRIP;
                line.action = visualization_msgs::Marker::ADD;

                line.scale.x = 0.08;

                line.color.r = 0.0;
                line.color.g = 0.5;
                line.color.b = 1.0;
                line.color.a = 1.0;

                geometry_msgs::Point p1;
                p1.x = drone.x;
                p1.y = drone.y;
                p1.z = 0.2;

                geometry_msgs::Point p2;
                p2.x = f.world_x;
                p2.y = f.world_y;
                p2.z = 0.2;

                line.points.push_back(p1);
                line.points.push_back(p2);

                ma.markers.push_back(line);

                // GOAL SPHERE

                visualization_msgs::Marker goal_marker;

                goal_marker.header.frame_id = "map";
                goal_marker.header.stamp = ros::Time::now();

                goal_marker.ns = "goals";
                goal_marker.id = marker_id++;

                goal_marker.type = visualization_msgs::Marker::SPHERE;
                goal_marker.action = visualization_msgs::Marker::ADD;

                goal_marker.pose.position.x = f.world_x;
                goal_marker.pose.position.y = f.world_y;
                goal_marker.pose.position.z = 0.3;

                goal_marker.pose.orientation.w = 1.0;

                goal_marker.scale.x = 0.35;
                goal_marker.scale.y = 0.35;
                goal_marker.scale.z = 0.35;

                goal_marker.color.r = 1.0;
                goal_marker.color.g = 1.0;
                goal_marker.color.b = 0.0;
                goal_marker.color.a = 1.0;

                ma.markers.push_back(goal_marker);
            }
        }

        marker_pub_.publish(ma);
    }

    void assignFrontiers(std::vector<FrontierCluster>& frontiers) {
        const double TIMEOUT_SEC = 10.0;
        const double FRONTIER_MATCH_DIST = 2.0;

        // Pass 1: validate in-progress goals against new frontier list

        for (auto& [name, drone] : drones_) {
            if (!drone.active || !drone.has_goal)
                continue;

            const auto& last = last_assigned_frontiers_[name];

            // Find the closest cluster in the new list to the drone's current target
            int    match_idx  = -1;
            double match_dist = std::numeric_limits<double>::max();

            for (int i = 0; i < (int)frontiers.size(); i++) {
                double dx = frontiers[i].world_x - last.world_x;
                double dy = frontiers[i].world_y - last.world_y;
                double d  = dx*dx + dy*dy;

                if (d < match_dist) {
                    match_dist = d;
                    match_idx = i;
                }
            }

            // Frontier explored / merged into map - free the drone immediately
            if (match_idx < 0 || match_dist > std::pow(FRONTIER_MATCH_DIST, 2)) {
                ROS_INFO("%s: target frontier explored, reassigning", name.c_str());
                drone.has_goal = false;
                drone.assigned_frontier = -1;
                continue;
            }

            // Frontier still live - hold it so Pass 2 doesn't double-assign it
            frontiers[match_idx].assigned = true;

            // Timeout: drone is probably stuck or the path is blocked
            double elapsed = (ros::Time::now() - drone.last_assigned_time).toSec();
            if (elapsed > TIMEOUT_SEC) {
                ROS_WARN("%s: pursuit timed out (%.0fs), forcing reassignment",
                        name.c_str(), elapsed);
                frontiers[match_idx].assigned = false;
                drone.has_goal                = false;
                drone.assigned_frontier       = -1;
            }
        }

        // Pass 2: assign free frontiers to drones that need one

        for (auto& [name, drone] : drones_) {
            if (!drone.active || drone.has_goal)
                continue;

            if ((ros::Time::now() - drone.last_assigned_time).toSec() < 2.0)
                continue;

            double best_score = -std::numeric_limits<double>::max();
            int    best_frontier_idx   = -1;

            for (int i = 0; i < (int)frontiers.size(); i++) {
                if (frontiers[i].assigned)
                    continue;

                double dx   = frontiers[i].world_x - drone.x;
                double dy   = frontiers[i].world_y - drone.y;
                double dist = std::sqrt(dx*dx + dy*dy);

                double score = 2.0*(double)frontiers[i].cells.size() - dist;

                if (score > best_score) {
                    best_score = score;
                    best_frontier_idx   = i;
                }
            }

            if (best_frontier_idx < 0)
                continue;

            auto& f = frontiers[best_frontier_idx];
            f.assigned = true;
            drone.assigned_frontier = best_frontier_idx;
            drone.has_goal = true;
            drone.last_assigned_time = ros::Time::now();
            last_assigned_frontiers_[name] = f;

            publishGoal(drone, f);
        }
    }

    void publishGoal(DroneState& drone, const FrontierCluster& f) {
        geometry_msgs::PoseStamped goal;

        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();

        goal.pose.position.x = f.world_x;
        goal.pose.position.y = f.world_y;
        goal.pose.position.z = 15.0;

        goal.pose.orientation.w = 1.0;

        goal_pubs_[drone.name].publish(goal);

        ROS_INFO("%s assigned goal (%.2f, %.2f)",
                 drone.name.c_str(),
                 goal.pose.position.x,
                 goal.pose.position.y);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator_node");
    Coordinator node;
    ros::spin();
    return 0;
}
