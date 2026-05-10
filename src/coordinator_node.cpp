#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/octomap.h>
#include <queue>
#include <random>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class Coordinator {
public:
    Coordinator() {
        octomap_sub_ = nh_.subscribe("/octomap_binary", 1,
                                     &Coordinator::octomapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_markers", 1);

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

        ROS_INFO("Initializing drones");

        std::vector<std::string> drone_names;

        if (!pnh_.getParam("drones", drone_names)) {
            ROS_FATAL("No drones param found");
            ros::shutdown();
            return;
        }

        for (const auto &name : drone_names) {
            ROS_INFO("Registering drone %s", name.c_str());

            drones_[name] = DroneState();
            drones_[name].name = name;

            odom_subs_[name] = nh_.subscribe<nav_msgs::Odometry>("/airsim_node/" + name + "/odom_local_ned", 1,
                                                                 boost::bind(&Coordinator::odomCallback, this, _1, name));

            ROS_INFO("Subscribed to %s", ("/airsim_node/" + name + "/odom_local_ned").c_str());

            path_pubs_[name] = nh_.advertise<nav_msgs::Path>("/" + name + "/path", 1, true);
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
        std::vector<octomap::point3d> voxels;

        double world_x = 0.0;
        double world_y = 0.0;
        double world_z = 0.0;

        bool assigned = false;
    };

    struct RRTNode {
        octomap::point3d pos;
        int parent;
    };

    struct BlacklistEntry {
        octomap::point3d pos;
        ros::Time expiry;
    };
    std::vector<BlacklistEntry> rrt_blacklist_;

    std::mt19937 rng_{std::random_device{}()};

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unordered_map<std::string, DroneState> drones_;

    std::unordered_map<std::string, std::vector<octomap::point3d>> active_paths_;
    std::unordered_map<std::string,
    std::vector<std::pair<octomap::point3d, octomap::point3d>>>
    active_rrt_trees_;

    ros::Subscriber octomap_sub_;
    ros::Publisher marker_pub_;

    std::unordered_map<std::string, ros::Subscriber> odom_subs_;
    std::unordered_map<std::string, ros::Publisher> path_pubs_;
    std::unordered_map<std::string, FrontierCluster> last_assigned_frontiers_;

    std::unordered_map<std::string, ros::Subscriber> gt_subs_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg,
                      const std::string &drone_name) {
        auto &drone = drones_[drone_name];

        geometry_msgs::PoseStamped pose_in, pose_map;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;

        try {
            tf_buffer_.transform(pose_in, pose_map, "map", ros::Duration(0.1));
            drone.x = pose_map.pose.position.x;
            drone.y = pose_map.pose.position.y;
            drone.z = pose_map.pose.position.z;
            drone.active = true;
        } catch (tf2::TransformException &ex) {
            return;
        }

        if (drone.has_goal) {
            auto &f = last_assigned_frontiers_[drone.name];

            // Calculate distance in the unified map frame
            double dist_sq = std::pow(drone.x - f.world_x, 2) +
                std::pow(drone.y - f.world_y, 2) +
                std::pow(drone.z - f.world_z, 2);

            if (dist_sq < 6.25) { // 2.5 meter tolerance
                drone.has_goal = false;
                drone.assigned_frontier = -1;
                ROS_INFO("%s reached goal successfully", drone.name.c_str());
            }
        }
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
        octomap::AbstractOcTree *tree = octomap_msgs::binaryMsgToMap(*msg);
        if (!tree)
            return;

        octomap::OcTree *octree = dynamic_cast<octomap::OcTree *>(tree);
        if (!octree)
            return;

        revalidateActivePaths(*octree);

        std::vector<FrontierCluster> frontiers = detect3DFrontiers(*octree);
        assignFrontiers(*octree, frontiers);
        publishPathMarkers();
        delete octree;
    }

    void revalidateActivePaths(octomap::OcTree &tree) {
        for (auto &[name, drone] : drones_) {
            if (!drone.has_goal || !drone.active)
                continue;

            auto it = active_paths_.find(name);
            if (it == active_paths_.end() || it->second.size() < 2)
                continue;

            const auto &path = it->second;
            for (size_t i = 0; i + 1 < path.size(); i++) {
                if (!isCollisionFree(tree, path[i], path[i + 1])) {
                    ROS_WARN("%s: path segment [%zu→%zu] blocked by new obstacle — replanning",
                             name.c_str(), i, i + 1);
                    drone.has_goal = false;
                    drone.assigned_frontier = -1;
                    active_paths_[name].clear();
                    break;
                }
            }
        }
    }

    void assignFrontiers(octomap::OcTree &tree,
                         std::vector<FrontierCluster> &frontiers) {
        const double TIMEOUT_SEC = 20.0;
        const double RESERVATION_RADIUS = 5.0;

        // PASS 1: Validate existing goals

        for (auto &[name, drone] : drones_) {
            if (!drone.active || !drone.has_goal)
                continue;

            const auto &last = last_assigned_frontiers_[name];

            // 1. check if the goal is still unexplored directly via the OctoMap
            octomap::point3d goal_pt(last.world_x, last.world_y, last.world_z);
            octomap::OcTreeNode *node = tree.search(goal_pt);

            if (node != nullptr) {
                if (tree.isNodeOccupied(node)) {
                    // Goal turned out to be an obstacle
                    ROS_INFO("%s: Target voxel is an obstacle! Reassigning...",
                             name.c_str());
                    drone.has_goal = false;
                    drone.assigned_frontier = -1;
                    continue;
                } else {
                    // Goal is free space. Is it still bordering unknown space?
                    if (!isFrontierVoxel(tree, goal_pt)) {
                        ROS_INFO("%s: Target area fully mapped! Reassigning...",
                                 name.c_str());
                        drone.has_goal = false;
                        drone.assigned_frontier = -1;
                        continue;
                    }
                }
            }

            // check for timeout
            double elapsed = (ros::Time::now() - drone.last_assigned_time).toSec();
            if (elapsed > TIMEOUT_SEC) {
                ROS_WARN("%s: pursuit timed out (%.0fs), forcing reassignment",
                         name.c_str(), elapsed);
                drone.has_goal = false;
                drone.assigned_frontier = -1;
                continue;
            }

            for (int i = 0; i < (int)frontiers.size(); i++) {
                double dist = std::sqrt(std::pow(frontiers[i].world_x - last.world_x, 2) +
                                        std::pow(frontiers[i].world_y - last.world_y, 2) +
                                        std::pow(frontiers[i].world_z - last.world_z, 2));

                if (dist < RESERVATION_RADIUS) {
                    frontiers[i].assigned = true;
                }
            }
        }

        // PASS 2: Assign free frontiers to idle drones

        for (auto &[name, drone] : drones_) {
            if (!drone.active || drone.has_goal)
                continue;

            double best_score = -std::numeric_limits<double>::max();
            int best_frontier_idx = -1;

            for (int i = 0; i < (int)frontiers.size(); i++) {
                if (frontiers[i].assigned)
                    continue;

                octomap::point3d fp(frontiers[i].world_x, frontiers[i].world_y,
                                    frontiers[i].world_z);
                if (isBlacklisted(fp))
                    continue;

                double dist = std::sqrt(std::pow(frontiers[i].world_x - drone.x, 2) +
                                        std::pow(frontiers[i].world_y - drone.y, 2) +
                                        std::pow(frontiers[i].world_z - drone.z, 2));

                double size_score = std::min((double)frontiers[i].voxels.size(), 300.0);

                double dist_penalty = dist * 3.0;

                const double target_z = 10.0;
                const double z_penalty_factor = 5.0;
                double dz = frontiers[i].world_z - target_z;
                double height_score = 20.0 * std::exp(-0.5 * (dz / z_penalty_factor) *
                                                      (dz / z_penalty_factor));

                double score = (size_score * 2.0) - dist_penalty + height_score;

                if (score > best_score) {
                    best_score = score;
                    best_frontier_idx = i;
                }
            }

            if (best_frontier_idx < 0)
                continue;

            auto &f = frontiers[best_frontier_idx];

            octomap::point3d start(drone.x, drone.y, drone.z);
            octomap::point3d goal(f.world_x, f.world_y, f.world_z);

            auto path = planRRT(drone.name, tree, start, goal);

            if (!path.empty()) {
                f.assigned = true;
                drone.assigned_frontier = best_frontier_idx;
                drone.has_goal = true;
                drone.last_assigned_time = ros::Time::now();
                last_assigned_frontiers_[name] = f;
                publishPath(drone.name, path);
            } else {
                octomap::point3d fp(f.world_x, f.world_y, f.world_z);
                ROS_WARN("%s: RRT failed for (%.1f, %.1f, %.1f). Blacklisting for 30s.",
                         name.c_str(), fp.x(), fp.y(), fp.z());
                rrt_blacklist_.push_back({fp, ros::Time::now() + ros::Duration(30.0)});
                f.assigned = true;
            }
        }
    }

    bool isBlacklisted(const octomap::point3d &pt) {
        ros::Time now = ros::Time::now();

        // prune expired entries
        rrt_blacklist_.erase(
            std::remove_if(rrt_blacklist_.begin(), rrt_blacklist_.end(),
                           [&](const BlacklistEntry &e) { return e.expiry < now; }),
            rrt_blacklist_.end()
        );

        const double BLACKLIST_RADIUS = 2.0;
        for (auto &e : rrt_blacklist_) {
            if ((e.pos - pt).norm() < BLACKLIST_RADIUS)
                return true;
        }
        return false;
    }

    std::vector<FrontierCluster> detect3DFrontiers(octomap::OcTree &tree) {
        std::vector<octomap::point3d> frontier_voxels;

        for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; it++) {
            if (tree.isNodeOccupied(*it))
                continue;

            octomap::point3d p = it.getCoordinate();

            if (p.z() > 60.0 || p.z() < 0.0)
                continue;

            if (isFrontierVoxel(tree, p))
                frontier_voxels.push_back(p);
        }

        return clusterFrontiers(tree, frontier_voxels);
    }

    bool isFrontierVoxel(octomap::OcTree &tree, const octomap::point3d &p) {
        double res = tree.getResolution();

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    octomap::point3d np(p.x() + dx * res, p.y() + dy * res,
                                        p.z() + dz * res);

                    auto node = tree.search(np);

                    if (!node)
                        return true;
                }
            }
        }

        return false;
    }

    std::vector<FrontierCluster>
    clusterFrontiers(octomap::OcTree &tree,
                     const std::vector<octomap::point3d> &voxels) {
        double res = tree.getResolution();

        std::unordered_map<octomap::OcTreeKey, size_t, octomap::OcTreeKey::KeyHash>
        key_to_idx;
        for (size_t i = 0; i < voxels.size(); i++) {
            key_to_idx[tree.coordToKey(voxels[i])] = i;
        }

        std::vector<bool> visited(voxels.size(), false);
        std::vector<FrontierCluster> clusters;

        // 26-connected neighbor offsets
        const int offsets[][3] = {
            {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0},
            {-1, 0, 1},   {-1, 1, -1}, {-1, 1, 0},  {-1, 1, 1},  {0, -1, -1},
            {0, -1, 0},   {0, -1, 1},  {0, 0, -1},  {0, 0, 1},   {0, 1, -1},
            {0, 1, 0},    {0, 1, 1},   {1, -1, -1}, {1, -1, 0},  {1, -1, 1},
            {1, 0, -1},   {1, 0, 0},   {1, 0, 1},   {1, 1, -1},  {1, 1, 0},
            {1, 1, 1},
        };

        for (size_t i = 0; i < voxels.size(); i++) {
            if (visited[i])
                continue;

            FrontierCluster cluster;
            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty()) {
                size_t idx = q.front();
                q.pop();
                cluster.voxels.push_back(voxels[idx]);

                octomap::OcTreeKey base = tree.coordToKey(voxels[idx]);
                for (auto &off : offsets) {
                    octomap::OcTreeKey nk;
                    nk[0] = base[0] + off[0];
                    nk[1] = base[1] + off[1];
                    nk[2] = base[2] + off[2];

                    auto it = key_to_idx.find(nk);
                    if (it != key_to_idx.end() && !visited[it->second]) {
                        visited[it->second] = true;
                        q.push(it->second);
                    }
                }
            }

            compute3DCentroid(cluster);
            if (cluster.voxels.size() >= 5)
                clusters.push_back(cluster);
        }
        return clusters;
    }

    void compute3DCentroid(FrontierCluster &cluster) {
        double sx = 0, sy = 0, sz = 0;
        for (auto &p : cluster.voxels) {
            sx += p.x();
            sy += p.y();
            sz += p.z();
        }
        int n = cluster.voxels.size();
        double avg_x = sx / n, avg_y = sy / n, avg_z = sz / n;

        double worst_dist_sq = -1.0;
        octomap::point3d best_voxel = cluster.voxels[0];

        for (auto &p : cluster.voxels) {
            double dist_sq = std::pow(p.x() - avg_x, 2) + std::pow(p.y() - avg_y, 2) +
                std::pow(p.z() - avg_z, 2);
            if (dist_sq > worst_dist_sq) {
                worst_dist_sq = dist_sq;
                best_voxel = p;
            }
        }

        cluster.world_x = best_voxel.x();
        cluster.world_y = best_voxel.y();
        cluster.world_z = std::max(best_voxel.z(), 5.0f);
    }

    bool isCollisionFree(octomap::OcTree &tree, const octomap::point3d &a,
                         const octomap::point3d &b, double drone_radius = 1.0) {
        octomap::point3d dir = b - a;
        double dist = dir.norm();
        dir.normalize();

        const double step = tree.getResolution() * 0.5;
        const double res = tree.getResolution();

        for (double d = 0.0; d <= dist; d += step) {
            octomap::point3d cp = a + dir * d;

            octomap::point3d bbx_min(cp.x() - drone_radius, cp.y() - drone_radius,
                                     cp.z() - drone_radius);
            octomap::point3d bbx_max(cp.x() + drone_radius, cp.y() + drone_radius,
                                     cp.z() + drone_radius);

            for (auto it = tree.begin_leafs_bbx(bbx_min, bbx_max),
            end = tree.end_leafs_bbx();
            it != end; ++it) {
                if (tree.isNodeOccupied(*it))
                    return false;
            }

            // treat unknown voxels that are face-adjacent to a known occupied voxel as occupied
            const int face_offsets[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

            octomap::OcTreeNode *cp_node = tree.search(cp);
            if (cp_node == nullptr) {
                for (auto &off : face_offsets) {
                    octomap::point3d neighbor(cp.x() + off[0] * res,
                                              cp.y() + off[1] * res,
                                              cp.z() + off[2] * res);
                    octomap::OcTreeNode *nb = tree.search(neighbor);
                    if (nb && tree.isNodeOccupied(nb))
                        return false;
                }
            }
        }
        return true;
    }

    std::vector<octomap::point3d> planRRT(const std::string &drone_name,
                                          octomap::OcTree &tree,
                                          octomap::point3d start,
                                          octomap::point3d goal) {
        // check if the goal itself is occupied. If so, find the nearest free neighbor
        octomap::OcTreeKey goal_key = tree.coordToKey(goal);
        auto goal_node = tree.search(goal_key);
        if (goal_node && tree.isNodeOccupied(goal_node)) {
            // move goal slightly toward the start to pull it out of the wall
            octomap::point3d push_dir = start - goal;
            push_dir.normalize();
            goal = goal + (push_dir * tree.getResolution() * 2.0);
        }

        const double MIN_FLIGHT_Z = 5.0;

        goal = octomap::point3d(goal.x(), goal.y(),
                                std::max((double)goal.z(), MIN_FLIGHT_Z));

        std::vector<RRTNode> nodes;
        nodes.push_back({start, -1});

        std::vector<std::pair<octomap::point3d, octomap::point3d>> tree_edges;

        double global_min_x, global_min_y, global_min_z;
        double global_max_x, global_max_y, global_max_z;
        tree.getMetricMin(global_min_x, global_min_y, global_min_z);
        tree.getMetricMax(global_max_x, global_max_y, global_max_z);

        double search_margin = 10.0;
        const double discovery_height_buffer = 20.0;

        double local_min_x = std::min(start.x(), goal.x()) - search_margin;
        double local_max_x = std::max(start.x(), goal.x()) + search_margin;

        double local_min_y = std::min(start.y(), goal.y()) - search_margin;
        double local_max_y = std::max(start.y(), goal.y()) + search_margin;

        double local_min_z =
            std::max(global_min_z, std::min(start.z(), goal.z()) - search_margin);
        double local_max_z =
            std::max(start.z(), goal.z()) + discovery_height_buffer;

        for (int iter = 0; iter < 5000; iter++) {
            octomap::point3d sample;

            std::uniform_real_distribution<double> bias(0.0, 1.0);
            if (bias(rng_) < 0.10) {
                sample = goal;
            } else {
                std::uniform_real_distribution<double> rx(local_min_x, local_max_x);
                std::uniform_real_distribution<double> ry(local_min_y, local_max_y);
                std::uniform_real_distribution<double> rz(std::max((double)local_min_z, MIN_FLIGHT_Z), local_max_z);
                sample = octomap::point3d(rx(rng_), ry(rng_), rz(rng_));
            }

            int nearest = 0;
            double best_dist = 1e9;

            for (int i = 0; i < nodes.size(); i++) {
                double d = (nodes[i].pos - sample).norm();

                if (d < best_dist) {
                    best_dist = d;
                    nearest = i;
                }
            }

            const double STEP = 2.0;
            octomap::point3d dir = sample - nodes[nearest].pos;
            if (dir.norm() > STEP) {
                dir.normalize();
                sample = nodes[nearest].pos + dir * STEP;
            }

            if (!isCollisionFree(tree, nodes[nearest].pos, sample))
                continue;

            nodes.push_back({sample, nearest});
            tree_edges.push_back({nodes[nearest].pos, sample});

            const double GOAL_THRESH = 1.0;
            if ((sample - goal).norm() < GOAL_THRESH) {
                std::vector<octomap::point3d> path;

                int idx = nodes.size() - 1;
                while (idx != -1) {
                    path.push_back(nodes[idx].pos);
                    idx = nodes[idx].parent;
                }

                std::reverse(path.begin(), path.end());
                if (isCollisionFree(tree, path.back(), goal)) {
                    path.push_back(goal);
                }

                active_rrt_trees_[drone_name] = tree_edges;

                return path;
            }
        }
        return {};
    }

    void publishPath(const std::string &drone,
                     const std::vector<octomap::point3d> &points) {
        nav_msgs::Path path;

        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        for (auto &p : points) {
            geometry_msgs::PoseStamped pose;

            pose.header = path.header;

            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();

            pose.pose.orientation.w = 1.0;

            path.poses.push_back(pose);
        }

        path_pubs_[drone].publish(path);

        active_paths_[drone] = points;
    }

    void publishPathMarkers() {
        visualization_msgs::MarkerArray ma;

        visualization_msgs::Marker clear;

        clear.action = visualization_msgs::Marker::DELETEALL;

        ma.markers.push_back(clear);

        int marker_id = 0;

        // draw RRT trees

        for (const auto &[drone, edges] : active_rrt_trees_) {
            visualization_msgs::Marker tree_marker;

            tree_marker.header.frame_id = "map";
            tree_marker.header.stamp = ros::Time::now();

            tree_marker.ns = "rrt_tree";

            tree_marker.id = marker_id++;

            tree_marker.type = visualization_msgs::Marker::LINE_LIST;

            tree_marker.action = visualization_msgs::Marker::ADD;

            tree_marker.scale.x = 0.03;

            // tree colors

            if (drone == "drone_1") {
                tree_marker.color.r = 1.0;
                tree_marker.color.g = 0.3;
                tree_marker.color.b = 0.3;
            } else {
                tree_marker.color.r = 0.3;
                tree_marker.color.g = 1.0;
                tree_marker.color.b = 1.0;
            }

            tree_marker.color.a = 0.35;

            for (const auto &edge : edges) {
                geometry_msgs::Point p1;
                geometry_msgs::Point p2;

                p1.x = edge.first.x();
                p1.y = edge.first.y();
                p1.z = edge.first.z();

                p2.x = edge.second.x();
                p2.y = edge.second.y();
                p2.z = edge.second.z();

                tree_marker.points.push_back(p1);
                tree_marker.points.push_back(p2);
            }

            ma.markers.push_back(tree_marker);
        }

        // draw final paths

        for (const auto &[drone, path] : active_paths_) {
            if (path.empty())
                continue;

            visualization_msgs::Marker line;

            line.header.frame_id = "map";
            line.header.stamp = ros::Time::now();

            line.ns = "rrt_paths";

            line.id = marker_id++;

            line.type = visualization_msgs::Marker::LINE_STRIP;

            line.action = visualization_msgs::Marker::ADD;

            line.scale.x = 0.15;

            // path colors

            if (drone == "drone_1") {
                line.color.r = 1.0;
                line.color.g = 0.0;
                line.color.b = 0.0;
            } else {
                line.color.r = 0.0;
                line.color.g = 1.0;
                line.color.b = 1.0;
            }

            line.color.a = 1.0;

            for (const auto &p : path) {
                geometry_msgs::Point gp;

                gp.x = p.x();
                gp.y = p.y();
                gp.z = p.z();

                line.points.push_back(gp);
            }

            ma.markers.push_back(line);

            // waypoint spheres

            for (const auto &p : path) {
                visualization_msgs::Marker wp;

                wp.header.frame_id = "map";
                wp.header.stamp = ros::Time::now();

                wp.ns = "rrt_waypoints";

                wp.id = marker_id++;

                wp.type = visualization_msgs::Marker::SPHERE;

                wp.action = visualization_msgs::Marker::ADD;

                wp.pose.position.x = p.x();
                wp.pose.position.y = p.y();
                wp.pose.position.z = p.z();

                wp.pose.orientation.w = 1.0;

                wp.scale.x = 0.35;
                wp.scale.y = 0.35;
                wp.scale.z = 0.35;

                wp.color.r = 1.0;
                wp.color.g = 1.0;
                wp.color.b = 0.0;
                wp.color.a = 1.0;

                ma.markers.push_back(wp);
            }
        }

        marker_pub_.publish(ma);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "coordinator_node");
    Coordinator node;
    ros::spin();
    return 0;
}
