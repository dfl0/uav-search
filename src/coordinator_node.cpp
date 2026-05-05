#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <vector>

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
    }

private:
    struct FrontierCluster {
        std::vector<std::pair<int,int>> cells;

        double centroid_x = 0.0;
        double centroid_y = 0.0;
    };

    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        detectFrontiers(*msg);
    }

    void detectFrontiers(const nav_msgs::OccupancyGrid& map) {
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

        ROS_INFO("Detected %d frontier cells", n_frontier_cells);

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

                if (cluster.cells.size() < 5)
                    continue;

                computeCentroid(cluster);

                clusters.push_back(cluster);
            }
        }

        ROS_INFO("Detected %lu frontier clusters", clusters.size());

        publishMarkers(clusters, map);
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
                     FrontierCluster& cluster
    ) {
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
        marker_pub_.publish(ma);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator_node");
    Coordinator node;
    ros::spin();
    return 0;
}
