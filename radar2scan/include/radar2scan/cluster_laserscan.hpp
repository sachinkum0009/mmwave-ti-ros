#ifndef CLUSTER_LASERSCAN_HPP_
#define CLUSTER_LASERSCAN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

namespace cluster_laserscan
{
    /**
     * @brief Structure to represent a 3D point
     */
    struct Point3D
    {
        float x;
        float y;
        float z;
        float intensity;
        int cluster_id;  // -1 for unassigned, -2 for noise
        
        Point3D() : x(0), y(0), z(0), intensity(0), cluster_id(-1) {}
        Point3D(float x_, float y_, float z_, float intensity_ = 0)
            : x(x_), y(y_), z(z_), intensity(intensity_), cluster_id(-1) {}
    };
    
    /**
     * @brief Structure to represent a cluster
     */
    struct Cluster
    {
        std::vector<Point3D> points;
        Point3D centroid;
        size_t point_count;
        
        Cluster() : point_count(0) {}
    };

    /**
     * @brief Node for clustering pointcloud data and generating LaserScan
     * 
     * This node performs the following steps:
     * 1. Subscribe to PointCloud2 data
     * 2. Apply Euclidean clustering to detect objects
     * 3. Calculate centroid for each cluster
     * 4. Generate a line of fixed length at each centroid
     * 5. Publish the lines as LaserScan data
     */
    class ClusterLaserScan : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor
         */
        ClusterLaserScan();
        
        /**
         * @brief Destructor
         */
        ~ClusterLaserScan();

    private:
        /**
         * @brief Callback for pointcloud data
         * 
         * @param msg Input pointcloud in PointCloud2 format
         */
        void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
        /**
         * @brief Extract points from PointCloud2 message
         * 
         * @param cloud_msg Input pointcloud message
         * @param points Output vector of 3D points
         */
        void extractPoints(const sensor_msgs::msg::PointCloud2 & cloud_msg,
                          std::vector<Point3D> & points);
        
        /**
         * @brief Perform Euclidean clustering on points
         * 
         * @param points Input points to cluster
         * @param clusters Output clusters
         */
        void performClustering(std::vector<Point3D> & points,
                              std::vector<Cluster> & clusters);
        
        /**
         * @brief Calculate distance between two points
         * 
         * @param p1 First point
         * @param p2 Second point
         * @return Euclidean distance
         */
        inline float euclideanDistance(const Point3D & p1, const Point3D & p2);
        
        /**
         * @brief Expand a cluster using region growing (DBSCAN-style)
         * 
         * @param points All points
         * @param point_idx Index of seed point
         * @param cluster_id Current cluster ID
         * @param neighbors Indices of neighbor points
         */
        void expandCluster(std::vector<Point3D> & points,
                          size_t point_idx,
                          int cluster_id,
                          std::vector<size_t> & neighbors);
        
        /**
         * @brief Find neighbors within epsilon distance
         * 
         * @param points All points
         * @param point_idx Index of query point
         * @return Indices of neighbor points
         */
        std::vector<size_t> regionQuery(const std::vector<Point3D> & points,
                                        size_t point_idx);
        
        /**
         * @brief Calculate centroid of a cluster
         * 
         * @param cluster Input cluster
         * @return Centroid point
         */
        Point3D calculateCentroid(const Cluster & cluster);
        
        /**
         * @brief Generate LaserScan from clusters
         * 
         * @param clusters Input clusters
         * @param laser_scan Output LaserScan message
         * @param header Header from input pointcloud
         */
        void generateLaserScan(const std::vector<Cluster> & clusters,
                              sensor_msgs::msg::LaserScan & laser_scan,
                              const std_msgs::msg::Header & header);
        
        /**
         * @brief Create line endpoints from centroid
         * 
         * @param centroid Center point of cluster
         * @param start_point Output start point of line
         * @param end_point Output end point of line
         */
        void createLineFromCentroid(const Point3D & centroid,
                                   Point3D & start_point,
                                   Point3D & end_point);

        // ROS2 subscriber and publisher
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_debug_pub_;
        
        // Clustering parameters
        double cluster_tolerance_;      // Epsilon - maximum distance between points in a cluster
        int min_cluster_size_;          // Minimum number of points to form a cluster
        int max_cluster_size_;          // Maximum number of points in a cluster
        
        // Line generation parameters
        double line_length_;            // Fixed length of the line at each cluster centroid
        std::string line_orientation_;  // "perpendicular" or "radial" to the centroid direction
        
        // LaserScan parameters
        double angle_min_;
        double angle_max_;
        double angle_increment_;
        double range_min_;
        double range_max_;
        std::string frame_id_;
        
        // Intensity filtering
        bool enable_intensity_filter_;
        double intensity_threshold_;
        
        // Debug options
        bool publish_cluster_debug_;
    };
    
} // namespace cluster_laserscan

#endif // CLUSTER_LASERSCAN_HPP_
