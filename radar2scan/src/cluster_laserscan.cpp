#include "radar2scan/cluster_laserscan.hpp"
#include <cmath>

namespace cluster_laserscan
{
    ClusterLaserScan::ClusterLaserScan()
    : Node("cluster_laserscan_node")
    {
        // Declare and get clustering parameters
        this->declare_parameter("cluster_tolerance", 0.5);
        this->declare_parameter("min_cluster_size", 3);
        this->declare_parameter("max_cluster_size", 500);
        
        // Declare and get line generation parameters
        this->declare_parameter("line_length", 0.5);
        this->declare_parameter("line_orientation", "perpendicular");
        
        // Declare and get LaserScan parameters
        this->declare_parameter("angle_min", -M_PI);
        this->declare_parameter("angle_max", M_PI);
        this->declare_parameter("angle_increment", 0.0087); // ~0.5 degrees
        this->declare_parameter("range_min", 0.1);
        this->declare_parameter("range_max", 10.0);
        this->declare_parameter("frame_id", "ti_mmwave_0");
        
        // Declare intensity filtering parameters
        this->declare_parameter("enable_intensity_filter", false);
        this->declare_parameter("intensity_threshold", 20.0);
        
        // Debug parameters
        this->declare_parameter("publish_cluster_debug", false);
        
        // Get parameter values
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        
        line_length_ = this->get_parameter("line_length").as_double();
        line_orientation_ = this->get_parameter("line_orientation").as_string();
        
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();
        angle_increment_ = this->get_parameter("angle_increment").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        enable_intensity_filter_ = this->get_parameter("enable_intensity_filter").as_bool();
        intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
        
        publish_cluster_debug_ = this->get_parameter("publish_cluster_debug").as_bool();
        
        // Create subscriber for pointcloud
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ti_mmwave/radar_scan_pcl",
            10,
            std::bind(&ClusterLaserScan::pointcloudCallback, this, std::placeholders::_1)
        );
        
        // Create publisher for laserscan
        laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/cluster_laserscan",
            10
        );
        
        // Create debug publisher if enabled
        if (publish_cluster_debug_) {
            cluster_debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/cluster_debug_pcl",
                10
            );
        }
        
        RCLCPP_INFO(this->get_logger(), "ClusterLaserScan node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /ti_mmwave/radar_scan_pcl");
        RCLCPP_INFO(this->get_logger(), "Publishing LaserScan to: /cluster_laserscan");
        RCLCPP_INFO(this->get_logger(), "Clustering parameters:");
        RCLCPP_INFO(this->get_logger(), "  - Cluster tolerance (epsilon): %.3f m", cluster_tolerance_);
        RCLCPP_INFO(this->get_logger(), "  - Min cluster size: %d points", min_cluster_size_);
        RCLCPP_INFO(this->get_logger(), "  - Max cluster size: %d points", max_cluster_size_);
        RCLCPP_INFO(this->get_logger(), "Line parameters:");
        RCLCPP_INFO(this->get_logger(), "  - Line length: %.3f m", line_length_);
        RCLCPP_INFO(this->get_logger(), "  - Line orientation: %s", line_orientation_.c_str());
        RCLCPP_INFO(this->get_logger(), "LaserScan parameters:");
        RCLCPP_INFO(this->get_logger(), "  - Angle range: [%.2f, %.2f] rad", angle_min_, angle_max_);
        RCLCPP_INFO(this->get_logger(), "  - Range limits: [%.2f, %.2f] m", range_min_, range_max_);
        
        if (enable_intensity_filter_) {
            RCLCPP_INFO(this->get_logger(), "Intensity filter ENABLED: threshold = %.2f", intensity_threshold_);
        }
    }

    ClusterLaserScan::~ClusterLaserScan()
    {
    }
    
    void ClusterLaserScan::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Extract points from PointCloud2
        std::vector<Point3D> points;
        extractPoints(*msg, points);
        
        if (points.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "No points to process after filtering");
            return;
        }
        
        // Perform clustering
        std::vector<Cluster> clusters;
        performClustering(points, clusters);
        
        RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters from %zu points", 
                    clusters.size(), points.size());
        
        // Generate LaserScan from clusters
        sensor_msgs::msg::LaserScan laser_scan;
        generateLaserScan(clusters, laser_scan, msg->header);
        
        // Publish LaserScan
        laserscan_pub_->publish(laser_scan);
    }
    
    void ClusterLaserScan::extractPoints(const sensor_msgs::msg::PointCloud2 & cloud_msg,
                                        std::vector<Point3D> & points)
    {
        points.clear();
        
        // Find field offsets
        int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
        for (size_t i = 0; i < cloud_msg.fields.size(); ++i) {
            if (cloud_msg.fields[i].name == "x") x_offset = cloud_msg.fields[i].offset;
            if (cloud_msg.fields[i].name == "y") y_offset = cloud_msg.fields[i].offset;
            if (cloud_msg.fields[i].name == "z") z_offset = cloud_msg.fields[i].offset;
            if (cloud_msg.fields[i].name == "intensity") intensity_offset = cloud_msg.fields[i].offset;
        }
        
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            RCLCPP_ERROR(this->get_logger(), "PointCloud2 missing x, y, or z fields");
            return;
        }
        
        bool has_intensity = (intensity_offset >= 0);
        size_t point_step = cloud_msg.point_step;
        size_t num_points = cloud_msg.width * cloud_msg.height;
        
        points.reserve(num_points);
        
        for (size_t i = 0; i < num_points; ++i) {
            size_t offset = i * point_step;
            
            float x = *reinterpret_cast<const float*>(&cloud_msg.data[offset + x_offset]);
            float y = *reinterpret_cast<const float*>(&cloud_msg.data[offset + y_offset]);
            float z = *reinterpret_cast<const float*>(&cloud_msg.data[offset + z_offset]);
            float intensity = 0.0f;
            
            if (has_intensity) {
                intensity = *reinterpret_cast<const float*>(&cloud_msg.data[offset + intensity_offset]);
            }
            
            // Skip invalid points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }
            
            // Apply intensity filter if enabled
            if (enable_intensity_filter_ && intensity < intensity_threshold_) {
                continue;
            }
            
            // Apply range filter
            float range = std::sqrt(x*x + y*y);
            if (range < range_min_ || range > range_max_) {
                continue;
            }
            
            points.emplace_back(x, y, z, intensity);
        }
    }
    
    void ClusterLaserScan::performClustering(std::vector<Point3D> & points,
                                            std::vector<Cluster> & clusters)
    {
        clusters.clear();
        
        if (points.empty()) {
            return;
        }
        
        int cluster_id = 0;
        
        // DBSCAN-style clustering
        for (size_t i = 0; i < points.size(); ++i) {
            // Skip already processed points
            if (points[i].cluster_id != -1) {
                continue;
            }
            
            // Find neighbors
            std::vector<size_t> neighbors = regionQuery(points, i);
            
            // Mark as noise if not enough neighbors
            if (neighbors.size() < static_cast<size_t>(min_cluster_size_)) {
                points[i].cluster_id = -2; // noise
                continue;
            }
            
            // Start a new cluster
            points[i].cluster_id = cluster_id;
            
            // Expand cluster
            expandCluster(points, i, cluster_id, neighbors);
            
            cluster_id++;
        }
        
        // Extract clusters
        std::map<int, Cluster> cluster_map;
        for (const auto & point : points) {
            if (point.cluster_id >= 0) {
                cluster_map[point.cluster_id].points.push_back(point);
            }
        }
        
        // Filter clusters by size and calculate centroids
        for (auto & pair : cluster_map) {
            Cluster & cluster = pair.second;
            cluster.point_count = cluster.points.size();
            
            // Check cluster size constraints
            if (cluster.point_count >= static_cast<size_t>(min_cluster_size_) &&
                cluster.point_count <= static_cast<size_t>(max_cluster_size_)) {
                cluster.centroid = calculateCentroid(cluster);
                clusters.push_back(cluster);
            }
        }
    }
    
    inline float ClusterLaserScan::euclideanDistance(const Point3D & p1, const Point3D & p2)
    {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    std::vector<size_t> ClusterLaserScan::regionQuery(const std::vector<Point3D> & points,
                                                     size_t point_idx)
    {
        std::vector<size_t> neighbors;
        const Point3D & query_point = points[point_idx];
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (i == point_idx) continue;
            
            if (euclideanDistance(query_point, points[i]) <= cluster_tolerance_) {
                neighbors.push_back(i);
            }
        }
        
        return neighbors;
    }
    
    void ClusterLaserScan::expandCluster(std::vector<Point3D> & points,
                                        size_t /* point_idx */,
                                        int cluster_id,
                                        std::vector<size_t> & neighbors)
    {
        size_t i = 0;
        while (i < neighbors.size()) {
            size_t neighbor_idx = neighbors[i];
            
            // If point was noise, add to cluster
            if (points[neighbor_idx].cluster_id == -2) {
                points[neighbor_idx].cluster_id = cluster_id;
            }
            // If point not yet processed
            else if (points[neighbor_idx].cluster_id == -1) {
                points[neighbor_idx].cluster_id = cluster_id;
                
                // Find neighbors of this point
                std::vector<size_t> neighbor_neighbors = regionQuery(points, neighbor_idx);
                
                // If this point has enough neighbors, add them to the cluster
                if (neighbor_neighbors.size() >= static_cast<size_t>(min_cluster_size_)) {
                    neighbors.insert(neighbors.end(), 
                                    neighbor_neighbors.begin(), 
                                    neighbor_neighbors.end());
                }
            }
            
            i++;
        }
    }
    
    Point3D ClusterLaserScan::calculateCentroid(const Cluster & cluster)
    {
        Point3D centroid;
        float sum_x = 0, sum_y = 0, sum_z = 0, sum_intensity = 0;
        
        for (const auto & point : cluster.points) {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            sum_intensity += point.intensity;
        }
        
        size_t n = cluster.points.size();
        centroid.x = sum_x / n;
        centroid.y = sum_y / n;
        centroid.z = sum_z / n;
        centroid.intensity = sum_intensity / n;
        
        return centroid;
    }
    
    void ClusterLaserScan::createLineFromCentroid(const Point3D & centroid,
                                                  Point3D & start_point,
                                                  Point3D & end_point)
    {
        float half_length = line_length_ / 2.0;
        
        if (line_orientation_ == "perpendicular") {
            // Create line perpendicular to radial direction
            // Radial angle from origin to centroid
            float angle = std::atan2(centroid.y, centroid.x);
            
            // Perpendicular angle (90 degrees offset)
            float perp_angle = angle + M_PI / 2.0;
            
            // Calculate line endpoints
            start_point.x = centroid.x - half_length * std::cos(perp_angle);
            start_point.y = centroid.y - half_length * std::sin(perp_angle);
            start_point.z = centroid.z;
            
            end_point.x = centroid.x + half_length * std::cos(perp_angle);
            end_point.y = centroid.y + half_length * std::sin(perp_angle);
            end_point.z = centroid.z;
        }
        else if (line_orientation_ == "radial") {
            // Create line along radial direction
            float angle = std::atan2(centroid.y, centroid.x);
            float range = std::sqrt(centroid.x * centroid.x + centroid.y * centroid.y);
            
            // Calculate line endpoints
            float start_range = range - half_length;
            float end_range = range + half_length;
            
            start_point.x = start_range * std::cos(angle);
            start_point.y = start_range * std::sin(angle);
            start_point.z = centroid.z;
            
            end_point.x = end_range * std::cos(angle);
            end_point.y = end_range * std::sin(angle);
            end_point.z = centroid.z;
        }
        else {
            // Default: horizontal line (perpendicular)
            start_point.x = centroid.x - half_length;
            start_point.y = centroid.y;
            start_point.z = centroid.z;
            
            end_point.x = centroid.x + half_length;
            end_point.y = centroid.y;
            end_point.z = centroid.z;
        }
    }
    
    void ClusterLaserScan::generateLaserScan(const std::vector<Cluster> & clusters,
                                            sensor_msgs::msg::LaserScan & laser_scan,
                                            const std_msgs::msg::Header & header)
    {
        // Initialize LaserScan message
        laser_scan.header = header;
        laser_scan.header.frame_id = frame_id_;
        
        laser_scan.angle_min = angle_min_;
        laser_scan.angle_max = angle_max_;
        laser_scan.angle_increment = angle_increment_;
        laser_scan.time_increment = 0.0;
        laser_scan.scan_time = 0.0;
        laser_scan.range_min = range_min_;
        laser_scan.range_max = range_max_;
        
        // Calculate number of beams
        size_t num_beams = std::ceil((angle_max_ - angle_min_) / angle_increment_);
        laser_scan.ranges.assign(num_beams, std::numeric_limits<float>::infinity());
        laser_scan.intensities.assign(num_beams, 0.0);
        
        // For each cluster, create a line and sample points along it
        for (const auto & cluster : clusters) {
            Point3D start_point, end_point;
            createLineFromCentroid(cluster.centroid, start_point, end_point);
            
            // Sample points along the line
            int num_samples = 20; // Number of points to sample along the line
            for (int i = 0; i <= num_samples; ++i) {
                float t = static_cast<float>(i) / num_samples;
                
                // Interpolate point along the line
                float x = start_point.x + t * (end_point.x - start_point.x);
                float y = start_point.y + t * (end_point.y - start_point.y);
                
                // Calculate angle and range
                float angle = std::atan2(y, x);
                float range = std::sqrt(x*x + y*y);
                
                // Check if angle is within scan range
                if (angle < angle_min_ || angle > angle_max_) {
                    continue;
                }
                
                // Calculate beam index
                int beam_idx = static_cast<int>((angle - angle_min_) / angle_increment_);
                if (beam_idx < 0 || beam_idx >= static_cast<int>(num_beams)) {
                    continue;
                }
                
                // Update laser scan if this range is closer
                if (range >= range_min_ && range <= range_max_) {
                    if (range < laser_scan.ranges[beam_idx]) {
                        laser_scan.ranges[beam_idx] = range;
                        laser_scan.intensities[beam_idx] = cluster.centroid.intensity;
                    }
                }
            }
        }
    }
    
} // namespace cluster_laserscan
