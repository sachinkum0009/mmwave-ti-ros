#ifndef RADAR_PROCESSING_HPP_
#define RADAR_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <limits>
#include <vector>

namespace radar_processing
{
    class RadarProcessor : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor
         */
        RadarProcessor();
        
        /**
         * @brief Destructor
         */
        ~RadarProcessor();

    private:
        /**
         * @brief Callback for radar pointcloud data
         * 
         * @param msg Input radar data in PointCloud2 format
         */
        void radarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
        /**
         * @brief Process radar data from PointCloud2 to LaserScan
         * 
         * @param radar_data Input radar data in PointCloud2 format
         * @param laser_scan Output laser scan data
         */
        void processRadarData(const sensor_msgs::msg::PointCloud2 & radar_data, 
                             sensor_msgs::msg::LaserScan & laser_scan);

        // ROS2 subscriber and publisher
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        
        // LaserScan parameters
        double angle_min_;
        double angle_max_;
        double angle_increment_;
        double range_min_;
        double range_max_;
        std::string frame_id_;
        
        // Intensity filtering parameters
        bool enable_intensity_filter_;
        double intensity_threshold_;
        
        // Visualization parameters
        double box_size_;
        
        // CUDA support
        bool use_cuda_;
        bool cuda_available_;
        unsigned char* d_data_;      // Device pointer for pointcloud data
        float* d_ranges_;            // Device pointer for ranges
        float* d_intensities_;       // Device pointer for intensities
        size_t d_data_size_;         // Allocated device memory size
        size_t d_ranges_size_;       // Allocated ranges array size
    };
}; // namespace radar_processing

#endif // RADAR_PROCESSING_HPP_