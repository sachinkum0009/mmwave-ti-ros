#ifndef RADAR_PROCESSING_HPP_
#define RADAR_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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
        
        // LaserScan parameters
        double angle_min_;
        double angle_max_;
        double angle_increment_;
        double range_min_;
        double range_max_;
        std::string frame_id_;
    };
}; // namespace radar_processing

#endif // RADAR_PROCESSING_HPP_