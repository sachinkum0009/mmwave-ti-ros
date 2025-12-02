#include "radar2scan/radar_processing.hpp"

namespace radar_processing
{
    RadarProcessor::RadarProcessor()
    : Node("radar_processor")
    {
        // Declare and get parameters
        this->declare_parameter("angle_min", -M_PI);
        this->declare_parameter("angle_max", M_PI);
        this->declare_parameter("angle_increment", 0.005); // ~0.3 degrees
        this->declare_parameter("range_min", 0.1);
        this->declare_parameter("range_max", 10.0);
        this->declare_parameter("frame_id", "ti_mmwave_0");
        
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();
        angle_increment_ = this->get_parameter("angle_increment").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Create subscriber for radar pointcloud
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ti_mmwave/radar_scan_pcl",
            10,
            std::bind(&RadarProcessor::radarCallback, this, std::placeholders::_1)
        );
        
        // Create publisher for laserscan
        laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/radar_scan",
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "RadarProcessor node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /ti_mmwave/radar_scan_pcl");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /radar_scan");
        RCLCPP_INFO(this->get_logger(), "Angle range: [%.2f, %.2f] rad", angle_min_, angle_max_);
        RCLCPP_INFO(this->get_logger(), "Range limits: [%.2f, %.2f] m", range_min_, range_max_);
    }

    RadarProcessor::~RadarProcessor()
    {
    }
    
    void RadarProcessor::radarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Create LaserScan message
        sensor_msgs::msg::LaserScan laser_scan;
        
        // Process the pointcloud data
        processRadarData(*msg, laser_scan);
        
        // Publish the laserscan
        laserscan_pub_->publish(laser_scan);
    }

    void RadarProcessor::processRadarData(const sensor_msgs::msg::PointCloud2 & radar_data, 
                                         sensor_msgs::msg::LaserScan & laser_scan)
    {
        // Initialize LaserScan message header
        laser_scan.header = radar_data.header;
        laser_scan.header.frame_id = frame_id_;
        
        // Set LaserScan parameters
        laser_scan.angle_min = angle_min_;
        laser_scan.angle_max = angle_max_;
        laser_scan.angle_increment = angle_increment_;
        laser_scan.time_increment = 0.0;
        laser_scan.scan_time = 0.0;
        laser_scan.range_min = range_min_;
        laser_scan.range_max = range_max_;
        
        // Calculate number of laser beams
        size_t num_beams = std::ceil((angle_max_ - angle_min_) / angle_increment_);
        laser_scan.ranges.assign(num_beams, std::numeric_limits<float>::infinity());
        laser_scan.intensities.assign(num_beams, 0.0);
        
        // Check if intensity field exists
        bool has_intensity = false;
        int intensity_offset = -1;
        for (size_t i = 0; i < radar_data.fields.size(); ++i) {
            if (radar_data.fields[i].name == "intensity") {
                has_intensity = true;
                intensity_offset = radar_data.fields[i].offset;
                break;
            }
        }
        
        // Get field offsets for x, y, z
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (size_t i = 0; i < radar_data.fields.size(); ++i) {
            if (radar_data.fields[i].name == "x") x_offset = radar_data.fields[i].offset;
            if (radar_data.fields[i].name == "y") y_offset = radar_data.fields[i].offset;
            if (radar_data.fields[i].name == "z") z_offset = radar_data.fields[i].offset;
        }
        
        // Calculate number of points
        size_t point_step = radar_data.point_step;
        size_t num_points = radar_data.width * radar_data.height;
        
        // Process points in parallel using OpenMP
        #pragma omp parallel for
        for (size_t i = 0; i < num_points; ++i) {
            size_t point_offset = i * point_step;
            
            // Extract x, y, z coordinates
            float x = *reinterpret_cast<const float*>(&radar_data.data[point_offset + x_offset]);
            float y = *reinterpret_cast<const float*>(&radar_data.data[point_offset + y_offset]);
            // float z = *reinterpret_cast<const float*>(&radar_data.data[point_offset + z_offset]);
            
            // Calculate 2D range (ignoring z component for 2D scan)
            float range = std::sqrt(x * x + y * y);
            
            // Filter out points outside range limits
            if (range < range_min_ || range > range_max_) {
                continue;
            }
            
            // Calculate angle (atan2 returns angle in range [-pi, pi])
            float angle = std::atan2(y, x);
            
            // Check if angle is within scan range
            if (angle < angle_min_ || angle > angle_max_) {
                continue;
            }
            
            // Calculate beam index
            int beam_index = static_cast<int>((angle - angle_min_) / angle_increment_);
            
            // Ensure beam index is within valid range
            if (beam_index >= 0 && beam_index < static_cast<int>(num_beams)) {
                // Thread-safe update using critical section
                #pragma omp critical
                {
                    // Keep the closest point for each beam
                    if (range < laser_scan.ranges[beam_index]) {
                        laser_scan.ranges[beam_index] = range;
                        
                        // Set intensity if available
                        if (has_intensity) {
                            float intensity = *reinterpret_cast<const float*>(&radar_data.data[point_offset + intensity_offset]);
                            laser_scan.intensities[beam_index] = intensity;
                        }
                    }
                }
            }
        }
    }

} // namespace radar_processing

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_processing::RadarProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}