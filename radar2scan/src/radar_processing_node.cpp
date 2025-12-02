#include "radar2scan/radar_processing.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_processing::RadarProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}