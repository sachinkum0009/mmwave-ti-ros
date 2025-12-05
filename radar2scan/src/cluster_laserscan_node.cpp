#include "radar2scan/cluster_laserscan.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cluster_laserscan::ClusterLaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
