from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file for cluster-based LaserScan generation from radar pointcloud.
    
    This node performs:
    1. Clustering of radar pointcloud data (DBSCAN algorithm)
    2. Centroid calculation for each cluster
    3. Line generation at each centroid with configurable length and orientation
    4. Publishing lines as LaserScan messages
    """
    
    # Get the package directory
    package_dir = get_package_share_directory('radar2scan')
    
    # Path to configuration file
    config_file = os.path.join(package_dir, 'config', 'radar2scan_params.yaml')
    
    # Create the cluster_laserscan node
    cluster_laserscan_node = Node(
        package='radar2scan',
        executable='cluster_laserscan_node',
        name='cluster_laserscan_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Optional remapping if you want to change topic names
            # ('/ti_mmwave/radar_scan_pcl', '/your_custom_pcl_topic'),
            # ('/cluster_laserscan', '/your_custom_scan_topic'),
        ]
    )
    
    return LaunchDescription([
        cluster_laserscan_node
    ])
