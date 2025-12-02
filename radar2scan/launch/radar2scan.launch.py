from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('radar2scan'),
            'config',
            'radar2scan_params.yaml'
        ]),
        description='Path to the radar2scan configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Radar processor node
    radar_processor_node = Node(
        package='radar2scan',
        executable='radar2scan_node',
        name='radar_processor',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
        remappings=[
            # Uncomment and modify if you need to remap topics
            # ('/ti_mmwave/radar_scan_pcl', '/your/custom/input/topic'),
            # ('/radar_scan', '/your/custom/output/topic'),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        radar_processor_node
    ])
