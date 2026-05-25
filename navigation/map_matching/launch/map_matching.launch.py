from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('map_matching')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg, 'config', 'map_matching.yaml']),
        description='Path to map_matching parameter YAML.',
    )
    map_arg = DeclareLaunchArgument(
        'full_map_path',
        default_value='',
        description='Override path to the reference PCD of the structure.',
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='/model/bluerov2/lidar/points',
        description='PointCloud2 topic carrying the submap.',
    )

    node = Node(
        package='map_matching',
        executable='map_matching_node',
        name='map_matching',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'full_map_path': LaunchConfiguration('full_map_path'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
            },
        ],
    )

    return LaunchDescription([config_arg, map_arg, cloud_topic_arg, node])
