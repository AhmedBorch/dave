import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('heading_publisher'),
        'config', 'heading_publisher.yaml',
    )

    noise_arg = DeclareLaunchArgument(
        'noise_std',
        default_value='0.05',
        description='Heading noise std dev (rad). Increase to simulate compass interference.',
    )

    node = Node(
        package='heading_publisher',
        executable='heading_publisher',
        name='heading_publisher',
        parameters=[params, {'noise_std': LaunchConfiguration('noise_std')}],
        output='screen',
    )

    return LaunchDescription([noise_arg, node])
