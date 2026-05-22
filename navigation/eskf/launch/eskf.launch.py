import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bluerov2',
        description='Robot namespace — used as frame_prefix',
    )

    eskf_params = os.path.join(
        get_package_share_directory('eskf'), 'config', 'eskf_params.yaml'
    )

    eskf_node = Node(
        package='eskf',
        executable='eskf_node',
        name='eskf_node',
        parameters=[
            eskf_params,
            {'frame_prefix': LaunchConfiguration('namespace')},
        ],
        output='screen',
    )

    return LaunchDescription([namespace_arg, eskf_node])
