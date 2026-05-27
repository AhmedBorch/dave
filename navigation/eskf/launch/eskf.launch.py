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

    map_pose_topic_arg = DeclareLaunchArgument(
        'map_pose_topic',
        default_value='',
        description='Topic for map-matcher pose updates (empty = disabled)',
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
            {'topics.map_pose': LaunchConfiguration('map_pose_topic')},
        ],
        output='screen',
    )

    return LaunchDescription([namespace_arg, map_pose_topic_arg, eskf_node])
