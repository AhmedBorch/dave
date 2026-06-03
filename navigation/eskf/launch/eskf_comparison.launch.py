"""Launch two ESKF instances simultaneously for comparison.

Both nodes keep the executable name 'eskf_node' so that the YAML parameter
file (keyed on '/**/eskf_node/ros__parameters') matches both.
They are separated by ROS 2 namespace:

  /eskf_base/eskf_node  — IMU + DVL + depth + heading only
  /eskf_map/eskf_node   — same sensors + map-matcher voted pose

Output topics are overridden so they land at well-known absolute paths:

  /model/bluerov2/eskf/odom      (base, no map)
  /model/bluerov2/eskf_map/odom  (with map)

Usage:
  ros2 launch eskf eskf_comparison.launch.py \\
      map_pose_topic:=/map_matching/average_pose
"""
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
        default_value='/map_matching/average_pose',
        description='Voted pose topic from the map-matcher (instance 2 only)',
    )

    eskf_params = os.path.join(
        get_package_share_directory('eskf'), 'config', 'eskf_params.yaml'
    )

    # --- Instance 1: no map-matcher ---
    eskf_base = Node(
        package='eskf',
        executable='eskf_node',
        name='eskf_node',
        namespace='eskf_base',   # FQN: /eskf_base/eskf_node  → YAML matches
        parameters=[
            eskf_params,
            {
                'frame_prefix': LaunchConfiguration('namespace'),
                'topics.map_pose': '',
                'topics.odom':  '/model/bluerov2/eskf/odom',
                'topics.pose':  '/model/bluerov2/eskf/pose',
                'topics.twist': '/model/bluerov2/eskf/twist',
                'publish_tf': True,
            },
        ],
        output='screen',
    )

    # --- Instance 2: with map-matcher pose updates ---
    eskf_map = Node(
        package='eskf',
        executable='eskf_node',
        name='eskf_node',
        namespace='eskf_map',    # FQN: /eskf_map/eskf_node   → YAML matches
        parameters=[
            eskf_params,
            {
                'frame_prefix': LaunchConfiguration('namespace'),
                'topics.map_pose': LaunchConfiguration('map_pose_topic'),
                'topics.odom':  '/model/bluerov2/eskf_map/odom',
                'topics.pose':  '/model/bluerov2/eskf_map/pose',
                'topics.twist': '/model/bluerov2/eskf_map/twist',
                # Disable TF to avoid collisions with eskf_base.
                'publish_tf': False,
            },
        ],
        output='screen',
    )

    return LaunchDescription([namespace_arg, map_pose_topic_arg, eskf_base, eskf_map])
