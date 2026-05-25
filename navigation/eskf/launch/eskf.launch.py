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
    return LaunchDescription([
        
        # ---------------------------------------------------------
        # INSTANCE 1: The Local Filter (Smooth, Odom -> Base_Link)
        # ---------------------------------------------------------
        Node(
            package='eskf',
            executable='eskf_node',
            name='eskf_local', # Give it a unique name
            parameters=[
                {'publish_tf': True},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'odom'} # Tells this filter its top-level frame is odom
            ],
            remappings=[
                # (Inside node name, Outside ROS2 topic name)
                ('imu', '/sensors/imu/data'),
                ('dvl', '/sensors/dvl/data'),
                ('pose', '/dummy_topic'), # Ignore pose for the local filter
                ('odometry/filtered', '/odometry/local') # Output topic
            ]
        ),

        # ---------------------------------------------------------
        # INSTANCE 2: The Global Filter (Jumps, Map -> Odom)
        # ---------------------------------------------------------
        Node(
            package='eskf',
            executable='eskf_node',
            name='eskf_global', # Unique name for the second instance
            parameters=[
                {'publish_tf': True},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'map'} # Tells this filter its top-level frame is map
            ],
            remappings=[
                ('imu', '/sensors/imu/data'),
                ('dvl', '/sensors/dvl/data'),
                ('pose', '/map_matcher/pose'), # THIS filter listens to the 3D Sonar!
                ('odometry/filtered', '/odometry/global') # Output topic
            ]
        )
    ])
