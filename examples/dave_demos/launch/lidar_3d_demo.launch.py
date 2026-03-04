import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to your new assets
    pkg_dave_worlds = get_package_share_directory('dave_worlds')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_dave_demos = get_package_share_directory('dave_demos')

    # Path to your new world file
    world_path = os.path.join(pkg_dave_worlds, 'worlds', 'lidar_3d.world')

    # 2. Launch Gazebo with the new world
    # We include the standard Gazebo Sim launch but pass our custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dave_demos, 'launch', "dave_sensor.launch.py")
        ),
        launch_arguments={
            "namespace": "lidar_3d",
            "world_name": "lidar_3d",
            "paused": "false",
            "debug": "true",
            "x": "5.8",
            "z": "2",
            "yaw": "3.14",
            'gz_args': f'-r {world_path} --render-engine ogre2'
        }.items(),
    )

    # This replaces the DAVE multibeam bridge with one for your 3D LiDAR
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # LiDAR LaserScan
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # LiDAR PointCloud
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Clock bridge (essential for TF and sensor timing)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
        ('/lidar', '/lidar_3d/lidar'),
        ('/lidar/points', '/lidar_3d/lidar/points')
        ],
        output='screen'
    )

    # Connects the sensor frame to the world frame for RViz visualization
    tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0.5',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'world',
        '--child-frame-id', 'lidar_3d/lidar_3d_base_link/gpu_lidar'
    ],
)

    return LaunchDescription([
        gz_sim,
        bridge,
        tf_node
    ])