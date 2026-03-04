import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to your new assets
    pkg_dave_worlds = get_package_share_directory('dave_worlds')
    pkg_dave_sensor_models = get_package_share_directory('dave_sensor_models')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to your new world file
    world_path = os.path.join(pkg_dave_worlds, 'worlds', 'lidar_3d.world')

    # 2. Launch Gazebo with the new world
    # We include the standard Gazebo Sim launch but pass our custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path} --render-engine ogre'
        }.items(),
    )

    # 3. ROS-Gazebo Bridge
    # This replaces the DAVE multibeam bridge with one for your 3D LiDAR
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # LiDAR LaserScan
            '/lidar_3d/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # LiDAR PointCloud
            '/lidar_3d/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Clock bridge (essential for TF and sensor timing)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # 4. Static TF Publisher
    # Connects the sensor frame to the world frame for RViz visualization
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'world', 'lidar_3d/link'],
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        tf_node
    ])