import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dave_demos = get_package_share_directory('dave_demos')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dave_demos, 'launch', 'dave_robot.launch.py')
        ),
        launch_arguments={
            'namespace': 'bluerov2_heavy_lidar_3d',
            'world_name': 'lidar_3d',
            'paused': 'false',
            'x': '0.0',
            'y': '0.0',
            'z': '-0.5',
        }.items(),
    )

    return LaunchDescription([robot_launch])
