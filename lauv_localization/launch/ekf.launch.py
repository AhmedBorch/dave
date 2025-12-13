from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/opt/dave_ws/src/dave/lauv_localization/config/ekf.yaml'],
    )

    return LaunchDescription([ekf_node])

