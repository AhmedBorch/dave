from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('goto_planner')

    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'joy_teleop.yaml']),
            description='Path to joy_teleop parameter YAML.',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path.',
        ),
    ]

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device': LaunchConfiguration('joy_dev')}],
        output='screen',
    )

    teleop_node = Node(
        package='goto_planner',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription(args + [joy_node, teleop_node])
