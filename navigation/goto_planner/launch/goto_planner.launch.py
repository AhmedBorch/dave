from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)

    # Only inject goal overrides when the user explicitly passes them on the
    # command line (non-empty string).  An empty default means "use the YAML
    # value" — this prevents the launch file from silently stomping whatever
    # goal is set in the YAML.
    goal_overrides = {}
    for key in ('goal_x', 'goal_y', 'goal_z'):
        val = LaunchConfiguration(key).perform(context)
        if val != '':
            goal_overrides[key] = float(val)

    params = [config_file]
    if goal_overrides:
        params.append(goal_overrides)

    node = Node(
        package='goto_planner',
        executable='goto_planner',
        name='goto_planner',
        output='screen',
        parameters=params,
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('goto_planner')

    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'goto_planner.yaml']),
            description='Path to goto_planner parameter YAML.',
        ),
        DeclareLaunchArgument(
            'goal_x', default_value='',
            description='Goal X (m). Overrides YAML when set.',
        ),
        DeclareLaunchArgument(
            'goal_y', default_value='',
            description='Goal Y (m). Overrides YAML when set.',
        ),
        DeclareLaunchArgument(
            'goal_z', default_value='',
            description='Goal Z (m). Overrides YAML when set.',
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
