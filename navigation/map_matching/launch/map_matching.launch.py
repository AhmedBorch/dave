from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    config_file   = LaunchConfiguration('config_file').perform(context)
    full_map_path = LaunchConfiguration('full_map_path').perform(context)
    cloud_topic   = LaunchConfiguration('input_cloud_topic').perform(context)

    # Only inject overrides when explicitly provided on the command line.
    # An empty string means "use the YAML value".
    overrides = {}
    if full_map_path:
        overrides['full_map_path'] = full_map_path
    if cloud_topic:
        overrides['input_cloud_topic'] = cloud_topic

    params = [config_file]
    if overrides:
        params.append(overrides)

    node = Node(
        package='map_matching',
        executable='map_matching_node',
        name='map_matching',
        output='screen',
        parameters=params,
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('map_matching')

    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'map_matching.yaml']),
            description='Path to map_matching parameter YAML.',
        ),
        DeclareLaunchArgument(
            'full_map_path',
            default_value='',
            description='Path to the reference PCD of the structure. Overrides YAML when set.',
        ),
        DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='',
            description='PointCloud2 topic. Overrides YAML when set.',
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
