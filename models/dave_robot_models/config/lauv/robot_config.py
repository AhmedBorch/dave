from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)

    # LAUV has 1 thruster and 4 fins based on the model.sdf
    thruster_joints = [f"/model/{namespace}/joint/thruster_0_joint"]
    
    fin_joints = []
    for fin in range(4):
        fin_joints.append(f"/model/{namespace}/joint/fin_{fin}_joint")

    lauv_arguments = (
        # Fin commands
        [f"{fin_joint}/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double" for fin_joint in fin_joints]
        +
        # Thruster commands
        [f"{thruster_joint}/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double" for thruster_joint in thruster_joints]
        # Sensor data
        + [
            f"/model/{namespace}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            f"/model/{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            f"/model/{namespace}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            # f"/model/{namespace}/dvl/velocity@dave_interfaces/msg/DVL@gz.msgs.DVL"        
            ]
    )

    lauv_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=lauv_arguments,
        output="screen",
        remappings=[
            (f"/model/{namespace}/joint_states", f"/{namespace}/joint_states"),
            (f"/model/{namespace}/odometry", f"/{namespace}/odometry"),
            (f"/model/{namespace}/imu", f"/{namespace}/imu"),
            # (f"/model/{namespace}/dvl/velocity", f"/{namespace}/dvl"),
        ],
    )

    return [lauv_bridge]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "namespace",
            default_value="lauv",
            description="Namespace for the LAUV robot",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])