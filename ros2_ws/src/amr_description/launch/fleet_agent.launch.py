"""
Launch ONE fleet_agent. The reusable unit - all other launch files include it.

The SAME code serves robot_1, robot_2 and robot_3. There is no per-robot copy
of anything: the robot_id parameter plus the namespace is the entire
difference. That is a hard requirement of the project and it is enforced here.

    ros2 launch amr_description fleet_agent.launch.py robot_id:=robot_2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("amr_description")

    robot_id = LaunchConfiguration("robot_id")
    mode = LaunchConfiguration("coordination_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pose_source = LaunchConfiguration("pose_source")
    grid_yaml = LaunchConfiguration("grid_yaml")
    params_file = LaunchConfiguration("params_file")
    gate = LaunchConfiguration("enable_cmd_vel_gate")
    home_x = LaunchConfiguration("home_x")
    home_y = LaunchConfiguration("home_y")
    home_yaw = LaunchConfiguration("home_yaw")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id", default_value="robot_1",
            description="Unique robot identity. Also used as the namespace."),
        DeclareLaunchArgument(
            "coordination_mode", default_value="proposed",
            description="'proposed' (full system) or 'baseline' (stop-and-wait)"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description=("MUST be true whenever Gazebo publishes /clock. If it "
                         "is false while Gazebo drives the clock, every TF "
                         "lookup fails with 'extrapolation into the future' - "
                         "an error that looks like a TF bug and costs a day.")),
        DeclareLaunchArgument(
            "pose_source", default_value="tf",
            description="tf for map-frame localization; odom for mock fixtures only."),
        DeclareLaunchArgument(
            "grid_yaml",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_description"), "config",
                "warehouse_grid.yaml"]),
            description="Occupancy + zone definitions. Same file on every robot."),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "fleet_params.yaml")),
        DeclareLaunchArgument(
            "enable_cmd_vel_gate", default_value="false",
            description=("false: publish motion_permit only, amr_navigation "
                         "owns cmd_vel. true: subscribe cmd_vel_raw and "
                         "publish gated cmd_vel. Agree with the nav team "
                         "before enabling.")),
        DeclareLaunchArgument(
            "home_x", default_value=".nan",
            description="Dock/charging pose x (m, map). Normally the spawn "
                        "pose; .nan (YAML NaN) disables return-to-dock."),
        DeclareLaunchArgument("home_y", default_value=".nan"),
        DeclareLaunchArgument(
            "home_yaw", default_value=".nan",
            description="Dock heading (rad); the robot aligns to it before "
                        "charging. .nan = any heading."),

        Node(
            package="amr_description",
            executable="fleet_agent",
            name="fleet_agent",
            # NAMESPACE: makes every RELATIVE topic resolve under /<robot_id>/.
            # 'odom' -> /robot_1/odom, 'motion_permit' -> /robot_1/motion_permit.
            # The /fleet/* topics use ABSOLUTE names in the code, so they are
            # NOT namespaced and remain shared across the fleet.
            namespace=robot_id,
            output="screen",
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    "robot_id": robot_id,
                    "coordination_mode": mode,
                    "use_sim_time": use_sim_time,
                    "pose_source": pose_source,
                    "grid_yaml": grid_yaml,
                    "enable_cmd_vel_gate": gate,
                    "home_x": ParameterValue(home_x, value_type=float),
                    "home_y": ParameterValue(home_y, value_type=float),
                    "home_yaw": ParameterValue(home_yaw, value_type=float),
                },
            ],
            # ROS 2 Jazzy's tf2 TransformListener subscribes to absolute /tf
            # names; remap those names into this robot's isolated TF tree.
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
    ])
