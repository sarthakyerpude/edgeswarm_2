"""
ONE agent + ONE mock body. The smallest thing that can run.

    ros2 launch amr_description single_robot_test.launch.py

Use this first, before three robots. It proves: the package built, the
interfaces generated, the grid loaded, parameters bound, and the node reached
its tick loop. If this fails, nothing else will work, and the failure is much
easier to read with one process than with eight.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rid = LaunchConfiguration("robot_id")
    grid = PathJoinSubstitution([
        FindPackageShare("amr_description"), "config", "warehouse_grid.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="robot_1"),

        Node(package="amr_description", executable="mock_robot",
             name="mock_robot", namespace=rid, output="screen",
             emulate_tty=True,
             parameters=[{"robot_id": rid, "start_x": -3.0, "start_y": -4.0,
                          "start_theta": 1.5708, "use_sim_time": False}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("amr_description"), "launch",
                "fleet_agent.launch.py"])),
            launch_arguments={"robot_id": rid, "use_sim_time": "false",
                              "grid_yaml": grid}.items()),

        Node(package="amr_description", executable="fleet_monitor",
             name="fleet_monitor", output="screen", emulate_tty=True,
             parameters=[{"use_sim_time": False, "report_interval_s": 2.0}]),
    ])
