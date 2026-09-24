"""
THREE FLEET AGENTS attached to the team's EXISTING Gazebo simulation.

    ros2 launch amr_description three_robots_gazebo.launch.py

WHAT THIS LAUNCHES:  three fleet_agents, a task generator, the monitor.
WHAT THIS DOES NOT LAUNCH:  Gazebo, the warehouse world, the robot models,
                            or ros_gz_bridge.

Those belong to warehouse_sim and amr_navigation, which this branch is not
permitted to modify. Start the simulation first, using whatever launch file
that team provides, then run this.

    Terminal 1:  ros2 launch warehouse_sim <their_world_launch>.py
    Terminal 2:  ros2 launch amr_description three_robots_gazebo.launch.py

PREREQUISITE - verify before running:
    ros2 topic list | grep -E "robot_[123]/(odom|scan)"
    ros2 topic hz /robot_1/odom
    ros2 topic hz /clock
If /robot_N/odom and /robot_N/scan are not present with those exact names, see
docs/GAZEBO_INTEGRATION.md for the required ros_gz_bridge configuration.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ROBOT_IDS = ["robot_1", "robot_2", "robot_3"]


def generate_launch_description():
    mode = LaunchConfiguration("coordination_mode")
    gate = LaunchConfiguration("enable_cmd_vel_gate")
    tasks = LaunchConfiguration("enable_tasks")
    monitor = LaunchConfiguration("enable_monitor")

    grid = PathJoinSubstitution([
        FindPackageShare("amr_description"), "config", "warehouse_grid.yaml"])

    ld = [
        DeclareLaunchArgument("coordination_mode", default_value="proposed"),
        DeclareLaunchArgument("enable_cmd_vel_gate", default_value="false"),
        DeclareLaunchArgument("enable_tasks", default_value="true"),
        DeclareLaunchArgument("enable_monitor", default_value="true"),
    ]

    for rid in ROBOT_IDS:
        ld.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("amr_description"), "launch",
                "fleet_agent.launch.py"])),
            launch_arguments={
                "robot_id": rid,
                "coordination_mode": mode,
                # TRUE because Gazebo publishes /clock. See the note in
                # fleet_agent.launch.py for why this matters so much.
                "use_sim_time": "true",
                "grid_yaml": grid,
                "enable_cmd_vel_gate": gate,
            }.items()))

    ld.append(TimerAction(period=8.0, actions=[
        Node(package="amr_description", executable="task_generator",
             name="task_generator", output="screen", emulate_tty=True,
             condition=IfCondition(tasks),
             parameters=[{"use_sim_time": True, "interval_s": 15.0,
                          "num_tasks": 20, "seed": 42}])]))

    ld.append(Node(
        package="amr_description", executable="fleet_monitor",
        name="fleet_monitor", output="screen", emulate_tty=True,
        condition=IfCondition(monitor),
        parameters=[{"use_sim_time": True, "report_interval_s": 3.0}]))

    return LaunchDescription(ld)
