"""
THREE-ROBOT FLEET WITH NO GAZEBO. Start here.

    ros2 launch amr_description three_robots_mock.launch.py

This launches three fleet_agents plus three mock_robot bodies, a task
generator and the read-only monitor. It needs no simulator, no bridge and no
warehouse model, so the coordination layer can be verified completely while
the Gazebo work proceeds in parallel.

Every behaviour in docs/TEST_PLAN.md can be demonstrated from here:
peer discovery, conflict detection, zone arbitration, deadlock resolution,
task auction, and robot-failure recovery.

Once this passes, switch to three_robots_gazebo.launch.py. If something breaks
after that switch, you know immediately that the fault is in the integration,
not in the coordination logic.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# start cells are (row, col); mock_robot converts via the same grid convention
ROBOTS = [
    {"id": "robot_1", "x": "1.0", "y": "1.0"},
    {"id": "robot_2", "x": "8.2", "y": "3.4"},
    {"id": "robot_3", "x": "16.2", "y": "3.4"},
]


def generate_launch_description():
    mode = LaunchConfiguration("coordination_mode")
    tasks = LaunchConfiguration("enable_tasks")
    monitor = LaunchConfiguration("enable_monitor")

    grid = PathJoinSubstitution([
        FindPackageShare("amr_description"), "config", "warehouse_grid.yaml"])

    ld = [
        DeclareLaunchArgument("coordination_mode", default_value="proposed"),
        DeclareLaunchArgument("enable_tasks", default_value="true"),
        DeclareLaunchArgument("enable_monitor", default_value="true"),
    ]

    for r in ROBOTS:
        ld.append(Node(
            package="amr_description", executable="mock_robot",
            name="mock_robot", namespace=r["id"], output="screen",
            emulate_tty=True,
            parameters=[{
                "robot_id": r["id"],
                "start_x": float(r["x"]),
                "start_y": float(r["y"]),
                # NOTE use_sim_time FALSE here: there is no Gazebo and nothing
                # publishes /clock, so nodes must use the wall clock. Setting
                # it true with no /clock publisher makes every timer stall at
                # time zero and the fleet appears frozen.
                "use_sim_time": False,
            }]))

        ld.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("amr_description"), "launch",
                "fleet_agent.launch.py"])),
            launch_arguments={
                "robot_id": r["id"],
                "coordination_mode": mode,
                "use_sim_time": "false",
                "grid_yaml": grid,
            }.items()))

    # Delay the task generator so all three agents have discovered each other
    # first. Announcing into an empty fleet just wastes the first auction.
    ld.append(TimerAction(period=6.0, actions=[
        Node(package="amr_description", executable="task_generator",
             name="task_generator", output="screen", emulate_tty=True,
             condition=IfCondition(tasks),
             parameters=[{"use_sim_time": False, "interval_s": 12.0,
                          "num_tasks": 15, "seed": 42}])]))

    ld.append(Node(
        package="amr_description", executable="fleet_monitor",
        name="fleet_monitor", output="screen", emulate_tty=True,
        condition=IfCondition(monitor),
        parameters=[{"use_sim_time": False, "report_interval_s": 3.0}]))

    return LaunchDescription(ld)
