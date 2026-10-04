"""Bring up three simulated AMRs, their local stacks, and fleet tasks.

The fleet/Nav2 side here is simulator-agnostic: it consumes the topic contract
(/clock and /robot_N/{odom,scan,tf,cmd_vel}) from whichever backend the ``sim``
argument selects. ``sim:=gazebo`` (default) starts Gazebo Harmonic via
launch/sim_gazebo.launch.py; ``sim:=external`` starts no simulator (Isaac Sim
on the Windows host, Webots, ...). Adding a backend means adding one
launch/sim_<name>.launch.py file — nothing here changes. Every robot receives
its own ROS namespace for sensors, odometry, TF, navigation, and permits; fleet
coordination topics remain absolute under ``/fleet``.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from amr_navigation_runtime.fleet_layout import ROBOTS


def _sim_backend(context):
    """Include launch/sim_<sim>.launch.py for the selected simulator backend."""
    sim = LaunchConfiguration('sim').perform(context)
    share = get_package_share_directory('amr_navigation_runtime')
    path = os.path.join(share, 'launch', f'sim_{sim}.launch.py')
    if not os.path.isfile(path):
        available = sorted(
            os.path.basename(p)[len('sim_'):-len('.launch.py')]
            for p in glob.glob(os.path.join(share, 'launch', 'sim_*.launch.py')))
        raise RuntimeError(
            f"Unknown simulator backend sim:={sim!r}. Available: "
            f"{', '.join(available)}. Add launch/sim_{sim}.launch.py to "
            f"amr_navigation_runtime to extend.")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={arg: LaunchConfiguration(arg) for arg in (
            'inject_blockage', 'blockage_delay_s',
            'blockage_x', 'blockage_y', 'blockage_z')}.items())]


def generate_launch_description():
    coordination_mode = LaunchConfiguration('coordination_mode')
    task_count = LaunchConfiguration('num_tasks')
    task_seed = LaunchConfiguration('task_seed')
    task_interval = LaunchConfiguration('task_interval_s')
    dashboard_host = LaunchConfiguration('dashboard_host')
    dashboard_port = LaunchConfiguration('dashboard_port')
    runtime_share = get_package_share_directory('amr_navigation_runtime')
    description_share = get_package_share_directory('amr_description')
    nav_launch = os.path.join(runtime_share, 'launch', 'navigation.launch.py')
    agent_launch = os.path.join(description_share, 'launch', 'fleet_agent.launch.py')

    actions = [
        DeclareLaunchArgument(
            'sim', default_value='gazebo',
            description='Simulator backend: selects launch/sim_<name>.launch.py '
                        '(gazebo = Gazebo Harmonic here; external = simulator '
                        'runs elsewhere, e.g. Isaac Sim on the Windows host).'),
        DeclareLaunchArgument('coordination_mode', default_value='proposed',
                              description='proposed or baseline'),
        DeclareLaunchArgument('start_tasks', default_value='true'),
        DeclareLaunchArgument('workload_id', default_value='seed_42'),
        DeclareLaunchArgument('num_tasks', default_value='20'),
        DeclareLaunchArgument('task_seed', default_value='42'),
        DeclareLaunchArgument('task_interval_s', default_value='12.0'),
        DeclareLaunchArgument(
            'readiness_timeout_s', default_value='10.0',
            description='Wall-clock seconds a robot_state/navigation_status '
                        'message stays fresh for the task feed. Readiness is '
                        'published on sim-time timers, so a slow simulator '
                        '(RTF < 1, e.g. WSL) needs more than the 1 s node '
                        'default or no task is ever announced.'),
        DeclareLaunchArgument(
            'dashboard_host', default_value='127.0.0.1',
            description='Dashboard bind address; use 0.0.0.0 for LAN access.'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        DeclareLaunchArgument(
            'webapp', default_value='true',
            description='Live web app (2D map + click-to-create tasks) on '
                        'dashboard_host:dashboard_port, started at t=0.'),
        DeclareLaunchArgument(
            'legacy_dashboard', default_value='false',
            description='Also start the old read-only dashboard on port 8081.'),
        DeclareLaunchArgument(
            'inject_blockage', default_value='false',
            description='Spawn a pallet across the central aisle for reroute validation.'),
        DeclareLaunchArgument(
            'blockage_delay_s', default_value='34.0',
            description='Wall-clock seconds after launch before spawning the test pallet.'),
        DeclareLaunchArgument('blockage_x', default_value='0.0'),
        DeclareLaunchArgument('blockage_y', default_value='2.0'),
        DeclareLaunchArgument('blockage_z', default_value='0.4'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open the fleet RViz view (map + live lidar points).'),
        # Simulator backend (after all argument declarations so the include
        # can forward them). Everything below is simulator-agnostic.
        OpaqueFunction(function=_sim_backend),
        # Up from t=0 so the browser shows robots coming online. It is an
        # event source like task_generator: it announces tasks, never assigns.
        Node(package='amr_description', executable='fleet_webapp',
             name='fleet_webapp', output='screen',
             condition=IfCondition(LaunchConfiguration('webapp')),
             parameters=[{'use_sim_time': True,
                          'host': dashboard_host,
                          'port': ParameterValue(dashboard_port, value_type=int),
                          'robot_ids': [robot[0] for robot in ROBOTS],
                          'home_xy': [float(v) for robot in ROBOTS
                                      for v in robot[1:3]],
                          'grid_yaml': os.path.join(
                              description_share, 'config',
                              'warehouse_grid.yaml')}]),
        Node(package='rviz2', executable='rviz2', name='fleet_rviz',
             output='log',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(runtime_share, 'rviz',
                                           'fleet_view.rviz')],
             parameters=[{'use_sim_time': True}]),
        # R8 rack slot names ("1R6") for the RViz view: latched MarkerArray.
        Node(package='amr_description', executable='rack_labels',
             name='rack_labels', output='log',
             parameters=[{'use_sim_time': True,
                          'grid_yaml': os.path.join(
                              description_share, 'config',
                              'warehouse_grid.yaml')}]),
        # R2 live robot hitboxes (body + yellow front edge + safety outline)
        # on /fleet/hitboxes, one node for the whole fleet.
        Node(package='amr_description', executable='hitbox_markers',
             name='hitbox_markers', output='log',
             parameters=[{'use_sim_time': True}]),
        # Start recorders before spawning robots so startup contacts are not
        # omitted from the safety result. They can subscribe before Gazebo's
        # bridge endpoints appear.
        Node(package='amr_navigation_runtime', executable='collision_recorder',
             name='collision_recorder', output='screen',
             parameters=[{'use_sim_time': True,
                          'robot_ids': [robot[0] for robot in ROBOTS],
                          'output_file': 'inter_robot_contacts.csv',
                          'events_output_file': 'contact_sensor_events.csv'}]),
        Node(package='amr_navigation_runtime', executable='experiment_recorder',
             name='experiment_recorder', output='screen',
             parameters=[{'use_sim_time': True,
                          'pair_id': LaunchConfiguration('workload_id'),
                          'coordination_mode': coordination_mode,
                          'expected_tasks': ParameterValue(task_count, value_type=int),
                          'robot_ids': [robot[0] for robot in ROBOTS],
                          'output_file': 'benchmark_runs.csv'}]),
    ]

    for index, (rid, x, y, yaw) in enumerate(ROBOTS):
        # Faster bring-up (was 45 + 10i / 80 s); the task feed still waits
        # for every robot's navigation readiness before announcing.
        actions.append(TimerAction(period=15.0 + index * 5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(agent_launch),
                launch_arguments={
                    'robot_id': rid,
                    'coordination_mode': coordination_mode,
                    'use_sim_time': 'true',
                    # Spawn pose doubles as the charging dock.
                    'home_x': str(x),
                    'home_y': str(y),
                    'home_yaw': str(yaw),
                }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    'namespace': rid,
                    'use_sim_time': 'True',
                    'publish_initial_pose': 'true',
                    'initial_x': x,
                    'initial_y': y,
                    'initial_yaw': yaw,
                }.items()),
        ]))

    actions.append(TimerAction(period=35.0, actions=[
        Node(package='amr_description', executable='task_generator',
             name='task_generator', output='screen', emulate_tty=True,
             condition=IfCondition(LaunchConfiguration('start_tasks')),
             parameters=[{'use_sim_time': True,
                          'interval_s': ParameterValue(task_interval, value_type=float),
                          'num_tasks': ParameterValue(task_count, value_type=int),
                          'seed': ParameterValue(task_seed, value_type=int),
                          'readiness_timeout_s': ParameterValue(
                              LaunchConfiguration('readiness_timeout_s'),
                              value_type=float),
                          'expected_robot_ids': [robot[0] for robot in ROBOTS]}]),
        Node(package='amr_description', executable='fleet_monitor',
             name='fleet_monitor', output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': True, 'report_interval_s': 3.0}]),
        Node(package='amr_description', executable='fleet_dashboard',
             name='fleet_dashboard', output='screen',
             condition=IfCondition(LaunchConfiguration('legacy_dashboard')),
             parameters=[{'host': dashboard_host, 'port': 8081}]),
    ]))
    return LaunchDescription(actions)
