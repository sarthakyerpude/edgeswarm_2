"""Gazebo Harmonic backend for three_amr.launch.py (sim:=gazebo).

Starts gz-sim with the warehouse world, one ros_gz parameter bridge (clock +
per-robot odom/scan/cmd_vel/tf/imu/contacts), staggered robot spawns, and the
optional dynamic-pallet blockage. Everything in this file is Gazebo-specific;
the fleet/Nav2 side lives in three_amr.launch.py and must not know which
simulator provides the topic contract.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_navigation_runtime.fleet_layout import CONTACT_SENSORS, ROBOTS


def _wsl_gpu_actions():
    """Route Gazebo rendering to the GPU via WSLg's D3D12 Mesa driver.

    Non-interactive WSL shells never source ~/.bashrc, so without these the
    sensors system falls back to llvmpipe software rendering and gpu_lidar
    starves. Only applied under WSL: on native Linux GALLIUM_DRIVER=d3d12
    does not exist and would break rendering.
    """
    try:
        with open('/proc/version', encoding='utf-8') as fh:
            if 'microsoft' not in fh.read().lower():
                return []
    except OSError:
        return []
    return [
        SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
        SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA'),
    ]


def generate_launch_description():
    sim_share = get_package_share_directory('warehouse_sim')
    runtime_share = get_package_share_directory('amr_navigation_runtime')
    world = os.path.join(sim_share, 'worlds', 'warehouse.sdf')
    gui_config = os.path.join(sim_share, 'config', 'gui_topview.config')
    model = os.path.join(runtime_share, 'models', 'amr_robot_imu', 'model.sdf')
    pallet = os.path.join(runtime_share, 'models', 'dynamic_pallet.sdf')
    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    bridge_params = {'bridge_names': ['clock']}

    def add_bridge(name, ros_topic, gz_topic, ros_type, gz_type, direction,
                   qos=None):
        bridge_params[f'bridges.{name}.ros_topic_name'] = ros_topic
        bridge_params[f'bridges.{name}.gz_topic_name'] = gz_topic
        bridge_params[f'bridges.{name}.ros_type_name'] = ros_type
        bridge_params[f'bridges.{name}.gz_type_name'] = gz_type
        bridge_params[f'bridges.{name}.direction'] = direction
        if qos:
            bridge_params[f'bridges.{name}.qos_profile'] = qos

    add_bridge('clock', '/clock', '/clock', 'rosgraph_msgs/msg/Clock',
               'gz.msgs.Clock', 'GZ_TO_ROS', 'CLOCK')
    for rid, _, _, _ in ROBOTS:
        names = bridge_params['bridge_names']
        names.append(f'{rid}_odom')
        add_bridge(f'{rid}_odom', f'/{rid}/odom', f'/model/{rid}/odometry',
                   'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS')
        names.append(f'{rid}_scan')
        add_bridge(f'{rid}_scan', f'/{rid}/scan',
                   f'/world/warehouse/model/{rid}/link/laser_link/sensor/lidar/scan',
                   'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan',
                   'GZ_TO_ROS', 'SENSOR_DATA')
        names.append(f'{rid}_cmd_vel')
        add_bridge(f'{rid}_cmd_vel', f'/{rid}/cmd_vel', f'/model/{rid}/cmd_vel',
                   'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ')
        # Bridge only this model's TF stream into its own namespace. A global
        # TF stream becomes ambiguous as soon as more than one AMR is spawned.
        names.append(f'{rid}_tf')
        add_bridge(f'{rid}_tf', f'/{rid}/tf', f'/model/{rid}/tf',
                   'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS')
        names.append(f'{rid}_imu')
        add_bridge(f'{rid}_imu', f'/{rid}/imu/data',
                   f'/world/warehouse/model/{rid}/link/base_link/sensor/imu/imu',
                   'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS',
                   'SENSOR_DATA')
        for link, sensor, suffix in CONTACT_SENSORS:
            name = f'{rid}_contact_{link}'
            names.append(name)
            add_bridge(name, f'/{rid}/{suffix}',
                       f'/world/warehouse/model/{rid}/link/{link}/sensor/{sensor}/contact',
                       'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts',
                       'GZ_TO_ROS')

    actions = [
        *_wsl_gpu_actions(),
        DeclareLaunchArgument(
            'inject_blockage', default_value='false',
            description='Spawn a pallet across the central aisle for reroute validation.'),
        DeclareLaunchArgument(
            'blockage_delay_s', default_value='34.0',
            description='Wall-clock seconds after launch before spawning the test pallet.'),
        DeclareLaunchArgument('blockage_x', default_value='0.0'),
        DeclareLaunchArgument('blockage_y', default_value='2.0'),
        DeclareLaunchArgument('blockage_z', default_value='0.4'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': f'-r {world} --gui-config {gui_config}'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='warehouse_bridge', parameters=[bridge_params],
             output='screen'),
    ]

    for index, (rid, x, y, yaw) in enumerate(ROBOTS):
        actions.append(TimerAction(period=15.0 + index * 10.0, actions=[Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', rid, '-file', model, '-x', x, '-y', y,
                       '-z', '0.05', '-Y', yaw])]))

    # Optional reproducible dynamic-obstacle scenario. It is opt-in so paired
    # throughput benchmarks keep identical unobstructed workloads by default.
    actions.append(TimerAction(
        period=LaunchConfiguration('blockage_delay_s'),
        actions=[Node(
            package='ros_gz_sim', executable='create', output='screen',
            condition=IfCondition(LaunchConfiguration('inject_blockage')),
            arguments=['-name', 'dynamic_pallet', '-file', pallet,
                       '-x', LaunchConfiguration('blockage_x'),
                       '-y', LaunchConfiguration('blockage_y'),
                       '-z', LaunchConfiguration('blockage_z')])]))

    return LaunchDescription(actions)
