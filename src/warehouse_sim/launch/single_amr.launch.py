"""single_amr.launch.py

Starts Gazebo Harmonic with worlds/warehouse.sdf, spawns one AMR from the
amr_description package at spawn_1 (see warehouse.sdf for the marker), and
starts the ros_gz_bridge parameter_bridge using config/bridge.yaml.

Usage:
    ros2 launch warehouse_sim single_amr.launch.py
    ros2 launch warehouse_sim single_amr.launch.py robot_name:=robot_2 x:=0.0 y:=-4.0
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    warehouse_sim_share = get_package_share_directory('warehouse_sim')
    amr_description_share = get_package_share_directory('amr_description')

    world_path = os.path.join(warehouse_sim_share, 'worlds', 'warehouse.sdf')
    gui_config = os.path.join(warehouse_sim_share, 'config', 'gui_topview.config')
    bridge_config_path = os.path.join(warehouse_sim_share, 'config', 'bridge.yaml')
    robot_sdf_path = os.path.join(
        amr_description_share, 'models', 'amr_robot', 'model.sdf'
    )

    # spawn_1 from warehouse.sdf: (-3.0, -4.0), facing north (+Y) into the aisles.
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot_1')
    x_arg = DeclareLaunchArgument('x', default_value='-3.0')
    y_arg = DeclareLaunchArgument('y', default_value='-4.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.05')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.5708')

    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # --- Gazebo ---------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_path} --gui-config {gui_config}'}.items(),
    )

    # --- Spawn the robot --------------------------------------------------
    # A short delay avoids racing Gazebo's own startup before the
    # /world/warehouse/create service is available.
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', robot_name,
                    '-file', robot_sdf_path,
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-Y', yaw,
                ],
                output='screen',
            )
        ],
    )

    # --- ROS <-> Gazebo bridge (cmd_vel, odom, scan, tf, clock) ------------
    # config/bridge.yaml hard-codes "robot_1". If you override robot_name
    # above, update bridge.yaml's topic names to match (or add a matching
    # block, same pattern, for the new name).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription(
        [
            *_wsl_gpu_actions(),
            robot_name_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            gz_sim,
            spawn_robot,
            bridge,
        ]
    )
