"""three_amr.launch.py

Starts Gazebo Harmonic with worlds/warehouse.sdf, spawns robot_1, robot_2
and robot_3 from amr_description at the three documented spawn points
(spawn_1/spawn_2/spawn_3 markers in warehouse.sdf), and starts the
ros_gz_bridge parameter_bridge covering all three robots' odom/scan/cmd_vel
plus the shared /clock.

Spawn points (from warehouse.sdf, south corridor, facing north / +Y):
    spawn_1: (-3.0, -4.0)   -> robot_1
    spawn_2: ( 0.0, -4.0)   -> robot_2
    spawn_3: ( 3.0, -4.0)   -> robot_3

Usage:
    ros2 launch warehouse_sim three_amr.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# (robot_name, x, y, z, yaw) - matches the spawn_1/spawn_2/spawn_3 markers
# in worlds/warehouse.sdf. All three face north (+Y) into the aisles.
SPAWN_POINTS = [
    ("robot_1", -3.0, -4.0, 0.05, 1.5708),
    ("robot_2", 0.0, -4.0, 0.05, 1.5708),
    ("robot_3", 3.0, -4.0, 0.05, 1.5708),
]

# Staggered so the three spawn calls don't race each other against Gazebo's
# /world/warehouse/create service.
SPAWN_DELAY_START = 3.0
SPAWN_DELAY_STEP = 1.0


def generate_launch_description():
    warehouse_sim_share = get_package_share_directory('warehouse_sim')
    amr_description_share = get_package_share_directory('amr_description')

    world_path = os.path.join(warehouse_sim_share, 'worlds', 'warehouse.sdf')
    bridge_config_path = os.path.join(warehouse_sim_share, 'config', 'bridge.yaml')
    robot_sdf_path = os.path.join(
        amr_description_share, 'models', 'amr_robot', 'model.sdf'
    )

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # Spawn all three robots, staggered 
    spawn_actions = []
    for i, (name, x, y, z, yaw) in enumerate(SPAWN_POINTS):
        delay = SPAWN_DELAY_START + i * SPAWN_DELAY_STEP
        spawn_actions.append(
            TimerAction(
                period=delay,
                actions=[
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        arguments=[
                            '-name', name,
                            '-file', robot_sdf_path,
                            '-x', str(x),
                            '-y', str(y),
                            '-z', str(z),
                            '-Y', str(yaw),
                        ],
                        output='screen',
                    )
                ],
            )
        )

    # ROS <-> Gazebo bridge (odom/scan/cmd_vel per robot + shared clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path, 'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([gz_sim, *spawn_actions, bridge])