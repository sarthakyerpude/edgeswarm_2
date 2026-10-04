"""Gazebo warehouse with an IMU-enabled AMR and namespaced ROS bridges.

This additive launch uses a copied model asset with the IMU sensor, leaving
amr_description/models/amr_robot/model.sdf and bridge.yaml unchanged.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _value(parts):
    return ParameterValue(PythonExpression(parts), value_type=str)


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
    warehouse_share = get_package_share_directory('warehouse_sim')
    runtime_share = get_package_share_directory('amr_navigation_runtime')
    world_path = os.path.join(warehouse_share, 'worlds', 'warehouse.sdf')
    gui_config = os.path.join(warehouse_share, 'config', 'gui_topview.config')
    model_path = os.path.join(runtime_share, 'models', 'amr_robot_imu', 'model.sdf')

    robot = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {world_path} --gui-config {gui_config}'}.items(),
    )
    spawn = TimerAction(period=3.0, actions=[Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', robot, '-file', model_path, '-x', x, '-y', y,
                   '-z', '0.05', '-Y', yaw],
    )])

    bridges = {
        'bridge_names': ['clock', 'odom', 'scan', 'cmd_vel', 'tf', 'imu'],
    }

    def add_bridge(name, ros_topic, gz_topic, ros_type, gz_type, direction,
                   qos=None):
        bridges[f'bridges.{name}.ros_topic_name'] = ros_topic
        bridges[f'bridges.{name}.gz_topic_name'] = gz_topic
        bridges[f'bridges.{name}.ros_type_name'] = ros_type
        bridges[f'bridges.{name}.gz_type_name'] = gz_type
        bridges[f'bridges.{name}.direction'] = direction
        if qos:
            bridges[f'bridges.{name}.qos_profile'] = qos

    add_bridge('clock', '/clock', '/clock', 'rosgraph_msgs/msg/Clock',
               'gz.msgs.Clock', 'GZ_TO_ROS', 'CLOCK')
    add_bridge('odom', _value(['"/" + "', robot, '" + "/odom"']),
               _value(['"/model/" + "', robot, '" + "/odometry"']),
               'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS')
    add_bridge('scan', _value(['"/" + "', robot, '" + "/scan"']),
               _value(['"/world/" + "', world, '" + "/model/" + "', robot,
                       '" + "/link/laser_link/sensor/lidar/scan"']),
               'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan',
               'GZ_TO_ROS', 'SENSOR_DATA')
    add_bridge('cmd_vel', _value(['"/" + "', robot, '" + "/cmd_vel"']),
               _value(['"/model/" + "', robot, '" + "/cmd_vel"']),
               'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ')
    # Bridge only this model's TF stream into its own namespace. A global TF
    # stream becomes ambiguous as soon as more than one AMR is spawned.
    add_bridge('tf', _value(['"/" + "', robot, '" + "/tf"']),
               _value(['"/model/" + "', robot, '" + "/tf"']),
               'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS')
    add_bridge('imu', _value(['"/" + "', robot, '" + "/imu/data"']),
               _value(['"/world/" + "', world, '" + "/model/" + "', robot,
                       '" + "/link/base_link/sensor/imu/imu"']),
               'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS', 'SENSOR_DATA')

    bridge_node = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='warehouse_bridge', parameters=[bridges], output='screen',
    )
    return LaunchDescription([
        *_wsl_gpu_actions(),
        DeclareLaunchArgument('robot_name', default_value='robot_1'),
        DeclareLaunchArgument('world_name', default_value='warehouse'),
        DeclareLaunchArgument('x', default_value='-3.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        gazebo, spawn, bridge_node,
    ])
