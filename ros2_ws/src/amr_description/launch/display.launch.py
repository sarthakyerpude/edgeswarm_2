"""Publish the Xacro robot model and joint states for RViz/TF inspection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    description = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('amr_description'), 'config', 'robot', 'amr.urdf.xacro']),
        ' prefix:=', prefix,
    ])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('prefix', default_value='robot_1/'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=namespace, parameters=[{
                 'robot_description': description, 'use_sim_time': use_sim_time,
             }], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace=namespace, parameters=[{'use_sim_time': use_sim_time}],
             output='screen'),
    ])
