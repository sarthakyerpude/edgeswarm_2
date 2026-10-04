"""Launch one opt-in AI-enabled fleet agent in its own robot namespace."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    grid_yaml = LaunchConfiguration('grid_yaml')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('grid_yaml', default_value=PathJoinSubstitution([
            FindPackageShare('amr_description'), 'config', 'warehouse_grid.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('amr_description'), 'config', 'fleet_params.yaml'])),
        Node(
            package='amr_navigation_runtime', executable='ai_fleet_agent',
            name='fleet_agent', namespace=robot_id, output='screen',
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[params_file, {
                'robot_id': robot_id,
                'grid_yaml': grid_yaml,
                'use_sim_time': use_sim_time,
                'enable_cmd_vel_gate': False,
            }],
        ),
    ])
