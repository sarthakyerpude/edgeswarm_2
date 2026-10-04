"""Start namespaced Nav2, robot TF publishers, and the permit velocity gate.

A Gazebo/hardware driver must supply relative `odom` and `scan`; the fleet
agent must supply relative `motion_permit`. The gate alone publishes `cmd_vel`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('nav_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_prefix = LaunchConfiguration('frame_prefix')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    description = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('amr_description'), 'config', 'robot', 'amr.urdf.xacro']),
        ' prefix:=', frame_prefix,
    ])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'namespace': namespace,
            # Nav2 defaults this to false. Supplying only `namespace` leaves
            # every robot's Nav2 nodes and actions at the root namespace.
            'use_namespace': 'True',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params,
            'autostart': 'True',
            'slam': 'False',
            'use_composition': 'False',
            # Self-heal: a crashed Nav2 server (e.g. controller_server
            # segfault, exit -11, seen 2026-10-04) is respawned and the
            # lifecycle manager re-activates it over its bond.
            'use_respawn': 'True',
        }.items(),
    )
    robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=namespace,
        parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )
    joint_states = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        namespace=namespace, parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    gate = Node(
        package='amr_navigation_runtime', executable='velocity_gate',
        name='velocity_gate', namespace=namespace,
        parameters=[PathJoinSubstitution([
            FindPackageShare('amr_navigation_runtime'), 'config', 'velocity_gate.yaml'])],
        output='screen',
    )
    goal_bridge = Node(
        package='amr_navigation_runtime', executable='fleet_goal_bridge',
        name='fleet_goal_bridge', namespace=namespace,
        # The spawn pose is the robot's dock: paths ending there are checked
        # with the precise goal checker (docking alignment).
        parameters=[{'use_sim_time': use_sim_time, 'robot_id': namespace,
                     'dock_x': ParameterValue(initial_x, value_type=float),
                     'dock_y': ParameterValue(initial_y, value_type=float)}],
        output='screen',
    )
    initial_pose = Node(
        package='amr_navigation_runtime', executable='initial_pose_publisher',
        name='initial_pose_publisher', namespace=namespace,
        condition=IfCondition(publish_initial_pose),
        parameters=[{
            'use_sim_time': use_sim_time,
            'x': ParameterValue(initial_x, value_type=float),
            'y': ParameterValue(initial_y, value_type=float),
            'yaw': ParameterValue(initial_yaw, value_type=float),
        }], output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('publish_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            FindPackageShare('amr_navigation_runtime'), 'config', 'warehouse_map.yaml'])),
        DeclareLaunchArgument('nav_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('amr_navigation_runtime'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        robot_state, joint_states, nav2, gate, goal_bridge, initial_pose,
    ])
