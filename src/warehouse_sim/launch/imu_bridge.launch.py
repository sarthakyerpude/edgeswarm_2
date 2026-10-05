"""Bridge one robot's Gazebo Harmonic IMU to /<robot_name>/imu/data."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')
    ros_topic = ParameterValue(
        PythonExpression(["'/' + '", robot_name, "' + '/imu/data'"]),
        value_type=str,
    )
    gz_topic = ParameterValue(
        PythonExpression([
            "'/world/' + '", world_name, "' + '/model/' + '", robot_name,
            "' + '/link/base_link/sensor/imu/imu'",
        ]),
        value_type=str,
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        parameters=[{
            'bridge_names': ['imu'],
            'bridges.imu.ros_topic_name': ros_topic,
            'bridges.imu.gz_topic_name': gz_topic,
            'bridges.imu.ros_type_name': 'sensor_msgs/msg/Imu',
            'bridges.imu.gz_type_name': 'gz.msgs.IMU',
            'bridges.imu.direction': 'GZ_TO_ROS',
            'bridges.imu.qos_profile': 'SENSOR_DATA',
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot_1'),
        DeclareLaunchArgument('world_name', default_value='warehouse'),
        bridge,
    ])
