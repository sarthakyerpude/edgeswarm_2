"""External-simulator backend for three_amr.launch.py (sim:=external).

Starts no simulator. Use this when the topic contract is provided by a
simulator this launch cannot manage — e.g. Isaac Sim running natively on the
Windows host, or Webots on Windows — and only the fleet/Nav2 side should run
here. The external simulator must provide, for robot_1..robot_3:

    /clock                     rosgraph_msgs/Clock
    /robot_N/odom              nav_msgs/Odometry   (~30 Hz, odom->base_footprint)
    /robot_N/scan              sensor_msgs/LaserScan (~10 Hz, frame laser_link)
    /robot_N/tf                tf2_msgs/TFMessage  (odom->base_footprint, bare frame names)
    /robot_N/cmd_vel           geometry_msgs/Twist (subscribed by the simulator)

and spawn the robots at the poses in amr_navigation_runtime.fleet_layout.
Contact streams (/robot_N/contacts*) are optional; without them the collision
recorders log nothing. The blockage arguments are accepted but ignored —
inject obstacles from the external simulator's own tooling.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('inject_blockage', default_value='false'),
        DeclareLaunchArgument('blockage_delay_s', default_value='34.0'),
        DeclareLaunchArgument('blockage_x', default_value='0.0'),
        DeclareLaunchArgument('blockage_y', default_value='2.0'),
        DeclareLaunchArgument('blockage_z', default_value='0.4'),
        LogInfo(msg='sim:=external — no simulator started here. Waiting on an '
                    'external sim for /clock and /robot_N/{odom,scan,tf,cmd_vel} '
                    '(see sim_external.launch.py docstring for the contract).'),
    ])
