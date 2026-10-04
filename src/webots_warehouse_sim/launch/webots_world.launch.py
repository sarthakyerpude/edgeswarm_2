"""Start Webots (natively on Windows when under WSL) with the EdgeSwarm
warehouse and attach the three robot drivers.

Provides the sim side of the fleet topic contract: Ros2Supervisor publishes
/clock; per robot the AmrDriver plugin + Ros2Lidar publish
/robot_N/{odom,tf,scan} and consume /robot_N/cmd_vel.

Arguments:
  webots_gui:=false       headless (no 3D window, sensors still rendered)
  webots_mode:=fast       unthrottled, faster than real time (default realtime)
  controller_url:=tcp://127.0.0.1:1234  manual override when WSL host-IP
                          auto-detection picks the wrong address.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

ROBOT_NAMES = ['robot_1', 'robot_2', 'robot_3']


def generate_launch_description():
    share = get_package_share_directory('webots_warehouse_sim')
    world = os.path.join(share, 'worlds', 'warehouse.wbt')
    robot_description = os.path.join(share, 'resource', 'amr_webots.urdf')

    # ros2_supervisor is intentionally OFF: under the Windows+WSL TCP setup it
    # cycled connect/exit("world not yet ready"), stalling the sim for every
    # other extern controller (driver segfaults). /clock comes from robot_1's
    # AmrDriver instead; spawn services are not needed by this project.
    webots = WebotsLauncher(
        world=world,
        mode=LaunchConfiguration('webots_mode'),
        gui=LaunchConfiguration('webots_gui'),
        ros2_supervisor=False,
    )

    actions = [
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument(
            'controller_url', default_value='',
            description='Override WEBOTS_CONTROLLER_URL host part, e.g. '
                        'tcp://127.0.0.1:1234 when WSL IP auto-detection fails.'),
        SetEnvironmentVariable(
            'WEBOTS_CONTROLLER_URL', LaunchConfiguration('controller_url'),
            condition=LaunchConfigurationNotEquals('controller_url', '')),
        webots,
        # Stop everything when the Webots window is closed.
        RegisterEventHandler(OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())])),
    ]

    # Controllers attach AFTER Webots finishes loading the world and its 12
    # lidar render targets: connecting during initialization overloads the
    # extern-controller server (one synchronized Hangup wave) and the robots'
    # lidar devices come back wedged until a later fresh respawn.
    controllers = [WebotsController(
        robot_name='item_manager',
        respawn=True,
        parameters=[{
            'robot_description': os.path.join(share, 'resource',
                                              'item_manager.urdf'),
            'use_sim_time': True,
        }],
    )]

    for name in ROBOT_NAMES:
        controllers.append(WebotsController(
            robot_name=name,
            namespace=name,
            respawn=True,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ))
        # Quadrant-merge LaserScan relay (see webots_warehouse_sim/scan_flip.py).
        controllers.append(Node(
            package='webots_warehouse_sim', executable='scan_flip',
            namespace=name, name='scan_flip', output='screen',
            parameters=[{'use_sim_time': True}],
        ))

    actions.append(TimerAction(period=10.0, actions=controllers))

    return LaunchDescription(actions)
