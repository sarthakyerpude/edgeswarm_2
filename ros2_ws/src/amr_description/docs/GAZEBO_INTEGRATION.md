# Gazebo Harmonic integration (ROS 2 Jazzy)

## Who owns what

```
Gazebo Harmonic  (gz-sim)          <- warehouse_sim team
        |  gz.msgs on Gazebo Transport
        v
   ros_gz_bridge  (parameter_bridge) <- warehouse_sim team
        |  ROS 2 messages over DDS
        v
   /robot_N/odom   /robot_N/scan   /clock
        |
        v
   fleet_agent  (THIS PACKAGE)      <- you
        |
        v
   /robot_N/motion_permit
        |
        v
   amr_navigation                   <- nav team
        |
        v
   /robot_N/cmd_vel
        |
        v
   ros_gz_bridge -> Gazebo
```

**This package launches no Gazebo and no bridge.** Those belong to
`warehouse_sim`, which this branch cannot modify. We only consume the topics.

## What we require from the Gazebo team

Exactly three things:

| ROS topic | ROS type | Direction |
|---|---|---|
| `/robot_N/odom` | `nav_msgs/msg/Odometry` | Gazebo -> ROS |
| `/robot_N/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> ROS |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS |

for `N` in `1, 2, 3`. Nothing else. We do not need `joint_states`, `tf` or
`tf_static` — coordination works in the `map` frame.

`/robot_N/cmd_vel` must be bridged **ROS -> Gazebo**, but that is consumed by
`amr_navigation`, not by us.

## Bridge configuration (for the warehouse_sim team's reference)

Gazebo Harmonic uses `ros_gz_bridge`, not the old `gazebo_ros_pkgs` plugins.
A YAML bridge config is cleaner than a long argument list:

```yaml
# warehouse_sim/config/bridge.yaml   (NOT our file - for their reference)
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/robot_1/odom"
  gz_topic_name: "/model/robot_1/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/robot_1/scan"
  gz_topic_name: "/world/warehouse/model/robot_1/link/laser_link/sensor/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/robot_1/cmd_vel"
  gz_topic_name: "/model/robot_1/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
# ... repeat for robot_2, robot_3
```

```python
Node(package='ros_gz_bridge', executable='parameter_bridge',
     parameters=[{'config_file': '/path/to/bridge.yaml'}],
     output='screen')
```

Find the exact Gazebo topic names with:
```bash
gz topic -l          # list all Gazebo topics
gz topic -i -t /model/robot_1/odometry   # inspect the type
```

## Verify BEFORE launching our agents

```bash
ros2 topic list | grep -E "robot_[123]/(odom|scan)"
ros2 topic hz /robot_1/odom       # expect ~30 Hz
ros2 topic hz /robot_1/scan       # expect ~10 Hz
ros2 topic hz /clock              # expect a high rate
ros2 topic echo /robot_1/odom --field pose.pose.position --once
```

If any is missing, stop and fix the bridge. Our agents will log
`odom timeout` and refuse to move, which is correct fail-safe behaviour but
tells you nothing new.

## `use_sim_time` — the bug that costs a day

Whenever Gazebo publishes `/clock`, **every** node must set
`use_sim_time: true`. Our launch files do this, but the mechanism is worth
understanding:

```
Gazebo runs at 0.7x real time (normal on modest hardware).
  Gazebo stamps /robot_1/scan with sim time 12.400
  A node with use_sim_time=false reads wall time 17.850
  It asks TF: "where was laser_link at 17.850?"
  TF only has data up to 12.400
  -> "Lookup would require extrapolation into the future"
  -> the error mentions TF, so you debug TF for a day instead of a boolean
```

```bash
ros2 param get /robot_1/fleet_agent use_sim_time    # must be True with Gazebo
```

Conversely, `three_robots_mock.launch.py` sets it **false**, because nothing
publishes `/clock` there. Setting it true with no clock publisher makes every
timer stall at time zero and the fleet appears frozen.

## Order of operations

```bash
# Terminal 1 - their simulation
ros2 launch warehouse_sim <their_world_launch>.py

# Terminal 2 - verify (see above)

# Terminal 3 - our coordination layer
ros2 launch amr_description three_robots_gazebo.launch.py
```

## If robot namespaces differ

The code never hard-codes `robot_1`. If the Gazebo team uses different names,
edit `ROBOT_IDS` in `launch/three_robots_gazebo.launch.py`. Nothing else changes.
