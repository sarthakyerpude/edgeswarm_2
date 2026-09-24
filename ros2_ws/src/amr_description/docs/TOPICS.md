# Topic contract

## The one rule

| Kind | Name style | Resolves to | Why |
|---|---|---|---|
| Per-robot | **relative** (`odom`) | `/robot_1/odom` | private to one robot |
| Fleet | **absolute** (`/fleet/robot_state`) | `/fleet/robot_state` | shared by all |

A leading `/` escapes the namespace. In `rclpy`:

```python
self.create_subscription(Odometry, 'odom', ...)               # -> /robot_1/odom
self.create_publisher(RobotState, '/fleet/robot_state', ...)  # -> /fleet/robot_state
```

**If fleet topics were namespaced, robots could not hear each other.** Every
robot would publish to its own private `/robot_1/fleet_state` and subscribe to
its own, so no message would ever cross. That one character is the whole design.

## Consumed (published by Gazebo via ros_gz_bridge, or by hardware drivers)

| Topic | Type | Rate | Owner |
|---|---|---|---|
| `/robot_N/odom` | `nav_msgs/Odometry` | ~30 Hz | Gazebo / wheel encoders |
| `/robot_N/scan` | `sensor_msgs/LaserScan` | ~10 Hz | Gazebo / RPLIDAR |
| `/clock` | `rosgraph_msgs/Clock` | ~250 Hz | Gazebo only |

We subscribe to `odom` and `scan`. We do **not** subscribe to `joint_states`,
`tf` or `tf_static` — coordination works in the `map` frame and needs no joint
information. Less coupling to another team's package is a feature.

## Produced, per robot

| Topic | Type | Rate | Consumer |
|---|---|---|---|
| `/robot_N/motion_permit` | `amr_description/MotionPermit` | 10 Hz | **amr_navigation** |
| `/robot_N/conflicts` | `amr_description/Conflict` | on detect | dashboard / logger |
| `/robot_N/coordination_path` | `nav_msgs/Path` | on replan | RViz / nav (advisory) |
| `/robot_N/fleet_diagnostics` | `std_msgs/String` (JSON) | 1 Hz | dashboard / metrics |

**`cmd_vel` is NOT published by default.** `amr_navigation` owns the actuator.
We publish a *decision*; they execute it. If the nav team wants us to enforce
it, set `enable_cmd_vel_gate:=true` and remap their output to `cmd_vel_raw`.

## Produced, fleet-wide (every robot publishes AND subscribes)

| Topic | Type | Rate | QoS |
|---|---|---|---|
| `/fleet/robot_state` | `RobotState` | 10 Hz | BEST_EFFORT / VOLATILE / KEEP_LAST(1) |
| `/fleet/intent` | `Intent` | on change + 1 Hz | RELIABLE / TRANSIENT_LOCAL / 10 |
| `/fleet/zone_request` | `ZoneRequest` | event | RELIABLE / TRANSIENT_LOCAL / 10 |
| `/fleet/zone_grant` | `ZoneGrant` | event | RELIABLE / TRANSIENT_LOCAL / 10 |
| `/fleet/map_update` | `MapUpdate` | event | RELIABLE / TRANSIENT_LOCAL / 50 |
| `/fleet/task_announce` | `Task` | event | RELIABLE / TRANSIENT_LOCAL / 10 |
| `/fleet/task_bid` | `TaskBid` | event | RELIABLE / TRANSIENT_LOCAL / 10 |
| `/fleet/task_award` | `TaskAward` | event | RELIABLE / TRANSIENT_LOCAL / 10 |
| `/fleet/task_complete` | `TaskComplete` | event | RELIABLE / TRANSIENT_LOCAL / 10 |

**`/fleet/*` is not a server.** It is a set of DDS topic names. No process sits
behind them. All three robots publish and all three subscribe; DDS delivers
peer-to-peer. Kill any robot and the others are unaffected because there was
never anything in the middle to break.

## Bandwidth

~30 msg/s per robot, roughly **43 kB/s for the whole fleet**. Trivial for Wi-Fi,
with large headroom.

## Services

| Service | Type | Note |
|---|---|---|
| `/robot_N/replan` | `amr_description/Replan` | affects ONLY that robot |

No robot ever calls another robot's service. Inter-robot coordination is
broadcast-only, which is what keeps it decentralised — a service call requires
knowing a specific server's name, reintroducing exactly the coupling we removed.
