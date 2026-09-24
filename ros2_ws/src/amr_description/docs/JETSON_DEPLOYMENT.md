# Jetson Nano deployment

## The honest constraint, stated first

Jetson Nano's latest JetPack (4.6.x) is **Ubuntu 18.04 with Python 3.6**.
ROS 2 Jazzy targets **Ubuntu 24.04 with Python 3.12**. There is no apt path
from one to the other, and NVIDIA has ended Nano support.

Three options, in order of preference:

| Option | Effort | Risk |
|---|---|---|
| **Docker** (`ros:jazzy` on L4T base) | low | container overhead on 4 GB |
| Build ROS 2 from source on 18.04 | very high | dependency hell; days of work |
| **`amr_fleet.core` + raw UDP, no ROS** | low | loses ROS tooling |

## Why the third option is the safety net

`amr_fleet/core/` imports no `rclpy`, no ROS messages, no hardware library —
enforced by `test/test_architecture.py`, which fails the build if anyone adds
such an import. It is plain Python that runs on 3.6.

So if Docker turns out to be too heavy on a 4 GB Nano, you write a
`UdpCommBus` that JSON-encodes the same dataclasses over UDP multicast, and
every algorithm runs unchanged. Roughly 150 lines.

**Build and test that fallback early, not in week 12.** A fallback you have
never run is not a fallback. The architecture test exists precisely to keep
this option open.

## Docker route

```bash
# On the Jetson (JetPack 4.6.x)
sudo docker run -it --rm --network host --privileged \
  -v /dev:/dev \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  arm64v8/ros:jazzy-ros-base
```

`--network host` is required — DDS discovery needs the real interface, and
Docker's default bridge network breaks multicast.

Inside:
```bash
apt update && apt install -y ros-jazzy-rmw-cyclonedds-cpp python3-yaml
mkdir -p /ws/src && cd /ws
# mount or clone amr_description into src/
colcon build --packages-select amr_description --symlink-install
source install/setup.bash
```

## Per-robot environment

```bash
export ROS_DOMAIN_ID=42                          # SAME on all three
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # SAME on all three
export CYCLONEDDS_URI=file:///ws/install/amr_description/share/amr_description/config/cyclonedds_wifi.xml
ros2 launch amr_description fleet_agent.launch.py \
  robot_id:=robot_1 use_sim_time:=false
```

`use_sim_time:=false` on hardware — there is no `/clock`. Setting it true with
no clock publisher freezes every timer at time zero and the robot appears dead.

## Network

```bash
ping 192.168.50.12                      # IP reachability FIRST
ros2 multicast send                     # on one machine
ros2 multicast receive                  # on another
```

`ros2 multicast` is the single most useful diagnostic for a hardware network.
Run it before blaming your code.

If multicast fails, use `config/cyclonedds_wifi.xml` (multicast off, explicit
peer list) or the RMW-independent Jazzy variables:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="192.168.50.11;192.168.50.12;192.168.50.13"
```

Check your access point for **AP isolation** — it silently blocks all
client-to-client traffic and is a very common cause of "discovery works at my
desk but not at the venue". Carry a cheap unmanaged switch and three USB
Ethernet adapters as a fallback.

## Clock sync

`Intent.t_enter`/`t_exit` are absolute epoch seconds, so skew directly corrupts
conflict prediction.

```bash
sudo apt install chrony
chronyc tracking          # verify offset < 20 ms before every run
```

The `safety_margin_s` (0.5 s) absorbs modest skew, but 20 ms is the budget you
should verify rather than assume.

## What changes from simulation to hardware

| Layer | Simulation | Hardware |
|---|---|---|
| odom | Gazebo → ros_gz_bridge | wheel encoders → ESP32 → serial driver |
| scan | Gazebo LiDAR plugin | `rplidar_ros` |
| cmd_vel | bridge → Gazebo | `amr_navigation` → ESP32 → motor driver |
| **coordination** | **`amr_fleet` unchanged** | **`amr_fleet` unchanged** |

The whole point: only the top and bottom rows change. Nothing in
`amr_fleet/core/` is aware of which one it is running under.

## Resource notes

One `fleet_agent` process is roughly 120 MB. On a 4 GB Nano dedicated to one
robot, you can afford to split `motor_controller` into its own process for
fault isolation — if the coordination process hangs, the motor process keeps
running its own watchdog. On the shared development PC, keep everything in one
process to save RAM.

`executor_threads` defaults to 3. On the Nano's 4 cores that is reasonable;
leave it unless you measure a reason to change.
