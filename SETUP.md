# EdgeSwarm — Setup & Run Guide

Three-robot decentralized AMR warehouse fleet: ROS 2 Jazzy + Nav2 + P2P fleet
coordination, with a **switchable simulator backend** (Gazebo Harmonic by
default). Workspace: `edgeswarm_2/ros2_ws`.

---

## 1. Prerequisites

- Windows 11 + WSL2 with **Ubuntu 24.04** (`wsl --install -d Ubuntu` if missing)
- A recent NVIDIA driver on the **Windows** side (GPU passthrough into WSL is automatic)
- Inside WSL: ROS 2 **Jazzy** ([install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)) plus:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                 python3-colcon-common-extensions
```

## 2. Build

```bash
cd /mnt/c/Users/sarth/Desktop/SIH/edgeswarm_3/edgeswarm_3/edgeswarm_2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Rebuild a single package after editing it, e.g.:

```bash
colcon build --packages-select amr_navigation_runtime
```

> Every new terminal needs both `source` lines (ROS, then the workspace).

## 3. GPU rendering in WSL (automatic)

The Gazebo backend injects `GALLIUM_DRIVER=d3d12` +
`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` by itself (WSL-only, see
`launch/sim_gazebo.launch.py`), so rendering and the `gpu_lidar` run on the
NVIDIA GPU in every context — no `.bashrc` tricks needed. Measured on the
RTX 4060 laptop: RTF ≈ 1.0 (was 0.45 on CPU rendering), scans steady at 10 Hz.

Sanity check if you suspect CPU fallback:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA glxinfo -B | head -8
# must say: Device: D3D12 (NVIDIA ...), Accelerated: yes
```

## 4. Run the fleet

Main entrypoint (3 robots + Nav2 + fleet + tasks + dashboard):

```bash
ros2 launch amr_navigation_runtime three_amr.launch.py
```

Bringup is staggered: robots spawn at 15/25/35 s, each fleet-agent + Nav2
stack at 45/55/65 s, task generator + monitor + dashboard at 80 s.
Dashboard: <http://127.0.0.1:8080>.

The Gazebo window opens with a straight-down view of the whole warehouse
(+y up, same orientation as the dashboard and RViz). The layout lives in
`warehouse_sim/config/gui_topview.config`. You can still orbit and zoom with
the mouse; relaunch to get the top view back.

### Simulator switch (`sim:=`)

The fleet/Nav2 side is simulator-agnostic; the `sim` argument picks which
backend provides `/clock` and `/robot_N/{odom,scan,tf,cmd_vel}`:

```bash
ros2 launch amr_navigation_runtime three_amr.launch.py sim:=gazebo    # default: Gazebo Harmonic here
ros2 launch amr_navigation_runtime three_amr.launch.py sim:=webots    # Webots on Windows (GPU), ROS in WSL
ros2 launch amr_navigation_runtime three_amr.launch.py sim:=external  # sim runs elsewhere (e.g. Isaac Sim on Windows)
```

### Webots backend one-time setup (`sim:=webots`)

1. Install Webots R2025a on **Windows** (default `C:\Program Files\Webots`) and pin
   `webots.exe` to the NVIDIA GPU (Settings → Display → Graphics → High performance).
2. In WSL: `sudo apt install ros-jazzy-webots-ros2 ros-jazzy-ros2run`
3. `C:\Users\<you>\.wslconfig` must have `networkingMode=NAT` and
   `dnsTunneling=false` (the controller finds the Windows host via
   /etc/resolv.conf's nameserver = the NAT gateway), memory ≥ 8GB; then `wsl --shutdown`.
4. Windows Firewall: allow inbound TCP 1234 (rule "Webots extern controller 1234").

The launch then starts Webots on Windows automatically. Useful args:
`webots_gui:=false` (headless), `webots_mode:=fast` (faster than real time),
`controller_url:=tcp://<ip>:1234` (manual override if IP auto-detection fails).
The backend pins the stack to CycloneDDS (`webots_warehouse_sim/resource/cyclonedds.xml`)
— Fast DDS's shared memory is unreliable under WSL at this node count.

An unknown value prints the available backends. To add one (Isaac, Webots, …):
drop a `launch/sim_<name>.launch.py` into `amr_navigation_runtime` that
provides the contract below, list it in `setup.py`, rebuild — then
`sim:=<name>`. Robot names/spawn poses come from
`amr_navigation_runtime/fleet_layout.py` (single source of truth).

**Contract every backend must provide (per robot_1..robot_3):**

| Topic | Type | Notes |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | all sim-flow nodes run `use_sim_time:=true` |
| `/robot_N/odom` | nav_msgs/Odometry | ~30 Hz, frames `odom` → `base_footprint` |
| `/robot_N/scan` | sensor_msgs/LaserScan | ~10 Hz, frame `laser_link`, 360 beams, 0.12–8 m |
| `/robot_N/tf` | tf2_msgs/TFMessage | `odom→base_footprint`, bare frame names |
| `/robot_N/cmd_vel` | geometry_msgs/Twist | subscribed by the sim (diff drive r=0.05 m, sep=0.38 m) |
| `/robot_N/contacts*` | ros_gz_interfaces/Contacts | optional — feeds the collision recorders |

### Useful arguments

```bash
start_tasks:=false          # bringup only, no task auction
coordination_mode:=baseline # stop-and-wait baseline (vs default 'proposed')
inject_blockage:=true       # pallet blocks the central aisle at ~34 s (reroute test)
num_tasks:=20 task_seed:=42 task_interval_s:=12.0
dashboard_port:=8080
```

### Other launches

```bash
# Single robot + Gazebo + bridge (quick smoke test):
ros2 launch warehouse_sim single_amr.launch.py

# Single robot with IMU model:
ros2 launch amr_navigation_runtime simulation_imu.launch.py

# No simulator at all — kinematic mock robots (fleet-logic regression, wall clock):
ros2 launch amr_description three_robots_mock.launch.py
```

## 5. Verify it's healthy (second terminal, both `source` lines first)

```bash
ros2 topic hz /robot_1/scan      # ~10 Hz
ros2 topic hz /robot_1/odom      # ~30 Hz
ros2 topic hz /clock             # high rate
gz topic -e -t /world/warehouse/stats | grep real_time_factor   # ~1.0
ros2 run tf2_ros tf2_echo map base_footprint --ros-args \
  -r /tf:=/robot_1/tf -r /tf_static:=/robot_1/tf_static          # fresh transform
```

Robots move only after their Nav2 stacks are up (≥ 65 s) and tasks flow (≥ 80 s).

## 6. Tests (no ROS or simulator needed)

```bash
cd edgeswarm_2/ros2_ws/src/amr_description
python3 -m pytest test/            # 62 tests; architecture test enforces the sim-free core
```

## 7. Stopping a run

`Ctrl+C` in the launch terminal. If stragglers survive (staggered timers can
outlive the parent):

```bash
pkill -9 -f "ros[-]args"; pkill -9 -f "g[z] sim"
```

(The `[x]` bracket trick stops `pkill` from killing your own shell.)

## 8. Troubleshooting

- **Robots frozen, logs mention TF extrapolation** → a node is missing
  `use_sim_time`. With Gazebo running, every sim-flow node must have it true.
- **Fleet frozen at time zero with the mock launch** → opposite problem:
  `three_robots_mock` runs wall-clock; don't set `use_sim_time` there.
- **No scans / lidar silent** → check the renderer (step 3). `llvmpipe` in
  `glxinfo -B` means CPU fallback.
- **Two simulators at once** (e.g. Gazebo here + Isaac on Windows) → never on
  the same `ROS_DOMAIN_ID`: two `/clock` publishers corrupt sim time.
- **Windows RAM pressure** → WSL reserves its full allocation; cap it in
  `C:\Users\<you>\.wslconfig` (`[wsl2]` → `memory=6GB`), then `wsl --shutdown`.
- **Benchmark CSVs** (`benchmark_runs.csv`, `inter_robot_contacts.csv`,
  `contact_sensor_events.csv`) land in the directory you launched from.

## 9. More documentation

- Topic/bridge contract: `edgeswarm_2/ros2_ws/src/amr_description/docs/` (TOPICS.md, GAZEBO_INTEGRATION.md, TEST_PLAN.md, EVALUATION.md)
- Project wiki: `SIH/wiki/` — run guide, debug session notes, and the Isaac Sim migration plan (`Migration-Gazebo-to-IsaacSim.md`)
