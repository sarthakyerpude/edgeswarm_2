# amr_description — decentralised fleet coordination

ROS 2 Jazzy · DDS peer-to-peer · **no central fleet server**

Part of **EdgeSwarm**: *Edge-AI Based Distributed Fleet Coordination for AMRs
in Smart Warehouses*. Branch `ankit/ros2-dds`.

---


## Run it in three commands

```bash
cd ~/edgeswarm_2/ros2_ws
colcon build --packages-select amr_description --symlink-install
source install/setup.bash
ros2 launch amr_description three_robots_mock.launch.py
```

No Gazebo. No bridge. No robot model. Three robots discovering each other,
sharing intent, arbitrating zones, resolving deadlocks and auctioning tasks.

```
ROBOT     STATUS    POSE                  BATT   WAIT    PRIO     AGE  LOST
robot_1   MOVING    (  1.00,  1.00)      100.0%    0.0  0.0895    12ms     0
robot_2   WAITING   (  8.20,  3.40)       99.8%    1.4  0.4345    15ms     0
robot_3   MOVING    ( 16.20,  3.40)       99.9%    0.0  0.0584    11ms     0
```

Then kill `robot_2`'s terminal. The other two carry on. **Record that.**

---

## What the package does

Each robot runs one `fleet_agent` process that independently:

- broadcasts its **state** (10 Hz) and its **intent** — a time-parameterised
  claim on grid cells, so peers can predict conflicts rather than react to them
- detects conflicts four ways: cell overlap, head-on swap, shared zone, and
  continuous time-to-collision
- computes a **deterministic priority** every robot can reproduce identically
- negotiates capacity-1 **zones** with an explicit request/grant handshake
- detects **deadlock** from a distributed wait-for graph, with zero extra
  messages
- shares **blocked aisles** with expiry, so peers reroute before arriving
- bids in an **auctioneer-free task auction**
- detects **peer failure** and releases that robot's dependencies

Output is a `MotionPermit` (GO / SLOW / STOP / YIELD / REROUTE plus a
continuous `speed_scale`). **`cmd_vel` is not published by default** —
`amr_navigation` keeps the actuator. Two packages fighting over one topic is a
bug you should never have to debug.

---

## Layout

```
amr_description/
├── CMakeLists.txt          extends your skeleton
├── package.xml             extends your skeleton
├── msg/                    11 interfaces, every field documented
├── srv/Replan.srv
├── amr_fleet/
│   ├── core/               PURE PYTHON — no rclpy, no ROS messages
│   │   ├── models.py       dataclasses
│   │   ├── gridmap.py      occupancy, zones, dynamic blockages
│   │   ├── priority.py     deterministic scoring + total order
│   │   ├── geometry.py     time-to-collision
│   │   ├── conflict.py     four detectors
│   │   ├── zone.py         Ricart-Agrawala mutual exclusion
│   │   ├── deadlock.py     wait-for graph
│   │   ├── tasks.py        auction
│   │   ├── peers.py        liveness / failure detection
│   │   ├── astar.py        planning + space-time intent
│   │   └── coordinator.py  the decision engine
│   └── nodes/              ROS 2 layer
│       ├── qos_profiles.py    import these; never inline a QoSProfile
│       ├── conversions.py     the only ROS↔core adapter
│       ├── fleet_agent_node.py
│       ├── task_generator_node.py
│       ├── fleet_monitor_node.py   read-only, zero publishers
│       └── mock_robot_node.py      Gazebo-free test fixture
├── scripts/                console entry points
├── config/                 grid, params, Cyclone DDS (local + wifi)
├── launch/                 4 launch files
├── test/                   62 tests, no ROS required
└── docs/                   9 documents
```

**`core/` never imports `rclpy`.** `test/test_architecture.py` fails the build
if anyone adds such an import. That single rule is what makes two claims true
rather than aspirational: the algorithms unit-test in 0.2 s with no ROS, and
they run on a Jetson Nano's Python 3.6 with no ROS at all if Docker proves too
heavy (see `docs/JETSON_DEPLOYMENT.md`).

---

## Three design decisions worth defending in a review

**The Python module is `amr_fleet`, not `amr_description`.**
`rosidl_generate_interfaces(amr_description ...)` generates a Python module
literally named `amr_description`. A hand-written package of the same name
would install to the same site-packages path and one would silently shadow the
other. Naming ours `amr_fleet` avoids the collision while keeping everything in
one ROS package — your folder restriction fully respected, no package created
outside it. (`docs/PACKAGE_LAYOUT.md`)

**Per-robot topics are relative; fleet topics are absolute.**

```python
self.create_subscription(Odometry, 'odom', ...)               # → /robot_1/odom
self.create_publisher(RobotState, '/fleet/robot_state', ...)  # → /fleet/robot_state
```

A leading `/` escapes the namespace. If fleet topics were namespaced, each
robot would publish to its own private topic and no message would ever cross.
That one character is the whole design. (`docs/TOPICS.md`)

**Priority decides fast; the handshake decides safe.** Implicit agreement works
only if every robot holds identical data. One lost packet and two robots
compute different winners and both enter the intersection. So a robot never
enters a zone on the basis of "I calculated that I have priority" — it waits
for `granted=true` from every peer that wants the same zone.
(`docs/ALGORITHMS.md`)

---


## Before the first Gazebo run: align the grid

`config/warehouse_grid.yaml` is aligned to the Gazebo team's **12 m × 10 m** warehouse centered at world `(0,0)`. The grid uses 0.1 m
cells with origin `[-6.0, -5.0]`, matching the supplied rack, aisle,
intersection, pickup, drop-off, and robot spawn geometry. Since this branch cannot
modify `warehouse_sim`, the Gazebo geometry remains the source of truth.

Convention: **grid row → world Y, grid column → world X.** Transposing this is
the most common source of "the robot drives into a rack", and it shows up far
from its cause.

```bash
python3 -m pytest test/test_grid_config.py -v
```

asserts every row is the same width, no zone overlaps a rack, every station is
on free space, and every station pair is mutually reachable. It caught two real
bugs while this package was written. Full procedure in `docs/GRID_ALIGNMENT.md`.

---

## Environment

```bash
export ROS_DOMAIN_ID=42                       # same on every robot; never 0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # same on every robot
```

Put both in `~/.bashrc`, not in individual terminals. A node started with Fast
DDS cannot reliably talk to one started with Cyclone, and the symptom — "two of
my three robots see each other" — is maddening to debug.

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

`ROS_DOMAIN_ID` is never hard-coded in source; it is deployment configuration.
(`docs/DDS.md`)

---

## Docs

| File | Read it when |
|---|---|
| `PACKAGE_LAYOUT.md` | you wonder why the module is named `amr_fleet` |
| `TOPICS.md` | wiring anything to this package |
| `ALGORITHMS.md` | writing the report, or defending a design choice |
| `TEST_PLAN.md` | verifying — 14 tests with expected output |
| `GRID_ALIGNMENT.md` | before the first Gazebo run |
| `GAZEBO_INTEGRATION.md` | the sim team asks what you need from them |
| `DDS.md` | discovery fails, or a topic silently has no subscribers |
| `JETSON_DEPLOYMENT.md` | moving to hardware |
| `GIT_WORKFLOW.md` | committing to `ankit/ros2-dds` |

---

## Status

| | |
|---|---|
| Unit tests | **56 passing** (`python3 -m pytest test/ -q`) |
| Runs without Gazebo | yes — `three_robots_mock.launch.py` |
| Files outside `amr_description` | **none** |
| Central server | none |
| `cmd_vel` published | no (opt-in gate, default off) |
| Grid aligned to Gazebo world | **not yet — placeholder** |
| Gazebo integration tested | not yet — `warehouse_sim` is empty |
