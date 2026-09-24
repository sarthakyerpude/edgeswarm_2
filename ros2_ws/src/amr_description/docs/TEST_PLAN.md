# Test plan

## Read this first: what is runnable today

I inspected the repo. `warehouse_sim` and `amr_navigation` are untouched
`ros2 pkg create` skeletons — two files each, no world, no robot model, no
bridge. `amr_description` had no `urdf/` either.

**So `/robot_1/odom` and `/robot_1/scan` do not exist yet.** The topic list in
the brief is the plan, not the current state.

This is why `mock_robot` exists. Tests 1-13 below run **today**, with no
Gazebo, no bridge and no robot model. Test 14 runs when the simulation lands.

---

## Test 1 - build

```bash
cd ~/edgeswarm_2/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select amr_description --symlink-install
source install/setup.bash
```

Expect `Finished <<< amr_description`. `--symlink-install` means editing a
Python file takes effect without rebuilding — but **changing a `.msg` always
needs a rebuild**.

Common failures:

| Error | Cause |
|---|---|
| `Could not find ament_cmake_python` | `sudo apt install ros-jazzy-ament-cmake-python` |
| `ModuleNotFoundError: amr_description.msg` | forgot `source install/setup.bash`, or `<member_of_group>` missing |
| `ModuleNotFoundError: amr_fleet` | Python package install step missing; verify `ament_get_python_install_dir()` and the `install(DIRECTORY amr_fleet ...)` line |
| `No such file or directory: scripts/fleet_agent` | lost the `+x` bit: `chmod +x scripts/*` |

## Test 2 - interface generation

```bash
ros2 interface list | grep amr_description      # expect 12 entries
ros2 interface show amr_description/msg/RobotState
ros2 interface proto amr_description/msg/Intent
```

## Test 3 - unit tests (no ROS runtime needed)

```bash
colcon test --packages-select amr_description
colcon test-result --verbose
# or directly, much faster while iterating:
cd src/amr_description && python3 -m pytest test/ -v
```

Expect **62 passed**. These cover priority determinism, the four conflict
detectors, zone mutual exclusion under every request ordering, wait-for-graph
cycles, the auction, the grid config, and the architecture boundary.

## Test 4 - single node startup

```bash
ros2 launch amr_description single_robot_test.launch.py
```

Expect within 2 s:
```
[fleet_agent]: grid 120x100 res=0.1m zones=['inter_X1', ...]
[fleet_agent]: fleet_agent up: robot_id=robot_1 mode=proposed tick=10.0Hz ns=/robot_1 domain=42 rmw=rmw_cyclonedds_cpp
[mock_robot]: mock_robot robot_1 at (1.00,1.00) [NO GAZEBO - test fixture]
[fleet_monitor]: FLEET MONITOR (1 robots seen)
```

If it exits with `grid_yaml parameter is empty or missing`, the launch file did
not resolve the installed config path — re-run `colcon build` and re-source.

## Test 5 - topic list and types

```bash
ros2 topic list -t | grep -E "fleet|robot_1"
```

Expect the nine `/fleet/*` topics plus `/robot_1/{odom,scan,motion_permit,conflicts,coordination_path,fleet_diagnostics}`.

**The check that matters:** `/fleet/*` must have NO robot prefix. If you see
`/robot_1/fleet/robot_state`, a relative name was used where an absolute one
was required and robots will never hear each other.

## Test 6 - QoS

```bash
ros2 topic info /fleet/robot_state --verbose
```

Expect `Reliability: BEST_EFFORT`, `Durability: VOLATILE`, and with three
robots running, **Publisher count: 3 / Subscription count: 3**.

```bash
ros2 topic info /fleet/zone_grant --verbose    # RELIABLE / TRANSIENT_LOCAL
```

**Publisher count > 0 with Subscription count = 0 is the signature of a QoS
mismatch**, and ROS 2 prints no error for it. That is the whole reason
`nodes/qos_profiles.py` exists.

## Test 7 - three-robot mutual visibility

```bash
ros2 launch amr_description three_robots_mock.launch.py
```

The monitor prints every 3 s:
```
ROBOT     STATUS    POSE                  BATT   WAIT    PRIO     AGE  LOST
robot_1   MOVING    (  1.00,  1.00)      100.0%    0.0  0.0895    12ms     0
robot_2   MOVING    (  8.20,  3.40)      100.0%    0.0  0.0779    15ms     0
robot_3   MOVING    ( 16.20,  3.40)      100.0%    0.0  0.0584    11ms     0
```

Three rows = each robot sees the other two. Confirm rate and loss:
```bash
ros2 topic hz /fleet/robot_state    # ~30 Hz total (3 x 10 Hz)
ros2 topic bw /fleet/robot_state    # ~4 KB/s
```
`LOST` must stay 0 on loopback. Non-zero on loopback means CPU starvation.

## Test 8 - intent sharing

```bash
ros2 topic echo /fleet/intent --once
```

`path_rows`/`path_cols` non-empty and `zones_needed` populated proves the
robot is broadcasting **future** occupancy, not just its current pose. That
distinction is the core of the project — position alone cannot predict conflict.

## Test 9 - conflict detection

```bash
ros2 topic echo /robot_1/conflicts
```

With `kind`: 0=CELL, 1=SWAP, 2=ZONE, 3=TTC. Check `i_have_priority` is
**opposite** on the two robots involved in the same conflict. If both report
`true`, determinism is broken — that would be a bug worth stopping for.

## Test 10 - zone request/grant

```bash
ros2 topic echo /fleet/zone_request
ros2 topic echo /fleet/zone_grant
```

Expected sequence for a contested zone:
```
zone_request  robot_id=robot_1 zone_id=aisle_A1 priority_score=0.226 lamport_ts=1 req_id=1
zone_request  robot_id=robot_2 zone_id=aisle_A1 priority_score=0.434 lamport_ts=1 req_id=1
zone_grant    granter_id=robot_1 requester_id=robot_2 granted=true
   -> robot_2 enters, robot_1 STOPs
   -> robot_2 releases, grant flows to robot_1, robot_1 proceeds
```

**The safety property:** at no instant do two robots hold the same zone. Watch
`motion_permit` on both — one GO, one STOP or SLOW.

## Test 11 - deadlock

Force it by giving two robots opposing goals through one aisle:

```bash
ros2 param set /robot_1/fleet_agent goal_cell "[8, 45]"
ros2 param set /robot_2/fleet_agent goal_cell "[8, 3]"
```

The monitor prints the wait-for graph and flags a cycle:
```
WAIT-FOR GRAPH:
    robot_1 --waits-for--> robot_2
    robot_2 --waits-for--> robot_1
  *** CYCLE DETECTED: robot_1 -> robot_2 -> robot_1
```

```bash
ros2 topic echo /robot_1/motion_permit --field deadlock_cycle
```

**Why this is decentralised:** the monitor is only *displaying* what each robot
computed independently. Kill the monitor and resolution still happens. Every
robot builds the same graph from the `waiting_for` field already present in
`RobotState`, and picks the same victim via the same total order — no
negotiation round, no extra messages.

Note the detector waits 1.0 s before believing a cycle. Two robots that both
just sent a ZoneRequest briefly point at each other; that resolves itself in one
round trip and must not trigger a reroute.

## Test 12 - blocked aisle

There is no LiDAR in mock mode, so inject the report directly:

```bash
ros2 topic pub --once /fleet/map_update amr_description/msg/MapUpdate \
"{reporter_id: 'robot_3', blocked_rows: [8,8,8], blocked_cols: [20,21,22],
  confidence: 0.9, expiry: 0.0}"
```

Set `expiry` to a Unix time ~30 s ahead (`date +%s` plus 30), or it is treated
as already stale and ignored — which is itself correct behaviour worth
observing once.

Expect robot_1 to log a replan and publish a `coordination_path` that avoids
those cells. **Expiry is mandatory**: without it, a forklift that pauses in an
aisle would block that aisle in every robot's map for the rest of the run.

## Test 13 - robot failure

```bash
# find and kill robot_2's agent
pkill -f "robot_id:=robot_2" || pkill -f fleet_agent   # or Ctrl-C its terminal
```

Within ~1.5 s the survivors log:
```
[robot_1]: PEER FAILURE: robot_2 silent for 1.52s
[robot_1]: released dependencies on failed peer robot_2
```

The monitor marks it `<== OFFLINE`. Verify:
- robot_1 and robot_3 keep moving,
- robot_2's zone is no longer waited on,
- its task is re-announced on `/fleet/task_announce`.

**Restart it** — it rejoins in ~200 ms with no reconfiguration, because DDS
rediscovers it and TRANSIENT_LOCAL replays the last coordination state.

**This is the demo to record.** Thirty seconds of killing a robot and watching
the others carry on is stronger evidence of "no single point of failure" than
any slide.

**False positives are minimised** by: a 1.5 s timeout (15 missed messages at
10 Hz), a two-stage ALIVE→SUSPECT→DEAD decision so a brief Wi-Fi stall does not
trigger reallocation, and instant recovery on one received message.

## Test 14 - Gazebo bridge (when the simulation exists)

Prerequisite check:
```bash
ros2 topic list | grep -E "robot_[123]/(odom|scan)"
ros2 topic hz /robot_1/odom     # ~30 Hz
ros2 topic hz /clock            # high rate
```

Then:
```bash
ros2 launch amr_description three_robots_gazebo.launch.py
ros2 param get /robot_1/fleet_agent use_sim_time    # MUST be True
```

If agents log `odom timeout`, the bridge is not publishing the expected topic
names. See `GAZEBO_INTEGRATION.md`.

---

## The comparison experiment

```bash
ros2 launch amr_description three_robots_mock.launch.py coordination_mode:=baseline
ros2 launch amr_description three_robots_mock.launch.py coordination_mode:=proposed
```

Identical sensing, planning and control; only the coordination block differs.
Changing exactly one variable is what makes the comparison valid.

Use the same `seed` in the task generator so both runs get an identical task
sequence. Collect from `/robot_N/fleet_diagnostics` (JSON at 1 Hz).

Run **30 trials per mode** with seeds 1-30, then report mean and standard
deviation, not a single run. Do not report a number you have not measured.
