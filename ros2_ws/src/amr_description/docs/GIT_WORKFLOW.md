# Git workflow for branch `ankit/ros2-dds`

Every path below is inside `ros2_ws/src/amr_description/`. Nothing in
`amr_navigation/` or `warehouse_sim/` is touched.

## First push

```bash
cd ~/edgeswarm_2                      # your clone of purify010622/edgeswarm_2
git checkout main && git pull origin main
git checkout -b ankit/ros2-dds

# copy the delivered package over the existing skeleton
cp -r /path/to/unzipped/amr_description/. ros2_ws/src/amr_description/

chmod +x ros2_ws/src/amr_description/scripts/*     # git tracks the +x bit

git status                            # REVIEW before staging
git add ros2_ws/src/amr_description
git status                            # confirm nothing outside that dir
```

That second `git status` is the important one. If anything under
`amr_navigation/` or `warehouse_sim/` appears, unstage it:
```bash
git restore --staged ros2_ws/src/amr_navigation ros2_ws/src/warehouse_sim
```

```bash
git commit -m "feat(amr_description): decentralised fleet coordination layer

Adds ROS 2 Jazzy / DDS coordination inside amr_description:

- 10 custom messages + Replan.srv on the shared /fleet/* bus
- amr_fleet.core: pure-Python algorithms, no rclpy (portable to Jetson)
  priority, conflict (cell/swap/zone/ttc), zone mutex (Ricart-Agrawala
  + Lamport), wait-for-graph deadlock, auctioneer-free task auction,
  peer liveness, A* + space-time intent
- amr_fleet.nodes: fleet_agent (one process per robot), task_generator,
  read-only fleet_monitor, Gazebo-free mock_robot
- shared QoS profiles, Cyclone DDS configs (local + wifi)
- 4 launch files, 56 unit tests, docs

No central fleet server. robot_id is a parameter; one codebase serves
robot_1/2/3. Extends the existing CMakeLists/package.xml skeleton."

git push -u origin ankit/ros2-dds
```

## Ongoing

```bash
git add ros2_ws/src/amr_description
git commit -m "fix(fleet): <what and why>"
git push
```

## Staying current with main

```bash
git fetch origin
git rebase origin/main       # keeps your branch a clean linear series
# if it goes wrong:  git rebase --abort
```

Prefer rebase over merge here: your branch only touches `amr_description`, so
conflicts with teammates' work in the other two packages should be zero, and
a linear history is much easier to review.

## Guard against touching other packages

```bash
git diff --name-only origin/main... | grep -v '^ros2_ws/src/amr_description/'
```
Empty output means you stayed inside your boundary. Worth running before every
push.

## Do not commit build artefacts

The repo's `.gitignore` already covers `ros2_ws/build/`, `install/`, `log/` and
`__pycache__/`. Verify with `git status` — if `build/` shows up, you are running
git from the wrong directory.

## PR description

State plainly: what was added, that nothing outside `amr_description` changed,
that `colcon test` gives 56 passing tests, and that
`ros2 launch amr_description three_robots_mock.launch.py` demonstrates the
whole system with no Gazebo required. That last point is what lets a reviewer
verify your work in one command.
