# Package layout: why the Python module is called `amr_fleet`

## The constraint

This branch may only modify `ros2_ws/src/amr_description/`. The old design
document put custom messages in a separate `robot_interfaces` package. That is
not permitted here, so the interfaces must live inside `amr_description`.

## Is that technically possible?

**Yes.** A single `ament_cmake` package can generate rosidl interfaces and install the
Python package, but ROS 2's current `ament_cmake_python` documentation warns
against calling `ament_python_install_package()` in the same CMake project as
`rosidl_generate_interfaces()`. This branch therefore uses
`ament_get_python_install_dir()` plus a normal CMake `install(DIRECTORY ...)`
for `amr_fleet`. No separate package is required, so the branch still respects
your directory restriction.

## The collision you must avoid

`rosidl_generate_interfaces(amr_description ...)` generates a Python module
**literally named `amr_description`**, installed to:

```
install/amr_description/lib/python3.12/site-packages/amr_description/
├── __init__.py          <-- generated
├── msg/
└── srv/
```

If the hand-written Python package were also called `amr_description`, it could
collide with the generated `amr_description` module in the Python install tree.
Keeping the handwritten package named `amr_fleet` avoids that collision and lets
both generated interfaces and coordination code be imported safely.

**The fix:** name the hand-written package `amr_fleet`.

```
install/amr_description/lib/python3.12/site-packages/
├── amr_description/     <-- generated interfaces (untouched)
│   ├── msg/
│   └── srv/
└── amr_fleet/           <-- our code (no collision)
    ├── core/
    └── nodes/
```

Both imports now work side by side:

```python
from amr_description.msg import RobotState   # generated
from amr_fleet.core.priority import wins     # ours
```

The ROS **package** is still `amr_description`. Only the Python **module** name
differs, which is invisible to `ros2 run`, `ros2 launch` and rosdep.

## One caveat to know about

A package that generates interfaces *and* uses them in its own Python nodes is
slightly unusual. For **Python** it works reliably at runtime. (For C++ it
would need `rosidl_get_typesupport_target()`; we have no C++, so this does not
apply.)

If your team later hits any interface-resolution oddity, the clean fix is to
split the messages into their own `amr_interfaces` package — but that requires
permission to create a directory outside `amr_description`. **Ask before doing
it. Do not assume permission.** Everything in this branch works without it.

## Directory map

```
amr_description/
├── CMakeLists.txt          MERGE required - see CMakeLists.txt.MERGE
├── package.xml             MERGE required - see package.xml.MERGE
├── msg/                    11 interface definitions
├── srv/                    Replan.srv
├── amr_fleet/
│   ├── core/               PURE PYTHON. No rclpy. Unit-testable, portable.
│   │   ├── models.py       dataclasses  (NOT types.py - shadows stdlib!)
│   │   ├── gridmap.py      occupancy grid, zones, dynamic blockages
│   │   ├── priority.py     deterministic scoring + total order
│   │   ├── geometry.py     TTC / closest point of approach
│   │   ├── conflict.py     4 conflict detectors
│   │   ├── zone.py         distributed mutual exclusion
│   │   ├── deadlock.py     wait-for graph
│   │   ├── tasks.py        auctioneer-free auction
│   │   ├── peers.py        liveness + failure detection
│   │   ├── astar.py        planning + space-time intent
│   │   └── coordinator.py  the decision engine
│   └── nodes/              ROS 2 layer ONLY
│       ├── qos_profiles.py
│       ├── conversions.py  the single ROS<->core adapter
│       ├── fleet_agent_node.py
│       ├── task_generator_node.py
│       ├── fleet_monitor_node.py
│       └── mock_robot_node.py
├── scripts/                console entry points (installed to lib/)
├── config/                 grid, params, Cyclone DDS XML
├── launch/                 4 launch files
├── test/                   62 tests, no ROS required
└── docs/
```
