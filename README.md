# EdgeSwarm

A decentralized fleet of three autonomous mobile robots for warehouse pick and
delivery, built on ROS 2 Jazzy, Nav2 and Webots. Robots coordinate peer to
peer with no central server: tasks are allocated by a distributed auction,
traffic flows on keep-right lanes with box junctions and right-of-way rules,
and collisions are prevented by a hitbox safety envelope computed from each
robot's real footprint and localization uncertainty.

## Fleet UI

![Warehouse map](docs/images/fleet-ui-map.png)

![Traffic view](docs/images/fleet-ui-traffic.png)

## Contents

1. [What you need](#1-what-you-need)
2. [Terminals used in this guide](#2-terminals-used-in-this-guide)
3. [One-time setup, Windows side](#3-one-time-setup-windows-side)
4. [One-time setup, WSL side](#4-one-time-setup-wsl-side)
5. [Running the project](#5-running-the-project)
6. [Using the fleet](#6-using-the-fleet)
7. [Tests and evaluation](#7-tests-and-evaluation)
8. [Rebuilding after code changes](#8-rebuilding-after-code-changes)
9. [Project structure](#9-project-structure)
10. [Key configuration files](#10-key-configuration-files)
11. [Troubleshooting](#11-troubleshooting)
12. [Known limitations](#12-known-limitations)

## 1. What you need

- Windows 11
- WSL2 with Ubuntu 24.04
- ROS 2 Jazzy inside WSL (install steps below)
- Webots R2025a installed on Windows (not inside WSL)
- About 8 GB of RAM available for WSL and roughly 10 GB of disk

## 2. Terminals used in this guide

Two different terminals are used. Every command block below is labeled.

- POWERSHELL (ADMIN): Windows PowerShell started with "Run as
  administrator". Used once, for the firewall rule.
- WSL TERMINAL: an Ubuntu shell. Open it from Windows Terminal by picking
  the Ubuntu profile, or type `wsl` in any PowerShell window. All build,
  run and test commands happen here.

In this guide the project is cloned to the Windows file system and accessed
from WSL under `/mnt/c`. The example path used everywhere below is:

- Windows view: `C:\Users\<you>\edgeswarm_2`
- WSL view of the same folder: `/mnt/c/Users/<you>/edgeswarm_2`

Replace `<you>` with your Windows user name. Any folder works; keep the two
views consistent.

## 3. One-time setup, Windows side

Step 3.1 - install WSL and Ubuntu 24.04 (skip if you have it):

```powershell
# POWERSHELL (ADMIN)
wsl --install -d Ubuntu-24.04
```

Reboot if Windows asks. Then create your Ubuntu user when the Ubuntu window
opens for the first time.

Step 3.2 - install Webots R2025a on Windows from
https://cyberbotics.com/doc/guide/installation-procedure. Use the default
location `C:\Program Files\Webots`. The simulation starts Webots from WSL
automatically; you never need to open Webots yourself.

Step 3.3 - allow the Webots controller port through the Windows firewall:

```powershell
# POWERSHELL (ADMIN)
netsh advfirewall firewall add rule name="Webots extern" dir=in action=allow protocol=TCP localport=1234
```

Step 3.4 - configure WSL memory and networking. Create or edit the file
`C:\Users\<you>\.wslconfig` (plain text, Notepad is fine) with exactly:

```ini
[wsl2]
memory=8GB
swap=8GB
networkingMode=NAT
dnsTunneling=false
```

Then apply it:

```powershell
# POWERSHELL (any)
wsl --shutdown
```

The next WSL terminal you open starts with the new settings. NAT mode and
dnsTunneling=false are required: the Webots driver inside WSL finds the
Windows host through the NAT gateway address.

## 4. One-time setup, WSL side

Step 4.1 - install ROS 2 Jazzy (skip if `/opt/ros/jazzy` exists):

```bash
# WSL TERMINAL
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-jazzy-desktop
```

Step 4.2 - clone the project:

```bash
# WSL TERMINAL
cd /mnt/c/Users/<you>
git clone https://github.com/sarthakyerpude/edgeswarm_2.git
cd edgeswarm_2
```

Step 4.3 - run the setup script. It installs the remaining apt packages
(Nav2, ros-gz, CycloneDDS, the Webots ROS 2 driver, colcon, pytest), builds
the workspace, and prints the Windows checklist from section 3 again:

```bash
# WSL TERMINAL, inside /mnt/c/Users/<you>/edgeswarm_2
./setup.sh
```

Expected last lines: `Build OK.` and `Setup complete. Start the fleet with
./run.sh`. The build creates `build/`, `install/` and `log/` folders in the
repo root; they are not committed.

## 5. Running the project

Everything is started by one script, from the repository root, in a WSL
terminal:

```bash
# WSL TERMINAL
cd /mnt/c/Users/<you>/edgeswarm_2
./run.sh
```

This starts, in one go: Webots (headless) with the warehouse world and three
robots, Nav2 for each robot, the fleet coordination nodes, the task
generator, RViz, and the web UI.

Startup takes 1 to 2 minutes. You will see `starting task feed` in the
terminal when the fleet is healthy and tasks begin.

Variants:

```bash
# WSL TERMINAL, repo root
./run.sh gazebo                    # use the Gazebo backend instead of Webots
./run.sh webots rviz:=false        # no RViz window, watch the web UI only
./run.sh webots webots_gui:=true   # also show the Webots window
```

Any additional `name:=value` pairs are passed to the ROS launch, for example
`task_interval_s:=15.0`.

The script is a thin wrapper. The manual equivalent, useful when you want
the two source lines in your own terminal session:

```bash
# WSL TERMINAL
cd /mnt/c/Users/<you>/edgeswarm_2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch amr_navigation_runtime three_amr.launch.py sim:=webots
```

Note: every new WSL terminal needs both `source` lines before any `ros2`
command works.

To stop the simulation press Ctrl+C in the terminal that runs it and wait a
few seconds for the nodes to shut down.

## 6. Using the fleet

Where to look while it runs:

- Web UI: open http://localhost:8080 in any Windows browser. It shows the
  live map, lanes, junction boxes, landmarks, robot hitboxes with heading,
  task list and per-robot status. Manoeuvre badges appear as U-TURN,
  REVERSING or OVERTAKING.
- RViz window: opens automatically unless `rviz:=false`. Shows hitboxes
  with front markers, planned paths, rack slot labels and live lidar
  points. Camera presets are in the Views panel.
- Terminal: fleet events are logged (WON, pickup reached, delivered,
  DEADLOCK, OVERTAKE and so on).

Creating a task: tasks are generated automatically. To create one manually,
click a rack section in the web UI map (sections are labeled 1R1 to 12R20),
or type a slot into the task bar.

Aborting a task: press the Abort button on a task row. Abort works only
before the robot has picked the item up; afterwards the robot refuses and
finishes the delivery.

## 7. Tests and evaluation

Pure python test suite (no simulator needed, about 90 seconds):

```bash
# WSL TERMINAL
cd /mnt/c/Users/<you>/edgeswarm_2/src/amr_description
source /opt/ros/jazzy/setup.bash
python3 -m pytest test/
```

Live scorecard (run while the simulation is up, in a SECOND WSL terminal):

```bash
# WSL TERMINAL (second one)
cd /mnt/c/Users/<you>/edgeswarm_2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/src/webots_warehouse_sim/resource/cyclonedds.xml
python3 tools/fleet_eval.py 480
```

It prints an 8 minute scorecard: deliveries, per-robot localization error,
lane centring, arrival accuracy, robot separations and contacts.

Lidar honesty check (same second-terminal environment):

```bash
# WSL TERMINAL (second one), same exports as above
python3 tools/lidar_probe.py
```

Offline traffic scenarios without ROS (fast, used for development):

```bash
# WSL TERMINAL
cd /mnt/c/Users/<you>/edgeswarm_2/src/amr_description
python3 test/fleet_sim.py random 600 --seed 7
```

## 8. Rebuilding after code changes

```bash
# WSL TERMINAL, repo root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install                                      # everything
colcon build --symlink-install --packages-select amr_description   # one package
```

Python files are symlinked, so plain .py edits inside already-built packages
take effect on the next launch without rebuilding. Rebuild when you change
message definitions (msg/), launch files, worlds, or add new files.

## 9. Project structure

```
run.sh                       start everything (section 5)
setup.sh                     one-time setup (section 4)
SETUP.md                     this guide's long-form companion
src/amr_description          fleet core: auction, traffic rules, safety
                             envelope, planner, ROS messages, grid config,
                             web UI, test suite and offline simulator
src/amr_navigation_runtime   Nav2 parameters, launch files, velocity gate,
                             goal bridge, AMCL map
src/webots_warehouse_sim     Webots world (racks, landmarks, robots) and
                             robot drivers
src/warehouse_sim            Gazebo world and bridges
tools/fleet_eval.py          live scorecard
tools/lidar_probe.py         lidar honesty check
docs/images/                 README screenshots
```

## 10. Key configuration files

- `src/amr_description/config/warehouse_grid.yaml`: occupancy grid, rack
  slots, lanes, junctions, turnarounds, landmarks. After changing the world,
  regenerate with `python3 src/amr_description/scripts/warehouse_map_tool.py
  write-grid` and `write-pgm`.
- `src/amr_navigation_runtime/config/nav2_params.yaml`: Nav2 controller,
  AMCL and collision monitor settings. Tuned values carry comments
  explaining why.
- `src/amr_description/config/fleet_params.yaml`: fleet cruise speed,
  docking tolerances, peer timeouts.
- `src/amr_navigation_runtime/config/velocity_gate.yaml`: the hard speed
  cap between Nav2 and the wheels.

## 11. Troubleshooting

- Web UI stops loading after a simulation restart: Windows' WSL port relay
  holds the dead connection. Close the browser tab completely and open
  http://localhost:8080 again.
- `ros2: command not found`: you forgot the source lines. Run
  `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash` in that
  terminal.
- Robots never spawn, log shows controller connection errors: the firewall
  rule for TCP 1234 is missing (section 3.3), or Webots is not at
  `C:\Program Files\Webots`.
- `AMENT_TRACE_SETUP_FILES: unbound variable`: you are sourcing ROS inside a
  script with `set -u`. Use ./run.sh, which handles it.
- Everything is slow, robots stop and report stale peers: WSL is memory or
  CPU starved. Give it 8 GB plus 8 GB swap (section 3.4), close RViz if the
  web UI is enough, and avoid heavy builds while the sim runs.
- Webots exits on its own after 10 to 16 minutes on some machines: known
  host limitation, just start again with ./run.sh.
- Build errors after pulling new code: rebuild everything once
  (`colcon build --symlink-install`); if messages changed, also restart any
  running terminals so stale environments are gone.

## 12. Known limitations

- Localization accuracy plateaus around 0.14 m mean and 0.4 m worst-case
  under load. Robots recover automatically from localization losses, but
  brief rack-face grazes can still occur at error peaks. The planned next
  step is scan-to-map ICP correction on top of AMCL.
- The degraded-localization fallback mode restarts a waiting robot about
  5 s after a junction clears, above the 1.5 s design target.
- One run supports three robots; scaling beyond needs discovery and
  bandwidth work noted in the project wiki.
