# Aligning `warehouse_grid.yaml` with the Gazebo world

## Why this file exists

The coordination layer reasons about a discrete grid: cells, zones, blocked
aisles. The Gazebo world in `warehouse_sim` is continuous geometry. The two must
describe the same warehouse.

Because this branch cannot modify `warehouse_sim`, the grid lives here and is
aligned by hand. This is a **5-minute, one-time task**. Do it before the first
integrated run.

## The convention (fixed in `core/gridmap.py`)

```
grid ROW    -> world Y
grid COLUMN -> world X

world_x = (col + 0.5) * resolution + origin[0]
world_y = (row + 0.5) * resolution + origin[1]
```

Transposing this is the single most common source of "the robot drives into a
rack" bugs, and it presents far downstream from its cause.

## The 5-minute check

**1. Get the world's extent.** Ask the Gazebo team, or read their `.sdf`. You
need the floor size in metres and which corner is the world origin.

**2. Set `meta` to match.**
```yaml
meta:
  resolution: 0.1          # 10 cm cells; aligns the supplied markers to cell centers
  width_cells: 120         # 120 * 0.1 = 12.0 m
  height_cells: 100        # 100 * 0.1 = 10.0 m
  origin: [-6.0, -5.0]     # world coords of grid cell (0,0)'s corner
```

**3. Verify with two robots and one command.** Start the simulation and drive
`robot_1` to a known landmark — say the front-left corner of the first rack.
Then:

```bash
ros2 topic echo /robot_1/odom --field pose.pose.position --once
```

Convert by hand and compare with the YAML:
```python
row = int((y - origin[1]) / resolution)
col = int((x - origin[0]) / resolution)
```
The cell you compute must be `'#'` (or immediately adjacent to one) in the
occupancy block. If it is in open space, your rows and columns are swapped or
your origin is wrong.

**4. Run the validator.**
```bash
colcon test --packages-select amr_description --pytest-args -k grid_config
```
`test_grid_config.py` asserts that every row is the same width, no zone
overlaps a rack, every station sits on free space, and every station pair is
mutually reachable. It caught two real bugs while this package was written.

## Drawing the occupancy block

`'#'` = static obstacle (rack or wall), `'.'` = free. Every row is exactly
`width_cells` characters. The current file is generated from the Gazebo team's
world-coordinate rectangles:

- outer world: `x=-6..+6`, `y=-5..+5`
- west racks: `x=-5.7..-1.0`
- east racks: `x=+1.0..+5.7`
- rack row 1: `y=3.2..3.8`
- rack row 2: `y=1.1..1.7`
- rack row 3: `y=-0.4..0.2`
- crossing corridor: `x=-1.0..+1.0`

The 0.1 m resolution was chosen because it maps the supplied markers exactly to
cell centers with the fixed center-coordinate convention. For example:

```text
world point                  grid cell
(-3.35,  3.95) pickup 1   -> (89, 26)
( 3.35,  3.95) pickup 2   -> (89, 93)
(-3.35,  2.45) pickup 3   -> (74, 26)
(-3.00, -2.80) dropoff 1  -> (22, 30)
( 0.00, -2.80) dropoff 2  -> (22, 60)
( 3.00, -2.80) dropoff 3  -> (22, 90)
```

The mock three-robot launch uses the same spawn positions as Gazebo:

```text
robot_1: (-3.0, -4.0), yaw +1.5708
robot_2: ( 0.0, -4.0), yaw +1.5708
robot_3: ( 3.0, -4.0), yaw +1.5708
```

## Defining zones

A zone is a named capacity-1 shared resource. Put them where robots physically
cannot pass each other.

```yaml
zones:
  inter_X1:
    type: intersection
    rect: [67, 50, 81, 69]   # x=-1..+1, y=1.7..3.2
    capacity: 1

  aisle_A1:
    type: single_lane
    rect: [67, 3, 81, 116]  # normal aisle y=1.7..3.2
    capacity: 1

  aisle_A2:
    type: single_lane
    rect: [52, 3, 60, 116]  # narrow aisle y=0.2..1.1
    capacity: 1
```

**Reserve a whole aisle, not its individual cells.** If two robots each reserved
half an aisle from opposite ends they would meet in the middle and neither
could cheaply back out. One zone per aisle makes head-on deadlock inside it
structurally impossible.

## If the Gazebo world changes

Re-run step 2 and the validator. Nothing else in this package needs touching —
the grid is the only place the warehouse geometry appears.
