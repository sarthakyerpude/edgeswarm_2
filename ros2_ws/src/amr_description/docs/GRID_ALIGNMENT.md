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
  resolution: 0.4          # pick so robot radius (0.16 m) fits a cell
  width_cells: 50          # width_cells * resolution == world width in X
  height_cells: 35         # height_cells * resolution == world depth in Y
  origin: [0.0, 0.0]       # world coords of grid cell (0,0)'s corner
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

`'#'` = static obstacle (rack or wall), `'.'` = free.

**Every row must be exactly `width_cells` characters.** The loader raises a
clear error if not, because a single short row silently shifts every coordinate
to its right.

Generating it with a short script beats typing it:

```python
W, H = 50, 35
RACK_ROWS = [(5,6), (10,11), (15,16), (20,21)]
RACKS = [(3,9), (14,9), (25,9), (36,10)]   # (start_col, length)

rows = []
for r in range(H):
    if r in (0, H-1):
        rows.append('#'*W); continue
    line = ['.']*W
    line[0] = line[W-1] = '#'
    if any(a <= r <= b for a, b in RACK_ROWS):
        for start, length in RACKS:
            for c in range(start, start+length):
                if 0 < c < W-1:
                    line[c] = '#'
    rows.append(''.join(line))

for i, row in enumerate(rows):
    print(f'  - "{row}"   # {i}')
```

## Defining zones

A zone is a named capacity-1 shared resource. Put them where robots physically
cannot pass each other.

```yaml
zones:
  inter_X1:
    type: intersection
    rect: [7, 12, 9, 13]     # [row_min, col_min, row_max, col_max] inclusive
    capacity: 1

  aisle_A2:
    type: single_lane
    rect: [13, 3, 13, 45]    # the WHOLE aisle as ONE zone
    capacity: 1
```

**Reserve a whole aisle, not its individual cells.** If two robots each reserved
half an aisle from opposite ends they would meet in the middle and neither
could cheaply back out. One zone per aisle makes head-on deadlock inside it
structurally impossible.

## If the Gazebo world changes

Re-run step 2 and the validator. Nothing else in this package needs touching —
the grid is the only place the warehouse geometry appears.
