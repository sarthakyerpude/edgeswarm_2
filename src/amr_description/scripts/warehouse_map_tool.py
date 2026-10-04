#!/usr/bin/env python3
"""Ground-truth map of the Webots warehouse, and alignment checks against it.

Parses webots_warehouse_sim/worlds/warehouse.wbt (the authoritative geometry),
rasterises every collision object at the fleet grid's resolution, and diffs
the result cell-by-cell against:
  * amr_description/config/warehouse_grid.yaml   (fleet A* grid, row 0 = min y)
  * amr_navigation_runtime/config/warehouse_map.pgm (Nav2/AMCL, PGM row 0 = max y)
It also checks that task/station cells and spawn poses sit on the world's
coloured floor markers.

Offline developer tool only: not installed by CMake, never imported by nodes.

  python3 warehouse_map_tool.py report             # catalogue + diffs + ASCII map
  python3 warehouse_map_tool.py write-pgm [PATH]   # regenerate the Nav2 PGM
  python3 warehouse_map_tool.py write-grid         # regenerate the grid yaml's
                                                   # occupancy rows from the wbt

Rasterisation rule: a cell is occupied iff its square overlaps an obstacle's
footprint with positive area (edge contact does not count). The world's walls
lie just OUTSIDE the 12 x 10 m interior, so the pure ground truth has no wall
cells inside the grid; both maps instead mark the outermost ring of interior
cells, whose outer edge IS the wall face. The PGM keeps that ring because AMCL
needs occupied cells where the lidar actually hits the walls.
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
WBT = os.path.join(SRC, 'webots_warehouse_sim', 'worlds', 'warehouse.wbt')
GRID_YAML = os.path.join(SRC, 'amr_description', 'config', 'warehouse_grid.yaml')
PGM = os.path.join(SRC, 'amr_navigation_runtime', 'config', 'warehouse_map.pgm')
MAP_YAML = os.path.join(SRC, 'amr_navigation_runtime', 'config', 'warehouse_map.yaml')
TASKGEN = os.path.join(SRC, 'amr_description', 'amr_fleet', 'nodes', 'task_generator_node.py')
LAYOUT = os.path.join(SRC, 'amr_navigation_runtime', 'amr_navigation_runtime', 'fleet_layout.py')

RES = 0.1
ORIGIN = (-6.0, -5.0)
W, H = 120, 100
EPS = 1e-6


# ----------------------------------------------------------------- wbt parse
def _top_level_nodes(text):
    """Yield (header, body) for every depth-0 `[DEF X] Type { ... }` node."""
    text = re.sub(r'#[^\n]*', '', text)
    i, depth, start, hdr_start = 0, 0, None, 0
    while i < len(text):
        ch = text[i]
        if ch == '{':
            if depth == 0:
                start = i
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                header = text[hdr_start:start].strip().split('\n')[-1].strip()
                yield header, text[start + 1:i]
                hdr_start = i + 1
        i += 1


def _vec(body, key):
    m = re.search(r'\b%s\s+([-\d.eE\s]+)' % key, body)
    return [float(v) for v in m.group(1).split()] if m else None


def parse_world(path=WBT):
    text = open(path).read()
    defs = {}
    for m in re.finditer(r'DEF\s+(\w+)\s+(Box|Cylinder)\s*\{([^{}]*)\}', text):
        defs[m.group(1)] = (m.group(2), m.group(3))

    def geometry(body):
        m = re.search(r'geometry\s+(?:DEF\s+\w+\s+)?(Box|Cylinder)\s*\{([^{}]*)\}', body)
        if m:
            kind, g = m.group(1), m.group(2)
        else:
            m = re.search(r'geometry\s+USE\s+(\w+)', body) or \
                re.search(r'children\s*\[\s*USE\s+(\w+)\s*\]', body)
            if not m:
                return None
            ref = m.group(1)
            # A `USE SPAWN_DISC` points at a Shape; find that Shape's geometry.
            if ref in defs:
                kind, g = defs[ref]
            else:
                sm = re.search(r'DEF\s+%s\s+Shape\s*\{.*?geometry\s+(Box|Cylinder)'
                               r'\s*\{([^{}]*)\}' % ref, text, re.S)
                kind, g = sm.group(1), sm.group(2)
        if kind == 'Box':
            return ('box', _vec(g, 'size'))
        return ('disc', _vec(g, 'radius')[0])

    def colour(body, nodes_text):
        c = _vec(body, 'baseColor')
        if c is None:
            m = re.search(r'children\s*\[\s*USE\s+(\w+)', body)
            if m:
                sm = re.search(r'DEF\s+%s\s+Shape\s*\{\s*appearance\s+PBRAppearance'
                               r'\s*\{([^{}]*)\}' % m.group(1), nodes_text, re.S)
                c = _vec(sm.group(1), 'baseColor') if sm else None
        return c[:3] if c else None

    comps = []
    for header, body in _top_level_nodes(text):
        kind = header.split()[-1] if header else ''
        if kind not in ('Solid', 'Pose', 'Robot'):
            continue
        t = _vec(body, 'translation') or [0.0, 0.0, 0.0]
        nm = re.search(r'\bname\s+"([^"]+)"', body)
        name = nm.group(1) if nm else header
        comps.append({
            'node': kind, 'name': name, 'x': t[0], 'y': t[1], 'z': t[2],
            'geom': geometry(body) if kind != 'Robot' else None,
            'collides': 'boundingObject' in body and kind == 'Solid',
            'colour': colour(body, text),
        })
    return comps


def classify(c):
    n = c['name']
    if c['node'] == 'Robot':
        return 'robot' if n.startswith('robot_') else 'supervisor'
    if n == 'floor':
        return 'floor'
    if n.startswith('wall_'):
        return 'wall'
    if n.startswith('rack_'):
        return 'rack'
    if n.startswith('movable'):
        return 'movable_box'
    if n.startswith('item_'):
        return 'item'
    r, g, b = c['colour'] or (0, 0, 0)
    if c['geom'] and c['geom'][0] == 'box':
        return 'intersection_marker'
    if r > 0.8 and g > 0.8 and b < 0.3:
        return 'spawn_marker'      # yellow
    if g > 0.8 and r < 0.3:
        return 'pickup_marker'     # green
    if b > 0.8 and r < 0.3:
        return 'dropoff_marker'    # blue
    return 'other'


def footprint(c):
    """Axis-aligned (xmin, xmax, ymin, ymax); all world boxes are unrotated."""
    kind, v = c['geom']
    if kind == 'box':
        return (c['x'] - v[0] / 2, c['x'] + v[0] / 2, c['y'] - v[1] / 2, c['y'] + v[1] / 2)
    return (c['x'] - v, c['x'] + v, c['y'] - v, c['y'] + v)


# ------------------------------------------------------------------- raster
def world_to_cell(x, y):
    # Same truncation as amr_fleet.core.gridmap.GridMap.world_to_cell; EPS
    # keeps exact cell-corner coordinates (e.g. -3.0) from flipping on FP noise.
    return (int((y - ORIGIN[1]) / RES + EPS), int((x - ORIGIN[0]) / RES + EPS))


def cell_to_world(r, c):
    return (ORIGIN[0] + (c + 0.5) * RES, ORIGIN[1] + (r + 0.5) * RES)


def rasterise(boxes, border=True):
    """grid[row][col] (row 0 = min y) of 0/1; border marks the wall-face ring."""
    g = [[0] * W for _ in range(H)]
    for (x0, x1, y0, y1) in boxes:
        for r in range(H):
            cy0 = ORIGIN[1] + r * RES
            if min(y1, cy0 + RES) - max(y0, cy0) <= EPS:
                continue
            for c in range(W):
                cx0 = ORIGIN[0] + c * RES
                if min(x1, cx0 + RES) - max(x0, cx0) > EPS:
                    g[r][c] = 1
    if border:
        for c in range(W):
            g[0][c] = g[H - 1][c] = 1
        for r in range(H):
            g[r][0] = g[r][W - 1] = 1
    return g


def ground_truth(comps, border=True, include_movable=False):
    kinds = {'wall', 'rack'} | ({'movable_box'} if include_movable else set())
    return rasterise([footprint(c) for c in comps
                      if c['collides'] and classify(c) in kinds], border)


def load_grid_yaml(path=GRID_YAML):
    rows, inside = [], False
    for line in open(path):
        if line.startswith('occupancy:'):
            inside = True
            continue
        if inside:
            m = re.match(r'\s*-\s*"([#.]+)"', line)
            if m:
                rows.append([1 if ch == '#' else 0 for ch in m.group(1)])
            elif line.strip() and not line.strip().startswith('#'):
                break
    return rows


def load_pgm(path=PGM, yaml_path=MAP_YAML):
    meta = {}
    for line in open(yaml_path):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    toks = open(path).read().split()
    assert toks[0] == 'P2', 'expected ASCII PGM'
    w, h, mx = int(toks[1]), int(toks[2]), int(toks[3])
    px = [int(t) for t in toks[4:4 + w * h]]
    occ_th = float(meta.get('occupied_thresh', 0.65))
    negate = int(meta.get('negate', 0))
    g = []
    for r in range(h):                       # flip: PGM row 0 is max y
        prow = px[(h - 1 - r) * w:(h - r) * w]
        g.append([1 if ((v / mx) if negate else (mx - v) / mx) > occ_th else 0 for v in prow])
    return g, meta, (w, h)


def write_pgm(grid, path=PGM):
    lines = ['P2', '%d %d' % (W, H), '255']
    for r in range(H - 1, -1, -1):
        lines.append(' '.join('0' if v else '254' for v in grid[r]))
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def write_grid_yaml(grid, path=GRID_YAML):
    """Replace the yaml's occupancy rows (row 0 = min y) with `grid`, in place;
    every other line (zones, stations, comments) is kept verbatim."""
    out, inside, done = [], False, False
    for line in open(path):
        if line.startswith('occupancy:'):
            inside = True
            out.append(line)
            continue
        if inside and re.match(r'\s*-\s*"[#.]+"', line):
            if not done:
                out.extend('  - "%s"\n' % ''.join('#' if v else '.' for v in row)
                           for row in grid)
                done = True
            continue
        if inside and line.strip() and not line.strip().startswith('#'):
            inside = False
        out.append(line)
    with open(path, 'w') as f:
        f.write(''.join(out))


def diff(a, b):
    """List of (row, col, a, b) where the grids disagree."""
    return [(r, c, a[r][c], b[r][c]) for r in range(H) for c in range(W) if a[r][c] != b[r][c]]


def regions(cells):
    """Group mismatch cells into row-runs: (row, col0, col1, value_in_a)."""
    out, cells = [], sorted(cells)
    for r, c, va, _ in cells:
        if out and out[-1][0] == r and out[-1][2] == c - 1 and out[-1][3] == va:
            out[-1][2] = c
        else:
            out.append([r, c, c, va])
    return out


def ascii_map(grid, marks=None, rstep=2, cstep=1):
    """North-up text map: '#' all blocked, '.' all free, '+' mixed block."""
    marks = marks or {}
    lines = []
    for r0 in range(H - rstep, -1, -rstep):
        line = ''
        for c0 in range(0, W, cstep):
            block = [(rr, cc) for rr in range(r0, r0 + rstep)
                     for cc in range(c0, min(W, c0 + cstep))]
            vals = [grid[rr][cc] for rr, cc in block]
            ch = '#' if all(vals) else ('.' if not any(vals) else '+')
            for cell in block:
                ch = marks.get(cell, ch)
            line += ch
        lines.append(line)
    return '\n'.join(lines)


# -------------------------------------------------------------- free spans
def free_run(grid, r, c, axis):
    """Contiguous free cells through (r, c) along 'row' (y) or 'col' (x).

    Returns (first, last) cell index of the run."""
    get = (lambda i: grid[i][c]) if axis == 'row' else (lambda i: grid[r][i])
    n = H if axis == 'row' else W
    lo = hi = r if axis == 'row' else c
    while lo - 1 >= 0 and not get(lo - 1):
        lo -= 1
    while hi + 1 < n and not get(hi + 1):
        hi += 1
    return lo, hi


def _ints(path, param):
    m = re.search(r'"%s",\s*\[([^\]]*)\]' % param, open(path).read())
    v = [int(x) for x in m.group(1).split(',')]
    return list(zip(v[0::2], v[1::2]))


def report():
    comps = parse_world()
    print('== components (world.wbt) ==')
    for c in comps:
        k = classify(c)
        fp = footprint(c) if c['geom'] else None
        cell = world_to_cell(c['x'], c['y'])
        print('%-20s %-14s x=%6.3f y=%6.3f  cell(r,c)=%-10s %s' % (
            c['name'], k, c['x'], c['y'], cell,
            '' if fp is None else 'x[%.3f,%.3f] y[%.3f,%.3f]%s' % (
                fp + (' COLLIDES' if c['collides'] else ' visual',))))

    gt = ground_truth(comps)
    gt_pure = ground_truth(comps, border=False)
    grid = load_grid_yaml()
    pgm, meta, size = load_pgm()
    print('\n== grid yaml vs ground truth (+wall-face ring) ==')
    print('size %dx%d; mismatches: %d' % (len(grid[0]), len(grid), len(diff(grid, gt))))
    for reg in regions(diff(grid, gt)):
        print('  row %d cols %d..%d grid=%s' % tuple(reg))
    print('grid vs PURE ground truth (no ring): %d cells differ (the ring = %d cells)'
          % (len(diff(grid, gt_pure)), 2 * W + 2 * H - 4))
    print('\n== warehouse_map.pgm vs ground truth ==')
    print('pgm size %s, yaml %s' % (size, meta))
    print('mismatches: %d' % len(diff(pgm, gt)))
    for reg in regions(diff(pgm, gt)):
        print('  row %d cols %d..%d pgm=%s' % tuple(reg))
    print('pgm vs grid yaml mismatches: %d' % len(diff(pgm, grid)))

    print('\n== markers vs config cells ==')
    markers = {k: [c for c in comps if classify(c) == k]
               for k in ('spawn_marker', 'pickup_marker', 'dropoff_marker')}
    for label, cells, k in (('pickup', _ints(TASKGEN, 'pickup_cells'), 'pickup_marker'),
                            ('dropoff', _ints(TASKGEN, 'dropoff_cells'), 'dropoff_marker')):
        for cell in cells:
            x, y = cell_to_world(*cell)
            best = min(markers[k], key=lambda m: math.hypot(m['x'] - x, m['y'] - y))
            d = math.hypot(best['x'] - x, best['y'] - y)
            r = best['geom'][1]
            print('%-7s cell %-9s centre (%.2f,%.2f) -> marker (%.2f,%.2f) off %.3f m (radius %.2f) %s'
                  % (label, cell, x, y, best['x'], best['y'], d, r, 'OK' if d < r else 'MISS'))
    for line in open(LAYOUT):
        m = re.match(r"\s*\('(robot_\d)',\s*'([-\d.]+)',\s*'([-\d.]+)',\s*'([-\d.]+)'\)", line)
        if m:
            x, y = float(m.group(2)), float(m.group(3))
            best = min(markers['spawn_marker'], key=lambda s: math.hypot(s['x'] - x, s['y'] - y))
            robot = next(c for c in comps if c['name'] == m.group(1))
            print('spawn %s (%.2f,%.2f) cell %s -> marker (%.2f,%.2f) off %.3f; wbt robot at (%.2f,%.2f)'
                  % (m.group(1), x, y, world_to_cell(x, y), best['x'], best['y'],
                     math.hypot(best['x'] - x, best['y'] - y), robot['x'], robot['y']))

    print('\n== free spans on the PURE ground truth (no ring) ==')
    for name, (x, y), axis in (('north aisle', (-3.35, 4.25), 'row'),
                               ('aisle A', (-3.35, 2.05), 'row'),
                               ('aisle B', (-3.35, -0.15), 'row'),
                               ('south area', (-3.35, -3.05), 'row'),
                               ('spine (row 1)', (0.05, 3.15), 'col'),
                               ('spine (row 2)', (0.05, 0.95), 'col'),
                               ('spine (row 3)', (0.05, -1.25), 'col'),
                               ('west end gap', (-5.95, 0.95), 'col'),
                               ('east end gap', (5.95, 0.95), 'col')):
        r, c = world_to_cell(x, y)
        lo, hi = free_run(gt_pure, r, c, axis)
        if axis == 'row':
            a, b = ORIGIN[1] + lo * RES, ORIGIN[1] + (hi + 1) * RES
            mid = ' centre cell row %s' % ((lo + hi) // 2 if (hi - lo) % 2 == 0
                                          else '%d/%d' % ((lo + hi) // 2, (lo + hi) // 2 + 1))
            print('%-16s rows %d..%d  y[%.2f,%.2f] width %.2f m%s'
                  % (name, lo, hi, a, b, b - a, mid))
        else:
            a, b = ORIGIN[0] + lo * RES, ORIGIN[0] + (hi + 1) * RES
            mid = ' centre cell col %s' % ((lo + hi) // 2 if (hi - lo) % 2 == 0
                                          else '%d/%d' % ((lo + hi) // 2, (lo + hi) // 2 + 1))
            print('%-16s cols %d..%d  x[%.2f,%.2f] width %.2f m%s'
                  % (name, lo, hi, a, b, b - a, mid))

    marks = {}
    for k, ch in (('spawn_marker', 'S'), ('pickup_marker', 'P'), ('dropoff_marker', 'D')):
        for m in markers[k]:
            marks[world_to_cell(m['x'], m['y'])] = ch
    box = next(c for c in comps if classify(c) == 'movable_box')
    marks[world_to_cell(box['x'], box['y'])] = 'B'
    print('\n== ASCII (1 col x 2 rows per char = 0.1 x 0.2 m, north up) ==')
    print(ascii_map(gt, marks))


def main(argv):
    if len(argv) >= 2 and argv[1] == 'write-grid':
        write_grid_yaml(ground_truth(parse_world()))
        print('wrote occupancy of', GRID_YAML)
    elif len(argv) >= 2 and argv[1] == 'write-pgm':
        out = argv[2] if len(argv) > 2 else PGM
        write_pgm(ground_truth(parse_world()), out)
        print('wrote', out)
    else:
        report()


if __name__ == '__main__':
    main(sys.argv)
