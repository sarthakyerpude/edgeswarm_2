"""Audit a robot's raw lidar scan and point cloud against ground-truth geometry.

Run with the robot parked at its spawn pose (launch with start_tasks:=false):
    python3 lidar_probe.py robot_3 3.0 -4.0 1.5708 [topic] [--cloud]
Reports, per 15-degree bearing bin, how many beams land within 0.3 m of the
true wall/rack range for the LaserScan labels, and with --cloud also for the
point cloud's own x,y. WARNING: subscribing to the point cloud makes
webots_ros2 enable it, and on this setup that crashed Webots (exit 1) a few
seconds later, so it is opt-in.
"""
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

WANT_CLOUD = '--cloud' in sys.argv
ARGS = [a for a in sys.argv[1:] if a != '--cloud']
ROBOT = ARGS[0] if ARGS else 'robot_3'
PX, PY, TH = (float(v) for v in (ARGS[1:4] if len(ARGS) > 3
                                  else ('3.0', '-4.0', '1.5708')))
TOPIC = ARGS[4] if len(ARGS) > 4 else 'scan_f_raw'

BOXES = [(-6.15, -6.0, -5.15, 5.15), (6.0, 6.15, -5.15, 5.15),
         (-6.15, 6.15, 5.0, 5.15), (-6.15, 6.15, -5.15, -5.0)]
for cy in (3.1, 0.9, -1.3):          # R7: 5.0 x 0.6 m racks, 1.6 m aisles
    BOXES += [(-5.8, -0.8, cy - 0.3, cy + 0.3), (0.8, 5.8, cy - 0.3, cy + 0.3)]
    # rack-end caps (wall_endcap_*) closing the 0.2 m end gaps
    BOXES += [(-6.0, -5.8, cy - 0.3, cy + 0.3), (5.8, 6.0, cy - 0.3, cy + 0.3)]
# R11 localization landmarks (landmark_* in warehouse.wbt): static solids in
# the south area that break the lidar self-similarity; beams on them are TRUE.
BOXES += [(1.2, 1.5, -4.7, -4.4),     # landmark_pillar_se
          (-6.0, -5.4, -2.5, -2.3),   # landmark_fin_w
          (5.6, 6.0, -3.7, -3.5)]     # landmark_fin_e


def cast(a):
    dx, dy = math.cos(a), math.sin(a)
    best = float('inf')
    for x1, x2, y1, y2 in BOXES:
        if abs(dx) > 1e-9:
            for xe in (x1, x2):
                t = (xe - PX) / dx
                if t > 0.05 and y1 <= PY + t * dy <= y2:
                    best = min(best, t)
        if abs(dy) > 1e-9:
            for ye in (y1, y2):
                t = (ye - PY) / dy
                if t > 0.05 and x1 <= PX + t * dx <= x2:
                    best = min(best, t)
    return best


def report(name, samples):
    """samples: list of (bearing_rad_in_robot_frame, range)."""
    bins = {}
    good = tot = 0
    for b, r in samples:
        if not math.isfinite(r) or r <= 0.12:
            continue
        tr = cast(b + TH)
        if not math.isfinite(tr) or tr > 7.5:
            continue
        key = int(math.degrees(b) // 15) * 15
        ok = abs(r - tr) < 0.3
        g, t = bins.get(key, (0, 0))
        bins[key] = (g + ok, t + 1)
        good += ok
        tot += 1
    print(f'== {name}: {good}/{tot} good ({100 * good / max(tot, 1):.0f}%)')
    for k in sorted(bins):
        g, t = bins[k]
        flag = 'OK ' if g / t > 0.7 else 'BAD'
        print(f'   {k:5d}..{k + 15:5d} deg: {g:3d}/{t:3d} {flag}')


class Probe(Node):
    def __init__(self):
        super().__init__('lidar_probe')
        self.scan = None
        self.cloud = None
        self.create_subscription(LaserScan, f'/{ROBOT}/{TOPIC}',
                                 lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        if WANT_CLOUD:
            self.create_subscription(PointCloud2,
                                     f'/{ROBOT}/{TOPIC}/point_cloud',
                                     lambda m: setattr(self, 'cloud', m),
                                     qos_profile_sensor_data)


def main():
    rclpy.init()
    node = Probe()
    end = time.time() + 8.0
    while time.time() < end and (
            node.scan is None or (WANT_CLOUD and node.cloud is None)):
        rclpy.spin_once(node, timeout_sec=0.2)

    s = node.scan
    if s is None:
        print('NO LaserScan received')
    else:
        print(f'LaserScan: n={len(s.ranges)} angle_min={math.degrees(s.angle_min):.1f} '
              f'inc={math.degrees(s.angle_increment):.3f} frame={s.header.frame_id}')
        report('LaserScan labels as published',
               [(s.angle_min + i * s.angle_increment, r)
                for i, r in enumerate(s.ranges)])

    c = node.cloud
    if not WANT_CLOUD:
        pass
    elif c is None:
        print('NO PointCloud2 received on '
              f'/{ROBOT}/{TOPIC}/point_cloud')
    else:
        offs = {f.name: f.offset for f in c.fields}
        print(f'PointCloud2: {c.width}x{c.height} fields={list(offs)} '
              f'step={c.point_step} frame={c.header.frame_id}')
        pts = []
        for i in range(c.width * c.height):
            base = i * c.point_step
            x, = struct.unpack_from('f', c.data, base + offs['x'])
            y, = struct.unpack_from('f', c.data, base + offs['y'])
            pts.append((x, y))
        for label, sx, sy in (('cloud as-is (x fwd, y left)', 1, 1),
                              ('cloud y-mirrored', 1, -1)):
            report(label, [(math.atan2(sy * y, sx * x), math.hypot(x, y))
                           for x, y in pts if math.isfinite(x)])
    rclpy.shutdown()


if __name__ == '__main__':
    main()
