# EdgeSwarm

A decentralized fleet of three autonomous mobile robots for warehouse pick and
delivery. Robots coordinate peer to peer: there is no central server. They
drive on keep-right lanes, resolve crossings at box junctions by a
deterministic right-of-way, and keep a safety envelope based on each robot's
real footprint and localization uncertainty.

## Fleet UI

![Warehouse map](docs/images/fleet-ui-map.png)

![Traffic view](docs/images/fleet-ui-traffic.png)

The web UI runs on http://localhost:8080. It shows the live map, lanes and
junctions, robot hitboxes with heading, task states, and badges for U-turn,
reversing and overtaking manoeuvres. Tasks can be created by clicking a rack
slot and aborted before pickup.

## Quick start

Inside WSL Ubuntu 24.04:

```bash
./setup.sh    # once: installs dependencies, builds, prints Windows steps
./run.sh      # starts Webots, Nav2, the fleet and the web UI
```

`./run.sh gazebo` starts the Gazebo backend instead. Extra launch arguments
pass through, for example `./run.sh webots rviz:=false`.

## What is inside

- Task auction without an auctioneer: every robot computes the same winner.
- Street model: one-way lane per direction in every 1.6 m aisle, junction
  boxes, U-turns and overtaking gated on real clearance checks.
- Oriented hitbox safety envelope, wheel-inclusive, inflated by broadcast
  localization sigma and message age. Zero contacts across all test runs.
- Rack slot addressing: 240 pick faces named 1R1 to 12R20.
- Deadline aging so old tasks win auctions and right of way.
- Offline fleet simulator (runs the real coordinator code at about 40x
  realtime) with lane discipline and contact audits; 400+ pytest tests.

## Layout

```
src/amr_description         fleet core, messages, config, web UI, tests
src/amr_navigation_runtime  Nav2 configs, launches, velocity gate, goal bridge
src/webots_warehouse_sim    Webots world and robot drivers
src/warehouse_sim           Gazebo world and bridges
tools/                      fleet_eval.py scorecard, lidar_probe.py
```

## Testing

```bash
cd src/amr_description && python3 -m pytest test/   # pure python suite
python3 tools/fleet_eval.py 480                     # live 8 min scorecard
```

See SETUP.md for the full guide.
