#!/usr/bin/env bash
# Run the EdgeSwarm fleet simulation.
#
# Usage:
#   ./run.sh                 # Webots backend, RViz on, web UI on :8080
#   ./run.sh gazebo          # Gazebo backend
#   ./run.sh webots rviz:=false webots_gui:=true   # any extra launch args
#
# Run this from WSL Ubuntu. Run ./setup.sh once before the first start.
set -euo pipefail
cd "$(dirname "$0")"

SIM="${1:-webots}"
case "$SIM" in webots|gazebo|external) shift || true ;; *) SIM=webots ;; esac

source /opt/ros/jazzy/setup.bash
if [ ! -f install/setup.bash ]; then
  echo "No install/ directory. Run ./setup.sh first." >&2
  exit 1
fi
source install/setup.bash

echo "Starting fleet (sim: $SIM). Web UI: http://localhost:8080"
exec ros2 launch amr_navigation_runtime three_amr.launch.py "sim:=$SIM" "$@"
