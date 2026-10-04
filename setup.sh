#!/usr/bin/env bash
# One-time setup for the EdgeSwarm fleet. Run inside WSL Ubuntu 24.04.
# Installs ROS dependencies, builds the workspace, and prints the
# Windows-side steps that cannot be automated from here.
set -euo pipefail
cd "$(dirname "$0")"

echo "== 1/4 Checking ROS 2 Jazzy"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy is not installed. Follow:"
  echo "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "== 2/4 Installing packages (sudo apt)"
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-webots-ros2-driver ros-jazzy-webots-ros2-msgs \
  python3-colcon-common-extensions python3-pytest

echo "== 3/4 Building the workspace"
colcon build --symlink-install
set +u
source install/setup.bash
set -u
echo "Build OK."

echo "== 4/4 Windows-side steps (do these once, outside WSL)"
cat <<'WIN'
1. Webots R2025a: install on Windows to C:\Program Files\Webots
   (the Webots backend starts it from WSL automatically).
2. Firewall: allow the Webots controller port. In an admin PowerShell:
   netsh advfirewall firewall add rule name="Webots extern" dir=in action=allow protocol=TCP localport=1234
3. WSL config (C:\Users\<you>\.wslconfig), then "wsl --shutdown":
   [wsl2]
   memory=8GB
   swap=8GB
   networkingMode=NAT
   dnsTunneling=false
4. Web UI after ./run.sh: http://localhost:8080
   If the page stops loading after a sim restart, close the tab and open it again.
WIN
echo "Setup complete. Start the fleet with ./run.sh"
