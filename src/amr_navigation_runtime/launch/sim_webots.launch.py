"""Webots backend for three_amr.launch.py (sim:=webots).

Thin include of webots_warehouse_sim/launch/webots_world.launch.py, which
starts Webots natively on Windows (when under WSL) with the EdgeSwarm
warehouse, three robots, drivers, and the /clock supervisor. Lives in its own
package so this one carries no webots_ros2 dependency unless selected.

Pass-through args: webots_gui:=false (headless), webots_mode:=fast
(faster than real time), controller_url:=tcp://127.0.0.1:1234 (WSL IP fix).
The blockage arguments are accepted but ignored for now — move the world's
movable_box from the Webots UI, or use the Ros2Supervisor spawn services.
"""

import os
import shutil
import socket
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# WebotsLauncher's extern-controller port (webots_world.launch.py default).
_WEBOTS_PORT = 1234
# Command-line signature of a Webots instance started by this stack: the
# launcher's generated temp world (URDF robots injected) or the source world.
_STALE_PATTERN = r'world_with_URDF_robot|warehouse\.wbt'


def _webots_hosts():
    """Addresses a Webots extern-controller server may be reachable at.

    Under WSL, Webots runs natively on Windows: it is reached through the
    Windows host's vNIC address (the WSL default gateway / resolv.conf
    nameserver), not through 127.0.0.1.
    """
    hosts = ['127.0.0.1']
    try:
        with open('/etc/resolv.conf') as handle:
            for line in handle:
                parts = line.split()
                if len(parts) >= 2 and parts[0] == 'nameserver':
                    hosts.append(parts[1])
    except OSError:
        pass
    return hosts


def _webots_port_busy(port=_WEBOTS_PORT):
    for host in _webots_hosts():
        try:
            with socket.create_connection((host, port), timeout=1.5):
                return True
        except OSError:
            continue
    return False


def _kill_stale_webots(context):
    """Refuse to attach the fleet to a leftover Webots from a previous run.

    A previous launch's Webots (a native Windows process when under WSL)
    routinely survives that launch's shutdown. It keeps extern-controller
    port 1234, so this run's WebotsLauncher silently falls back to another
    port while every driver connects to the STALE world: sim time resumes
    mid-run, the robots sit wherever the old run left them, and AMCL —
    initialized at the nominal spawn poses — starts metres off and never
    converges (runs webots_final2/3: 8.55 m instant divergence). The stale
    instance also spins a full core waiting on its dead synchronized
    controllers, which is what made those startups slow in the first place.

    So: if something is already listening on the port, kill Webots processes
    matching this project's command-line signature (Windows side via WSL
    interop), then verify the port is free; otherwise abort loudly instead of
    starting a run that is guaranteed to mislocalize.
    """
    # Run the cleanup unconditionally: a leftover instance pushed off port
    # 1234 cannot capture this run's controllers, but it still burns a full
    # core (slow startup). At this point no Webots belongs to this launch
    # yet, so anything matching the signature is stale by definition.
    powershell = shutil.which('powershell.exe')  # present under WSL interop
    if powershell:
        script = (
            "Get-CimInstance Win32_Process -Filter \"Name='webots.exe' or "
            "Name='webots-bin.exe'\" | Where-Object { $_.CommandLine -match "
            "'world_with_URDF_robot|warehouse\\.wbt' } | ForEach-Object { "
            "try { Stop-Process -Id $_.ProcessId -Force -ErrorAction Stop; "
            "Write-Output ('killed ' + $_.ProcessId) } catch {} }")
        try:
            result = subprocess.run(
                [powershell, '-NoProfile', '-Command', script],
                capture_output=True, text=True, timeout=45)
            if result.stdout.strip():
                print('[sim_webots] ' + result.stdout.strip().replace(
                    '\n', '; '))
        except (OSError, subprocess.SubprocessError) as error:
            print(f'[sim_webots] stale-Webots cleanup failed: {error}')
    else:
        # Native Linux: Webots runs locally; match the same signature.
        subprocess.run(['pkill', '-f', _STALE_PATTERN], check=False)
    for _ in range(20):
        if not _webots_port_busy():
            print('[sim_webots] port is free; starting a fresh Webots.')
            return []
        time.sleep(0.5)
    raise RuntimeError(
        f'Something is still listening on 127.0.0.1:{_WEBOTS_PORT} after '
        'stale-Webots cleanup. Starting now would attach the robot drivers '
        'to that foreign/stale simulation (wrong robot poses, mid-run sim '
        'clock) and AMCL would diverge immediately. Close the process '
        'holding the port (e.g. an old webots.exe / webots-bin.exe on the '
        'Windows host) and relaunch.')


def generate_launch_description():
    world_launch = os.path.join(
        get_package_share_directory('webots_warehouse_sim'),
        'launch', 'webots_world.launch.py')

    cyclonedds_xml = os.path.join(
        get_package_share_directory('webots_warehouse_sim'),
        'resource', 'cyclonedds.xml')

    return LaunchDescription([
        # The project's DDS setup (see webots_warehouse_sim/resource/cyclonedds.xml + SETUP.md):
        # Fast DDS under WSL hit shared-memory failures and multi-minute
        # service discovery with this ~40-node graph; Cyclone with a raised
        # participant index is what the Gazebo stack always ran on.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + cyclonedds_xml),
        DeclareLaunchArgument('inject_blockage', default_value='false'),
        DeclareLaunchArgument('blockage_delay_s', default_value='34.0'),
        DeclareLaunchArgument('blockage_x', default_value='0.0'),
        DeclareLaunchArgument('blockage_y', default_value='2.0'),
        DeclareLaunchArgument('blockage_z', default_value='0.4'),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('inject_blockage')),
            msg='sim:=webots ignores inject_blockage for now — teleport the '
                'movable_box from the Webots UI instead.'),
        # Must run before WebotsLauncher: a surviving Webots from an earlier
        # run holds port 1234 and this run's drivers would attach to its
        # stale world (robots off-spawn, mid-run clock -> AMCL divergence).
        OpaqueFunction(function=_kill_stale_webots),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(world_launch)),
    ])
