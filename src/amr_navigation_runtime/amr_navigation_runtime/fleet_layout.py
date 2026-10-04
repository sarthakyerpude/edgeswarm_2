"""Shared fleet layout constants for launch files and sim adapters.

Single source of truth so the simulator adapters (sim_*.launch.py) and the
sim-agnostic bringup (three_amr.launch.py) can never disagree on robot names
or spawn poses. Every simulator backend must spawn these robots at these
poses; the fleet/Nav2 side starts agents for exactly this list.
"""

# (robot_id, x, y, yaw) — spawn markers in warehouse coordinates.
ROBOTS = [
    ('robot_1', '-3.0', '-4.0', '1.5708'),
    ('robot_2', '0.0', '-4.0', '1.5708'),
    ('robot_3', '3.0', '-4.0', '1.5708'),
]

# (link, gz sensor name, ROS topic suffix) for the per-link contact streams.
# Gazebo-specific today; an Isaac/Webots adapter must either synthesize the
# same streams or the recorders need their own adapter (see wiki migration plan).
CONTACT_SENSORS = [
    ('base_link', 'chassis_contact', 'contacts'),
    ('wheel_left', 'wheel_contact', 'contacts/wheel_left'),
    ('wheel_right', 'wheel_contact', 'contacts/wheel_right'),
    ('caster', 'caster_contact', 'contacts/caster'),
]
