"""
amr_fleet - decentralised fleet coordination for AMRs.

This Python package is installed FROM the `amr_description` ROS package but is
deliberately named differently. See docs/PACKAGE_LAYOUT.md for the reason
(short version: rosidl generates a Python module literally named
`amr_description`, so a hand-written module of the same name would collide
with it at install time and one would silently shadow the other).

Layout
------
amr_fleet.core   : PURE PYTHON. No rclpy, no ROS message imports.
                   Unit-testable with plain pytest, and runnable on a Jetson
                   with no ROS installed at all.
amr_fleet.nodes  : the ROS 2 layer. Converts ROS messages <-> core dataclasses
                   and owns all rclpy objects.

That split is what makes "the same coordination code runs in Gazebo and on
hardware" a fact rather than an aspiration.
"""

__version__ = "0.1.0"
