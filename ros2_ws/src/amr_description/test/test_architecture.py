"""
THE MOST IMPORTANT TEST IN THE REPOSITORY.

It enforces that amr_fleet/core/ never imports rclpy, ROS messages, or any
hardware library. That single rule is what makes these two claims true rather
than aspirational:

  1. The coordination algorithms can be unit-tested with plain pytest, with no
     ROS installed and no simulator running.
  2. The same algorithms can later run on a Jetson over a different transport
     without a rewrite.

Architecture discipline that is not tested decays within a week. Run this in CI
or before every commit.
"""
import ast
import pathlib

import pytest

FORBIDDEN = {
    "rclpy", "rosidl_runtime_py", "ament_index_python",
    "amr_description",          # the generated ROS message module
    "std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs", "builtin_interfaces",
    "serial", "RPi", "Jetson", "smbus", "gpiozero",
    "cv2", "gz", "gazebo_msgs",
}

CORE = pathlib.Path(__file__).resolve().parents[1] / "amr_fleet" / "core"


def _imports(path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                yield node.lineno, a.name.split(".")[0]
        elif isinstance(node, ast.ImportFrom):
            if node.level and node.level > 0:
                continue                       # relative import, always fine
            if node.module:
                yield node.lineno, node.module.split(".")[0]


def test_core_has_no_platform_imports():
    assert CORE.is_dir(), f"core directory not found at {CORE}"
    violations = []
    for path in sorted(CORE.rglob("*.py")):
        for lineno, mod in _imports(path):
            if mod in FORBIDDEN:
                violations.append(f"{path.name}:{lineno} imports '{mod}'")
    assert not violations, (
        "amr_fleet/core must stay platform-free:\n  " + "\n  ".join(violations))


def test_core_modules_import_standalone():
    """core must import with NO ROS on sys.path at all."""
    import importlib
    import sys
    sys.path.insert(0, str(CORE.parents[1]))
    for name in ["models", "gridmap", "priority", "geometry", "conflict",
                 "zone", "deadlock", "tasks", "peers", "astar", "coordinator"]:
        importlib.import_module(f"amr_fleet.core.{name}")


def test_no_module_shadows_stdlib():
    """A core module named e.g. types.py or queue.py shadows the stdlib and
    breaks imports in confusing ways. This bit us once already."""
    import sys
    stdlib = set(sys.stdlib_module_names) if hasattr(sys, "stdlib_module_names") else {
        "types", "queue", "time", "json", "math", "socket", "select", "copy"}
    clashes = [p.stem for p in CORE.glob("*.py") if p.stem in stdlib]
    assert not clashes, f"core modules shadow stdlib modules: {clashes}"


def test_cmake_does_not_mix_rosidl_with_ament_python_install_package():
    cmake = (CORE.parents[1] / "CMakeLists.txt").read_text()
    assert "ament_python_install_package(amr_fleet)" not in cmake
    assert "ament_get_python_install_dir(PYTHON_INSTALL_DIR)" in cmake
