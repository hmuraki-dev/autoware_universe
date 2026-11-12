"""
pytest setup for the ROS-free unit tests (test_traffic_light_*.py).

Makes `autoware_carla_interface` importable from `src/` without building the package. The
package's `modules/__init__.py` imports ROS (rclpy, sensor_msgs, ament_index_python), so on
machines without ROS (e.g. a Windows dev PC) the two package levels are registered as bare
packages instead, without running their `__init__.py`. The pure modules under test
(`traffic_light_matcher.py`, `traffic_light_publisher.py`) then import normally. Where ROS is
sourced, the real package is used and nothing is replaced.

Run with:
    python3 -m pytest test/test_traffic_light_matcher.py test/test_traffic_light_publisher.py
"""

import importlib
import os
import sys
import types

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


def _install_bare_packages():
    """Register the package levels without executing their __init__.py."""
    for name in list(sys.modules):
        if name == "autoware_carla_interface" or name.startswith("autoware_carla_interface."):
            del sys.modules[name]
    for name, rel in (
        ("autoware_carla_interface", "autoware_carla_interface"),
        ("autoware_carla_interface.modules", os.path.join("autoware_carla_interface", "modules")),
    ):
        package = types.ModuleType(name)
        package.__path__ = [os.path.join(_SRC, rel)]
        sys.modules[name] = package


try:
    importlib.import_module("autoware_carla_interface.modules")
except ImportError:
    _install_bare_packages()
