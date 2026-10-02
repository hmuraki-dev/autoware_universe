"""
Minimal `carla` / `lxml` stand-ins for the stub tests in this directory.

The vendored `sumo_integration` modules import `carla` (and `sumo_simulation.py` imports
`lxml.etree`) at import time. On machines without them (e.g. a Windows dev PC) the stub tests
install these stand-ins first; where the real modules exist (the Linux machine), they are used and
nothing is replaced. Only what the stub tests and the code paths they exercise need is provided.
"""

import os
import sys
import types

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))


def install_stand_ins():
    """Minimal carla / lxml stand-ins, only if the real modules are missing."""
    try:
        import carla  # noqa: F401  pylint: disable=import-outside-toplevel,unused-import,import-error
    except ImportError:
        carla = types.ModuleType('carla')

        class _Vec(object):
            def __init__(self, x=0.0, y=0.0, z=0.0):
                self.x, self.y, self.z = x, y, z

        class Rotation(object):
            def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
                self.pitch, self.yaw, self.roll = pitch, yaw, roll

        class Transform(object):
            def __init__(self, location=None, rotation=None):
                self.location = location if location is not None else _Vec()
                self.rotation = rotation if rotation is not None else Rotation()

        carla.Location = carla.Vector3D = _Vec
        carla.Rotation = Rotation
        carla.Transform = Transform
        carla.VehicleLightState = types.SimpleNamespace(NONE=0)
        carla.TrafficLightState = types.SimpleNamespace(Red=0, Yellow=1, Green=2, Off=3,
                                                        Unknown=4)
        sys.modules['carla'] = carla
    try:
        import lxml.etree  # noqa: F401  pylint: disable=import-outside-toplevel,unused-import
    except ImportError:
        # sumo_simulation.py only needs the module to exist at import time; aliasing the standard
        # library's ElementTree keeps sumolib (which prefers lxml when importable) working.
        import xml.etree.ElementTree  # pylint: disable=import-outside-toplevel
        lxml = types.ModuleType('lxml')
        lxml.etree = xml.etree.ElementTree
        sys.modules['lxml'] = lxml
        sys.modules['lxml.etree'] = lxml.etree
