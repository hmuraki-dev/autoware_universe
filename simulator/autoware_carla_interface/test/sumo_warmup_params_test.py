#!/usr/bin/env python3
"""
Step S1 checks for the SUMO warmup / EGO safe spawn parameters (see
docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md, section 2.2 / Step S1).

1. `ego_spawn_gate.validate_warmup_params()`: accepted / rejected combinations.
2. The four parameters are declared consistently in all three places they must appear:
   the launch <arg> (default), the <param> of the autoware_carla_interface node, and the
   ROS parameter table in carla_ros.py (default and type).

Needs neither carla, traci nor ROS: `ego_spawn_gate` has no such imports, and the launch file /
carla_ros.py are read as text. Not wired into colcon/ament (no pytest infra exists for this package
yet); run manually with:
    python3 test/sumo_warmup_params_test.py

Exits with a non-zero status and an AssertionError if any check fails.
"""

import ast
import os
import re
import sys
import xml.etree.ElementTree as ET

_PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PACKAGE_DIR, 'src'))

from autoware_carla_interface.sumo_integration.ego_spawn_gate import (  # noqa: E402  pylint: disable=wrong-import-position
    is_random_spawn_point,
    validate_warmup_params,
)

_LAUNCH_FILE = os.path.join(_PACKAGE_DIR, 'launch', 'autoware_carla_interface.launch.xml')
_CARLA_ROS_FILE = os.path.join(_PACKAGE_DIR, 'src', 'autoware_carla_interface', 'carla_ros.py')

# name -> (expected default, expected rclpy type name)
_EXPECTED_PARAMS = {
    'sumo_warmup_time': (0, 'INTEGER'),
    'ego_spawn_front_margin': (20.0, 'DOUBLE'),
    'ego_spawn_rear_margin': (20.0, 'DOUBLE'),
    'ego_spawn_wait_timeout': (60, 'INTEGER'),
}

_SPAWN_POINT = '229.78,-2.02,0.0,0.0,0.0,180.0'
_VALID = {
    'use_sumo': True,
    'sumo_warmup_time': 100,
    'ego_spawn_front_margin': 20.0,
    'ego_spawn_rear_margin': 20.0,
    'ego_spawn_wait_timeout': 60,
    'spawn_point': _SPAWN_POINT,
    'tls_manager': 'sumo',
}


def _validate(**overrides):
    kwargs = dict(_VALID)
    kwargs.update(overrides)
    validate_warmup_params(**kwargs)


def _expect_error(substring, **overrides):
    try:
        _validate(**overrides)
    except ValueError as e:
        assert substring in str(e), 'expected %r in error, got %r' % (substring, str(e))
        return
    raise AssertionError('expected ValueError (%s) for %r' % (substring, overrides))


def test_spawn_point_parsing():
    assert not is_random_spawn_point(_SPAWN_POINT)
    assert is_random_spawn_point('None')  # launch default
    assert is_random_spawn_point('')
    assert is_random_spawn_point('1,2,3')


def test_warmup_disabled_accepts_anything():
    # Default (warmup disabled) must not reject runs that work today (requirement 10).
    _validate(sumo_warmup_time=0, use_sumo=False, spawn_point='None', tls_manager='carla',
              ego_spawn_front_margin=-1.0, ego_spawn_rear_margin=-1.0, ego_spawn_wait_timeout=0)


def test_warmup_enabled_valid():
    _validate()
    _validate(tls_manager='none')
    _validate(ego_spawn_front_margin=0.0, ego_spawn_rear_margin=0.0, ego_spawn_wait_timeout=1)


def test_warmup_enabled_rejected():
    _expect_error('sumo_warmup_time must be >= 0', sumo_warmup_time=-1)
    _expect_error('sumo_warmup_time must be >= 0', sumo_warmup_time=-1, use_sumo=False)
    _expect_error('requires use_sumo=True', use_sumo=False)
    _expect_error('requires spawn_point', spawn_point='None')
    _expect_error('tls_manager', tls_manager='carla')
    _expect_error('ego_spawn_front_margin', ego_spawn_front_margin=-0.1)
    _expect_error('ego_spawn_rear_margin', ego_spawn_rear_margin=-0.1)
    _expect_error('ego_spawn_wait_timeout', ego_spawn_wait_timeout=0)


def _launch_eval(expression):
    """Evaluates the subset of `$(eval "...")` / `$(var x)` used for these params."""
    match = re.fullmatch(r'\$\(eval "(.*)"\)', expression)
    return eval(match.group(1)) if match else expression  # pylint: disable=eval-used


def test_launch_file():
    root = ET.parse(_LAUNCH_FILE).getroot()
    args = {arg.get('name'): arg.get('default') for arg in root.iter('arg')}
    nodes = [node for node in root.iter('node') if node.get('exec') == 'autoware_carla_interface']
    assert len(nodes) == 1, 'expected one autoware_carla_interface node, found %d' % len(nodes)
    assert nodes[0].get('on_exit') == 'shutdown', \
        'a test start failure (exit of the bridge) must stop the whole launch (section 2.8)'
    params = {param.get('name'): param.get('value') for param in nodes[0].iter('param')}

    for name, (default, type_name) in _EXPECTED_PARAMS.items():
        assert name in args, 'launch <arg> %s missing' % name
        assert float(args[name]) == default, 'launch default of %s: %r' % (name, args[name])
        assert name in params, '<param> %s missing in the autoware_carla_interface node' % name
        assert '$(var %s)' % name in params[name], '<param> %s does not use $(var %s)' % (name, name)

        # What the node receives for the default and for an integer override (e.g. :=25).
        # yaml-style typing: '25' -> int, '25.0' -> float; it must match the declared type.
        for given in (args[name], '25'):
            value = str(_launch_eval(params[name].replace('$(var %s)' % name, given)))
            parsed = float(value) if '.' in value else int(value)
            if type_name == 'DOUBLE':
                assert isinstance(parsed, float), '%s:=%s reaches the node as %r' % (name, given, value)
            else:
                assert isinstance(parsed, int), '%s:=%s reaches the node as %r' % (name, given, value)


def test_carla_ros_parameter_table():
    with open(_CARLA_ROS_FILE, encoding='utf-8') as f:
        source = f.read()
    for name, (default, type_name) in _EXPECTED_PARAMS.items():
        match = re.search(r'"%s": \(rclpy\.Parameter\.Type\.(\w+), ([^)]+)\)' % name, source)
        assert match, 'carla_ros.py does not declare %s' % name
        assert match.group(1) == type_name, '%s declared as %s' % (name, match.group(1))
        declared_default = ast.literal_eval(match.group(2))
        assert declared_default == default and type(declared_default) is type(default), \
            '%s default %r' % (name, declared_default)


def main():
    tests = [
        test_spawn_point_parsing,
        test_warmup_disabled_accepts_anything,
        test_warmup_enabled_valid,
        test_warmup_enabled_rejected,
        test_launch_file,
        test_carla_ros_parameter_table,
    ]
    for test in tests:
        test()
        print('PASS %s' % test.__name__)
    print('All %d checks passed.' % len(tests))


if __name__ == '__main__':
    main()
