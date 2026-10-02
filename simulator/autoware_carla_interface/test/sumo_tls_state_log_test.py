#!/usr/bin/env python3
"""
Checks for the SUMO -> CARLA traffic-light state conversion log
(`BridgeHelper.get_carla_traffic_light_state()`, `sumo_integration/bridge_helper.py`).

When different SUMO link states are assigned to one CARLA landmark (e.g. {'G', 'r'}), CARLA is
forced to red. This used to be logged as a WARNING on every synchronization step (about once per
step in Town01). Now:

- the WARNING is emitted once per (landmark, state combination), naming the landmark,
- repetitions are logged at DEBUG level,
- the returned CARLA states are unchanged.

Uses minimal carla/lxml stand-ins where the real modules are missing (see stand_ins.py); needs
traci/sumolib (SUMO_HOME). Not wired into colcon/ament; run manually with:
    python3 test/sumo_tls_state_log_test.py

Exits with a non-zero status and an AssertionError if any check fails.
"""

import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stand_ins import install_stand_ins  # noqa: E402  pylint: disable=wrong-import-position

install_stand_ins()

import carla  # noqa: E402  pylint: disable=wrong-import-position,import-error

from autoware_carla_interface.sumo_integration.bridge_helper import (  # noqa: E402  pylint: disable=wrong-import-position
    BridgeHelper,
)

Red = carla.TrafficLightState.Red
Green = carla.TrafficLightState.Green
Yellow = carla.TrafficLightState.Yellow


class _Capture(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self, level=logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)

    def count(self, level, text):
        return sum(1 for r in self.records if r.levelno == level and text in r.getMessage())


def _capture():
    BridgeHelper._warned_mixed_tl_states.clear()  # pylint: disable=protected-access
    root = logging.getLogger()
    handler = _Capture()
    root.addHandler(handler)
    root.setLevel(logging.DEBUG)
    return root, handler


def test_returned_states_unchanged():
    root, handler = _capture()
    try:
        convert = BridgeHelper.get_carla_traffic_light_state
        assert convert({'G', 'r'}, '360') == Red
        assert convert({'G', 'y'}, '360') == Red
        assert convert({'G', 'g'}, '360') == Green
        assert convert({'y'}, '360') == Yellow
        assert convert('r') == Red, 'a single state string and no landmark id still work'
        assert convert(set()) == carla.TrafficLightState.Unknown
    finally:
        root.removeHandler(handler)


def test_warning_once_per_landmark_and_combination():
    root, handler = _capture()
    try:
        convert = BridgeHelper.get_carla_traffic_light_state
        for _ in range(100):  # 100 synchronization steps, Town01-like
            convert({'G', 'r'}, '360')
            convert({'r', 'G'}, '374')
        convert({'G', 'y'}, '360')
        convert({'y', 'G'}, '360')

        warnings = [r for r in handler.records if r.levelno == logging.WARNING]
        assert len(warnings) == 3, [r.getMessage() for r in warnings]
        messages = sorted(r.getMessage() for r in warnings)
        assert "['G', 'r'] are assigned to the same CARLA landmark 360." in messages[0], messages
        assert "['G', 'r'] are assigned to the same CARLA landmark 374." in messages[1], messages
        assert "['G', 'y'] are assigned to the same CARLA landmark 360." in messages[2], messages
        assert all('Logged once per landmark and state combination' in m for m in messages)
        assert handler.count(logging.DEBUG, 'forced to red') == 99 + 99 + 1, \
            'repetitions are kept at DEBUG level'
    finally:
        root.removeHandler(handler)


def main():
    tests = [test_returned_states_unchanged, test_warning_once_per_landmark_and_combination]
    for test in tests:
        test()
        print('PASS %s' % test.__name__)
    print('All %d checks passed.' % len(tests))


if __name__ == '__main__':
    main()
