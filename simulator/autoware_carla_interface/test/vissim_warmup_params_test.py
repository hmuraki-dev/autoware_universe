#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Stub test for the Vissim warmup / EGO safe spawn parameters in carla_autoware.py (see
docs/Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md Step V1):

- InitializeInterface reads vissim_warmup_time / ego_spawn_front_margin / ego_spawn_rear_margin /
  ego_spawn_wait_timeout;
- its startup validation of them (use_vissim only, and the spawn-related ones only while the
  warmup is enabled), including the fixed spawn_point the warmup requires.

Needs neither ROS 2, CARLA, nor a vissim adapter: `carla`, `zmq`, `msgpack` and the ROS-dependent
`carla_ros`/`modules.*` modules are replaced by mocks, only for the duration of run() (via
mock.patch.dict(sys.modules), so nothing leaks into other tests if this file is imported by a test
runner). Run with:
    python3 test/vissim_warmup_params_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

import os
import sys
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

_STUBBED_MODULES = [
    'carla',
    'zmq',
    'msgpack',
    'autoware_carla_interface.carla_ros',
    'autoware_carla_interface.modules',
    'autoware_carla_interface.modules.carla_data_provider',
    'autoware_carla_interface.modules.carla_wrapper',
]

_SPAWN_POINT = '229.8,-2.0,0.3,0.0,0.0,180.0'

# Parameters as returned by carla_ros2_interface.get_param(), with valid Vissim settings and the
# warmup disabled (the defaults of carla_ros.py / the launch file).
_BASE_PARAMS = {
    'host': 'localhost',
    'port': 2000,
    'timeout': 20,
    'sync_mode': True,
    'fixed_delta_seconds': 0.05,
    'carla_map': 'Town01',
    'ego_vehicle_role_name': 'ego_vehicle',
    'vehicle_type': 'vehicle.toyota.prius',
    'spawn_point': 'None',
    'use_traffic_manager': False,
    'max_real_delta_seconds': 0.05,
    'use_vissim': True,
    'vissim_adapter_host': '127.0.0.1',
    'vissim_adapter_port': 5555,
    'vissim_connect_timeout_ms': 60000,
    'vissim_rpc_timeout_ms': 2000,
    'vissim_simulator_vehicles': 1,
    'sync_traffic_lights': False,
    'vissim_sim_period': 600,
    'vissim_max_consecutive_failures': 3,
    'vissim_warmup_time': 0,
    'ego_spawn_front_margin': 20.0,
    'ego_spawn_rear_margin': 20.0,
    'ego_spawn_wait_timeout': 60,
}

# A valid warmup configuration, on top of _BASE_PARAMS.
_WARMUP = {'vissim_warmup_time': 100, 'spawn_point': _SPAWN_POINT}

# ==================================================================================================
# -- helpers -----------------------------------------------------------------------------------------
# ==================================================================================================


def _make_interface(ca, **overrides):
    ca.carla_ros2_interface.return_value.get_param.return_value = dict(_BASE_PARAMS, **overrides)
    return ca.InitializeInterface()


def _expect_value_error(ca, overrides):
    try:
        _make_interface(ca, **overrides)
    except ValueError:
        return
    raise AssertionError('expected ValueError for %r' % overrides)


# ==================================================================================================
# -- checks ------------------------------------------------------------------------------------------
# ==================================================================================================


def check_params_are_read(ca):
    interface = _make_interface(ca, vissim_warmup_time=120, ego_spawn_front_margin=25.0,
                                ego_spawn_rear_margin=15.5, ego_spawn_wait_timeout=90,
                                spawn_point=_SPAWN_POINT)
    assert interface.vissim_warmup_time == 120
    assert interface.ego_spawn_front_margin == 25.0
    assert interface.ego_spawn_rear_margin == 15.5
    assert interface.ego_spawn_wait_timeout == 90


def check_defaults_keep_warmup_disabled(ca):
    # The defaults (warmup disabled, random spawn point) must stay valid, exactly as before the
    # warmup parameters existed.
    interface = _make_interface(ca)
    assert interface.vissim_warmup_time == 0


def check_valid_warmup_accepted(ca):
    _make_interface(ca, **_WARMUP)
    _make_interface(ca, **dict(_WARMUP, ego_spawn_front_margin=0.0, ego_spawn_rear_margin=0.0,
                               ego_spawn_wait_timeout=1))
    # Spaces around the comma-separated values are accepted, as by _parse_spawn_point().
    _make_interface(ca, **dict(_WARMUP, spawn_point='229.8, -2.0, 0.3, 0.0, 0.0, 180.0'))


def check_invalid_warmup_rejected(ca):
    for overrides in [
            {'vissim_warmup_time': -1},
            dict(_WARMUP, ego_spawn_wait_timeout=0),
            dict(_WARMUP, ego_spawn_wait_timeout=-5),
            dict(_WARMUP, ego_spawn_front_margin=-0.1),
            dict(_WARMUP, ego_spawn_rear_margin=-1.0),
            dict(_WARMUP, spawn_point='None'),  # random spawn point
            dict(_WARMUP, spawn_point=''),
            dict(_WARMUP, spawn_point='229.8,-2.0,0.3'),  # too few values
            dict(_WARMUP, spawn_point='229.8,-2.0,0.3,0.0,0.0,180.0,1.0'),  # too many values
            dict(_WARMUP, spawn_point='[229.8,-2.0,0.3,0.0,0.0,180.0]'),  # not plain numbers
    ]:
        _expect_value_error(ca, overrides)


def check_spawn_params_ignored_while_warmup_disabled(ca):
    # Without the warmup, the spawn-related parameters are not used, so invalid values must not
    # break a launch that does not use the warmup.
    _make_interface(ca, vissim_warmup_time=0, ego_spawn_wait_timeout=0,
                    ego_spawn_front_margin=-1.0, ego_spawn_rear_margin=-1.0, spawn_point='None')


def check_validation_is_noop_without_vissim(ca):
    # CARLA-only operation is unaffected by the warmup parameters, even if they are invalid.
    _make_interface(ca, use_vissim=False, vissim_warmup_time=-1)
    _make_interface(ca, use_vissim=False, vissim_warmup_time=100, spawn_point='None',
                    ego_spawn_wait_timeout=0)


# ==================================================================================================
# -- entry point -------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    stubs = {name: mock.MagicMock() for name in _STUBBED_MODULES}
    with mock.patch.dict(sys.modules, stubs):
        from autoware_carla_interface import carla_autoware as ca

        check_params_are_read(ca)
        check_defaults_keep_warmup_disabled(ca)
        check_valid_warmup_accepted(ca)
        check_invalid_warmup_rejected(ca)
        check_spawn_params_ignored_while_warmup_disabled(ca)
        check_validation_is_noop_without_vissim(ca)
    print('All vissim warmup parameter checks passed.')


if __name__ == '__main__':
    run()
