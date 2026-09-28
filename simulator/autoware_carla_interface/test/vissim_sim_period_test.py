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
Stub test for the Vissim simulation period management in carla_autoware.py (see
docs/Vissim_CARLA_Autoware_シミュレーション期間管理_実装計画_v1.0.md Steps V5/V7):

- InitializeInterface's startup validation of vissim_sim_period / fixed_delta_seconds /
  vissim_max_consecutive_failures (use_vissim only), and sim_period being passed on to
  PTVVissimSimulation;
- SensorLoop stopping once the co-simulation period has elapsed, or after too many consecutive
  failed vissim adapter ticks - and never stopping on its own without Vissim.

Needs neither ROS 2, CARLA, nor a vissim adapter: `carla`, `zmq`, `msgpack` and the ROS-dependent
`carla_ros`/`modules.*` modules are replaced by mocks, only for the duration of run() (via
mock.patch.dict(sys.modules), so nothing leaks into other tests if this file is imported by a test
runner). Run with:
    python3 test/vissim_sim_period_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

import os
import sys
import types
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

# Parameters as returned by carla_ros2_interface.get_param(), with valid Vissim settings.
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
}

# ==================================================================================================
# -- helpers -----------------------------------------------------------------------------------------
# ==================================================================================================


def _make_interface(ca, **overrides):
    ca.carla_ros2_interface.return_value.get_param.return_value = dict(_BASE_PARAMS, **overrides)
    return ca.InitializeInterface()


def _make_sensor_loop(ca, vissim, max_consecutive_failures=3):
    loop = ca.SensorLoop()
    loop.running = True
    loop.sensor = mock.MagicMock()
    loop.ego_actor = mock.MagicMock()
    loop.vissim_sync = mock.MagicMock()
    loop.vissim_sync.vissim = vissim
    loop.vissim_max_consecutive_failures = max_consecutive_failures
    return loop


def _run_until_stopped(loop, max_iterations=100):
    """Ticks the loop (with a strictly increasing timestamp) until it stops. Returns the count."""
    for iteration in range(1, max_iterations + 1):
        loop._tick_sensor(types.SimpleNamespace(elapsed_seconds=float(iteration)))
        if not loop.running:
            return iteration
    raise AssertionError('loop did not stop within %d iterations' % max_iterations)


# ==================================================================================================
# -- checks ------------------------------------------------------------------------------------------
# ==================================================================================================


def check_startup_validation_rejects_invalid_params(ca):
    _make_interface(ca)  # valid defaults: no error

    for overrides in [
            {'vissim_sim_period': 0},
            {'vissim_sim_period': -1},
            {'vissim_sim_period': 2678391},  # + 10 s margin exceeds Vissim's 2678400 s maximum
            {'fixed_delta_seconds': 0.03},  # not 1/N s
            {'fixed_delta_seconds': 0.02},  # resolution 50, above 20
            {'vissim_max_consecutive_failures': 0},
    ]:
        try:
            _make_interface(ca, **overrides)
        except ValueError:
            continue
        raise AssertionError('expected ValueError for %r' % overrides)


def check_startup_validation_is_noop_without_vissim(ca):
    # CARLA-only operation is unaffected by the Vissim parameters, even if they are invalid.
    _make_interface(ca, use_vissim=False, vissim_sim_period=0, fixed_delta_seconds=0.03,
                    vissim_max_consecutive_failures=0)


def check_sim_period_passed_to_vissim_simulation(ca):
    interface = _make_interface(ca, vissim_sim_period=60)
    package = 'autoware_carla_interface.vissim_integration'
    with mock.patch(package + '.vissim_simulation.PTVVissimSimulation') as vissim_cls, \
            mock.patch(package + '.carla_simulation.CarlaSimulation'), \
            mock.patch(package + '.simulation_synchronization.SimulationSynchronization'):
        interface._init_vissim_integration(mock.MagicMock())
    vissim_args = vissim_cls.call_args[0][0]
    assert vissim_args.sim_period == 60, vars(vissim_args)
    assert vissim_args.step_length == 0.05, vars(vissim_args)


def check_sensor_loop_stops_when_period_elapsed(ca):
    vissim = types.SimpleNamespace(tick_count=0, end_tick=5, consecutive_failures=0)
    loop = _make_sensor_loop(ca, vissim)

    def successful_vissim_tick():
        vissim.tick_count += 1

    loop.vissim_sync.sync_vissim_to_carla.side_effect = successful_vissim_tick
    assert _run_until_stopped(loop) == 5
    assert vissim.tick_count == 5


def check_sensor_loop_stops_after_consecutive_failures(ca):
    vissim = types.SimpleNamespace(tick_count=0, end_tick=1000, consecutive_failures=0)
    loop = _make_sensor_loop(ca, vissim, max_consecutive_failures=3)

    def failed_vissim_tick():
        vissim.consecutive_failures += 1

    loop.vissim_sync.sync_vissim_to_carla.side_effect = failed_vissim_tick
    assert _run_until_stopped(loop) == 3
    assert vissim.tick_count == 0


def check_sensor_loop_without_vissim_never_stops(ca):
    loop = ca.SensorLoop()
    loop.running = True
    loop.sensor = mock.MagicMock()
    loop.ego_actor = mock.MagicMock()
    for iteration in range(1, 50):
        loop._tick_sensor(types.SimpleNamespace(elapsed_seconds=float(iteration)))
    assert loop.running


def check_run_bridge_sets_failure_limit(ca):
    interface = _make_interface(ca, vissim_max_consecutive_failures=7)
    interface.sensor_wrapper = mock.MagicMock()
    interface.ego_actor = mock.MagicMock()

    class _NeverRunningSensorLoop(ca.SensorLoop):
        # run_bridge() sets running = True and then loops while it is true; ignoring the
        # assignment makes run_bridge() return right after wiring up the loop.
        running = property(lambda self: False, lambda self, value: None)

    with mock.patch.object(ca, 'SensorLoop', _NeverRunningSensorLoop):
        interface.run_bridge()
    assert interface.bridge_loop.vissim_max_consecutive_failures == 7


# ==================================================================================================
# -- entry point -------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    stubs = {name: mock.MagicMock() for name in _STUBBED_MODULES}
    with mock.patch.dict(sys.modules, stubs):
        from autoware_carla_interface import carla_autoware as ca

        check_startup_validation_rejects_invalid_params(ca)
        check_startup_validation_is_noop_without_vissim(ca)
        check_sim_period_passed_to_vissim_simulation(ca)
        check_sensor_loop_stops_when_period_elapsed(ca)
        check_sensor_loop_stops_after_consecutive_failures(ca)
        check_sensor_loop_without_vissim_never_stops(ca)
        check_run_bridge_sets_failure_limit(ca)
    print('All vissim simulation period checks passed.')


if __name__ == '__main__':
    run()
