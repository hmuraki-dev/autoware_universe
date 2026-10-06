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
  failed vissim adapter ticks - and never stopping on its own without Vissim;
- the period extension for the Vissim warmup / EGO safe spawn (see
  docs/Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md Step V2):
  get_vissim_sim_params()'s warmup_time/wait_timeout, their startup validation and propagation to
  PTVVissimSimulation, and end_tick/start_period().

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
    'vissim_warmup_time': 0,
    'ego_spawn_front_margin': 20.0,
    'ego_spawn_rear_margin': 20.0,
    'ego_spawn_wait_timeout': 60,
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


def check_sim_params_with_warmup():
    # docs/Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md Step V2: the
    # warmup time and the spawn wait timeout extend the period written into the network file.
    from autoware_carla_interface.vissim_integration.vissim_simulation import \
        get_vissim_sim_params

    # Without them (the upstream call signature), the result is unchanged: period + 10 s margin.
    assert get_vissim_sim_params(0.05, 600) == (610, 20)
    assert get_vissim_sim_params(0.05, 600, 0, 0) == (610, 20)
    assert get_vissim_sim_params(0.05, 600, 100, 60) == (770, 20)
    assert get_vissim_sim_params(0.1, 600, warmup_time=300) == (910, 10)
    # The total may reach Vissim's 2678400 s maximum, but not exceed it.
    assert get_vissim_sim_params(0.05, 2678300, 60, 30) == (2678400, 20)

    for args in [
            (0.05, 600, -1, 0),
            (0.05, 600, 0, -1),
            (0.05, 600, 1.5, 0),  # not an int
            (0.05, 600, True, 0),  # bool is not accepted as an int
            (0.05, 600, 0, None),
            (0.05, 2678300, 60, 31),  # total exceeds the maximum
    ]:
        try:
            get_vissim_sim_params(*args)
        except ValueError:
            continue
        raise AssertionError('expected ValueError for %r' % (args, ))


def check_startup_validation_with_warmup(ca):
    warmup = {'vissim_warmup_time': 100, 'spawn_point': '229.8,-2.0,0.3,0.0,0.0,180.0'}
    _make_interface(ca, **warmup)  # 100 + 60 + 600 + 10 s: valid
    # The total period exceeds Vissim's maximum only because of the warmup and the wait timeout.
    try:
        _make_interface(ca, vissim_sim_period=2678300, ego_spawn_wait_timeout=31,
                        **dict(warmup, vissim_warmup_time=60))
    except ValueError:
        pass
    else:
        raise AssertionError('expected ValueError for a total period above the maximum')
    # Without the warmup, the wait timeout does not extend the period: the same values are valid.
    _make_interface(ca, vissim_sim_period=2678300, ego_spawn_wait_timeout=1000)


def check_warmup_passed_to_vissim_simulation(ca):
    package = 'autoware_carla_interface.vissim_integration'

    def vissim_args_for(**overrides):
        interface = _make_interface(ca, **overrides)
        with mock.patch(package + '.vissim_simulation.PTVVissimSimulation') as vissim_cls, \
                mock.patch(package + '.carla_simulation.CarlaSimulation'), \
                mock.patch(package + '.simulation_synchronization.SimulationSynchronization'):
            interface._init_vissim_integration(mock.MagicMock())
        return vissim_cls.call_args[0][0]

    vissim_args = vissim_args_for(vissim_warmup_time=100, ego_spawn_wait_timeout=45,
                                  spawn_point='229.8,-2.0,0.3,0.0,0.0,180.0')
    assert (vissim_args.warmup_time, vissim_args.wait_timeout) == (100, 45), vars(vissim_args)
    # Warmup disabled: neither extends the period, whatever ego_spawn_wait_timeout holds.
    vissim_args = vissim_args_for(vissim_warmup_time=0, ego_spawn_wait_timeout=45)
    assert (vissim_args.warmup_time, vissim_args.wait_timeout) == (0, 0), vars(vissim_args)


def check_end_tick_and_start_period():
    from autoware_carla_interface.vissim_integration.vissim_simulation import \
        PTVVissimSimulation

    # Bypasses __init__ (which connects to the vissim adapter): only the period bookkeeping it
    # sets up is needed here - 60 s at 20 ticks per second.
    vissim = PTVVissimSimulation.__new__(PTVVissimSimulation)
    vissim._period_ticks = 60 * 20
    vissim._period_start_tick = 0
    vissim._tick_count = 0

    # Without start_period() (no warmup), the period starts at tick 0, as before.
    assert vissim.end_tick == 1200
    vissim._tick_count = 500
    assert vissim.end_tick == 1200

    # After a warmup, the period starts when start_period() is called.
    vissim._tick_count = 2345
    vissim.start_period()
    assert vissim.end_tick == 2345 + 1200
    vissim._tick_count = 3000
    assert vissim.end_tick == 3545


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
        check_sim_params_with_warmup()
        check_startup_validation_with_warmup(ca)
        check_warmup_passed_to_vissim_simulation(ca)
        check_end_tick_and_start_period()
        check_sensor_loop_stops_when_period_elapsed(ca)
        check_sensor_loop_stops_after_consecutive_failures(ca)
        check_sensor_loop_without_vissim_never_stops(ca)
        check_run_bridge_sets_failure_limit(ca)
    print('All vissim simulation period checks passed.')


if __name__ == '__main__':
    run()
