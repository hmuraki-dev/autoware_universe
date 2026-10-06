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
Stub test for the EGO spawn of carla_autoware.py's InitializeInterface (see
docs/Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md Step V3):

- load_world() initializes the Vissim integration and then spawns the EGO (with its sensors and,
  if enabled, the Traffic Manager NPCs) through _spawn_ego_and_sensors(), in that order, as it did
  before the EGO spawn was split out of it;
- _spawn_ego_and_sensors() itself: fixed or random spawn point, sensors, Traffic Manager, and a
  failed EGO spawn;
- with the Vissim warmup (Step V4): load_world() leaves the EGO spawn to run_bridge(), which runs
  the warmup and the catch-up spawn, spawns the EGO, starts the co-simulation period and only then
  the regular loop; stop requests during the warmup; the exit code of a failed test start.

Needs neither ROS 2, CARLA, nor a vissim adapter: `carla`, `zmq`, `msgpack` and the ROS-dependent
`carla_ros`/`modules.*` modules are replaced by mocks, only for the duration of run() (via
mock.patch.dict(sys.modules), so nothing leaks into other tests if this file is imported by a test
runner). Run with:
    python3 test/vissim_warmup_ego_spawn_test.py
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

# Parameters as returned by carla_ros2_interface.get_param() (Vissim disabled).
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
    'use_vissim': False,
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
    # Fresh mocks per interface, so that call assertions only see this check's calls.
    ca.CarlaDataProvider.reset_mock()
    ca.SensorWrapper.reset_mock()
    ca.carla.reset_mock()
    return ca.InitializeInterface()


# ==================================================================================================
# -- checks ------------------------------------------------------------------------------------------
# ==================================================================================================


def check_load_world_spawns_ego_after_vissim_init(ca):
    interface = _make_interface(ca)
    calls = mock.MagicMock()
    with mock.patch.object(ca.time, 'sleep'), \
            mock.patch.object(interface, '_init_vissim_integration',
                              calls._init_vissim_integration), \
            mock.patch.object(interface, '_spawn_ego_and_sensors', calls._spawn_ego_and_sensors):
        interface.load_world()

    client = ca.carla.Client.return_value
    assert calls.mock_calls == [
        mock.call._init_vissim_integration(client),
        mock.call._spawn_ego_and_sensors(client),
    ], calls.mock_calls


def check_spawn_at_fixed_spawn_point(ca):
    interface = _make_interface(ca, spawn_point=_SPAWN_POINT)
    client = mock.MagicMock()
    with mock.patch.object(interface, '_setup_traffic_manager') as setup_traffic_manager:
        interface._spawn_ego_and_sensors(client)

    request = ca.CarlaDataProvider.request_new_actor
    assert request.call_count == 1, request.mock_calls
    args, kwargs = request.call_args
    assert args[0] == 'vehicle.toyota.prius' and args[2] == 'ego_vehicle', args
    assert kwargs == {'random_location': False}, kwargs
    spawn_transform = args[1]
    assert spawn_transform is ca.carla.Transform.return_value

    ego = request.return_value
    assert interface.ego_actor is ego
    assert interface.interface.ego_actor is ego
    assert interface.interface.physics_control is ego.get_physics_control.return_value

    ca.SensorWrapper.assert_called_once_with(interface.interface)
    assert interface.sensor_wrapper is ca.SensorWrapper.return_value
    interface.sensor_wrapper.setup_sensors.assert_called_once_with(ego, False)

    setup_traffic_manager.assert_not_called()


def check_spawn_at_random_spawn_point(ca):
    interface = _make_interface(ca)  # spawn_point 'None'
    with mock.patch.object(interface, '_setup_traffic_manager'):
        interface._spawn_ego_and_sensors(mock.MagicMock())
    _, kwargs = ca.CarlaDataProvider.request_new_actor.call_args
    assert kwargs == {'random_location': True}, kwargs


def check_traffic_manager_after_sensors(ca):
    interface = _make_interface(ca, use_traffic_manager=True)
    client = mock.MagicMock()
    order = mock.MagicMock()
    ca.SensorWrapper.return_value.setup_sensors.side_effect = \
        lambda *args: order.setup_sensors()
    try:
        with mock.patch.object(interface, '_setup_traffic_manager',
                               order._setup_traffic_manager):
            interface._spawn_ego_and_sensors(client)
    finally:
        ca.SensorWrapper.return_value.setup_sensors.side_effect = None
    assert order.mock_calls == [
        mock.call.setup_sensors(),
        mock.call._setup_traffic_manager(client),
    ], order.mock_calls


def check_ego_spawn_failure_raises(ca):
    interface = _make_interface(ca, spawn_point=_SPAWN_POINT)
    ca.CarlaDataProvider.request_new_actor.return_value = None
    try:
        interface._spawn_ego_and_sensors(mock.MagicMock())
    except ca.EgoSpawnGateError as e:
        assert 'failed to spawn the EGO vehicle' in str(e), e
    else:
        raise AssertionError('expected EgoSpawnGateError')
    finally:
        ca.CarlaDataProvider.request_new_actor.return_value = mock.DEFAULT
    ca.SensorWrapper.assert_not_called()


# -- Vissim warmup (Step V4) --------------------------------------------------------------------------

_WARMUP = {'use_vissim': True, 'vissim_warmup_time': 100, 'spawn_point': _SPAWN_POINT}

_GATE_CLASS = 'autoware_carla_interface.vissim_integration.ego_spawn_gate.EgoSpawnGate'


class _FakeLocation(object):
    x, y, z = 229.8, -2.0, 0.5


def _make_warmup_interface(ca, **overrides):
    """An interface past load_world() with the warmup enabled (vissim objects mocked)."""
    interface = _make_interface(ca, **dict(_WARMUP, **overrides))
    interface._client = mock.MagicMock(name='client')
    interface.vissim_sync = mock.MagicMock(name='vissim_sync')
    interface.vissim_sim = mock.MagicMock(name='vissim_sim')
    interface.vissim_sim.vehicle_ids = {1, 2, 3}
    interface.vissim_sim.end_tick = 3200
    return interface


def _never_running_sensor_loop(ca):
    class _NeverRunningSensorLoop(ca.SensorLoop):
        # run_bridge() sets running = True and then loops while it is true; ignoring the
        # assignment makes run_bridge() return right after wiring up the loop.
        running = property(lambda self: False, lambda self, value: None)
    return _NeverRunningSensorLoop


def check_load_world_defers_ego_with_warmup(ca):
    interface = _make_interface(ca, **_WARMUP)
    with mock.patch.object(ca.time, 'sleep'), \
            mock.patch.object(interface, '_init_vissim_integration') as init_vissim, \
            mock.patch.object(interface, '_spawn_ego_and_sensors') as spawn:
        interface.load_world()
    client = ca.carla.Client.return_value
    init_vissim.assert_called_once_with(client)
    spawn.assert_not_called()
    assert interface._client is client


def check_run_bridge_warms_up_then_spawns_ego(ca):
    interface = _make_warmup_interface(ca)
    order = mock.MagicMock()
    order.attach_mock(interface.vissim_sim.start_period, 'start_period')

    def spawn(client):
        order._spawn_ego_and_sensors(client)
        interface.ego_actor = mock.MagicMock(name='ego')
        interface.ego_actor.get_location.return_value = _FakeLocation()

    with mock.patch(_GATE_CLASS) as gate_cls, \
            mock.patch.object(interface, '_spawn_ego_and_sensors', side_effect=spawn), \
            mock.patch.object(ca, 'SensorLoop', _never_running_sensor_loop(ca)):
        gate = gate_cls.return_value
        def warmup():
            order.warmup()
            return True

        gate.warmup.side_effect = warmup
        gate.catch_up.side_effect = lambda: order.catch_up()
        gate.sim_time = 107.4
        interface.run_bridge()

    args, kwargs = gate_cls.call_args
    assert args == (interface.vissim_sync, 100, 0.05, 3), args
    assert kwargs['should_stop']() is False
    interface._stop_requested = True
    assert kwargs['should_stop']() is True

    assert order.mock_calls == [
        mock.call.warmup(),
        mock.call.catch_up(),
        mock.call._spawn_ego_and_sensors(interface._client),
        mock.call.start_period(),
    ], order.mock_calls
    # The regular loop then starts with the EGO spawned after the warmup.
    assert interface.bridge_loop is not None
    assert interface.bridge_loop.ego_actor is interface.ego_actor
    assert interface.bridge_loop.vissim_sync is interface.vissim_sync


def check_run_bridge_stopped_during_warmup(ca):
    interface = _make_warmup_interface(ca)
    with mock.patch(_GATE_CLASS) as gate_cls, \
            mock.patch.object(interface, '_spawn_ego_and_sensors') as spawn:
        gate_cls.return_value.warmup.return_value = False
        interface.run_bridge()
    gate_cls.return_value.catch_up.assert_not_called()
    spawn.assert_not_called()
    interface.vissim_sim.start_period.assert_not_called()
    assert interface.bridge_loop is None


def check_run_bridge_stopped_during_catch_up(ca):
    interface = _make_warmup_interface(ca)

    def catch_up():
        interface._stop_loop(None, None)

    with mock.patch(_GATE_CLASS) as gate_cls, \
            mock.patch.object(interface, '_spawn_ego_and_sensors') as spawn:
        gate_cls.return_value.warmup.return_value = True
        gate_cls.return_value.catch_up.side_effect = catch_up
        interface.run_bridge()
    spawn.assert_not_called()
    assert interface.bridge_loop is None


def check_run_bridge_without_warmup_skips_gate(ca):
    interface = _make_interface(ca, use_vissim=True)  # warmup disabled
    interface.vissim_sync = mock.MagicMock()
    with mock.patch(_GATE_CLASS) as gate_cls, \
            mock.patch.object(ca, 'SensorLoop', _never_running_sensor_loop(ca)):
        interface.run_bridge()
    gate_cls.assert_not_called()
    assert interface.bridge_loop is not None


def check_stop_loop_before_bridge_loop(ca):
    interface = _make_interface(ca)
    assert interface.bridge_loop is None and interface._stop_requested is False
    interface._stop_loop(None, None)  # e.g. SIGINT during the warmup: must not raise
    assert interface._stop_requested is True


def check_main_exits_non_zero_when_test_start_fails(ca):
    bridge = mock.MagicMock()
    bridge.run_bridge.side_effect = ca.EgoSpawnGateError('no safe gap')
    with mock.patch.object(ca, 'InitializeInterface', return_value=bridge), \
            mock.patch.object(ca.signal, 'signal'), \
            mock.patch.object(ca.sys.stdout, 'reconfigure', create=True):
        try:
            ca.main()
        except SystemExit as e:
            assert e.code == 1, e.code
        else:
            raise AssertionError('expected SystemExit(1)')
    bridge._cleanup.assert_called_once_with()


# ==================================================================================================
# -- entry point -------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    stubs = {name: mock.MagicMock() for name in _STUBBED_MODULES}
    with mock.patch.dict(sys.modules, stubs):
        from autoware_carla_interface import carla_autoware as ca

        check_load_world_spawns_ego_after_vissim_init(ca)
        check_spawn_at_fixed_spawn_point(ca)
        check_spawn_at_random_spawn_point(ca)
        check_traffic_manager_after_sensors(ca)
        check_ego_spawn_failure_raises(ca)
        check_load_world_defers_ego_with_warmup(ca)
        check_run_bridge_warms_up_then_spawns_ego(ca)
        check_run_bridge_stopped_during_warmup(ca)
        check_run_bridge_stopped_during_catch_up(ca)
        check_run_bridge_without_warmup_skips_gate(ca)
        check_stop_loop_before_bridge_loop(ca)
        check_main_exits_non_zero_when_test_start_fails(ca)
    print('All EGO spawn checks passed.')


if __name__ == '__main__':
    run()
