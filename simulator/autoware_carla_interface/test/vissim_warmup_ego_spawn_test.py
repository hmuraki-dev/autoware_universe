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
- _spawn_ego_and_sensors() itself: fixed or random spawn point, sensors, Traffic Manager.

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
    print('All EGO spawn checks passed.')


if __name__ == '__main__':
    run()
