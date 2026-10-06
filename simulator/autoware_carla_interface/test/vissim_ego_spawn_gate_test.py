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
Stub test for the Vissim warmup / catch-up spawn (see
docs/Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md Step V4):

- vissim_integration/ego_spawn_gate.py's EgoSpawnGate: warmup tick count, progress lines, stop
  request, consecutive tick failures, catch-up;
- SimulationSynchronization.spawn_all_vissim_actors_in_carla(): which vissim vehicles/pedestrians
  get spawned in CARLA, and the bookkeeping of those that cannot be;
- PTVVissimSimulation.vehicle_ids / pedestrian_ids.

Needs neither ROS 2, CARLA, nor a vissim adapter: `carla`, `zmq` and `msgpack` are replaced by
mocks, only for the duration of run() (via mock.patch.dict(sys.modules), so nothing leaks into
other tests if this file is imported by a test runner). Run with:
    python3 test/vissim_ego_spawn_gate_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

import os
import sys
import types
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

_STUBBED_MODULES = ['carla', 'zmq', 'msgpack']

# ==================================================================================================
# -- fakes -------------------------------------------------------------------------------------------
# ==================================================================================================


class FakeVissim(object):
    """Stands in for PTVVissimSimulation: tick() succeeds unless a failure is scripted."""

    def __init__(self, failing_ticks=(), vehicles_per_tick=0.5):
        self.tick_count = 0
        self.consecutive_failures = 0
        self.tick_calls = 0
        self._failing_ticks = set(failing_ticks)  # tick_calls (1-based) that fail
        self._vehicles_per_tick = vehicles_per_tick

    def tick(self):
        self.tick_calls += 1
        if self.tick_calls in self._failing_ticks:
            self.consecutive_failures += 1
        else:
            self.tick_count += 1
            self.consecutive_failures = 0

    @property
    def vehicle_ids(self):
        return set(range(int(self.tick_count * self._vehicles_per_tick)))

    @property
    def pedestrian_ids(self):
        return set()


def _make_gate(egsg, vissim, warmup_time=100, step_length=0.05, max_failures=3,
               should_stop=lambda: False):
    sync = mock.MagicMock()
    sync.vissim = vissim
    log = []
    gate = egsg.EgoSpawnGate(sync, warmup_time, step_length, max_failures, should_stop,
                             log=log.append)
    return gate, sync, log


# ==================================================================================================
# -- checks: EgoSpawnGate ------------------------------------------------------------------------------
# ==================================================================================================


def check_warmup_ticks_vissim_alone(egsg):
    vissim = FakeVissim()
    gate, sync, log = _make_gate(egsg, vissim, warmup_time=100, step_length=0.05)
    assert gate.warmup() is True
    assert vissim.tick_count == 2000 and vissim.tick_calls == 2000
    assert gate.sim_time == 100.0
    # CARLA is not touched during the warmup.
    assert sync.carla.mock_calls == [], sync.carla.mock_calls
    assert log[0] == '[VISSIM WARMUP] start: warmup_time=100 s (2000 ticks)', log[0]
    progress = [line for line in log if line.startswith('[VISSIM WARMUP] t=')]
    # Every 10 simulated seconds, except at the end (reported as 'completed' instead).
    assert [line.split()[2] for line in progress] == ['t=%.1f' % t for t in range(10, 100, 10)], \
        progress
    assert 'vehicles=100 ' in progress[0], progress[0]  # 200 ticks * 0.5 vehicles per tick
    assert log[-1].startswith('[VISSIM WARMUP] completed: t=100.0 s vehicles=1000 '), log[-1]


def check_warmup_with_other_resolution(egsg):
    vissim = FakeVissim()
    gate, _, _ = _make_gate(egsg, vissim, warmup_time=30, step_length=0.1)
    assert gate.warmup() is True
    assert vissim.tick_count == 300


def check_warmup_counts_from_current_tick(egsg):
    # The warmup lasts warmup_time from wherever vissim is, not up to an absolute tick.
    vissim = FakeVissim()
    vissim.tick_count = 7
    gate, _, _ = _make_gate(egsg, vissim, warmup_time=1, step_length=0.05)
    assert gate.warmup() is True
    assert vissim.tick_count == 27


def check_warmup_retries_failed_ticks(egsg):
    # Isolated failures (below the limit) are retried: the warmup still covers warmup_time.
    vissim = FakeVissim(failing_ticks={5, 6, 50})
    gate, _, _ = _make_gate(egsg, vissim, warmup_time=10, step_length=0.05, max_failures=3)
    assert gate.warmup() is True
    assert vissim.tick_count == 200 and vissim.tick_calls == 203


def check_warmup_gives_up_after_consecutive_failures(egsg):
    vissim = FakeVissim(failing_ticks={11, 12, 13})
    gate, _, _ = _make_gate(egsg, vissim, warmup_time=10, step_length=0.05, max_failures=3)
    try:
        gate.warmup()
    except egsg.EgoSpawnGateError as e:
        assert '3 consecutive failed' in str(e), e
    else:
        raise AssertionError('expected EgoSpawnGateError')
    assert vissim.tick_count == 10


def check_warmup_stops_on_request(egsg):
    vissim = FakeVissim()
    gate, _, log = _make_gate(egsg, vissim, warmup_time=100,
                              should_stop=lambda: vissim.tick_count >= 42)
    assert gate.warmup() is False
    assert vissim.tick_count == 42
    assert log[-1] == '[VISSIM WARMUP] stopped at t=2.1 s', log[-1]


def check_catch_up_spawns_then_ticks_carla(egsg):
    vissim = FakeVissim()
    gate, sync, log = _make_gate(egsg, vissim)
    sync.spawn_all_vissim_actors_in_carla.return_value = {
        'vehicles': 83, 'vehicles_not_spawned': {7, 3}, 'pedestrians': 4,
        'pedestrians_not_spawned': set()}

    order = mock.MagicMock()
    order.attach_mock(sync.spawn_all_vissim_actors_in_carla, 'spawn_all')
    order.attach_mock(sync.carla.world.tick, 'world_tick')
    order.attach_mock(sync.carla.update_actor_diff, 'update_actor_diff')
    result = gate.catch_up()

    assert result is sync.spawn_all_vissim_actors_in_carla.return_value
    assert order.mock_calls == [mock.call.spawn_all(), mock.call.world_tick(),
                                mock.call.update_actor_diff()], order.mock_calls
    assert log[0] == ('[VISSIM WARMUP] caught up: carla_spawned=83 vissim_only=2 pedestrians=4 '
                      'pedestrians_vissim_only=0'), log[0]
    assert log[1] == '[VISSIM WARMUP] vissim vehicle(s) without a CARLA counterpart: [3, 7]', log


# ==================================================================================================
# -- checks: spawn_all_vissim_actors_in_carla() --------------------------------------------------------
# ==================================================================================================


def _make_sync(ss, vehicle_ids, pedestrian_ids):
    sync = ss.SimulationSynchronization.__new__(ss.SimulationSynchronization)
    sync.vissim = mock.MagicMock()
    sync.vissim.vehicle_ids = set(vehicle_ids)
    sync.vissim.pedestrian_ids = set(pedestrian_ids)
    sync.vissim.get_actor.side_effect = lambda i: types.SimpleNamespace(
        id=i, type='v%d' % i, get_transform=mock.MagicMock())
    sync.vissim.get_pedestrian.side_effect = lambda i: types.SimpleNamespace(id=i, type='p%d' % i)
    sync.carla = mock.MagicMock()
    sync.vissim2carla_ids = {}
    sync.carla2vissim_ids = {}
    sync.vissim2carla_ped_ids = {}
    return sync


def check_spawn_all_vissim_actors(ss):
    sync = _make_sync(ss, vehicle_ids={1, 2, 3, 4, 5, 6}, pedestrian_ids={100, 101})
    sync.vissim2carla_ids = {1: 501}  # already mirrored
    sync.carla2vissim_ids = {900: 6}  # carla-origin (EGO) in vissim: never mirrored back
    sync.vissim2carla_ped_ids = {100: 700}

    def blueprint_for(vissim_actor):
        return None if vissim_actor.id == 4 else 'bp-%d' % vissim_actor.id  # 4: unknown type

    def spawn(blueprint, transform):
        return ss.INVALID_ACTOR_ID if blueprint == 'bp-5' else 1000 + int(blueprint[3:])

    with mock.patch.object(ss.BridgeHelper, 'get_carla_blueprint', side_effect=blueprint_for), \
            mock.patch.object(ss.BridgeHelper, 'get_carla_transform'), \
            mock.patch.object(ss.BridgeHelper, 'get_carla_pedestrian_blueprint',
                              return_value='walker'), \
            mock.patch.object(ss.BridgeHelper, 'get_carla_pedestrian_transform'):
        sync.carla.spawn_actor.side_effect = lambda bp, tf: (
            1101 if bp == 'walker' else spawn(bp, tf))
        result = sync.spawn_all_vissim_actors_in_carla()

    assert sync.vissim2carla_ids == {1: 501, 2: 1002, 3: 1003}, sync.vissim2carla_ids
    assert sync.vissim2carla_ped_ids == {100: 700, 101: 1101}, sync.vissim2carla_ped_ids
    assert result == {'vehicles': 2, 'vehicles_not_spawned': {4, 5}, 'pedestrians': 1,
                      'pedestrians_not_spawned': set()}, result
    # Does not tick CARLA itself (the gate does, after it).
    sync.carla.tick.assert_not_called()
    sync.carla.update_actor_diff.assert_not_called()


def check_spawn_all_with_nothing_new(ss):
    sync = _make_sync(ss, vehicle_ids={1}, pedestrian_ids=set())
    sync.vissim2carla_ids = {1: 501}
    with mock.patch.object(ss.BridgeHelper, 'get_carla_blueprint') as get_blueprint:
        result = sync.spawn_all_vissim_actors_in_carla()
    get_blueprint.assert_not_called()
    sync.carla.spawn_actor.assert_not_called()
    assert result == {'vehicles': 0, 'vehicles_not_spawned': set(), 'pedestrians': 0,
                      'pedestrians_not_spawned': set()}, result


def check_vissim_actor_id_accessors(vs):
    vissim = vs.PTVVissimSimulation.__new__(vs.PTVVissimSimulation)
    vissim._vissim_vehicles = {3: object(), 9: object()}
    vissim._vissim_pedestrians = {12: object()}
    assert vissim.vehicle_ids == {3, 9}
    assert vissim.pedestrian_ids == {12}
    # Copies: changing them does not affect the simulation's own state.
    vissim.vehicle_ids.add(99)
    assert vissim.vehicle_ids == {3, 9}


# ==================================================================================================
# -- entry point -------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    stubs = {name: mock.MagicMock() for name in _STUBBED_MODULES}
    with mock.patch.dict(sys.modules, stubs):
        from autoware_carla_interface.vissim_integration import ego_spawn_gate as egsg
        from autoware_carla_interface.vissim_integration import simulation_synchronization as ss
        from autoware_carla_interface.vissim_integration import vissim_simulation as vs

        check_warmup_ticks_vissim_alone(egsg)
        check_warmup_with_other_resolution(egsg)
        check_warmup_counts_from_current_tick(egsg)
        check_warmup_retries_failed_ticks(egsg)
        check_warmup_gives_up_after_consecutive_failures(egsg)
        check_warmup_stops_on_request(egsg)
        check_catch_up_spawns_then_ticks_carla(egsg)
        check_spawn_all_vissim_actors(ss)
        check_spawn_all_with_nothing_new(ss)
        check_vissim_actor_id_accessors(vs)
    print('All EGO spawn gate checks passed.')


if __name__ == '__main__':
    run()
