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
- PTVVissimSimulation.vehicle_ids / pedestrian_ids;
- the safe-gap check evaluate_spawn_gap() / center_from_front() (Step V5): clearances against the
  margins, same-lane / heading / range filtering, overlap of the outlines, vehicles without a
  CARLA counterpart, rotated and oblique headings.

Needs neither ROS 2, CARLA, nor a vissim adapter: `carla`, `zmq` and `msgpack` are replaced by
mocks, only for the duration of run() (via mock.patch.dict(sys.modules), so nothing leaks into
other tests if this file is imported by a test runner). Run with:
    python3 test/vissim_ego_spawn_gate_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

import math
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
        id=i, type='v%d' % i, get_transform=lambda: 'vissim-tf-%d' % i,
        get_velocity=lambda: 'vissim-vel-%d' % i)
    sync.vissim.get_pedestrian.side_effect = lambda i: types.SimpleNamespace(
        id=i, type='p%d' % i, get_velocity=lambda: 'vissim-ped-vel-%d' % i)
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

    def carla_transform(vissim_transform, extent=None):
        # Tags the result with whether it was corrected with the CARLA bounding box (center) or
        # not (vissim's front-center-bumper position, as used for spawning).
        return (vissim_transform, 'center' if extent is not None else 'front', extent)

    def carla_actor(carla_actor_id):
        return types.SimpleNamespace(
            bounding_box=types.SimpleNamespace(extent='extent-%d' % carla_actor_id))

    with mock.patch.object(ss.BridgeHelper, 'get_carla_blueprint', side_effect=blueprint_for), \
            mock.patch.object(ss.BridgeHelper, 'get_carla_transform',
                              side_effect=carla_transform), \
            mock.patch.object(ss.BridgeHelper, 'get_carla_velocity',
                              side_effect=lambda v: 'carla-' + v), \
            mock.patch.object(ss.BridgeHelper, 'get_carla_pedestrian_blueprint',
                              return_value='walker'), \
            mock.patch.object(ss.BridgeHelper, 'get_carla_pedestrian_transform',
                              side_effect=lambda p: 'ped-tf-%d' % p.id):
        sync.carla.spawn_actor.side_effect = lambda bp, tf: (
            1101 if bp == 'walker' else spawn(bp, tf))
        sync.carla.get_actor.side_effect = carla_actor
        result = sync.spawn_all_vissim_actors_in_carla()

    assert sync.vissim2carla_ids == {1: 501, 2: 1002, 3: 1003}, sync.vissim2carla_ids
    assert sync.vissim2carla_ped_ids == {100: 700, 101: 1101}, sync.vissim2carla_ped_ids
    assert result == {'vehicles': 2, 'vehicles_not_spawned': {4, 5}, 'pedestrians': 1,
                      'pedestrians_not_spawned': set()}, result

    # Vehicles are spawned at vissim's front-center-bumper position...
    spawned_at = {tf[0]: tf[1] for _, tf in
                  (c.args for c in sync.carla.spawn_actor.call_args_list) if tf != 'ped-tf-101'}
    assert spawned_at == {'vissim-tf-2': 'front', 'vissim-tf-3': 'front',
                          'vissim-tf-5': 'front'}, spawned_at
    # ...then, before CARLA is ticked, moved to their center position using their own bounding
    # box - only those actually spawned (not 1: already mirrored, not 4/5: not spawned).
    synchronized = sorted(c.args for c in sync.carla.synchronize_vehicle.call_args_list)
    assert synchronized == [
        (1002, ('vissim-tf-2', 'center', 'extent-1002'), 'carla-vissim-vel-2'),
        (1003, ('vissim-tf-3', 'center', 'extent-1003'), 'carla-vissim-vel-3'),
    ], synchronized
    # Pedestrians likewise (spawned lifted by CARLA_SPAWN_OFFSET_Z, then moved to the ground).
    sync.carla.synchronize_pedestrian.assert_called_once_with(1101, 'ped-tf-101',
                                                             'carla-vissim-ped-vel-101')
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
# -- checks: evaluate_spawn_gap() (Step V5) -----------------------------------------------------------
# ==================================================================================================

# EGO heading +x at the origin; 4 m x 2 m, so that clearances come out exact. Lane width 3.5 m.
_EGO_LENGTH, _EGO_WIDTH, _LANE_WIDTH = 4.0, 2.0, 3.5


def _ego(egsg, x=0.0, y=0.0, yaw=0.0):
    return egsg.EgoSpawnPose(x, y, yaw, _EGO_LENGTH, _EGO_WIDTH, _LANE_WIDTH)


def _car(egsg, label, x, y, yaw=0.0, length=4.0, width=2.0, source='carla'):
    return egsg.GapVehicle(label, x, y, yaw, length, width, source)


def _gap(egsg, vehicles, ego=None, front_margin=20.0, rear_margin=20.0, **kwargs):
    return egsg.evaluate_spawn_gap(ego or _ego(egsg), vehicles, front_margin, rear_margin,
                                   **kwargs)


def check_gap_no_vehicles(egsg):
    result = _gap(egsg, [])
    assert result.safe and result.front is None and result.rear is None, result
    assert result.overlapping == []
    assert result.describe() == 'front=none rear=none overlap=none result=SAFE', result.describe()


def check_gap_front_clearance(egsg):
    # Front clearance = s - 4/2 - 4/2 = s - 4: below / exactly / above the 20 m margin.
    for s, safe in ((23.9, False), (24.0, True), (30.0, True)):
        result = _gap(egsg, [_car(egsg, 'f', s, 0.0)])
        assert result.safe is safe, (s, result)
        assert result.rear is None
        assert result.front.vehicle.label == 'f'
        assert abs(result.front.clearance - (s - 4.0)) < 1e-9, result.front
    result = _gap(egsg, [_car(egsg, 'vissim:12/carla:40', 12.4, 0.0)])
    assert result.describe() == ('front=vissim:12/carla:40 clearance=8.4 m rear=none '
                                 'overlap=none result=WAIT'), result.describe()


def check_gap_rear_clearance(egsg):
    for s, safe in ((-23.9, False), (-24.0, True), (-30.0, True)):
        result = _gap(egsg, [_car(egsg, 'r', s, 0.0)])
        assert result.safe is safe, (s, result)
        assert result.front is None
        assert abs(result.rear.clearance - (-s - 4.0)) < 1e-9, result.rear


def check_gap_margins_apply_per_side(egsg):
    vehicles = [_car(egsg, 'f', 14.0, 0.0), _car(egsg, 'r', -34.0, 0.0)]  # 10 m / 30 m gaps
    assert not _gap(egsg, vehicles).safe
    assert _gap(egsg, vehicles, front_margin=10.0, rear_margin=30.0).safe
    assert not _gap(egsg, vehicles, front_margin=10.0, rear_margin=30.1).safe


def check_gap_uses_vehicle_lengths(egsg):
    # A 12 m bus whose center is 30 m ahead: gap = 30 - 2 - 6 = 22 m.
    result = _gap(egsg, [_car(egsg, 'bus', 30.0, 0.0, length=12.0, width=2.55)])
    assert abs(result.front.clearance - 22.0) < 1e-9 and result.safe, result


def check_gap_picks_nearest_vehicle_per_side(egsg):
    vehicles = [_car(egsg, 'far', 60.0, 0.0), _car(egsg, 'near', 30.0, 0.0),
                _car(egsg, 'rear-far', -80.0, 0.0), _car(egsg, 'rear-near', -26.0, 0.0)]
    result = _gap(egsg, vehicles)
    assert result.front.vehicle.label == 'near' and result.rear.vehicle.label == 'rear-near'
    assert result.safe, result


def check_gap_ignores_adjacent_lane(egsg):
    # 3.5 m to the side (the next lane), right next to the EGO: neither front/rear nor overlapping.
    for d in (3.5, -3.5, 1.75):
        result = _gap(egsg, [_car(egsg, 'side', 6.0, d)])
        assert result.front is None and result.overlapping == [], (d, result)
        assert result.safe, (d, result)
    # Slightly off-center in the EGO's own lane still counts.
    result = _gap(egsg, [_car(egsg, 'own', 10.0, 1.7)])
    assert result.front is not None and not result.safe, result


def check_gap_ignores_opposite_and_crossing_traffic(egsg):
    # Opposite direction in the lane next door, and a crossing vehicle far enough not to overlap.
    for yaw in (180.0, -135.0, 90.0, 46.0):
        result = _gap(egsg, [_car(egsg, 'other', 10.0, 0.0, yaw=yaw)])
        assert result.front is None, (yaw, result)
    # Within the 45 degree tolerance (e.g. a slight curve), it counts.
    result = _gap(egsg, [_car(egsg, 'same', 10.0, 0.0, yaw=44.0)])
    assert result.front is not None and not result.safe, result
    result = _gap(egsg, [_car(egsg, 'same', 10.0, 0.0, yaw=-359.0)])  # same heading, wrapped
    assert result.front is not None, result


def check_gap_overlap_rejects_whatever_the_lane(egsg):
    # A vehicle in the next lane sticking out into the EGO's (|d| = 2.0 >= 1.75: not front/rear),
    # 2.2 m wide: its outline spans d = 0.9 .. 3.1, the EGO's d = -1 .. 1.
    sticking_out = _car(egsg, 'wide', 1.0, 2.0, width=2.2)
    result = _gap(egsg, [sticking_out])
    assert result.front is None and result.rear is None, result
    assert result.overlapping == [sticking_out] and not result.safe, result
    assert result.describe() == 'front=none rear=none overlap=wide result=WAIT', result.describe()
    # Crossing traffic over the EGO's spawn point (e.g. in an intersection) as well.
    crossing = _car(egsg, 'crossing', 0.5, 0.0, yaw=90.0)
    assert _gap(egsg, [crossing]).overlapping == [crossing]
    # A diagonal vehicle near the EGO's front-right corner, separated from it only along its own
    # axes (its projections on the EGO's axes do overlap): not overlapping.
    diagonal = _car(egsg, 'diagonal', 3.35, 2.5, yaw=45.0)
    assert _gap(egsg, [diagonal]).overlapping == [], diagonal
    # A vehicle exactly touching the EGO's outline does not overlap (but leaves no gap).
    touching = _car(egsg, 'touching', 4.0, 0.0)
    result = _gap(egsg, [touching])
    assert result.overlapping == [] and result.front.clearance == 0.0 and not result.safe


def check_gap_search_range(egsg):
    assert _gap(egsg, [_car(egsg, 'f', 100.5, 0.0)]).front is None
    assert _gap(egsg, [_car(egsg, 'f', 100.0, 0.0)]).front is not None  # range is inclusive
    assert _gap(egsg, [_car(egsg, 'r', -100.5, 0.0)]).rear is None
    # The range is configurable (e.g. for tests).
    assert _gap(egsg, [_car(egsg, 'f', 40.0, 0.0)], search_range=30.0).front is None


def check_gap_in_rotated_frame(egsg):
    # EGO heading +y (yaw 90, south in CARLA's frame) at (100, 50).
    ego = _ego(egsg, x=100.0, y=50.0, yaw=90.0)
    ahead = _car(egsg, 'ahead', 100.0, 70.0, yaw=90.0)  # 20 m further along +y
    behind = _car(egsg, 'behind', 100.0, 20.0, yaw=90.0)  # 30 m back
    beside = _car(egsg, 'beside', 103.5, 50.0, yaw=90.0)  # next lane
    result = _gap(egsg, [ahead, behind, beside], ego=ego)
    assert result.front.vehicle is ahead and abs(result.front.clearance - 16.0) < 1e-9, result
    assert result.rear.vehicle is behind and abs(result.rear.clearance - 26.0) < 1e-9, result
    assert result.overlapping == [] and not result.safe  # 16 m < 20 m ahead


def check_gap_in_oblique_frame(egsg):
    # EGO heading 30 degrees: positions built here from the heading independently of the module.
    yaw = 30.0
    fx, fy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))  # forward
    rx, ry = -fy, fx  # right, in CARLA's left-handed frame
    ego = _ego(egsg, x=10.0, y=-5.0, yaw=yaw)

    def at(s, d):
        return 10.0 + fx * s + rx * d, -5.0 + fy * s + ry * d

    ahead = _car(egsg, 'ahead', *at(20.0, 0.0), yaw=yaw)
    beside = _car(egsg, 'beside', *at(3.0, 3.5), yaw=yaw)  # next lane, right next to the EGO
    behind = _car(egsg, 'behind', *at(-30.0, 0.5), yaw=yaw)
    result = _gap(egsg, [ahead, beside, behind], ego=ego)
    assert result.front.vehicle is ahead and abs(result.front.clearance - 16.0) < 1e-9, result
    assert result.rear.vehicle is behind and abs(result.rear.clearance - 26.0) < 1e-9, result
    assert abs(result.rear.d - 0.5) < 1e-9, result.rear
    assert result.overlapping == [], result
    # Moved half a lane closer, the vehicle beside sticks into the EGO's outline.
    closer = _car(egsg, 'closer', *at(0.0, 1.8), yaw=yaw)
    assert _gap(egsg, [closer], ego=ego).overlapping == [closer]


def check_vissim_only_vehicle(egsg):
    # No CARLA counterpart: center derived from vissim's front bumper with the assumed length.
    length = egsg.EGO_SPAWN_UNKNOWN_VEHICLE_LENGTH_M
    assert length == 12.2 and egsg.EGO_SPAWN_UNKNOWN_VEHICLE_WIDTH_M == 2.6
    x, y = egsg.center_from_front(30.0, 0.0, 0.0, length)
    assert abs(x - 23.9) < 1e-9 and abs(y) < 1e-9, (x, y)
    x, y = egsg.center_from_front(0.0, 30.0, 90.0, length)  # heading +y
    assert abs(x) < 1e-9 and abs(y - 23.9) < 1e-9, (x, y)

    vissim_only = egsg.GapVehicle('vissim:77', 23.9, 0.0, 0.0, length,
                                  egsg.EGO_SPAWN_UNKNOWN_VEHICLE_WIDTH_M, 'vissim_only')
    result = _gap(egsg, [vissim_only])
    # 23.9 - 2 - 6.1 = 15.8 m: too close, although its front bumper is 30 m ahead.
    assert abs(result.front.clearance - 15.8) < 1e-9 and not result.safe, result


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
        check_gap_no_vehicles(egsg)
        check_gap_front_clearance(egsg)
        check_gap_rear_clearance(egsg)
        check_gap_margins_apply_per_side(egsg)
        check_gap_uses_vehicle_lengths(egsg)
        check_gap_picks_nearest_vehicle_per_side(egsg)
        check_gap_ignores_adjacent_lane(egsg)
        check_gap_ignores_opposite_and_crossing_traffic(egsg)
        check_gap_overlap_rejects_whatever_the_lane(egsg)
        check_gap_search_range(egsg)
        check_gap_in_rotated_frame(egsg)
        check_gap_in_oblique_frame(egsg)
        check_vissim_only_vehicle(egsg)
    print('All EGO spawn gate checks passed.')


if __name__ == '__main__':
    run()
