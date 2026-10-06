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
Vissim warmup / EGO safe spawn, run before the regular co-simulation loop (see docs/
Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md).

Not vendored from the upstream CARLA repository (unlike the other files in this directory): it
only drives the vendored SimulationSynchronization / PTVVissimSimulation / CarlaSimulation from the
outside, before autoware_carla_interface's own main loop starts.

    WARMUP            : tick Vissim alone for warmup_time seconds (CARLA is not ticked, no pacing)
    catch-up          : spawn in CARLA every vissim actor that entered the network meanwhile
    WAIT_FOR_SAFE_GAP : check the gap around the EGO's spawn point; while not safe, run one
                        regular synchronization step (without the EGO) and check again
    (then the caller spawns the EGO and starts the co-simulation period)

The safe-gap check (evaluate_spawn_gap()) is a pure function on plain 2D poses in CARLA's world
frame (x forward/east, y right/south, yaw in degrees, clockwise from +x), so that it can be unit
tested without CARLA.

Deliberately free of carla / zmq imports, so that it can be imported (and unit tested) without
CARLA or a vissim adapter.
"""

import collections
import math
import time

# Simulated seconds between two warmup progress lines.
WARMUP_REPORT_INTERVAL_S = 10

# Safe-gap check (see the plan doc section 2.2 / 2.6). Kept here rather than in the vendored
# constants.py, since they are specific to this repository.
# Longitudinal range (m, center to center) in which vehicles ahead/behind the EGO are looked for.
EGO_SPAWN_SEARCH_RANGE_M = 100.0
# Maximum heading difference (deg) for a vehicle to count as going the EGO's way (excludes the
# opposite lane, and crossing traffic).
EGO_SPAWN_HEADING_TOLERANCE_DEG = 45.0
# Assumed size (m) of a vissim vehicle with no CARLA counterpart (unknown type, spawn failure):
# the longest vissim vehicle of Town01 (300: bus, 12.14 m) and the widest (2.55 m), rounded up.
EGO_SPAWN_UNKNOWN_VEHICLE_LENGTH_M = 12.2
EGO_SPAWN_UNKNOWN_VEHICLE_WIDTH_M = 2.6

# A vehicle for the safe-gap check, in CARLA's world frame. `x`/`y` is the center of its bounding
# box. `label` identifies it in logs (e.g. 'vissim:123/carla:456'); `source` is 'carla' (pose and
# size of its CARLA counterpart) or 'vissim_only' (no CARLA counterpart: pose derived from vissim,
# size assumed - see center_from_front()).
GapVehicle = collections.namedtuple('GapVehicle',
                                    ['label', 'x', 'y', 'yaw', 'length', 'width', 'source'])

# The EGO's planned spawn pose and size, and the width of the lane it is spawned in.
EgoSpawnPose = collections.namedtuple('EgoSpawnPose',
                                      ['x', 'y', 'yaw', 'length', 'width', 'lane_width'])

# The nearest vehicle ahead/behind in the EGO's lane and the gap (m, bumper to bumper) to it.
GapNeighbor = collections.namedtuple('GapNeighbor', ['vehicle', 'clearance', 's', 'd'])


class GapResult(collections.namedtuple('GapResult', ['safe', 'front', 'rear', 'overlapping'])):
    """
    Outcome of evaluate_spawn_gap(): `safe`, the nearest vehicles ahead/behind in the EGO's lane
    (GapNeighbor, or None when there is none in range), and the vehicles whose outline overlaps the
    EGO's planned one (list of GapVehicle, whatever their lane).
    """

    __slots__ = ()

    def describe(self):
        """One log line, e.g. 'front=vissim:12/carla:40 clearance=8.4 m rear=none ... result=WAIT'."""
        parts = []
        for name, neighbor in (('front', self.front), ('rear', self.rear)):
            if neighbor is None:
                parts.append('%s=none' % name)
            else:
                parts.append('%s=%s clearance=%.1f m' % (name, neighbor.vehicle.label,
                                                         neighbor.clearance))
        parts.append('overlap=%s' % (','.join(v.label for v in self.overlapping) or 'none'))
        parts.append('result=%s' % ('SAFE' if self.safe else 'WAIT'))
        return ' '.join(parts)


def _unit_vectors(yaw_deg):
    """(forward, right) unit vectors of a heading, in CARLA's left-handed world frame."""
    yaw = math.radians(yaw_deg)
    forward = (math.cos(yaw), math.sin(yaw))
    right = (-math.sin(yaw), math.cos(yaw))
    return forward, right


def _heading_difference(yaw_a_deg, yaw_b_deg):
    """Absolute difference between two headings, in [0, 180] degrees."""
    return abs((yaw_a_deg - yaw_b_deg + 180.0) % 360.0 - 180.0)


def _corners(x, y, yaw_deg, length, width):
    forward, right = _unit_vectors(yaw_deg)
    hl, hw = length / 2.0, width / 2.0
    return [(x + forward[0] * a + right[0] * b, y + forward[1] * a + right[1] * b)
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def _outlines_overlap(a, b):
    """
    Whether two oriented rectangles (x, y, yaw, length, width) overlap - separating axis test.
    Rectangles that only touch (zero-area contact) do not count as overlapping.
    """
    corners_a, corners_b = _corners(*a), _corners(*b)
    for yaw in (a[2], b[2]):
        for axis in _unit_vectors(yaw):
            proj_a = [px * axis[0] + py * axis[1] for px, py in corners_a]
            proj_b = [px * axis[0] + py * axis[1] for px, py in corners_b]
            if max(proj_a) <= min(proj_b) + 1e-9 or max(proj_b) <= min(proj_a) + 1e-9:
                return False
    return True


def bounding_box_center(x, y, yaw_deg, offset_x, offset_y):
    """
    The world position of a bounding box center given the actor's origin and heading, and the
    box's offset from that origin in the actor's own frame (CARLA's bounding_box.location: x
    forward, y right).
    """
    forward, right = _unit_vectors(yaw_deg)
    return (x + forward[0] * offset_x + right[0] * offset_y,
            y + forward[1] * offset_x + right[1] * offset_y)


def center_from_front(front_x, front_y, yaw_deg, length):
    """
    The bounding box center of a vehicle given its front-center-bumper position (vissim's
    reference point, already converted to CARLA's world frame) and its length.
    """
    forward, _ = _unit_vectors(yaw_deg)
    return front_x - forward[0] * length / 2.0, front_y - forward[1] * length / 2.0


def evaluate_spawn_gap(ego, vehicles, front_margin, rear_margin,
                       search_range=EGO_SPAWN_SEARCH_RANGE_M,
                       heading_tolerance_deg=EGO_SPAWN_HEADING_TOLERANCE_DEG):
    """
    Decides whether the EGO can be spawned at its planned pose (see the plan doc section 2.6).

    A vehicle is in the EGO's lane when, in the frame of the EGO's planned pose (s forward, d
    sideways), |d| < lane_width / 2, |s| <= search_range and its heading is within
    heading_tolerance_deg of the EGO's. This assumes a straight road around the spawn point. Of
    those, the vehicle ahead (s >= 0) / behind (s < 0) with the smallest bumper-to-bumper gap is the
    front / rear vehicle:

        front clearance = s - L_ego / 2 - L_front / 2
        rear clearance  = -s - L_ego / 2 - L_rear / 2

    The spawn is safe when no vehicle at all (whatever its lane) overlaps the EGO's planned
    outline, the front clearance is >= front_margin and the rear clearance >= rear_margin; a side
    with no vehicle in range counts as clear.

        :param EgoSpawnPose ego: the EGO's planned pose and size, and the lane width there.
        :param vehicles: iterable of GapVehicle (all vehicles around, in any lane).
        :return: GapResult.
    """
    forward, right = _unit_vectors(ego.yaw)
    ego_outline = (ego.x, ego.y, ego.yaw, ego.length, ego.width)
    front = rear = None
    overlapping = []

    for vehicle in vehicles:
        if _outlines_overlap(ego_outline,
                             (vehicle.x, vehicle.y, vehicle.yaw, vehicle.length, vehicle.width)):
            overlapping.append(vehicle)

        dx, dy = vehicle.x - ego.x, vehicle.y - ego.y
        s = dx * forward[0] + dy * forward[1]
        d = dx * right[0] + dy * right[1]
        if (abs(d) >= ego.lane_width / 2.0 or abs(s) > search_range or
                _heading_difference(vehicle.yaw, ego.yaw) > heading_tolerance_deg):
            continue

        clearance = abs(s) - ego.length / 2.0 - vehicle.length / 2.0
        neighbor = GapNeighbor(vehicle, clearance, s, d)
        if s >= 0:
            if front is None or clearance < front.clearance:
                front = neighbor
        elif rear is None or clearance < rear.clearance:
            rear = neighbor

    safe = (not overlapping and (front is None or front.clearance >= front_margin) and
            (rear is None or rear.clearance >= rear_margin))
    return GapResult(safe, front, rear, overlapping)


class EgoSpawnGateError(RuntimeError):
    """
    Raised when the co-simulation cannot be started (the test start failed): the vissim adapter
    failed too many ticks in a row during the warmup or the wait, no safe gap appeared within the
    wait timeout, or the EGO could not be measured / spawned.
    """


class EgoSpawnGate(object):
    """
    Runs the Vissim warmup, the catch-up spawn in CARLA that follows it, and the wait for a safe
    EGO spawn gap.

        :param sync: SimulationSynchronization (its .vissim / .carla are used as well).
        :param int warmup_time: seconds to run Vissim alone (> 0).
        :param float step_length: fixed delta seconds of the co-simulation (1/N s).
        :param int max_consecutive_failures: give up after this many failed vissim ticks in a row.
        :param should_stop: callable returning True once a stop was requested (e.g. SIGINT).
        :param log: callable taking one line of text (print by default).
    """

    def __init__(self, sync, warmup_time, step_length, max_consecutive_failures, should_stop,
                 log=print):
        self._sync = sync
        self._vissim = sync.vissim
        self._carla = sync.carla
        self._sim_res = int(round(1.0 / step_length))
        self._warmup_time = warmup_time
        self._max_consecutive_failures = max_consecutive_failures
        self._should_stop = should_stop
        self._log = log

    @property
    def sim_time(self):
        """Vissim simulation time (s), from the number of successful vissim ticks."""
        return self._vissim.tick_count / float(self._sim_res)

    def warmup(self):
        """
        Ticks Vissim alone (CARLA is not ticked, and there is no real-time pacing) for
        warmup_time simulated seconds.

            :return: True once the warmup is complete, False if a stop was requested meanwhile.
            :raises EgoSpawnGateError: if max_consecutive_failures vissim ticks fail in a row.
        """
        warmup_ticks = self._warmup_time * self._sim_res
        end_tick = self._vissim.tick_count + warmup_ticks
        report_every = WARMUP_REPORT_INTERVAL_S * self._sim_res
        start_wall = time.monotonic()
        self._log('[VISSIM WARMUP] start: warmup_time=%d s (%d ticks)' %
                  (self._warmup_time, warmup_ticks))

        while self._vissim.tick_count < end_tick:
            if self._should_stop():
                self._log('[VISSIM WARMUP] stopped at t=%.1f s' % self.sim_time)
                return False

            tick_count = self._vissim.tick_count
            self._vissim.tick()
            if self._vissim.consecutive_failures >= self._max_consecutive_failures:
                raise EgoSpawnGateError(
                    'giving up after %d consecutive failed vissim adapter tick(s) during the '
                    'warmup (t=%.1f s). The vissim adapter (and Vissim) may need to be restarted.'
                    % (self._vissim.consecutive_failures, self.sim_time))

            if self._vissim.tick_count != tick_count and \
                    self._vissim.tick_count % report_every == 0 and \
                    self._vissim.tick_count < end_tick:
                self._log('[VISSIM WARMUP] t=%.1f s vehicles=%d pedestrians=%d (wall %.1f s)' %
                          (self.sim_time, len(self._vissim.vehicle_ids),
                           len(self._vissim.pedestrian_ids), time.monotonic() - start_wall))

        self._log('[VISSIM WARMUP] completed: t=%.1f s vehicles=%d pedestrians=%d (wall %.1f s)' %
                  (self.sim_time, len(self._vissim.vehicle_ids), len(self._vissim.pedestrian_ids),
                   time.monotonic() - start_wall))
        return True

    def catch_up(self):
        """
        Spawns in CARLA every vissim actor that entered the network during the warmup, then
        ticks CARLA once and refreshes its actor diff, so that the actors just spawned are known
        as already-mirrored vissim actors (and never auto-adopted into vissim like the EGO).

            :return: the dict returned by SimulationSynchronization.spawn_all_vissim_actors_in_carla().
        """
        result = self._sync.spawn_all_vissim_actors_in_carla()
        self._carla.world.tick()
        self._carla.update_actor_diff()
        self._log('[VISSIM WARMUP] caught up: carla_spawned=%d vissim_only=%d pedestrians=%d '
                  'pedestrians_vissim_only=%d' %
                  (result['vehicles'], len(result['vehicles_not_spawned']),
                   result['pedestrians'], len(result['pedestrians_not_spawned'])))
        if result['vehicles_not_spawned']:
            self._log('[VISSIM WARMUP] vissim vehicle(s) without a CARLA counterpart: %s' %
                      sorted(result['vehicles_not_spawned']))
        return result

    def collect_gap_vehicles(self):
        """
        Returns the vissim vehicles currently in the network as GapVehicle, for
        evaluate_spawn_gap() (see the plan doc sections 2.6.2 / 2.6.3):

        - mirrored in CARLA: the pose and size of the CARLA actor's bounding box ('carla');
        - without a CARLA counterpart (unknown type, spawn failure): the pose derived from vissim's
          front-center-bumper position, with the assumed EGO_SPAWN_UNKNOWN_VEHICLE_* size
          ('vissim_only').
        """
        from .bridge_helper import BridgeHelper  # imports carla: only needed from here on

        vehicles = []
        for vissim_id in sorted(self._vissim.vehicle_ids):
            carla_id = self._sync.vissim2carla_ids.get(vissim_id)
            carla_actor = self._carla.get_actor(carla_id) if carla_id is not None else None
            if carla_actor is not None:
                transform = carla_actor.get_transform()
                box = carla_actor.bounding_box
                x, y = bounding_box_center(transform.location.x, transform.location.y,
                                           transform.rotation.yaw, box.location.x, box.location.y)
                vehicles.append(GapVehicle('vissim:%s/carla:%s' % (vissim_id, carla_id), x, y,
                                           transform.rotation.yaw, 2.0 * box.extent.x,
                                           2.0 * box.extent.y, 'carla'))
            else:
                front = BridgeHelper.get_carla_transform(
                    self._vissim.get_actor(vissim_id).get_transform())
                yaw = front.rotation.yaw
                x, y = center_from_front(front.location.x, front.location.y, yaw,
                                         EGO_SPAWN_UNKNOWN_VEHICLE_LENGTH_M)
                vehicles.append(GapVehicle('vissim:%s' % vissim_id, x, y, yaw,
                                           EGO_SPAWN_UNKNOWN_VEHICLE_LENGTH_M,
                                           EGO_SPAWN_UNKNOWN_VEHICLE_WIDTH_M, 'vissim_only'))
        return vehicles

    def wait_for_safe_gap(self, ego, front_margin, rear_margin, wait_timeout):
        """
        Checks the gap around the EGO's planned spawn pose; while it is not safe, advances the
        co-simulation by one regular synchronization step (vissim -> CARLA, CARLA tick, CARLA ->
        vissim; without the EGO, and without ticking GameTime or publishing anything) and checks
        again - with no real-time pacing (see the plan doc section 2.6.5).

        Logs the check right away, whenever its result changes, once per simulated second, and
        once safe.

            :param EgoSpawnPose ego: the EGO's planned spawn pose and size.
            :param int wait_timeout: give up after this many simulated seconds.
            :return: the safe GapResult, or None if a stop was requested meanwhile.
            :raises EgoSpawnGateError: if no safe gap appears within wait_timeout, or if
                max_consecutive_failures vissim ticks fail in a row.
        """
        max_steps = wait_timeout * self._sim_res
        step = 0
        last_key = None
        while True:
            result = evaluate_spawn_gap(ego, self.collect_gap_vehicles(), front_margin,
                                        rear_margin)
            # "Changed" means a different outcome or different vehicles involved - not merely
            # clearances that moved a little, which they do at nearly every step.
            key = (result.safe,
                   result.front.vehicle.label if result.front is not None else None,
                   result.rear.vehicle.label if result.rear is not None else None,
                   tuple(v.label for v in result.overlapping))
            if result.safe or key != last_key or step % self._sim_res == 0:
                self._log('[EGO SPAWN CHECK] t=%.2f s %s' % (self.sim_time, result.describe()))
            last_key = key
            if result.safe:
                return result

            if step >= max_steps:
                raise EgoSpawnGateError(
                    'no safe gap found within ego_spawn_wait_timeout=%d s (t=%.2f s): %s' %
                    (wait_timeout, self.sim_time, result.describe()))
            if self._should_stop():
                self._log('[EGO SPAWN CHECK] stopped at t=%.2f s' % self.sim_time)
                return None

            self._sync.sync_vissim_to_carla()
            self._carla.world.tick()
            self._sync.sync_carla_to_vissim()
            if self._vissim.consecutive_failures >= self._max_consecutive_failures:
                raise EgoSpawnGateError(
                    'giving up after %d consecutive failed vissim adapter tick(s) while waiting '
                    'for a safe EGO spawn gap (t=%.2f s). The vissim adapter (and Vissim) may need '
                    'to be restarted.' % (self._vissim.consecutive_failures, self.sim_time))
            step += 1
