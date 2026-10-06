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

    WARMUP    : tick Vissim alone for warmup_time seconds (CARLA is not ticked, no pacing)
    catch-up  : spawn in CARLA every vissim actor that entered the network meanwhile
    (then the caller spawns the EGO and starts the co-simulation period)

Deliberately free of carla / zmq imports, so that it can be imported (and unit tested) without
CARLA or a vissim adapter.
"""

import time

# Simulated seconds between two warmup progress lines.
WARMUP_REPORT_INTERVAL_S = 10


class EgoSpawnGateError(RuntimeError):
    """
    Raised when the co-simulation cannot be started (the test start failed): the vissim adapter
    failed too many ticks in a row during the warmup, or the EGO could not be spawned.
    """


class EgoSpawnGate(object):
    """
    Runs the Vissim warmup and the catch-up spawn in CARLA that follows it.

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
