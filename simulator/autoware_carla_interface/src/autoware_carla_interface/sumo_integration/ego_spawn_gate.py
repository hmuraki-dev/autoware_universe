"""
SUMO warmup and EGO safe spawn gate.

See docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md. Before the normal
co-simulation loop (`run_bridge()`), SUMO is advanced alone for `sumo_warmup_time` seconds
(WARMUP), then EGO is spawned only once the gap around its spawn point is large enough
(WAIT_FOR_SAFE_GAP -> SPAWN_EGO).

Not part of the vendored upstream bridge (this file is new in autoware_carla_interface). Unlike
`sumo_simulation.py`, this module must stay importable without `traci`/`sumolib`/`carla`, so the
pure parts (parameter validation, gap evaluation) can be tested anywhere. `EgoSpawnGate` only
uses the SUMO/CARLA objects it is given (duck typing), so it can be tested with fakes as well.
"""

import time

# Gap evaluation constants (section 2.2). Fixed values, deliberately not launch args.
EGO_SPAWN_SEARCH_RANGE_M = 100.0  # longitudinal range in which leaders/followers are searched
EGO_SPAWN_HEADING_TOLERANCE_DEG = 45.0  # max heading difference to count as the same direction

# Simulated seconds between WARMUP progress log lines (section 2.3).
WARMUP_PROGRESS_INTERVAL_S = 10.0

# tls_manager values with which the warmup cannot be used (section 1.6): with 'carla', SUMO's
# traffic lights are switched off and only follow CARLA, which is not ticked during the warmup.
_TLS_MANAGERS_WITHOUT_WARMUP = ('carla',)


def is_random_spawn_point(spawn_point):
    """
    Return True if `spawn_point` makes the EGO spawn at a random location.

    Mirrors `InitializeInterface._parse_spawn_point()`: anything other than six comma-separated
    items (e.g. the launch default "None") means a random spawn point.
    """
    return len(spawn_point.split(',')) != 6


def validate_warmup_params(use_sumo, sumo_warmup_time, ego_spawn_front_margin,
                           ego_spawn_rear_margin, ego_spawn_wait_timeout, spawn_point,
                           tls_manager):
    """
    Check the warmup / EGO safe spawn parameters (section 2.2).

    Raises ValueError with a message naming the offending parameter. With the default
    `sumo_warmup_time == 0` (warmup disabled), only the sign of `sumo_warmup_time` is checked, so
    the other parameters cannot change the behavior of a run without warmup.
    """
    if sumo_warmup_time < 0:
        raise ValueError(
            'sumo_warmup_time must be >= 0 (0 disables the warmup), got %r' % (sumo_warmup_time,))
    if sumo_warmup_time == 0:
        return

    if not use_sumo:
        raise ValueError(
            'sumo_warmup_time > 0 requires use_sumo=True (got sumo_warmup_time=%r)'
            % (sumo_warmup_time,))
    if is_random_spawn_point(spawn_point):
        raise ValueError(
            'sumo_warmup_time > 0 requires spawn_point to be set as "x,y,z,roll,pitch,yaw": the gap '
            'check needs a fixed EGO spawn location (got spawn_point=%r)' % (spawn_point,))
    if tls_manager in _TLS_MANAGERS_WITHOUT_WARMUP:
        raise ValueError(
            'sumo_warmup_time > 0 cannot be used with tls_manager=%r: SUMO traffic lights are '
            'switched off and follow CARLA, which is not ticked during the warmup. Use '
            'tls_manager=sumo or none.' % (tls_manager,))
    if ego_spawn_front_margin < 0:
        raise ValueError(
            'ego_spawn_front_margin must be >= 0, got %r' % (ego_spawn_front_margin,))
    if ego_spawn_rear_margin < 0:
        raise ValueError(
            'ego_spawn_rear_margin must be >= 0, got %r' % (ego_spawn_rear_margin,))
    if ego_spawn_wait_timeout < 1:
        raise ValueError(
            'ego_spawn_wait_timeout must be >= 1, got %r' % (ego_spawn_wait_timeout,))


class EgoSpawnGate(object):
    """
    WARMUP -> catch-up -> SPAWN_EGO, run once before the normal co-simulation loop.

    Sections 2.1/2.3/2.4/2.7. Step S3: EGO is spawned right after the catch-up; the gap check
    (WAIT_FOR_SAFE_GAP, Step S5/S6) is not implemented yet.

    Nothing here touches GameTime / CarlaDataProvider.on_carla_tick() or the sensors, so the ROS
    clock does not advance and nothing is published until run_bridge() starts (section 1.2).
    """

    def __init__(self, sumo_sync, world, warmup_time, step_length, spawn_ego, stop_requested,
                 log_info):
        """
        Args
        ----
            sumo_sync: the SimulationSynchronization; its `.sumo` (SumoSimulation) is ticked alone
                during the warmup.
            world: the carla.World; ticked only after the warmup.
            warmup_time: seconds of SUMO time to run alone (sumo_warmup_time, > 0).
            step_length: seconds per step (fixed_delta_seconds, also SUMO's --step-length).
            spawn_ego: callable spawning EGO and its sensors; raises if EGO cannot be spawned.
            stop_requested: callable returning True once SIGINT/SIGTERM has been received.
            log_info: callable taking one message string.

        """
        self._sync = sumo_sync
        self._sumo = sumo_sync.sumo
        self._world = world
        self._step_length = step_length
        self._warmup_steps = int(round(warmup_time / step_length))
        self._progress_every = max(1, int(round(WARMUP_PROGRESS_INTERVAL_S / step_length)))
        self._spawn_ego = spawn_ego
        self._stop_requested = stop_requested
        self._log = log_info

    def run(self):
        """
        Returns True once EGO has been spawned, False if a stop was requested before that.
        """
        if not self._warmup():
            return False
        self._catch_up()
        if self._stop_requested():
            self._log('[SUMO WARMUP] stop requested; EGO is not spawned')
            return False
        # One normal synchronization step (without EGO), so that the actors spawned by the
        # catch-up - placed SPAWN_OFFSET_Z above their position, like any SUMO actor spawned in
        # CARLA - are moved onto the road before EGO is spawned next to them.
        self._sync_step_without_ego()
        if self._stop_requested():
            self._log('[SUMO WARMUP] stop requested; EGO is not spawned')
            return False
        self._spawn_ego()
        self._log('[EGO SPAWN] t=%.2f s vehicles=%d' %
                  (self._sumo.get_time(), len(self._sumo.get_vehicle_ids())))
        return True

    def _warmup(self):
        """SUMO alone, as fast as possible: no CARLA tick, no pacing sleep (section 2.3)."""
        self._log('[SUMO WARMUP] start: warmup_time=%g s (%d steps)' %
                  (self._warmup_steps * self._step_length, self._warmup_steps))
        wall_start = time.monotonic()
        for step in range(1, self._warmup_steps + 1):
            if self._stop_requested():
                self._log('[SUMO WARMUP] stop requested at t=%.2f s' % self._sumo.get_time())
                return False
            self._sumo.tick()
            if step % self._progress_every == 0 and step != self._warmup_steps:
                self._log('[SUMO WARMUP] t=%.1f s vehicles=%d persons=%d (wall %.1f s)' %
                          (self._sumo.get_time(), len(self._sumo.get_vehicle_ids()),
                           len(self._sumo.get_person_ids()), time.monotonic() - wall_start))
        self._log('[SUMO WARMUP] completed: t=%.1f s vehicles=%d persons=%d (wall %.1f s)' %
                  (self._sumo.get_time(), len(self._sumo.get_vehicle_ids()),
                   len(self._sumo.get_person_ids()), time.monotonic() - wall_start))
        return True

    def _catch_up(self):
        """Spawn in CARLA everything that departed during the warmup (section 2.4)."""
        wall_start = time.monotonic()
        counts = self._sync.spawn_all_sumo_actors_in_carla()
        self._log('[SUMO WARMUP] caught up: carla_spawned=%d sumo_only=%d pedestrians=%d '
                  'pedestrians_sumo_only=%d (wall %.1f s)' %
                  (counts['vehicles'], counts['vehicles_not_in_carla'], counts['pedestrians'],
                   counts['pedestrians_not_in_carla'], time.monotonic() - wall_start))

    def _sync_step_without_ego(self):
        """The normal SUMO<->CARLA step of SensorLoop._tick_sensor(), minus GameTime/sensors."""
        self._sync.sync_sumo_to_carla()
        self._world.tick()
        self._sync.sync_carla_to_sumo()
