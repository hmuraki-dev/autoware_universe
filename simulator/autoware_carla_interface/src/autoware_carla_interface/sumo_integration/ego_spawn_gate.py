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

import collections
import math
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


# ==================================================================================================
# -- gap evaluation (section 2.6, Step S5) ----------------------------------------------------------
# ==================================================================================================

# One actor around the EGO spawn point, in CARLA coordinates (meters / degrees). x, y is the center
# of the actor's footprint; length/width its full size. `source` is 'carla' (position and size
# from the CARLA actor), 'sumo_only' (a SUMO vehicle not mirrored in CARLA: center converted from
# SUMO, SUMO size, section 2.6.3) or 'pedestrian' (only checked for overlap, never a leader or
# follower). `actor_id` is the display id used in the logs, e.g. "sumo:in1_A.3/carla:123".
GapActor = collections.namedtuple('GapActor', 'actor_id x y yaw length width source')

# The leader or follower found in the EGO lane: `s` is its longitudinal offset from the EGO spawn
# point along the EGO heading (positive ahead), `clearance` the bumper-to-bumper distance.
GapNeighbor = collections.namedtuple('GapNeighbor', 'actor s clearance')

# safe: True if EGO may be spawned. front/rear: GapNeighbor or None (no vehicle in range).
# overlaps: actors whose footprint intersects the EGO footprint at the spawn point.
GapResult = collections.namedtuple('GapResult', 'safe front rear overlaps')


def _heading_difference_deg(yaw_a, yaw_b):
    """Absolute difference of two headings in degrees, in [0, 180]."""
    return abs((yaw_a - yaw_b + 180.0) % 360.0 - 180.0)


def _footprint_corners(x, y, yaw_deg, length, width):
    yaw = math.radians(yaw_deg)
    fx, fy = math.cos(yaw), math.sin(yaw)  # forward
    rx, ry = -fy, fx  # 90 degrees from forward
    hl, hw = length / 2.0, width / 2.0
    return [(x + sl * hl * fx + sw * hw * rx, y + sl * hl * fy + sw * hw * ry)
            for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def _footprints_overlap(a, b):
    """Separating axis test for two rectangles given as 4 corners each (touching = no overlap)."""
    for rect in (a, b):
        for i in range(4):
            ex, ey = rect[(i + 1) % 4][0] - rect[i][0], rect[(i + 1) % 4][1] - rect[i][1]
            axis = (-ey, ex)
            proj_a = [px * axis[0] + py * axis[1] for px, py in a]
            proj_b = [px * axis[0] + py * axis[1] for px, py in b]
            if max(proj_a) <= min(proj_b) or max(proj_b) <= min(proj_a):
                return False
    return True


def evaluate_spawn_gap(ego_x, ego_y, ego_yaw, ego_length, ego_width, lane_width, actors,
                       front_margin, rear_margin, search_range=EGO_SPAWN_SEARCH_RANGE_M,
                       heading_tolerance=EGO_SPAWN_HEADING_TOLERANCE_DEG):
    """
    Decide whether EGO can be spawned at (ego_x, ego_y, ego_yaw) (section 2.6).

    Pure function in CARLA coordinates; assumes the spawn point is on a straight road (2.6.1).

    - Leader/follower (2.6.1): among the non-pedestrian actors with |d| < lane_width / 2,
      |s| <= search_range and a heading within heading_tolerance of the EGO heading, the nearest
      one with s >= 0 is the leader and the nearest one with s < 0 the follower (s: longitudinal,
      d: lateral offset in the EGO frame).
    - Clearances (2.6.2): leader: s - ego_length/2 - length/2, follower: |s| - ego_length/2 -
      length/2. A missing leader/follower satisfies its side.
    - Overlap (2.6.2): any actor (any lane, any heading, pedestrians included) whose footprint
      intersects the EGO footprint makes the result unsafe.

    Safe iff no overlap, leader clearance >= front_margin and follower clearance >= rear_margin.

        :return: GapResult.
    """
    yaw = math.radians(ego_yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    ego_corners = _footprint_corners(ego_x, ego_y, ego_yaw, ego_length, ego_width)

    front = rear = None
    overlaps = []
    for actor in actors:
        if _footprints_overlap(ego_corners, _footprint_corners(actor.x, actor.y, actor.yaw,
                                                               actor.length, actor.width)):
            overlaps.append(actor)
        if actor.source == 'pedestrian':
            continue

        dx, dy = actor.x - ego_x, actor.y - ego_y
        s = dx * fx + dy * fy
        d = -dx * fy + dy * fx
        if (abs(d) >= lane_width / 2.0 or abs(s) > search_range
                or _heading_difference_deg(actor.yaw, ego_yaw) > heading_tolerance):
            continue

        clearance = abs(s) - ego_length / 2.0 - actor.length / 2.0
        if s >= 0.0:
            if front is None or s < front.s:
                front = GapNeighbor(actor, s, clearance)
        elif rear is None or s > rear.s:
            rear = GapNeighbor(actor, s, clearance)

    safe = (not overlaps
            and (front is None or front.clearance >= front_margin)
            and (rear is None or rear.clearance >= rear_margin))
    return GapResult(safe, front, rear, overlaps)


def describe_gap_result(result):
    """One-line summary for the [EGO SPAWN CHECK] log (section 2.9), without the time stamp."""
    def neighbor(name, n):
        if n is None:
            return '%s=-' % name
        return '%s=%s clearance=%.1f m' % (name, n.actor.actor_id, n.clearance)

    overlap = ','.join(a.actor_id for a in result.overlaps) or 'none'
    return '%s %s overlap=%s result=%s' % (neighbor('front', result.front),
                                           neighbor('rear', result.rear), overlap,
                                           'SAFE' if result.safe else 'WAIT')


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
