"""
SUMO warmup and EGO safe spawn gate.

See docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md. Before the normal
co-simulation loop (`run_bridge()`), SUMO is advanced alone for `sumo_warmup_time` seconds
(WARMUP), then EGO is spawned only once the gap around its spawn point is large enough
(WAIT_FOR_SAFE_GAP -> SPAWN_EGO).

Not part of the vendored upstream bridge (this file is new in autoware_carla_interface). Unlike
`sumo_simulation.py`, this module must stay importable without `traci`/`sumolib`/`carla`, so the
pure parts (parameter validation, gap evaluation) can be tested anywhere.
"""

# Gap evaluation constants (section 2.2). Fixed values, deliberately not launch args.
EGO_SPAWN_SEARCH_RANGE_M = 100.0  # longitudinal range in which leaders/followers are searched
EGO_SPAWN_HEADING_TOLERANCE_DEG = 45.0  # max heading difference to count as the same direction

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
