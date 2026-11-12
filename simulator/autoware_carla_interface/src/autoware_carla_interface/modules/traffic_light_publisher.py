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

"""ROS/CARLA-free logic for publishing CARLA traffic-light states to Autoware.

See docs/CARLA_Autoware_信号状態publish_実装計画_v1.0.md (T3). Upstream keeps this logic
inside ``carla_ros.py``; it is split out here so the group-id resolution order, the
per-group aggregation and the state conversion can be unit-tested without ROS or CARLA.
The geometric matching itself lives in ``traffic_light_matcher.py`` (ported unchanged
from upstream).

Inputs are plain values (CARLA states by name, positions as tuples), so the caller in
``carla_ros.py`` only has to read the CARLA actors and build the ROS message.
"""

import os

from .traffic_light_matcher import MatchResult
from .traffic_light_matcher import match_traffic_lights
from .traffic_light_matcher import parse_id_map_override

# Constants of autoware_perception_msgs/msg/TrafficLightElement, repeated here so this
# module stays importable without ROS. test_traffic_light_publisher.py checks them
# against the message definition when the message package is available.
COLOR_UNKNOWN = 0
COLOR_RED = 1
COLOR_AMBER = 2
COLOR_GREEN = 3
SHAPE_CIRCLE = 1
STATUS_UNKNOWN = 0
STATUS_SOLID_OFF = 1
STATUS_SOLID_ON = 2

_STATE_TO_ELEMENT = {
    "Red": (COLOR_RED, STATUS_SOLID_ON),
    "Yellow": (COLOR_AMBER, STATUS_SOLID_ON),
    "Green": (COLOR_GREEN, STATUS_SOLID_ON),
    # CARLA's Off is a known state (the signal is dark), not a lamp of unknown color.
    "Off": (COLOR_UNKNOWN, STATUS_SOLID_OFF),
}

# When several CARLA lights resolve to the same group id but show different aspects,
# the most restrictive one is published (lower rank wins). Upstream lets the last
# light win, which depends on actor order.
_RESTRICTIVENESS = {
    (COLOR_RED, STATUS_SOLID_ON): 0,
    (COLOR_AMBER, STATUS_SOLID_ON): 1,
    (COLOR_UNKNOWN, STATUS_UNKNOWN): 2,
    (COLOR_UNKNOWN, STATUS_SOLID_OFF): 3,
    (COLOR_GREEN, STATUS_SOLID_ON): 4,
}


def carla_state_to_element(state):
    """Map a ``carla.TrafficLightState`` (or its name) to ``(color, status)``.

    Red / Yellow / Green are lit lamps (SOLID_ON), Off is SOLID_OFF, and any state this
    bridge cannot interpret (e.g. ``Unknown``) is reported as UNKNOWN / UNKNOWN.
    """
    name = getattr(state, "name", None) or str(state)
    return _STATE_TO_ELEMENT.get(name, (COLOR_UNKNOWN, STATUS_UNKNOWN))


def carla_head_map_point(light_box_locations, fallback_location, origin_x=0.0, origin_y=0.0):
    """Return a CARLA light's head position ``(x, y)`` in the Autoware map frame.

    ``light_box_locations`` are the CARLA ``(x, y)`` centres of ``get_light_boxes()``
    (the physical light heads); their mean is used because the actor origin sits at the
    pole base. ``fallback_location`` (the actor location) is used when there is no box.
    The conversion matches ``carla_location_to_ros_point``: ``x + origin_x``,
    ``-y + origin_y`` (docs T1: the offset is 0 for Town01).
    """
    points = list(light_box_locations or [])
    if points:
        x = sum(p[0] for p in points) / len(points)
        y = sum(p[1] for p in points) / len(points)
    else:
        x, y = fallback_location[0], fallback_location[1]
    return (x + origin_x, -y + origin_y)


# tls_manager values with which traffic_light.force_green is refused: with 'sumo', SUMO
# keeps overwriting the CARLA lights every step; with 'carla', the frozen all-green CARLA
# lights would be copied into SUMO and let SUMO traffic cross intersections head-on.
_TLS_MANAGERS_WITHOUT_FORCE_GREEN = ("sumo", "carla")


def validate_traffic_light_params(
    publish,
    map_path,
    match_distance,
    match_ratio,
    force_green=False,
    use_sumo=False,
    tls_manager="none",
):
    """Raise ``ValueError`` if the traffic_light.* parameters cannot work.

    Checked at startup so a typo fails the launch instead of silently publishing
    nothing. With ``publish`` and ``force_green`` False (default) every value is accepted.
    """
    if force_green and use_sumo and tls_manager in _TLS_MANAGERS_WITHOUT_FORCE_GREEN:
        raise ValueError(
            f"traffic_light.force_green cannot be used with use_sumo and "
            f"tls_manager={tls_manager!r}: the traffic lights are synchronized between SUMO "
            f"and CARLA. Use tls_manager=none, or disable traffic_light.force_green."
        )
    if not publish:
        return
    map_path = str(map_path or "").strip()
    if map_path and not os.path.isfile(map_path):
        raise ValueError(
            f"traffic_light.map_path '{map_path}' is not a file. Give the lanelet2 .osm "
            f"used by Autoware, or leave it empty to use the OpenDRIVE signal ids."
        )
    if not match_distance > 0.0:
        raise ValueError(f"traffic_light.match_distance must be > 0 (got {match_distance})")
    if not 0.0 < match_ratio <= 1.0:
        raise ValueError(f"traffic_light.match_ratio must be in (0, 1] (got {match_ratio})")


def _parse_opendrive_id(opendrive_id):
    """Return the OpenDRIVE signal id as int, or None when it is missing or not numeric."""
    try:
        return int(str(opendrive_id).strip())
    except (TypeError, ValueError):
        return None


class TrafficLightResolution:
    """How each CARLA light resolved to Autoware group ids, for one run.

    ``assignments`` maps a CARLA actor id to its sorted group ids; lights missing from
    it are not published. ``overridden`` holds the actor ids pinned by ``id_map``.
    ``match_result`` is the position matcher's result (None when no map was given), and
    ``fallback_count`` is the number of lights that used their OpenDRIVE id directly.
    """

    def __init__(self):
        self.assignments = {}
        self.overridden = set()
        self.match_result = None
        self.fallback_count = 0


def resolve_traffic_light_groups(
    lights,
    id_map="",
    map_lights=None,
    match_distance=5.0,
    match_ratio=0.6,
    on_invalid=None,
):
    """Resolve each CARLA light to its Autoware group id(s).

    ``lights`` is an iterable of ``(actor_id, opendrive_id, (x, y))`` with the head
    position already in the map frame (see :func:`carla_head_map_point`). Per light:

    1. ``id_map`` (``opendrive_id:group_id[|group_id...],...``) wins. Malformed entries
       are skipped and reported through ``on_invalid``.
    2. Otherwise, if ``map_lights`` (parsed lanelet2 heads) is given, the light is
       matched by position; ambiguous / too-far lights are left out.
    3. Otherwise, the OpenDRIVE signal id is used as the group id.

    Returns a :class:`TrafficLightResolution`.
    """
    lights = list(lights)
    resolution = TrafficLightResolution()
    override = parse_id_map_override(id_map, on_invalid=on_invalid)

    to_resolve = []
    for actor_id, opendrive_id, point in lights:
        parsed_id = _parse_opendrive_id(opendrive_id)
        if parsed_id is not None and parsed_id in override:
            resolution.assignments[actor_id] = list(override[parsed_id])
            resolution.overridden.add(actor_id)
        else:
            to_resolve.append((actor_id, parsed_id, point))

    if map_lights is not None:
        result = match_traffic_lights(
            to_resolve,
            map_lights,
            distance_threshold=match_distance,
            ambiguity_ratio=match_ratio,
        )
        resolution.match_result = result
        resolution.assignments.update(result.assignments)
    else:
        for actor_id, parsed_id, _ in to_resolve:
            if parsed_id is not None:
                resolution.assignments[actor_id] = [parsed_id]
                resolution.fallback_count += 1

    return resolution


def resolution_report(resolution, map_lights=None):
    """Human-readable startup report as a list of ``(level, message)``.

    ``level`` is ``"info"`` or ``"warning"`` so the caller can forward each line to the
    matching logger method.
    """
    lines = []
    result = resolution.match_result
    if result is None:
        lines.append(
            (
                "info",
                f"Traffic lights: publishing {resolution.fallback_count} CARLA lights using "
                f"the OpenDRIVE signal id as the group id (no traffic_light.map_path), "
                f"{len(resolution.overridden)} overridden by traffic_light.id_map",
            )
        )
        return lines

    by_status = {}
    for entry in result.entries:
        by_status.setdefault(entry["status"], []).append(entry)
    ambiguous = by_status.get(MatchResult.AMBIGUOUS, [])
    too_far = by_status.get(MatchResult.TOO_FAR, [])
    no_head = by_status.get(MatchResult.NO_HEAD, [])
    map_summary = (
        f" (map has {len(map_lights)} heads / {map_lights.group_count} groups)"
        if map_lights is not None
        else ""
    )
    lines.append(
        (
            "info",
            f"Traffic-light matching: {result.matched_actor_count} matched, "
            f"{len(ambiguous)} ambiguous, {len(too_far)} too far, "
            f"{len(resolution.overridden)} overridden{map_summary}",
        )
    )
    for entry in ambiguous:
        lines.append(
            (
                "warning",
                f"  ambiguous CARLA light (opendrive_id={entry['opendrive_id']}): nearest "
                f"{entry['nearest']:.2f} m vs {entry['second']:.2f} m to a different signal; "
                f"not published (pin it via traffic_light.id_map if needed)",
            )
        )
    for entry in too_far:
        lines.append(
            (
                "warning",
                f"  unmatched CARLA light (opendrive_id={entry['opendrive_id']}): nearest map "
                f"head {entry['nearest']:.2f} m away exceeds the match distance; not published",
            )
        )
    if no_head:
        lines.append(
            (
                "warning",
                f"  the map has no traffic-light heads; {len(no_head)} CARLA lights not "
                f"published (does traffic_light.map_path contain traffic_light "
                f"regulatory elements?)",
            )
        )
    return lines


def build_group_states(actor_groups, actor_states):
    """Aggregate per-actor states into ``{group_id: (color, status)}``, sorted by id.

    ``actor_groups`` is ``{actor_id: [group_id, ...]}`` and ``actor_states`` is
    ``{actor_id: carla state or state name}``. Actors without groups or without a state
    are skipped. When several lights share a group id and disagree, the most
    restrictive aspect wins (red > amber > unknown > off > green).
    """
    group_states = {}
    for actor_id, group_ids in actor_groups.items():
        if not group_ids or actor_id not in actor_states:
            continue
        element = carla_state_to_element(actor_states[actor_id])
        for group_id in group_ids:
            current = group_states.get(group_id)
            if current is None or _RESTRICTIVENESS[element] < _RESTRICTIVENESS[current]:
                group_states[group_id] = element
    return dict(sorted(group_states.items()))
