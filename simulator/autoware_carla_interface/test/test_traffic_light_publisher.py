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

"""Unit tests for the ROS/CARLA-free traffic-light publishing logic (docs T3/T8)."""

import types

from autoware_carla_interface.modules import traffic_light_publisher as tlp
from autoware_carla_interface.modules.traffic_light_matcher import MapTrafficLights
from autoware_carla_interface.modules.traffic_light_matcher import MatchResult
import pytest


def _map(heads):
    """Build a MapTrafficLights from {way_id: ((x, y), {group_id, ...})}."""
    m = MapTrafficLights()
    for way_id, (pos, groups) in heads.items():
        m.head_positions[way_id] = pos
        m.head_groups[way_id] = set(groups)
    return m


# --- state conversion ------------------------------------------------------------------


@pytest.mark.parametrize(
    "state, expected",
    [
        ("Red", (tlp.COLOR_RED, tlp.STATUS_SOLID_ON)),
        ("Yellow", (tlp.COLOR_AMBER, tlp.STATUS_SOLID_ON)),
        ("Green", (tlp.COLOR_GREEN, tlp.STATUS_SOLID_ON)),
        ("Off", (tlp.COLOR_UNKNOWN, tlp.STATUS_SOLID_OFF)),
        ("Unknown", (tlp.COLOR_UNKNOWN, tlp.STATUS_UNKNOWN)),
        ("SomethingElse", (tlp.COLOR_UNKNOWN, tlp.STATUS_UNKNOWN)),
    ],
)
def test_state_names_map_to_elements(state, expected):
    assert tlp.carla_state_to_element(state) == expected


def test_state_enum_with_name_attribute():
    # carla.TrafficLightState values expose .name (e.g. 'Red').
    assert tlp.carla_state_to_element(types.SimpleNamespace(name="Red")) == (
        tlp.COLOR_RED,
        tlp.STATUS_SOLID_ON,
    )


def test_real_carla_states_when_available():
    carla = pytest.importorskip("carla")
    if not hasattr(carla, "TrafficLight"):
        pytest.skip("carla stand-in, not the real module")
    s = carla.TrafficLightState
    assert tlp.carla_state_to_element(s.Red) == (tlp.COLOR_RED, tlp.STATUS_SOLID_ON)
    assert tlp.carla_state_to_element(s.Yellow) == (tlp.COLOR_AMBER, tlp.STATUS_SOLID_ON)
    assert tlp.carla_state_to_element(s.Green) == (tlp.COLOR_GREEN, tlp.STATUS_SOLID_ON)
    assert tlp.carla_state_to_element(s.Off) == (tlp.COLOR_UNKNOWN, tlp.STATUS_SOLID_OFF)
    assert tlp.carla_state_to_element(s.Unknown) == (tlp.COLOR_UNKNOWN, tlp.STATUS_UNKNOWN)


def test_constants_match_the_message_definition_when_available():
    msg = pytest.importorskip("autoware_perception_msgs.msg")
    element = msg.TrafficLightElement
    assert tlp.COLOR_UNKNOWN == element.UNKNOWN
    assert tlp.COLOR_RED == element.RED
    assert tlp.COLOR_AMBER == element.AMBER
    assert tlp.COLOR_GREEN == element.GREEN
    assert tlp.SHAPE_CIRCLE == element.CIRCLE
    assert tlp.STATUS_UNKNOWN == element.UNKNOWN
    assert tlp.STATUS_SOLID_OFF == element.SOLID_OFF
    assert tlp.STATUS_SOLID_ON == element.SOLID_ON


# --- head position ----------------------------------------------------------------------


def test_head_point_is_mean_of_light_boxes_with_y_flipped():
    # docs T1: map_x = carla_x, map_y = -carla_y for Town01.
    point = tlp.carla_head_map_point([(10.0, 4.0), (12.0, 6.0)], (0.0, 0.0))
    assert point == (11.0, -5.0)


def test_head_point_falls_back_to_actor_location_and_applies_origin():
    point = tlp.carla_head_map_point([], (3.0, 2.0), origin_x=100.0, origin_y=50.0)
    assert point == (103.0, 48.0)


# --- group id resolution ----------------------------------------------------------------


def test_fallback_uses_opendrive_id_without_map():
    lights = [(60, "360", (0.0, 0.0)), (61, 361, (5.0, 0.0)), (62, "", (9.0, 0.0))]
    res = tlp.resolve_traffic_light_groups(lights)
    assert res.assignments == {60: [360], 61: [361]}
    # A light without a numeric OpenDRIVE id cannot be published.
    assert 62 not in res.assignments
    assert res.fallback_count == 2
    assert res.match_result is None


def test_id_map_takes_precedence_over_fallback():
    lights = [(60, "360", (0.0, 0.0)), (61, "361", (5.0, 0.0))]
    res = tlp.resolve_traffic_light_groups(lights, id_map="360:9000|9001")
    assert res.assignments == {60: [9000, 9001], 61: [361]}
    assert res.overridden == {60}
    assert res.fallback_count == 1


def test_id_map_takes_precedence_over_position_matching():
    m = _map({"w1": ((0.0, 0.0), {500}), "w2": ((50.0, 0.0), {600})})
    # Light 60 sits on head w1 but is pinned to 777; light 61 matches w2 by position.
    lights = [(60, "360", (0.0, 0.0)), (61, "361", (50.2, 0.0))]
    res = tlp.resolve_traffic_light_groups(lights, id_map="360:777", map_lights=m)
    assert res.assignments == {60: [777], 61: [600]}
    # Overridden lights are not passed to the matcher at all.
    assert [e["actor_id"] for e in res.match_result.entries] == [61]


def test_position_matching_drops_ambiguous_and_too_far_lights():
    m = _map({"w1": ((0.0, 0.0), {1}), "w2": ((2.0, 0.0), {2}), "w3": ((100.0, 0.0), {3})})
    lights = [
        (10, "10", (100.1, 0.0)),  # matched to w3
        (11, "11", (1.0, 0.0)),  # halfway between w1 and w2 -> ambiguous
        (12, "12", (50.0, 30.0)),  # nothing within 5 m -> too far
    ]
    res = tlp.resolve_traffic_light_groups(lights, map_lights=m)
    assert res.assignments == {10: [3]}
    statuses = {e["actor_id"]: e["status"] for e in res.match_result.entries}
    assert statuses == {10: MatchResult.MATCHED, 11: MatchResult.AMBIGUOUS, 12: MatchResult.TOO_FAR}
    # With a map, the OpenDRIVE id is never used as a fallback.
    assert res.fallback_count == 0


def test_match_distance_and_ratio_are_passed_through():
    m = _map({"w1": ((0.0, 0.0), {1}), "w2": ((10.0, 0.0), {2})})
    lights = [(10, "10", (4.0, 0.0))]
    # Default ratio 0.6: 4 > 0.6 * 6 -> ambiguous.
    assert tlp.resolve_traffic_light_groups(lights, map_lights=m).assignments == {}
    # Looser ratio accepts it; a tighter distance rejects it again.
    loose = tlp.resolve_traffic_light_groups(lights, map_lights=m, match_ratio=0.8)
    assert loose.assignments == {10: [1]}
    near = tlp.resolve_traffic_light_groups(
        lights, map_lights=m, match_ratio=0.8, match_distance=3.0
    )
    assert near.assignments == {}


def test_malformed_id_map_entries_are_reported_and_skipped():
    reported = []
    lights = [(60, "360", (0.0, 0.0))]
    res = tlp.resolve_traffic_light_groups(lights, id_map="360,361:5", on_invalid=reported.append)
    assert res.assignments == {60: [360]}
    assert len(reported) == 1 and "'360'" in reported[0]


# --- startup report ---------------------------------------------------------------------


def test_report_counts_and_warnings_for_position_matching():
    m = _map({"w1": ((0.0, 0.0), {1}), "w2": ((2.0, 0.0), {2})})
    lights = [(10, "10", (0.0, 0.0)), (11, "11", (1.0, 0.0)), (12, "12", (90.0, 0.0))]
    res = tlp.resolve_traffic_light_groups(lights, id_map="99:5", map_lights=m)
    lines = tlp.resolution_report(res, m)
    assert lines[0] == (
        "info",
        "Traffic-light matching: 1 matched, 1 ambiguous, 1 too far, 0 overridden "
        "(map has 2 heads / 2 groups)",
    )
    warnings = [text for level, text in lines if level == "warning"]
    assert any("ambiguous" in t and "opendrive_id=11" in t for t in warnings)
    assert any("unmatched" in t and "opendrive_id=12" in t for t in warnings)


def test_report_warns_when_map_has_no_traffic_lights():
    # Like the stock Town01 map (docs section 1): no traffic_light regulatory elements.
    m = MapTrafficLights()
    res = tlp.resolve_traffic_light_groups([(10, "10", (0.0, 0.0))], map_lights=m)
    assert res.assignments == {}
    warnings = [text for level, text in tlp.resolution_report(res, m) if level == "warning"]
    assert len(warnings) == 1 and "no traffic-light heads" in warnings[0]


def test_report_for_fallback():
    res = tlp.resolve_traffic_light_groups([(60, "360", (0.0, 0.0))])
    assert tlp.resolution_report(res) == [
        (
            "info",
            "Traffic lights: publishing 1 CARLA lights using the OpenDRIVE signal id as the "
            "group id (no traffic_light.map_path), 0 overridden by traffic_light.id_map",
        )
    ]


# --- per-group aggregation --------------------------------------------------------------


def test_group_states_fan_out_shared_heads_and_sort_by_id():
    groups = {60: [501, 500], 61: [300]}
    states = {60: "Red", 61: "Green"}
    assert tlp.build_group_states(groups, states) == {
        300: (tlp.COLOR_GREEN, tlp.STATUS_SOLID_ON),
        500: (tlp.COLOR_RED, tlp.STATUS_SOLID_ON),
        501: (tlp.COLOR_RED, tlp.STATUS_SOLID_ON),
    }
    assert list(tlp.build_group_states(groups, states)) == [300, 500, 501]


@pytest.mark.parametrize(
    "first, second, expected",
    [
        ("Green", "Red", (tlp.COLOR_RED, tlp.STATUS_SOLID_ON)),
        ("Red", "Green", (tlp.COLOR_RED, tlp.STATUS_SOLID_ON)),
        ("Green", "Yellow", (tlp.COLOR_AMBER, tlp.STATUS_SOLID_ON)),
        ("Green", "Unknown", (tlp.COLOR_UNKNOWN, tlp.STATUS_UNKNOWN)),
        ("Green", "Off", (tlp.COLOR_UNKNOWN, tlp.STATUS_SOLID_OFF)),
        ("Off", "Unknown", (tlp.COLOR_UNKNOWN, tlp.STATUS_UNKNOWN)),
    ],
)
def test_conflicting_lights_in_one_group_publish_the_most_restrictive(first, second, expected):
    groups = {1: [700], 2: [700]}
    assert tlp.build_group_states(groups, {1: first, 2: second}) == {700: expected}


def test_lights_without_groups_or_state_are_skipped():
    groups = {1: [], 2: [800], 3: [900]}
    states = {1: "Red", 2: "Green"}  # actor 3 has no state
    assert tlp.build_group_states(groups, states) == {800: (tlp.COLOR_GREEN, tlp.STATUS_SOLID_ON)}


# --- startup parameter check ------------------------------------------------------------


def test_params_are_not_checked_when_publishing_is_disabled():
    tlp.validate_traffic_light_params(False, "/no/such/map.osm", -1.0, 5.0)


def test_valid_params_pass(tmp_path):
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text("<osm/>")
    tlp.validate_traffic_light_params(True, str(osm), 5.0, 0.6)
    tlp.validate_traffic_light_params(True, "", 5.0, 1.0)


@pytest.mark.parametrize(
    "map_path, distance, ratio, message",
    [
        ("/no/such/map.osm", 5.0, 0.6, "is not a file"),
        ("", 0.0, 0.6, "match_distance"),
        ("", 5.0, 0.0, "match_ratio"),
        ("", 5.0, 1.5, "match_ratio"),
    ],
)
def test_invalid_params_are_rejected(map_path, distance, ratio, message):
    with pytest.raises(ValueError, match=message):
        tlp.validate_traffic_light_params(True, map_path, distance, ratio)


@pytest.mark.parametrize("tls_manager", ["sumo", "carla"])
def test_force_green_is_refused_with_sumo_synchronized_traffic_lights(tls_manager):
    with pytest.raises(ValueError, match="force_green"):
        tlp.validate_traffic_light_params(
            False, "", 5.0, 0.6, force_green=True, use_sumo=True, tls_manager=tls_manager
        )


@pytest.mark.parametrize(
    "use_sumo, tls_manager",
    [(True, "none"), (False, "sumo"), (False, "carla"), (False, "none")],
)
def test_force_green_is_accepted_without_traffic_light_sync(use_sumo, tls_manager):
    # tls_manager only matters with use_sumo; without SUMO the CARLA lights are alone.
    tlp.validate_traffic_light_params(
        True, "", 5.0, 0.6, force_green=True, use_sumo=use_sumo, tls_manager=tls_manager
    )


def test_sumo_tls_manager_is_fine_without_force_green():
    # Publishing the SUMO-driven CARLA states is the main co-simulation use case.
    tlp.validate_traffic_light_params(True, "", 5.0, 0.6, use_sumo=True, tls_manager="sumo")
