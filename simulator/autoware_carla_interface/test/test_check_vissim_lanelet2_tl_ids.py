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
Unit tests for tools/check_vissim_lanelet2_tl_ids.py (ROS- and CARLA-free).

Run with:
    python3 -m pytest test/test_check_vissim_lanelet2_tl_ids.py
"""

import json
import os
import sys

import pytest

_TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools")
sys.path.insert(0, _TOOLS)
import check_vissim_lanelet2_tl_ids as tool  # noqa: E402

# Two intersections. Intersection A: lights 1 and 2 (an eastbound and a westbound approach),
# intersection B: light 3. Coordinates are Vissim / map; CARLA y is the negative.
#   light 1: eastbound (CARLA yaw 0), light at map (10, -2), stop line x = 2 (y -4..0)
#   light 2: westbound (CARLA yaw 180), light at map (-10, 2), stop line x = -2 (y 0..4)
#   light 3: eastbound, light at map (110, -2), stop line x = 102
_LIGHTS = {
    "1": ((10.0, -2.0), 0.0, (2.0, -4.0), (2.0, 0.0)),
    "2": ((-10.0, 2.0), 180.0, (-2.0, 0.0), (-2.0, 4.0)),
    "3": ((110.0, -2.0), 0.0, (102.0, -4.0), (102.0, 0.0)),
}
_GROUPS = {"1": ["1", "2"], "2": ["1", "2"], "3": ["3"]}


def _carla_json(tmp_path, with_group=True):
    lights = []
    for landmark, ((x, y), yaw, _a, _b) in _LIGHTS.items():
        light = {
            "opendrive_id": landmark,
            "location": [x, -y, 0.0],
            "boxes": [],
            "stops": [{"x": x - 8.0 if yaw == 0 else x + 8.0, "y": -y, "z": 0.0, "yaw": yaw}],
        }
        if with_group:
            light["group"] = _GROUPS[landmark]
        lights.append(light)
    path = tmp_path / "carla_tl.json"
    path.write_text(json.dumps({"map": "Carla/Maps/TownX", "traffic_lights": lights}))
    return str(path)


def _osm(tmp_path, landmarks=("1", "2", "3"), extra_stop_for=None):
    """Lanelet2 map: one regulatory element per light (+ one sharing its heads if asked)."""
    nodes, ways, relations = [], [], []
    next_id = [1000]

    def new_id():
        next_id[0] += 1
        return next_id[0]

    def node(x, y):
        node_id = new_id()
        nodes.append(
            f'<node id="{node_id}" lat="0" lon="0"><tag k="local_x" v="{x}"/>'
            f'<tag k="local_y" v="{y}"/></node>'
        )
        return node_id

    def way(points, kind):
        way_id = new_id()
        refs = "".join(f'<nd ref="{node(x, y)}"/>' for x, y in points)
        ways.append(f'<way id="{way_id}">{refs}<tag k="type" v="{kind}"/></way>')
        return way_id

    def regelem(regelem_id, head, a, b):
        line = way([a, b], "stop_line")
        relations.append(
            f'<relation id="{regelem_id}"><member type="way" ref="{head}" role="refers"/>'
            f'<member type="way" ref="{line}" role="ref_line"/>'
            '<tag k="type" v="regulatory_element"/><tag k="subtype" v="traffic_light"/></relation>'
        )

    for landmark in landmarks:
        (x, y), _yaw, a, b = _LIGHTS[landmark]
        head = way([(x, y), (x, y + 0.5)], "traffic_light")
        regelem(int(landmark), head, a, b)
        if landmark == extra_stop_for:
            regelem(5000, head, (a[0] + 0.5, a[1]), (b[0] + 0.5, b[1]))
    path = tmp_path / "map.osm"
    path.write_text("<osm>" + "".join(nodes + ways + relations) + "</osm>")
    return str(path)


def _mapping(tmp_path, data):
    path = tmp_path / "signal_mapping.json"
    path.write_text(json.dumps(data))
    return str(path)


_GOOD_MAPPING = {"1": {"1": [1], "2": [2]}, "2": {"1": [3]}}


def _inpx(tmp_path, heads, pedestrian_links=()):
    """
    Vissim network. Link 11 runs east along y = -2 from x = 0, link 12 west along y = 2 from
    x = 0, link 13 east along y = -2 from x = 100, link 14 (pedestrian area) north at x = 5.
    heads: [(no, link, pos, "SC SG")].
    """
    geometry = {
        "11": [(0.0, -2.0), (20.0, -2.0)],
        "12": [(0.0, 2.0), (-20.0, 2.0)],
        "13": [(100.0, -2.0), (120.0, -2.0)],
        "14": [(5.0, -6.0), (5.0, 6.0)],
    }
    links = []
    for no, points in geometry.items():
        ped = ' isPedArea="true"' if no in pedestrian_links else ""
        pts = "".join(f'<linkPolyPoint x="{x}" y="{y}" zOffset="0"/>' for x, y in points)
        links.append(
            f'<link no="{no}"{ped}><geometry><linkPolyPts>{pts}</linkPolyPts></geometry></link>'
        )
    signal_heads = "".join(
        f'<signalHead no="{no}" lane="{link} 1" pos="{pos}" sg="{sg}"/>'
        for no, link, pos, sg in heads
    )
    path = tmp_path / "net.inpx"
    path.write_text(
        f"<network><links>{''.join(links)}</links>"
        f"<signalHeads>{signal_heads}</signalHeads></network>"
    )
    return str(path)


# Heads 3 m past each stop line: light 1 stop line x = 2 -> head at x = 5 (link 11 pos 5),
# light 2 stop line x = -2 (westbound) -> head at x = -5 (link 12 pos 5), light 3 -> x = 105.
_GOOD_HEADS = [(1, 11, 5.0, "1 1"), (2, 12, 5.0, "1 2"), (3, 13, 5.0, "2 1")]


def test_consistent_inputs_are_ok(tmp_path, capsys):
    ok = tool.check(
        _mapping(tmp_path, _GOOD_MAPPING),
        _osm(tmp_path),
        _carla_json(tmp_path),
        _inpx(tmp_path, _GOOD_HEADS),
    )
    out = capsys.readouterr().out
    assert ok, out
    assert "3 landmarks in 3 signal groups of 2 signal controllers" in out
    assert "2 intersections" in out
    assert out.rstrip().endswith("OK")


def test_landmarks_on_one_side_only(tmp_path, capsys):
    mapping = {"1": {"1": [1], "2": [2, 7]}, "2": {"1": [3]}}
    assert not tool.check(_mapping(tmp_path, mapping), _osm(tmp_path, ("1", "2")))
    out = capsys.readouterr().out
    assert "NG only in signal_mapping.json: ['3', '7']" in out


def test_carla_light_missing_from_both(tmp_path, capsys):
    mapping = {"1": {"1": [1], "2": [2]}}
    osm = _osm(tmp_path, ("1", "2"))
    assert not tool.check(_mapping(tmp_path, mapping), osm, _carla_json(tmp_path))
    out = capsys.readouterr().out
    assert "NG in CARLA but not in signal_mapping.json: ['3']" in out
    assert "NG in CARLA but not in Lanelet2: ['3']" in out


def test_landmark_in_two_signal_groups(tmp_path, capsys):
    mapping = {"1": {"1": [1, 2], "2": [2]}, "2": {"1": [3]}}
    assert not tool.check(_mapping(tmp_path, mapping), _osm(tmp_path))
    assert "NG in several signal groups: ['2']" in capsys.readouterr().out


def test_controller_with_two_intersections(tmp_path, capsys):
    # Today's Town01 layout: one signal controller for every intersection.
    mapping = {"1": {"1": [1, 3], "2": [2]}}
    assert not tool.check(_mapping(tmp_path, mapping), _osm(tmp_path), _carla_json(tmp_path))
    out = capsys.readouterr().out
    assert "NG signal controllers with lights of several intersections: " in out
    assert "['SC 1 (2 intersections)']" in out


def test_intersection_split_over_two_controllers(tmp_path, capsys):
    mapping = {"1": {"1": [1]}, "2": {"1": [2]}, "3": {"1": [3]}}
    assert not tool.check(_mapping(tmp_path, mapping), _osm(tmp_path), _carla_json(tmp_path))
    assert "NG intersections split over several signal controllers: ['1/2 (SC [1, 2])']" in (
        capsys.readouterr().out
    )


def test_carla_json_without_group_skips_the_intersection_checks(tmp_path, capsys):
    mapping = {"1": {"1": [1, 3], "2": [2]}}
    carla = _carla_json(tmp_path, with_group=False)
    ok = tool.check(_mapping(tmp_path, mapping), _osm(tmp_path), carla)
    out = capsys.readouterr().out
    assert ok, out
    assert "WARNING: --carla-json has no 'group'" in out


def test_regulatory_elements_sharing_heads_are_one_light(tmp_path, capsys):
    osm = _osm(tmp_path, extra_stop_for="2")
    assert tool.check(_mapping(tmp_path, _GOOD_MAPPING), osm, _carla_json(tmp_path))
    assert "3 traffic lights in 4 regulatory elements" in capsys.readouterr().out


def test_group_id_offset(tmp_path, capsys):
    lights, count = tool.lanelet2_traffic_lights(_osm(tmp_path), group_id_offset=1)
    assert set(lights) == {"0", "1", "2"} and count == 3


def test_signal_head_group_differs_from_mapping(tmp_path, capsys):
    heads = [(1, 11, 5.0, "1 2"), (2, 12, 5.0, "1 2"), (3, 13, 5.0, "2 1")]
    assert not tool.check(
        _mapping(tmp_path, _GOOD_MAPPING),
        _osm(tmp_path),
        _carla_json(tmp_path),
        _inpx(tmp_path, heads)
    )
    assert "NG signal heads whose signal group differs from signal_mapping.json: " in (
        capsys.readouterr().out
    )


def test_heads_of_one_landmark_in_two_signal_groups(tmp_path, capsys):
    heads = _GOOD_HEADS + [(4, 11, 6.0, "1 2")]
    assert not tool.check(
        _mapping(tmp_path, _GOOD_MAPPING),
        _osm(tmp_path),
        _carla_json(tmp_path),
        _inpx(tmp_path, heads)
    )
    out = capsys.readouterr().out
    assert "NG landmarks whose vehicle signal heads are in different signal groups: " in out
    assert "['1 [(1, 1), (1, 2)]']" in out


def test_signal_head_too_far_and_landmark_without_head(tmp_path, capsys):
    heads = [(1, 11, 5.0, "1 1"), (2, 12, 5.0, "1 2"), (3, 13, 20.0, "2 1")]  # head 3 at x = 120
    assert not tool.check(
        _mapping(tmp_path, _GOOD_MAPPING),
        _osm(tmp_path),
        _carla_json(tmp_path),
        _inpx(tmp_path, heads),
        max_distance=5.0,
    )
    out = capsys.readouterr().out
    assert "NG vehicle signal heads farther than 5.0 m: ['head 3 (10.0 m from 3)']" in out
    assert "NG mapped landmarks without a vehicle signal head: ['3']" in out


def test_pedestrian_heads_are_left_out_of_the_landmark_checks(tmp_path, capsys):
    # Head 9 sits on the pedestrian link next to light 1, in SG 2 (the other approach's group):
    # as a vehicle head it would put landmark 1 in two signal groups.
    heads = _GOOD_HEADS + [(9, 14, 6.0, "1 2")]
    ok = tool.check(
        _mapping(tmp_path, _GOOD_MAPPING), _osm(tmp_path), _carla_json(tmp_path),
        _inpx(tmp_path, heads, pedestrian_links=("14",)),
    )
    out = capsys.readouterr().out
    assert ok, out
    assert "3 vehicle and 1 pedestrian signal heads" in out
    assert "pedestrian head 9 (link 14): SC 1 SG 2, nearest light 1" in out


def test_pedestrian_head_in_another_intersections_controller(tmp_path, capsys):
    heads = _GOOD_HEADS + [(9, 14, 6.0, "2 1")]
    assert not tool.check(
        _mapping(tmp_path, _GOOD_MAPPING), _osm(tmp_path), _carla_json(tmp_path),
        _inpx(tmp_path, heads, pedestrian_links=("14",)),
    )
    out = capsys.readouterr().out
    assert "NG pedestrian signal heads in another intersection's controller: " in out
    assert "['head 9 (SC 2, light 1)']" in out


def test_stop_offset_is_signed_along_the_direction_of_travel(tmp_path, capsys):
    # Head 1 is 3 m past its stop line (eastbound), head 2 3 m past (westbound), head 3 1 m
    # before its line (link 13 pos 1 -> x = 101, line at x = 102).
    heads = [(1, 11, 5.0, "1 1"), (2, 12, 5.0, "1 2"), (3, 13, 1.0, "2 1")]
    assert tool.check(
        _mapping(tmp_path, _GOOD_MAPPING),
        _osm(tmp_path),
        _carla_json(tmp_path),
        _inpx(tmp_path, heads),
        report_stop_offset=True,
    )
    out = capsys.readouterr().out
    assert "1: head   1 link    11 pos    5.00  SC  1 SG 1   +3.00 m *" in out
    assert "2: head   2 link    12 pos    5.00  SC  1 SG 2   +3.00 m *" in out
    assert "3: head   3 link    13 pos    1.00  SC  2 SG 1   -1.00 m *" in out
    assert "range -1.00 .. +3.00 m, 3 of 3 outside 0..2 m" in out


def test_stop_offset_uses_the_nearest_stop_line_of_the_light():
    light = {"stops": [{"yaw": 0.0}]}
    lines = [((2.0, -4.0), (2.0, 0.0)), ((30.0, -4.0), (30.0, 0.0))]
    assert tool.stop_offset((29.0, -2.0), light, lines) == pytest.approx(-1.0)
    assert tool.stop_offset((29.0, -2.0), light, []) is None


def test_main_exit_status(tmp_path):
    mapping = _mapping(tmp_path, _GOOD_MAPPING)
    osm = _osm(tmp_path)
    assert tool.main([mapping, osm, "--carla-json", _carla_json(tmp_path)]) == 0
    assert tool.main([_mapping(tmp_path, {"1": {"1": [1, 2, 3]}}), osm, "--carla-json",
                      _carla_json(tmp_path)]) == 1
