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

"""Unit tests for tools/carla_tl_to_lanelet2.py (docs T2); no CARLA or ROS needed."""

import json
import math
import os
import sys
import textwrap
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools")
)

from autoware_carla_interface.modules.traffic_light_matcher import (  # noqa: E402
    load_map_traffic_lights,
)
import carla_tl_to_lanelet2 as tool  # noqa: E402

# A straight two-lane road along map x in [0, 100]: lanelet 100 runs east (+x) on
# y in [0, 4], lanelet 200 runs west (-x) on y in [-4, 0]. lat/lon follow the same scale
# as the CARLA maps (lon = 9e-6 * x, lat = 9e-6 * y).
_MAP = textwrap.dedent("""\
    <?xml version="1.0"?>
    <osm version="0.6" generator="lanelet2">
      <node id="1" lat="0.0" lon="0.0"><tag k="ele" v="0"/><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
      <node id="2" lat="0.0" lon="0.0009"><tag k="ele" v="0"/><tag k="local_x" v="100"/><tag k="local_y" v="0"/></node>
      <node id="3" lat="0.000036" lon="0.0"><tag k="ele" v="0"/><tag k="local_x" v="0"/><tag k="local_y" v="4"/></node>
      <node id="4" lat="0.000036" lon="0.0009"><tag k="ele" v="0"/><tag k="local_x" v="100"/><tag k="local_y" v="4"/></node>
      <node id="5" lat="-0.000036" lon="0.0"><tag k="ele" v="0"/><tag k="local_x" v="0"/><tag k="local_y" v="-4"/></node>
      <node id="6" lat="-0.000036" lon="0.0009"><tag k="ele" v="0"/><tag k="local_x" v="100"/><tag k="local_y" v="-4"/></node>
      <way id="10"><nd ref="3"/><nd ref="4"/></way>
      <way id="11"><nd ref="1"/><nd ref="2"/></way>
      <way id="20"><nd ref="6"/><nd ref="5"/></way>
      <way id="21"><nd ref="2"/><nd ref="1"/></way>
      <relation id="100">
        <member type="way" ref="10" role="left"/>
        <member type="way" ref="11" role="right"/>
        <tag k="subtype" v="road"/>
        <tag k="type" v="lanelet"/>
      </relation>
      <relation id="200">
        <member type="way" ref="20" role="left"/>
        <member type="way" ref="21" role="right"/>
        <tag k="subtype" v="road"/>
        <tag k="type" v="lanelet"/>
      </relation>
    </osm>
    """)


def _box(x, y, z=2.6, yaw=90.0):
    return {"x": x, "y": y, "z": z, "extent": [0.225, 0.139, 0.61], "yaw": yaw}


def _stop(x, y, yaw, z=0.0):
    return {"x": x, "y": y, "z": z, "yaw": yaw, "lane_width": 4.0, "road_id": 1, "lane_id": 1}


def _light(opendrive_id, boxes, stops, actor_id=1):
    return {
        "opendrive_id": opendrive_id,
        "actor_id": actor_id,
        "location": [boxes[0]["x"], boxes[0]["y"], 0.0] if boxes else [0.0, 0.0, 0.0],
        "boxes": boxes,
        "stops": stops,
    }


# Eastbound: map (50, 2) is CARLA (50, -2), map +x is CARLA yaw 0.
_EAST_STOP = _stop(50.0, -2.0, 0.0)
# Westbound: map (50, -2) is CARLA (50, 2), map -x is CARLA yaw 180.
_WEST_STOP = _stop(50.0, 2.0, 180.0)


def _load(tmp_path, text=_MAP):
    path = tmp_path / "lanelet2_map.osm"
    path.write_text(text)
    return tool.Lanelet2Map.load(str(path))


def _relation(root, rel_id):
    return next(r for r in root.findall("relation") if r.get("id") == str(rel_id))


def _members(rel):
    return [(m.get("type"), m.get("ref"), m.get("role")) for m in rel.findall("member")]


def _tags(element):
    return {t.get("k"): t.get("v") for t in element.findall("tag")}


def _way_points(root, way_id):
    nodes = {n.get("id"): n for n in root.findall("node")}
    way = next(w for w in root.findall("way") if w.get("id") == str(way_id))
    return [
        tuple(float(_tags(nodes[nd.get("ref")])[k]) for k in ("local_x", "local_y", "ele"))
        for nd in way.findall("nd")
    ]


# --- geometry ---------------------------------------------------------------------------


def test_head_line_is_bottom_edge_through_box_centre():
    (a, b), height = tool.head_line(_box(10.0, -5.0, z=2.6, yaw=90.0))
    # yaw 90 in CARLA points to map -y; the edge spans the 0.45 m width of the head.
    assert height == pytest.approx(1.22)
    assert a[2] == b[2] == pytest.approx(2.6 - 0.61)
    assert ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) == pytest.approx((10.0, 5.0))
    assert math.dist(a[:2], b[:2]) == pytest.approx(0.45)
    assert a[0] == pytest.approx(b[0])


def test_stop_line_spans_the_lane_perpendicular_to_travel():
    left, right = tool.stop_line(_EAST_STOP)
    assert left[:2] == pytest.approx((50.0, 4.0))
    assert right[:2] == pytest.approx((50.0, 0.0))


def test_lanelet_lookup_uses_position_and_direction(tmp_path):
    lanelet_map = _load(tmp_path)
    assert lanelet_map.lanelet_at((50.0, 2.0), (1.0, 0.0)) == "100"
    assert lanelet_map.lanelet_at((50.0, -2.0), (-1.0, 0.0)) == "200"
    # Right place, wrong direction, or outside every lanelet: no match.
    assert lanelet_map.lanelet_at((50.0, 2.0), (-1.0, 0.0)) is None
    assert lanelet_map.lanelet_at((50.0, 9.0), (1.0, 0.0)) is None


def test_lat_lon_scale_is_fitted_from_the_map(tmp_path):
    lanelet_map = _load(tmp_path)
    assert lanelet_map.lon_per_x == pytest.approx(9e-6)
    assert lanelet_map.lat_per_y == pytest.approx(9e-6)


# --- conversion -------------------------------------------------------------------------


def test_adds_regulatory_element_with_opendrive_id_and_attaches_it(tmp_path):
    lanelet_map = _load(tmp_path)
    data = {"traffic_lights": [_light("360", [_box(55.0, -6.0)], [_EAST_STOP])]}
    report = tool.add_traffic_lights(lanelet_map, data)
    root = lanelet_map.root

    assert report.regulatory_elements == [(360, "360", 100)]
    rel = _relation(root, 360)
    assert _tags(rel) == {"type": "regulatory_element", "subtype": "traffic_light"}
    members = _members(rel)
    assert [role for _, _, role in members] == ["refers", "ref_line"]
    head_way, stop_way = members[0][1], members[1][1]

    head_way_el = next(w for w in root.findall("way") if w.get("id") == head_way)
    assert _tags(head_way_el) == {
        "type": "traffic_light",
        "subtype": "red_yellow_green",
        "height": "1.220",
    }
    assert _tags(next(w for w in root.findall("way") if w.get("id") == stop_way)) == {
        "type": "stop_line"
    }
    stop_points = _way_points(root, stop_way)
    assert [p[:2] for p in stop_points] == [pytest.approx((50.0, 4.0)), pytest.approx((50.0, 0.0))]

    # Lanelet 100 refers to it, before its tags; lanelet 200 is untouched.
    lanelet = _relation(root, 100)
    assert ("relation", "360", "regulatory_element") in _members(lanelet)
    assert [c.tag for c in lanelet][-2:] == ["tag", "tag"]
    assert all(m[2] != "regulatory_element" for m in _members(_relation(root, 200)))


def test_new_ids_are_unique_and_osm_order_is_kept(tmp_path):
    lanelet_map = _load(tmp_path)
    data = {
        "traffic_lights": [
            _light("360", [_box(55.0, -6.0)], [_EAST_STOP], actor_id=1),
            _light("361", [_box(45.0, 6.0, yaw=270.0)], [_WEST_STOP], actor_id=2),
        ]
    }
    tool.add_traffic_lights(lanelet_map, data)
    root = lanelet_map.root
    ids = [int(e.get("id")) for e in root]
    assert len(ids) == len(set(ids))
    order = [e.tag for e in root]
    assert order == sorted(order, key=["node", "way", "relation"].index)
    # New nodes carry lat/lon from the fitted scale.
    node = next(n for n in root.findall("node") if int(n.get("id")) > 361)
    tags = _tags(node)
    assert float(node.get("lon")) == pytest.approx(float(tags["local_x"]) * 9e-6)
    assert float(node.get("lat")) == pytest.approx(float(tags["local_y"]) * 9e-6)


def test_group_id_offset_and_collision(tmp_path):
    data = {"traffic_lights": [_light("100", [_box(55.0, -6.0)], [_EAST_STOP])]}
    # OpenDRIVE id 100 collides with lanelet 100 (lanelet2 ids are global).
    with pytest.raises(tool.ConversionError, match="already used"):
        tool.add_traffic_lights(_load(tmp_path), data)
    lanelet_map = _load(tmp_path)
    report = tool.add_traffic_lights(lanelet_map, data, group_id_offset=10000)
    assert report.regulatory_elements == [(10100, "100", 100)]


def test_refuses_a_map_that_already_has_traffic_lights(tmp_path):
    lanelet_map = _load(tmp_path)
    data = {"traffic_lights": [_light("360", [_box(55.0, -6.0)], [_EAST_STOP])]}
    tool.add_traffic_lights(lanelet_map, data)
    with pytest.raises(tool.ConversionError, match="already has traffic-light"):
        tool.add_traffic_lights(lanelet_map, data)


def test_unattached_and_skipped_lights_are_reported(tmp_path):
    lanelet_map = _load(tmp_path)
    data = {
        "traffic_lights": [
            # Stop position off the road.
            _light("360", [_box(55.0, -6.0)], [_stop(50.0, -20.0, 0.0)], actor_id=1),
            _light("abc", [_box(55.0, -6.0)], [_EAST_STOP], actor_id=2),
            _light("362", [], [_EAST_STOP], actor_id=3),
        ]
    }
    report = tool.add_traffic_lights(lanelet_map, data)
    assert report.regulatory_elements == []
    assert [(o, i) for o, i, _ in report.unattached] == [("360", 0)]
    assert sorted(o for o, _ in report.skipped) == ["362", "abc"]


def test_stop_line_forward_offset_moves_the_line_along_the_lane(tmp_path):
    left, right = tool.stop_line(_WEST_STOP, forward_offset=6.0)
    # Westbound: forward is map -x.
    assert left[:2] == pytest.approx((44.0, -4.0))
    assert right[:2] == pytest.approx((44.0, 0.0))

    lanelet_map = _load(tmp_path)
    data = {"traffic_lights": [_light("360", [_box(55.0, -6.0)], [_EAST_STOP])]}
    report = tool.add_traffic_lights(lanelet_map, data, stop_line_forward_offset=6.0)
    assert report.regulatory_elements == [(360, "360", 100)]
    stop_way = _members(_relation(lanelet_map.root, 360))[1][1]
    assert [p[:2] for p in _way_points(lanelet_map.root, stop_way)] == [
        pytest.approx((56.0, 4.0)),
        pytest.approx((56.0, 0.0)),
    ]


def test_stop_line_moved_out_of_its_lanelet_is_not_added(tmp_path):
    # Lanelet 100 ends at x = 100; 60 m forward from x = 50 leaves it.
    lanelet_map = _load(tmp_path)
    data = {"traffic_lights": [_light("360", [_box(55.0, -6.0)], [_EAST_STOP])]}
    report = tool.add_traffic_lights(lanelet_map, data, stop_line_forward_offset=60.0)
    assert report.regulatory_elements == []
    assert [(o, i) for o, i, _ in report.unattached] == [("360", 0)]
    assert all(
        m[2] != "regulatory_element" for m in _members(_relation(lanelet_map.root, 100))
    )


def test_several_stop_positions_get_one_element_each(tmp_path):
    # Only the first attached stop carries the OpenDRIVE id; the head is shared.
    lanelet_map = _load(tmp_path)
    light = _light("360", [_box(55.0, -6.0)], [_stop(50.0, -20.0, 0.0), _EAST_STOP, _WEST_STOP])
    report = tool.add_traffic_lights(lanelet_map, {"traffic_lights": [light]})
    assert [(r[0] == 360, r[2]) for r in report.regulatory_elements] == [(True, 100), (False, 200)]
    second = _relation(lanelet_map.root, report.regulatory_elements[1][0])
    first = _relation(lanelet_map.root, 360)
    assert _members(first)[0] == _members(second)[0]


def test_output_matches_the_bridge_position_matcher(tmp_path):
    # The written head sits exactly where the bridge puts the CARLA head.
    lanelet_map = _load(tmp_path)
    data = {"traffic_lights": [_light("360", [_box(55.0, -6.0)], [_EAST_STOP])]}
    tool.add_traffic_lights(lanelet_map, data)
    out = tmp_path / "out.osm"
    tool.write_map(lanelet_map, str(out))
    heads = load_map_traffic_lights(str(out))
    assert list(heads.head_groups.values()) == [{360}]
    assert list(heads.head_positions.values())[0] == pytest.approx((55.0, 6.0))


# --- command line -----------------------------------------------------------------------


def test_main_writes_next_to_the_input_and_refuses_to_overwrite(tmp_path, capsys):
    map_dir = tmp_path / "Town01"
    map_dir.mkdir()
    (map_dir / "lanelet2_map.osm").write_text(_MAP)
    data = {
        "map": "Carla/Maps/Town01",
        "traffic_lights": [_light("360", [_box(55.0, -6.0)], [_EAST_STOP])],
    }
    js = tmp_path / "tl.json"
    js.write_text(json.dumps(data))

    argv = [str(map_dir / "lanelet2_map.osm"), "--from-json", str(js)]
    assert tool.main(argv) == 0
    out = map_dir / "lanelet2_map_with_tl.osm"
    assert out.exists()
    assert (map_dir / "lanelet2_map.osm").read_text() == _MAP
    ET.parse(str(out))
    assert "WARNING" not in capsys.readouterr().out

    with pytest.raises(SystemExit):
        tool.main(argv)
    assert tool.main(argv + ["--force"]) == 0


def test_main_returns_error_on_id_collision(tmp_path):
    (tmp_path / "lanelet2_map.osm").write_text(_MAP)
    js = tmp_path / "tl.json"
    js.write_text(json.dumps({"traffic_lights": [_light("100", [_box(55.0, -6.0)], [_EAST_STOP])]}))
    assert tool.main([str(tmp_path / "lanelet2_map.osm"), "--from-json", str(js)]) == 1
    assert not (tmp_path / "lanelet2_map_with_tl.osm").exists()
