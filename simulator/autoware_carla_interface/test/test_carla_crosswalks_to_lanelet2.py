"""Unit tests for tools/carla_crosswalks_to_lanelet2.py; no CARLA or ROS needed."""

import json
import os
import sys
import textwrap

import pytest

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools")
)

import carla_crosswalks_to_lanelet2 as tool  # noqa: E402
import carla_tl_to_lanelet2 as tl_tool  # noqa: E402

# A straight two-lane road along map x in [0, 100]: lanelet 100 runs east (+x) on y in [0, 4],
# lanelet 200 runs west (-x) on y in [-4, 0] (the same map as test_carla_tl_to_lanelet2.py).
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


def _carla_polygon(map_corners):
    """Map-frame corners -> closed CARLA polygon (carla_y = -map_y)."""
    points = [[x, -y, 0.0] for x, y in map_corners]
    return points + [points[0]]


# Across the road at x = 50: 2.5 m wide in x, from y = -5 to y = 5 (10 m long)
_ACROSS = _carla_polygon([(50.0, -5.0), (52.5, -5.0), (52.5, 5.0), (50.0, 5.0)])
# Beside the road (y = 10 to 20): overlaps no road lanelet
_OFF_ROAD = _carla_polygon([(20.0, 10.0), (22.5, 10.0), (22.5, 20.0), (20.0, 20.0)])


@pytest.fixture
def map_path(tmp_path):
    path = tmp_path / "Town99" / "lanelet2_map.osm"
    path.parent.mkdir()
    path.write_text(_MAP, encoding="utf-8")
    return path


def test_split_polygons():
    assert [len(p) for p in tool.split_polygons(_ACROSS + _OFF_ROAD)] == [5, 5]
    with pytest.raises(tl_tool.ConversionError):
        tool.split_polygons(_ACROSS + _OFF_ROAD[:3])


def test_bounds_are_the_long_sides_with_left_on_the_left():
    left, right = tool.crosswalk_bounds(_ACROSS)
    # The long sides run along y; both the same way
    assert {round(left[0][0], 3), round(right[0][0], 3)} == {50.0, 52.5}
    assert (left[1][1] - left[0][1]) * (right[1][1] - right[0][1]) > 0
    direction = (left[1][0] - left[0][0], left[1][1] - left[0][1])
    offset = (left[0][0] - right[0][0], left[0][1] - right[0][1])
    # left minus right points to the left of the direction (positive cross product)
    assert direction[0] * offset[1] - direction[1] * offset[0] > 0


def test_corners_other_than_four_are_refused():
    with pytest.raises(tl_tool.ConversionError):
        tool.crosswalk_bounds(_ACROSS[:3] + [_ACROSS[0]])


def test_crosswalk_lanelet_is_added_over_both_lanes(map_path):
    lanelet_map = tl_tool.Lanelet2Map.load(map_path)
    report = tool.add_crosswalks(lanelet_map, {"map": "Town99", "points": _ACROSS})
    assert len(report.crosswalks) == 1
    lanelet_id, length, width, roads = report.crosswalks[0]
    assert lanelet_id > 200
    assert (round(length, 3), round(width, 3)) == (10.0, 2.5)
    assert roads == [100, 200]
    rel = next(r for r in lanelet_map.root.findall("relation") if r.get("id") == str(lanelet_id))
    tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
    assert tags == tool.CROSSWALK_TAGS
    roles = sorted(m.get("role") for m in rel.findall("member"))
    assert roles == ["left", "right"]


def test_a_map_with_crosswalks_is_refused(map_path, tmp_path):
    out = tmp_path / "Town99" / "with_crosswalk.osm"
    data = tmp_path / "cw.json"
    data.write_text(json.dumps({"map": "Town99", "points": _ACROSS}), encoding="utf-8")
    assert tool.main([str(map_path), "--from-json", str(data), "--output", str(out)]) == 0
    again = tmp_path / "Town99" / "again.osm"
    assert tool.main([str(out), "--from-json", str(data), "--output", str(again)]) == 1


def test_an_off_road_crosswalk_is_reported(map_path, tmp_path):
    data = tmp_path / "cw.json"
    data.write_text(json.dumps({"map": "Town99", "points": _ACROSS + _OFF_ROAD}), encoding="utf-8")
    out = tmp_path / "Town99" / "with_crosswalk.osm"
    assert tool.main([str(map_path), "--from-json", str(data), "--output", str(out)]) == 2
    lanelet_map = tl_tool.Lanelet2Map.load(out)
    crosswalks = [
        rel
        for rel in lanelet_map.root.findall("relation")
        if {t.get("k"): t.get("v") for t in rel.findall("tag")}.get("subtype") == "crosswalk"
    ]
    assert len(crosswalks) == 2


def test_stop_lines_are_not_put_on_crosswalk_lanelets(map_path):
    # A crosswalk-subtype lanelet along lanelet 100 (y in [2, 4]) whose centre line is nearer to
    # the point than the road's: the traffic-light tool must still pick road lanelet 100
    lanelet_map = tl_tool.Lanelet2Map.load(map_path)
    builder = tl_tool._OsmBuilder(lanelet_map, first_free_id=1000)
    left = builder.add_way([(0.0, 4.0, 0.0), (100.0, 4.0, 0.0)], tool.BOUND_TAGS)
    right = builder.add_way([(0.0, 2.0, 0.0), (100.0, 2.0, 0.0)], tool.BOUND_TAGS)
    builder.add_lanelet(1100, left, right, tool.CROSSWALK_TAGS)
    builder.finish()
    reloaded = tl_tool.Lanelet2Map(lanelet_map.tree)
    assert "1100" in reloaded.lanelets
    assert reloaded.lanelet_at((50.0, 3.0), (1.0, 0.0)) == "100"
