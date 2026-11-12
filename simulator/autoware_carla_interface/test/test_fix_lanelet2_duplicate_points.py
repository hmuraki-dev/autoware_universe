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

"""Unit tests for tools/fix_lanelet2_duplicate_points.py; no CARLA, SUMO or ROS needed."""

import os
import sys
import textwrap
import xml.etree.ElementTree as ET

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools")
)

import fix_lanelet2_duplicate_points as tool  # noqa: E402


def _node(node_id, x, y):
    return (
        f'<node id="{node_id}" lat="0" lon="0">'
        f'<tag k="local_x" v="{x}"/><tag k="local_y" v="{y}"/></node>'
    )


# Way 10 (left bound of lanelet 100) starts with 1 and 2 at the same position, like Town05.
# Node 1 is also the end of way 11, the left bound of the preceding lanelet 101.
# Way 12 (right bound of lanelet 100) has 6 and 7 at the same position in the middle.
_MAP = textwrap.dedent(f"""\
    <osm>
      {_node(1, 0, 0)}{_node(2, 0, 0)}{_node(3, 1, 0)}{_node(4, 2, 0)}
      {_node(5, 0, 3)}{_node(6, 1, 3)}{_node(7, 1, 3)}{_node(8, 2, 3)}{_node(9, -1, 0)}
      <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/></way>
      <way id="11"><nd ref="9"/><nd ref="1"/></way>
      <way id="12"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/></way>
      <relation id="100">
        <member type="way" ref="10" role="left"/><member type="way" ref="12" role="right"/>
        <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
      </relation>
      <relation id="101">
        <member type="way" ref="11" role="left"/><member type="way" ref="12" role="right"/>
        <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
      </relation>
    </osm>
""")


def _refs(root, way_id):
    return [nd.get("ref") for nd in root.find(f"way[@id='{way_id}']").findall("nd")]


def test_keeps_the_way_end_point_and_drops_the_duplicate():
    root = ET.fromstring(_MAP)
    removed = tool.fix_duplicate_points(root)
    assert removed == {"10": ["2"], "12": ["7"]}
    assert _refs(root, "10") == ["1", "3", "4"]
    assert _refs(root, "11") == ["9", "1"]
    assert _refs(root, "12") == ["5", "6", "8"]
    # the nodes stay in the map
    assert root.find("node[@id='2']") is not None


def test_reports_the_lanelets_using_the_fixed_ways():
    root = ET.fromstring(_MAP)
    removed = tool.fix_duplicate_points(root)
    assert tool.lanelets_using(root, set(removed)) == ["100", "101"]


def test_check_mode_and_rerun_on_fixed_map(tmp_path):
    src = tmp_path / "map.osm"
    out = tmp_path / "fixed.osm"
    src.write_text(_MAP)
    assert tool.main([str(src), "--check"]) == 1
    assert tool.main([str(src), str(out)]) == 0
    assert tool.main([str(out), "--check"]) == 0
