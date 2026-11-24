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
"""tools/add_lanelet2_line_types.py: 車線の境界線に線の種類を付け、車線変更できるようにする。"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

lanelet2 = pytest.importorskip("lanelet2")

TOOL = Path(__file__).resolve().parents[1] / "tools" / "add_lanelet2_line_types.py"
spec = importlib.util.spec_from_file_location("add_lanelet2_line_types", TOOL)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

DEG = 1.0 / 111320.0


def write_map(path, lanelets, lines):
    """lines: {way id: [(x, y), ...]}、lanelets: {id: (left way, right way)} の地図(境界線に種類なし)。"""
    root = ET.Element("osm", version="0.6")
    node_ids = {}
    for way_id, points in lines.items():
        way = ET.SubElement(root, "way", id=str(way_id), version="1")
        for x, y in points:
            if (x, y) not in node_ids:
                node_ids[(x, y)] = len(node_ids) + 1
                node = ET.Element("node", id=str(node_ids[(x, y)]), lat=str(y * DEG), lon=str(x * DEG), version="1")
                root.insert(0, node)
            ET.SubElement(way, "nd", ref=str(node_ids[(x, y)]))
    for lanelet_id, (left, right) in lanelets.items():
        relation = ET.SubElement(root, "relation", id=str(lanelet_id), version="1")
        ET.SubElement(relation, "member", type="way", role="left", ref=str(left))
        ET.SubElement(relation, "member", type="way", role="right", ref=str(right))
        ET.SubElement(relation, "tag", k="type", v="lanelet")
        ET.SubElement(relation, "tag", k="subtype", v="road")
    ET.ElementTree(root).write(path, encoding="UTF-8", xml_declaration=True)


def way_types(path):
    root = ET.parse(path).getroot()
    types = {}
    for way in root.findall("way"):
        tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        if "type" in tags:
            types[int(way.get("id"))] = (tags["type"], tags.get("subtype"))
    return types


def routing_graph(path):
    return tool.load_graph(path)


def test_two_lanes_of_the_same_direction_get_a_dashed_line(tmp_path):
    """同じ向きの 2 車線の間は破線(車線変更できる)、外側は実線。"""
    src, out = tmp_path / "in.osm", tmp_path / "out.osm"
    write_map(src, {100: (1, 2), 101: (2, 3)},
              {1: [(0, 7), (30, 7)], 2: [(0, 3.5), (30, 3.5)], 3: [(0, 0), (30, 0)]})
    lanelet_map, graph = routing_graph(src)
    assert graph.right(lanelet_map.laneletLayer[100]) is None     # 種類なし: 車線変更できない
    assert tool.main([str(src), "--check"]) == 1
    assert tool.main([str(src), str(out)]) == 0
    assert way_types(out) == {1: tool.SOLID, 2: tool.DASHED, 3: tool.SOLID}
    lanelet_map, graph = routing_graph(out)
    assert graph.right(lanelet_map.laneletLayer[100]).id == 101
    assert tool.main([str(out), "--check"]) == 0


def test_junction_lanes_and_typed_lines_are_not_dashed(tmp_path):
    """分かれた先の車線(交差点の中)と、その隣の車線の間は実線。もとから種類のある線は変えない。"""
    src, out = tmp_path / "in.osm", tmp_path / "out.osm"
    write_map(src, {100: (1, 2), 101: (2, 3), 110: (11, 12), 111: (13, 14), 120: (12, 15)},
              {1: [(0, 7), (10, 7)], 2: [(0, 3.5), (10, 3.5)], 3: [(0, 0), (10, 0)],
               # 100 は 110(直進)と 111(左へ分かれる)へ続く。101 は 120 へ続き、120 は 110 と隣り合う
               11: [(10, 7), (20, 7)], 12: [(10, 3.5), (20, 3.5)], 15: [(10, 0), (20, 0)],
               13: [(10, 7), (18, 15)], 14: [(10, 3.5), (20, 12)]})
    root = ET.parse(src).getroot()
    ET.SubElement(root.find("way[@id='1']"), "tag", k="type", v="road_border")
    ET.ElementTree(root).write(src, encoding="UTF-8", xml_declaration=True)
    lanelet_map, graph = routing_graph(src)
    assert tool.is_junction(graph, lanelet_map.laneletLayer[110])
    assert not tool.is_junction(graph, lanelet_map.laneletLayer[100])
    assert tool.main([str(src), str(out)]) == 0
    types = way_types(out)
    assert types[1] == ("road_border", None)      # もとから種類がある
    assert types[2] == tool.DASHED                # 100 と 101 の間
    assert types[12] == tool.SOLID                # 110(分かれた先)と 120 の間
