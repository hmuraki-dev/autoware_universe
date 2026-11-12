#!/usr/bin/env python3
"""
Add CARLA's traffic lights to a Lanelet2 map as Autoware traffic-light regulatory elements.

See docs/CARLA_Autoware_信号状態publish_実装計画_v1.0.md (T2). The stock CARLA Lanelet2 maps
(e.g. ~/autoware_map/Town01/lanelet2_map.osm) contain only road lanelets, so Autoware's
traffic_light module has nothing to stop at and the bridge's position matching
(traffic_light.map_path) has nothing to match against. For every CARLA traffic light this
tool adds:

- one `traffic_light` linestring per light head (get_light_boxes()): the bottom edge of the
  head, tagged with its height,
- one `stop_line` linestring per stop position (get_stop_waypoints()), across the lane,
  optionally moved forward along the lane by --stop-line-forward-offset,
- one `regulatory_element` (subtype=traffic_light) per stop position, referring to the
  heads (role `refers`) and the stop line (role `ref_line`),
- a reference to that regulatory element on the lanelet containing the stop position
  (the lanelet approaching the intersection, matched by position and direction).

The regulatory-element id is the OpenDRIVE signal id (+ --group-id-offset), so the bridge's
OpenDRIVE-id fallback and its position matching both resolve to it.

CARLA's stop positions lie about 7 m before the light heads (6.7-8.9 m in Town01), so Autoware
stops well back from the intersection. --stop-line-forward-offset moves every stop line that
far forward along the lane; a stop line that would leave its lanelet (i.e. enter the
intersection) is not added and is reported as unattached. In Town01 the stop positions lie
7.0-13.1 m before the end of their lanelet, so up to about 6 m keeps every stop line. A light with several
stop positions gets one regulatory element per stop position; only the first one attached
to a lanelet carries the OpenDRIVE id, so such maps need traffic_light.map_path (position matching).

Coordinates: map = (carla_x, -carla_y) (docs T1). New nodes get local_x/local_y/ele tags
and lat/lon from the scale fitted to the input map's existing nodes.

The input map is never modified; the result is written to --output (default:
lanelet2_map_with_tl.osm next to the input). Keep it in the same map directory, because
the CARLA map name is taken from the directory name, and point Autoware at it with
lanelet2_map_file:=lanelet2_map_with_tl.osm.

Usage (CARLA server running with the matching town loaded):
    python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/Town01/lanelet2_map.osm
Offline, from data saved earlier with --dump-json:
    python3 tools/carla_tl_to_lanelet2.py MAP.osm --from-json town01_tl.json
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

# A stop position is attached to a lanelet only if the lane directions agree within this
# angle (cos 45 deg); CARLA and the lanelet run the same way along an approach lane.
_MIN_DIRECTION_DOT = math.cos(math.radians(45.0))


class ConversionError(Exception):
    """The map cannot be extended (e.g. an id collision)."""


# --- CARLA side ---------------------------------------------------------------------------


def collect_carla_traffic_lights(host, port, timeout):
    """Read every traffic light of the loaded CARLA world as plain data (see from_json)."""
    import carla  # pylint: disable=import-outside-toplevel

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    lights = world.get_actors().filter("traffic.traffic_light")
    if len(lights) == 0 and world.get_settings().synchronous_mode:
        # A synchronous world that has not been ticked since it was loaded lists no actors
        # (docs T1). Only tick when nothing else is driving the world.
        print("CARLA world is synchronous and lists no actors yet; ticking it once.")
        world.tick()
        lights = world.get_actors().filter("traffic.traffic_light")

    data = []
    for light in lights:
        data.append(
            {
                "opendrive_id": light.get_opendrive_id(),
                "actor_id": light.id,
                "location": [
                    light.get_location().x,
                    light.get_location().y,
                    light.get_location().z,
                ],
                "boxes": [
                    {
                        "x": b.location.x,
                        "y": b.location.y,
                        "z": b.location.z,
                        "extent": [b.extent.x, b.extent.y, b.extent.z],
                        "yaw": b.rotation.yaw,
                    }
                    for b in light.get_light_boxes()
                ],
                "stops": [
                    {
                        "x": w.transform.location.x,
                        "y": w.transform.location.y,
                        "z": w.transform.location.z,
                        "yaw": w.transform.rotation.yaw,
                        "lane_width": w.lane_width,
                        "road_id": w.road_id,
                        "lane_id": w.lane_id,
                    }
                    for w in light.get_stop_waypoints()
                ],
            }
        )
    return {"map": world.get_map().name, "traffic_lights": data}


# --- geometry -----------------------------------------------------------------------------


def carla_to_map(x, y):
    """CARLA (x, y) -> Autoware map (x, y) (docs T1: y flipped, no origin offset)."""
    return (x, -y)


def carla_yaw_to_map_dir(yaw_deg):
    """Return the unit direction in the map frame of a CARLA yaw [deg]."""
    yaw = math.radians(yaw_deg)
    return (math.cos(yaw), -math.sin(yaw))


def head_line(box):
    """Bottom edge of a light head as ((x, y, z), (x, y, z)) and its height, in the map frame.

    The edge runs along the wider horizontal side of the box, through the box centre, so
    its midpoint is exactly the head position the bridge matches against.
    """
    ex, ey, ez = box["extent"]
    yaw = box["yaw"] if ex >= ey else box["yaw"] + 90.0
    half = max(ex, ey)
    dx, dy = carla_yaw_to_map_dir(yaw)
    cx, cy = carla_to_map(box["x"], box["y"])
    z = box["z"] - ez
    return ((cx - dx * half, cy - dy * half, z), (cx + dx * half, cy + dy * half, z)), 2.0 * ez


def stop_point(stop, forward_offset=0.0):
    """Map (x, y) of a stop position, moved forward_offset [m] along the driving direction."""
    dx, dy = carla_yaw_to_map_dir(stop["yaw"])
    x, y = carla_to_map(stop["x"], stop["y"])
    return (x + dx * forward_offset, y + dy * forward_offset)


def stop_line(stop, forward_offset=0.0):
    """Stop line across the lane at a stop position, as two (x, y, z) points (left, right).

    forward_offset [m] moves the line forward along the driving direction.
    """
    dx, dy = carla_yaw_to_map_dir(stop["yaw"])
    # Left of the driving direction in the (right-handed) map frame.
    lx, ly = -dy, dx
    half = stop["lane_width"] / 2.0
    cx, cy = stop_point(stop, forward_offset)
    z = stop["z"]
    return ((cx + lx * half, cy + ly * half, z), (cx - lx * half, cy - ly * half, z))


def _point_in_polygon(px, py, polygon):
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = (
        0.0
        if length2 == 0
        else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    )
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _direction_near(polyline, p):
    """Return the unit direction of the polyline segment closest to p."""
    best = min(
        range(len(polyline) - 1), key=lambda i: _segment_distance(p, polyline[i], polyline[i + 1])
    )
    a, b = polyline[best], polyline[best + 1]
    length = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    return ((b[0] - a[0]) / length, (b[1] - a[1]) / length)


# --- map side -----------------------------------------------------------------------------


class Lanelet2Map:
    """The parts of a Lanelet2 .osm this tool reads, plus the parsed XML tree."""

    def __init__(self, tree):
        self.tree = tree
        self.root = tree.getroot()
        self.nodes = {}  # id -> (x, y)
        lat_lon_samples = []
        for node in self.root.findall("node"):
            tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
            if "local_x" not in tags or "local_y" not in tags:
                continue
            x, y = float(tags["local_x"]), float(tags["local_y"])
            self.nodes[node.get("id")] = (x, y)
            if node.get("lat") is not None and node.get("lon") is not None:
                lat_lon_samples.append((x, y, float(node.get("lat")), float(node.get("lon"))))
        self.lon_per_x, self.lat_per_y = _fit_lat_lon_scale(lat_lon_samples)
        self.ways = {
            w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in self.root.findall("way")
        }
        self.lanelets = {}  # relation id -> (relation element, left points, right points)
        for rel in self.root.findall("relation"):
            tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            roles = {m.get("role"): m.get("ref") for m in rel.findall("member")}
            if "left" not in roles or "right" not in roles:
                continue
            left = [self.nodes[r] for r in self.ways.get(roles["left"], []) if r in self.nodes]
            right = [self.nodes[r] for r in self.ways.get(roles["right"], []) if r in self.nodes]
            if len(left) >= 2 and len(right) >= 2:
                self.lanelets[rel.get("id")] = (rel, left, right)
        self.used_ids = {
            int(e.get("id")) for tag in ("node", "way", "relation") for e in self.root.findall(tag)
        }

    @classmethod
    def load(cls, path):
        return cls(ET.parse(path))

    def contains(self, lanelet_id, point):
        """Whether point lies inside the lanelet."""
        _, left, right = self.lanelets[lanelet_id]
        return _point_in_polygon(point[0], point[1], left + right[::-1])

    def lanelet_at(self, point, direction):
        """Id of the lanelet containing point whose direction agrees, or None.

        If several do (overlapping lanelets in a junction), the one whose centre line is
        closest to the point wins.
        """
        candidates = []
        for lanelet_id, (_, left, right) in self.lanelets.items():
            if not _point_in_polygon(point[0], point[1], left + right[::-1]):
                continue
            ldir = _direction_near(left, point)
            if ldir[0] * direction[0] + ldir[1] * direction[1] < _MIN_DIRECTION_DOT:
                continue
            offset = abs(
                min(_segment_distance(point, left[i], left[i + 1]) for i in range(len(left) - 1))
                - min(
                    _segment_distance(point, right[i], right[i + 1]) for i in range(len(right) - 1)
                )
            )
            candidates.append((offset, lanelet_id))
        return min(candidates)[1] if candidates else None


def _fit_lat_lon_scale(samples):
    """Least-squares scales with lon = k_x * local_x and lat = k_y * local_y."""
    sxx = sum(x * x for x, _, _, _ in samples)
    syy = sum(y * y for _, y, _, _ in samples)
    k_x = sum(x * lon for x, _, _, lon in samples) / sxx if sxx else 0.0
    k_y = sum(y * lat for _, y, lat, _ in samples) / syy if syy else 0.0
    return k_x, k_y


# --- conversion ---------------------------------------------------------------------------


class Report:
    """What the conversion did, for the console summary and the tests."""

    def __init__(self):
        self.regulatory_elements = []  # (regelem id, opendrive id, lanelet id)
        self.unattached = []  # (opendrive id, stop index, reason)
        self.skipped = []  # (opendrive id, reason)


def add_traffic_lights(lanelet_map, carla_data, group_id_offset=0, stop_line_forward_offset=0.0):
    """Add the CARLA traffic lights to lanelet_map (in place) and return a Report.

    stop_line_forward_offset [m] moves every stop line forward along the lane. A stop line
    that would leave the lanelet of its stop position is not added (reported as unattached).

    Raises ConversionError if a regulatory-element id derived from an OpenDRIVE id is
    already used by the map, or if the map already has traffic-light regulatory elements.
    """
    root = lanelet_map.root
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") == "regulatory_element" and tags.get("subtype") == "traffic_light":
            raise ConversionError(
                f"the map already has traffic-light regulatory elements (e.g. relation "
                f'{rel.get("id")}); give the original map without them'
            )

    lights = sorted(
        carla_data["traffic_lights"], key=lambda light: _sort_key(light["opendrive_id"])
    )
    report = Report()
    reserved = {}
    for light in lights:
        try:
            group_id = int(str(light["opendrive_id"]).strip()) + group_id_offset
        except ValueError:
            report.skipped.append((light["opendrive_id"], "OpenDRIVE id is not an integer"))
            continue
        if not light["boxes"] or not light["stops"]:
            report.skipped.append((light["opendrive_id"], "no light head or no stop position"))
            continue
        if group_id in lanelet_map.used_ids or group_id in reserved.values():
            raise ConversionError(
                f'regulatory-element id {group_id} (OpenDRIVE id {light["opendrive_id"]} + '
                f"offset {group_id_offset}) is already used in the map; choose a "
                f"--group-id-offset above {max(lanelet_map.used_ids)}"
            )
        reserved[id(light)] = group_id

    builder = _OsmBuilder(
        lanelet_map, first_free_id=max(lanelet_map.used_ids | set(reserved.values())) + 1
    )
    for light in lights:
        if id(light) not in reserved:
            continue
        head_way_ids = []
        for box in light["boxes"]:
            (a, b), height = head_line(box)
            head_way_ids.append(
                builder.add_way(
                    [a, b],
                    {
                        "type": "traffic_light",
                        "subtype": "red_yellow_green",
                        "height": f"{height:.3f}",
                    },
                )
            )
        reserved_id_used = False
        for index, stop in enumerate(light["stops"]):
            point = carla_to_map(stop["x"], stop["y"])
            lanelet_id = lanelet_map.lanelet_at(point, carla_yaw_to_map_dir(stop["yaw"]))
            if lanelet_id is None:
                report.unattached.append(
                    (
                        light["opendrive_id"],
                        index,
                        "no lanelet with the same direction contains the stop position",
                    )
                )
                continue
            # The lanelet is found from CARLA's stop position; the moved stop line must stay in it.
            if not lanelet_map.contains(lanelet_id, stop_point(stop, stop_line_forward_offset)):
                report.unattached.append(
                    (
                        light["opendrive_id"],
                        index,
                        f"the stop line moved {stop_line_forward_offset} m forward leaves "
                        f"lanelet {lanelet_id}",
                    )
                )
                continue
            stop_way_id = builder.add_way(
                list(stop_line(stop, stop_line_forward_offset)), {"type": "stop_line"}
            )
            # The first attached stop position carries the OpenDRIVE-derived id.
            regelem_id = builder.new_id() if reserved_id_used else reserved[id(light)]
            reserved_id_used = True
            builder.add_regulatory_element(regelem_id, head_way_ids, stop_way_id)
            builder.attach(lanelet_id, regelem_id)
            report.regulatory_elements.append((regelem_id, light["opendrive_id"], int(lanelet_id)))
    builder.finish()
    return report


def _sort_key(opendrive_id):
    text = str(opendrive_id).strip()
    return (0, int(text), "") if text.lstrip("-").isdigit() else (1, 0, text)


class _OsmBuilder:
    """Appends nodes / ways / relations to the tree, keeping OSM's node-way-relation order."""

    def __init__(self, lanelet_map, first_free_id):
        self.map = lanelet_map
        self.next_id = first_free_id
        self.new_nodes, self.new_ways, self.new_relations = [], [], []

    def new_id(self):
        new_id = self.next_id
        self.next_id += 1
        return new_id

    def _add_node(self, point):
        x, y, z = point
        node_id = self.new_id()
        node = ET.Element(
            "node",
            {
                "id": str(node_id),
                "visible": "true",
                "version": "1",
                "lat": f"{y * self.map.lat_per_y:.11f}",
                "lon": f"{x * self.map.lon_per_x:.11f}",
            },
        )
        for key, value in (("ele", f"{z:.6f}"), ("local_x", f"{x:.6f}"), ("local_y", f"{y:.6f}")):
            ET.SubElement(node, "tag", {"k": key, "v": value})
        self.new_nodes.append(node)
        return node_id

    def add_way(self, points, tags):
        node_ids = [self._add_node(p) for p in points]
        way_id = self.new_id()
        way = ET.Element("way", {"id": str(way_id), "visible": "true", "version": "1"})
        for node_id in node_ids:
            ET.SubElement(way, "nd", {"ref": str(node_id)})
        for key, value in tags.items():
            ET.SubElement(way, "tag", {"k": key, "v": value})
        self.new_ways.append(way)
        return way_id

    def add_regulatory_element(self, regelem_id, head_way_ids, stop_way_id):
        rel = ET.Element("relation", {"id": str(regelem_id), "visible": "true", "version": "1"})
        for way_id in head_way_ids:
            ET.SubElement(rel, "member", {"type": "way", "ref": str(way_id), "role": "refers"})
        ET.SubElement(rel, "member", {"type": "way", "ref": str(stop_way_id), "role": "ref_line"})
        ET.SubElement(rel, "tag", {"k": "subtype", "v": "traffic_light"})
        ET.SubElement(rel, "tag", {"k": "type", "v": "regulatory_element"})
        self.new_relations.append(rel)

    def attach(self, lanelet_id, regelem_id):
        rel = self.map.lanelets[lanelet_id][0]
        member = ET.Element(
            "member", {"type": "relation", "ref": str(regelem_id), "role": "regulatory_element"}
        )
        # Members before tags, as lanelet2 writes them.
        first_tag = next((i for i, child in enumerate(rel) if child.tag == "tag"), len(rel))
        rel.insert(first_tag, member)

    def finish(self):
        root = self.map.root
        children = list(root)
        last_node = max((i for i, c in enumerate(children) if c.tag == "node"), default=-1)
        for offset, node in enumerate(self.new_nodes):
            root.insert(last_node + 1 + offset, node)
        children = list(root)
        last_way = max((i for i, c in enumerate(children) if c.tag in ("node", "way")), default=-1)
        for offset, way in enumerate(self.new_ways):
            root.insert(last_way + 1 + offset, way)
        for rel in self.new_relations:
            root.append(rel)


def write_map(lanelet_map, path):
    ET.indent(lanelet_map.tree, space="  ")
    lanelet_map.tree.write(path, encoding="UTF-8", xml_declaration=True)


# --- command line -------------------------------------------------------------------------


def _print_report(report, carla_data):
    print(f'CARLA map {carla_data.get("map")}: {len(carla_data["traffic_lights"])} traffic lights')
    print(f"  added {len(report.regulatory_elements)} traffic-light regulatory elements")
    for regelem_id, opendrive_id, lanelet_id in report.regulatory_elements:
        print(
            f"    regulatory element {regelem_id} (OpenDRIVE {opendrive_id}) -> lanelet {lanelet_id}"
        )
    for opendrive_id, index, reason in report.unattached:
        print(f"  WARNING: OpenDRIVE {opendrive_id} stop #{index} not added: {reason}")
    for opendrive_id, reason in report.skipped:
        print(f"  WARNING: OpenDRIVE {opendrive_id} skipped: {reason}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("map", help="input Lanelet2 map (.osm); not modified")
    parser.add_argument(
        "-o", "--output", help="output .osm (default: lanelet2_map_with_tl.osm next to the input)"
    )
    parser.add_argument("--host", default="localhost", help="CARLA host (default: localhost)")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port (default: 2000)")
    parser.add_argument("--timeout", type=float, default=20.0, help="CARLA client timeout [s]")
    parser.add_argument(
        "--from-json", help="read the CARLA traffic lights from this file instead of CARLA"
    )
    parser.add_argument("--dump-json", help="also save the CARLA traffic lights read to this file")
    parser.add_argument(
        "--group-id-offset",
        type=int,
        default=0,
        help="regulatory-element id = OpenDRIVE id + this (default: 0)",
    )
    parser.add_argument(
        "--stop-line-forward-offset",
        type=float,
        default=0.0,
        help="move every stop line this far [m] forward along the lane from CARLA's stop "
        "position (default: 0)",
    )
    parser.add_argument("--force", action="store_true", help="overwrite --output if it exists")
    args = parser.parse_args(argv)

    output = args.output or os.path.join(
        os.path.dirname(os.path.abspath(args.map)), "lanelet2_map_with_tl.osm"
    )
    if os.path.abspath(output) == os.path.abspath(args.map):
        parser.error("--output must differ from the input map")
    if os.path.exists(output) and not args.force:
        parser.error(f"{output} exists; pass --force to overwrite it")

    if args.from_json:
        with open(args.from_json, encoding="utf-8") as f:
            carla_data = json.load(f)
    else:
        carla_data = collect_carla_traffic_lights(args.host, args.port, args.timeout)
    if args.dump_json:
        with open(args.dump_json, "w", encoding="utf-8") as f:
            json.dump(carla_data, f, indent=1)

    map_town = os.path.basename(os.path.dirname(os.path.abspath(args.map)))
    carla_town = os.path.basename(str(carla_data.get("map", "")))
    if carla_town and map_town != carla_town:
        print(f"WARNING: CARLA has {carla_town} loaded but the map directory is {map_town}")

    lanelet_map = Lanelet2Map.load(args.map)
    try:
        report = add_traffic_lights(
            lanelet_map, carla_data, args.group_id_offset, args.stop_line_forward_offset
        )
    except ConversionError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    write_map(lanelet_map, output)
    _print_report(report, carla_data)
    print(f"wrote {output}")
    return 0 if not report.unattached else 2


if __name__ == "__main__":
    sys.exit(main())
