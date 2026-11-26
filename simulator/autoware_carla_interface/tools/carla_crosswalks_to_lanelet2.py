#!/usr/bin/env python3
"""
Add CARLA's crosswalks to a Lanelet2 map as Autoware crosswalk lanelets.

The stock CARLA Lanelet2 maps (e.g. ~/autoware_map/Town10HD/lanelet2_map.osm) contain only road
lanelets, while the OpenDRIVE file of the town has crosswalk objects (16 in Town10HD). CARLA
returns their outlines with Map.get_crosswalks(): a flat list of points in which each crosswalk is a
closed polygon (the first point repeated at the end). For every crosswalk this tool adds:

- two linestrings (type=pedestrian_marking): the two long sides of the outline, both running the
  same way, as the left and right bound,
- one lanelet (subtype=crosswalk, participant:pedestrian=yes, one_way=no) between them.

Autoware's crosswalk module finds a crosswalk lanelet through the routing graph: a crosswalk
lanelet that overlaps a road lanelet on the path conflicts with it in the pedestrian graph. No
crosswalk regulatory element is added (it is optional; without it the module stops in front of
the crosswalk lanelet). Every crosswalk should overlap at least one road lanelet; the report lists
the overlapped road lanelets and warns about a crosswalk that overlaps none.

Order of the map tools: add the crosswalks to the original map first, then the traffic lights
(carla_tl_to_lanelet2.py, which skips crosswalk lanelets when it looks for the lanelet of a stop
position), so that the final map keeps its name lanelet2_map_with_tl.osm.

Coordinates: map = (carla_x, -carla_y), as in carla_tl_to_lanelet2.py (= OpenDRIVE (x, y)).

Usage (CARLA server running with the matching town loaded):
    python3 tools/carla_crosswalks_to_lanelet2.py ~/autoware_map/Town10HD/lanelet2_map.osm \\
        --dump-json ~/autoware_map/Town10HD/carla_crosswalks.json
Offline, from data saved earlier with --dump-json:
    python3 tools/carla_crosswalks_to_lanelet2.py MAP.osm --from-json carla_crosswalks.json
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from carla_tl_to_lanelet2 import (  # noqa: E402
    ConversionError,
    Lanelet2Map,
    _OsmBuilder,
    _point_in_polygon,
    carla_to_map,
    write_map,
)

CROSSWALK_TAGS = {
    "type": "lanelet",
    "subtype": "crosswalk",
    "speed_limit": "10",
    "location": "urban",
    "one_way": "no",
    "participant:pedestrian": "yes",
}
BOUND_TAGS = {"type": "pedestrian_marking"}
# Points per metre when testing whether a crosswalk overlaps a road lanelet
_SAMPLE_STEP_M = 0.5


# --- CARLA side ---------------------------------------------------------------------------


def collect_carla_crosswalks(host, port, timeout):
    """Read the crosswalk outlines of the loaded CARLA map as plain data (see split_polygons)."""
    import carla  # pylint: disable=import-outside-toplevel

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    carla_map = client.get_world().get_map()
    points = [[p.x, p.y, p.z] for p in carla_map.get_crosswalks()]
    return {"map": carla_map.name, "points": points}


def split_polygons(points):
    """Flat point list of Map.get_crosswalks() -> closed polygons (first point repeated last).

    Raises ConversionError if the list does not end with a closed polygon.
    """
    polygons, current = [], []
    for point in points:
        current.append(tuple(point))
        if len(current) > 1 and _same(current[-1], current[0]):
            polygons.append(current)
            current = []
    if current:
        raise ConversionError(f"the crosswalk points end with an open polygon of {len(current)} points")
    return polygons


def _same(a, b, tolerance=1e-3):
    return all(abs(x - y) <= tolerance for x, y in zip(a, b))


# --- geometry -----------------------------------------------------------------------------


def crosswalk_bounds(polygon):
    """Closed CARLA polygon of 4 corners -> (left, right) bounds in the map frame.

    The bounds are the two long sides, both running the same way (the walking direction across
    the road); left is the one on the left of that direction. Raises ConversionError if the
    polygon does not have 4 corners.
    """
    corners = [(*carla_to_map(x, y), z) for x, y, z in polygon[:-1]]
    if len(corners) != 4:
        raise ConversionError(f"a crosswalk has {len(corners)} corners (only 4 are supported)")
    sides = [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    lengths = [math.dist(a[:2], b[:2]) for a, b in sides]
    # The long sides are either 0 and 2 or 1 and 3
    first = 0 if lengths[0] + lengths[2] >= lengths[1] + lengths[3] else 1
    side_a = sides[first]
    side_b = sides[first + 2][::-1]  # the opposite side, turned to run the same way as side_a
    direction = (side_a[1][0] - side_a[0][0], side_a[1][1] - side_a[0][1])
    # side_b is on the left of side_a if the cross product of direction and (b - a) is positive
    offset = (side_b[0][0] - side_a[0][0], side_b[0][1] - side_a[0][1])
    if direction[0] * offset[1] - direction[1] * offset[0] > 0:
        return list(side_b), list(side_a)
    return list(side_a), list(side_b)


def _samples(left, right):
    """Points on the crosswalk (its centre line and both bounds), every _SAMPLE_STEP_M."""
    out = []
    pairs = [
        (left[0], left[1]),
        (right[0], right[1]),
        (_mid(left[0], right[0]), _mid(left[1], right[1])),
    ]
    for a, b in pairs:
        n = max(1, int(math.dist(a[:2], b[:2]) / _SAMPLE_STEP_M))
        out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]
    return out


def _mid(a, b):
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2)


def _centre(left, right):
    """(x, y) of the centre of the crosswalk."""
    points = left + right
    return (sum(p[0] for p in points) / 4, sum(p[1] for p in points) / 4)


def overlapped_road_lanelets(lanelet_map, left, right):
    """Ids of the road lanelets that contain a point of the crosswalk (sorted)."""
    points = _samples(left, right)
    found = set()
    for lanelet_id, (rel, l_pts, r_pts) in lanelet_map.lanelets.items():
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("subtype", "road") != "road":
            continue
        polygon = l_pts + r_pts[::-1]
        if any(_point_in_polygon(x, y, polygon) for x, y in points):
            found.add(int(lanelet_id))
    return sorted(found)


# --- conversion ---------------------------------------------------------------------------


class Report:
    """What the conversion did, for the console summary and the tests."""

    def __init__(self):
        self.crosswalks = []  # (lanelet id, length [m], width [m], overlapped road lanelet ids)


def add_crosswalks(lanelet_map, carla_data):
    """Add the CARLA crosswalks to lanelet_map (in place) and return a Report.

    Raises ConversionError if the map already has crosswalk lanelets or a crosswalk is not a
    quadrilateral.
    """
    for rel in lanelet_map.root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") == "lanelet" and tags.get("subtype") == "crosswalk":
            raise ConversionError(
                f'the map already has crosswalk lanelets (e.g. relation {rel.get("id")}); '
                "give the original map without them"
            )
    bounds = [crosswalk_bounds(p) for p in split_polygons(carla_data["points"])]
    # Sort by the centre so that the ids do not depend on CARLA's order
    bounds.sort(key=lambda lr: tuple(round(c, 3) for c in _centre(*lr)))
    builder = _OsmBuilder(lanelet_map, first_free_id=max(lanelet_map.used_ids) + 1)
    report = Report()
    for left, right in bounds:
        roads = overlapped_road_lanelets(lanelet_map, left, right)
        left_id = builder.add_way(left, BOUND_TAGS)
        right_id = builder.add_way(right, BOUND_TAGS)
        lanelet_id = builder.new_id()
        builder.add_lanelet(lanelet_id, left_id, right_id, CROSSWALK_TAGS)
        report.crosswalks.append(
            (
                lanelet_id,
                math.dist(left[0][:2], left[1][:2]),
                math.dist(_mid(left[0], left[1])[:2], _mid(right[0], right[1])[:2]),
                roads,
            )
        )
    builder.finish()
    return report


# --- command line -------------------------------------------------------------------------


def _print_report(report, carla_data):
    print(f'CARLA map {carla_data.get("map")}: {len(report.crosswalks)} crosswalks')
    for lanelet_id, length, width, roads in report.crosswalks:
        print(
            f"  crosswalk lanelet {lanelet_id}: {length:.1f} m x {width:.1f} m, "
            f"over road lanelets {roads}"
        )
        if not roads:
            print(f"  WARNING: crosswalk lanelet {lanelet_id} overlaps no road lanelet")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("map", help="input Lanelet2 map (.osm); not modified")
    parser.add_argument(
        "-o",
        "--output",
        help="output .osm (default: lanelet2_map_with_crosswalk.osm next to the input)",
    )
    parser.add_argument("--host", default="localhost", help="CARLA host (default: localhost)")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port (default: 2000)")
    parser.add_argument("--timeout", type=float, default=20.0, help="CARLA client timeout [s]")
    parser.add_argument("--from-json", help="read the CARLA crosswalks from this file instead of CARLA")
    parser.add_argument("--dump-json", help="also save the CARLA crosswalks read to this file")
    parser.add_argument("--force", action="store_true", help="overwrite --output if it exists")
    args = parser.parse_args(argv)

    output = args.output or os.path.join(
        os.path.dirname(os.path.abspath(args.map)), "lanelet2_map_with_crosswalk.osm"
    )
    if os.path.abspath(output) == os.path.abspath(args.map):
        parser.error("--output must differ from the input map")
    if os.path.exists(output) and not args.force:
        parser.error(f"{output} exists; pass --force to overwrite it")

    if args.from_json:
        with open(args.from_json, encoding="utf-8") as f:
            carla_data = json.load(f)
    else:
        carla_data = collect_carla_crosswalks(args.host, args.port, args.timeout)
    if args.dump_json:
        with open(args.dump_json, "w", encoding="utf-8") as f:
            json.dump(carla_data, f, indent=1)

    map_town = os.path.basename(os.path.dirname(os.path.abspath(args.map)))
    carla_town = os.path.basename(str(carla_data.get("map", "")))
    if carla_town and map_town != carla_town:
        print(f"WARNING: CARLA has {carla_town} loaded but the map directory is {map_town}")

    lanelet_map = Lanelet2Map.load(args.map)
    try:
        report = add_crosswalks(lanelet_map, carla_data)
    except ConversionError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    write_map(lanelet_map, output)
    _print_report(report, carla_data)
    print(f"wrote {output}")
    return 0 if all(roads for *_, roads in report.crosswalks) else 2


if __name__ == "__main__":
    sys.exit(main())
