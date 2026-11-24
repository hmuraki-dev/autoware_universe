#!/usr/bin/env python3
"""
Give the lane boundaries of a CARLA Lanelet2 map a line type, so that Autoware can change lanes.

The Lanelet2 maps converted from CARLA (Town01, Town10HD, ...) have no `type`/`subtype` on any
lane boundary. The Lanelet2 traffic rules treat such a boundary as not passable, so the routing
graph has no lane change anywhere: `RoutingGraph.right()`/`left()` are empty and only
`adjacentRight()`/`adjacentLeft()` know the neighbor. Autoware then plans a route over the
neighboring lane (it is in the route segment) but its lane change module never starts, and the
vehicle stops at the end of its own lane (seen 2026-11-24 on Town10HD, lanelet 891 -> 860).

The tool tags every boundary that has no `type` yet:
  - line_thin / dashed: the boundary between two lanelets of the same direction
    (`adjacentRight`), when neither of them is a junction lanelet. Lane change is allowed.
  - line_thin / solid: every other boundary (outer edges, the boundary to the opposite
    direction, and boundaries of junction lanelets). Lane change is not allowed.
A lanelet is a junction lanelet when it branches or merges: its predecessor has more than one
successor, or its successor has more than one predecessor (CARLA junction connectors).
The maps carry no turn_direction, so this is the only way to tell them apart.
Boundaries that already have a `type` are left as they are.

Needs the lanelet2 Python bindings (source /opt/ros/humble/setup.bash).

Usage:
    python3 tools/add_lanelet2_line_types.py IN.osm OUT.osm
    python3 tools/add_lanelet2_line_types.py IN.osm --check
Exit status: with --check, 1 if some boundary has no type, else 0.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from collections import Counter

DASHED = ("line_thin", "dashed")
SOLID = ("line_thin", "solid")


def load_graph(path):
    """(lanelet map, routing graph) of the map, read with the lanelet2 bindings."""
    import lanelet2
    from lanelet2.io import Origin
    from lanelet2.projection import LocalCartesianProjector

    lanelet_map, _ = lanelet2.io.loadRobust(str(path), LocalCartesianProjector(Origin(0, 0)))
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.Vehicle)
    return lanelet_map, lanelet2.routing.RoutingGraph(lanelet_map, rules)


def is_junction(graph, lanelet):
    return (any(len(graph.following(p)) > 1 for p in graph.previous(lanelet))
            or any(len(graph.previous(s)) > 1 for s in graph.following(lanelet)))


def line_types(lanelet_map, graph):
    """{boundary way id: (type, subtype)} for every lanelet boundary, and the dashed pairs."""
    types, pairs = {}, []
    for lanelet in lanelet_map.laneletLayer:
        for bound in (lanelet.leftBound, lanelet.rightBound):
            types.setdefault(bound.id, SOLID)
    for lanelet in lanelet_map.laneletLayer:
        neighbor = graph.adjacentRight(lanelet) or graph.right(lanelet)
        if neighbor is None or is_junction(graph, lanelet) or is_junction(graph, neighbor):
            continue
        types[lanelet.rightBound.id] = DASHED
        pairs.append((lanelet.id, neighbor.id))
    return types, pairs


def apply(root, types):
    """Tag the ways in place; return (Counter of the tags written, ids of ways that kept their type)."""
    written, kept = Counter(), []
    for way in root.findall("way"):
        wanted = types.get(int(way.get("id")))
        if wanted is None:
            continue
        tags = {tag.get("k"): tag for tag in way.findall("tag")}
        if "type" in tags:
            kept.append(way.get("id"))
            continue
        for key, value in zip(("type", "subtype"), wanted):
            ET.SubElement(way, "tag", k=key, v=value)
        written[wanted] += 1
    return written, kept


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input")
    parser.add_argument("output", nargs="?")
    parser.add_argument("--check", action="store_true", help="only report, write nothing")
    args = parser.parse_args(argv)
    if not args.check and not args.output:
        parser.error("OUT.osm is required unless --check")

    tree = ET.parse(args.input)
    lanelet_map, graph = load_graph(args.input)
    types, pairs = line_types(lanelet_map, graph)
    written, kept = apply(tree.getroot(), types)
    untyped = sum(written.values())
    print(f"boundaries: {len(types)}, without type: {untyped}, already typed (kept): {len(kept)}")
    for (kind, subtype), count in sorted(written.items()):
        print(f"  {kind} {subtype}: {count}")
    print(f"lane change allowed between {len(pairs)} lanelet pairs (left -> right): "
          + ", ".join(f"{a}->{b}" for a, b in sorted(pairs)[:20]) + (" ..." if len(pairs) > 20 else ""))
    if args.check:
        return 1 if untyped else 0
    ET.indent(tree, space="  ")
    tree.write(args.output, encoding="UTF-8", xml_declaration=True)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
