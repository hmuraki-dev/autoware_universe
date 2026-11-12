#!/usr/bin/env python3
"""
Remove consecutive duplicate points from the ways of a Lanelet2 map.

Some CARLA Lanelet2 maps (e.g. Town05: 48 of 486 lanelets, Town10HD: 10 of 168) have a bound
whose first two points share the same position. The Autoware lanelet2_map_loader regenerates
each lanelet centerline (center_line_resolution) and puts NaN into the first centerline point of
such a lanelet. Any route through it then gets a NaN length, the mission planner's route cost
comparison never succeeds, it returns an empty path, and component_container_mt
(mission_planner) crashes with SIGSEGV in RouteHandler::getMainLanelets. In RViz the 2D Goal
Pose then sets no route.

Within a run of points at the same (local_x, local_y), the tool keeps one point: an end point of
the way if there is one (it connects the lanelet to its neighbors), else the point used by the
most ways, else the first. Nodes are left in the map; only the way references are removed.

Usage:
    python3 tools/fix_lanelet2_duplicate_points.py IN.osm OUT.osm [--eps 1e-6]
    python3 tools/fix_lanelet2_duplicate_points.py IN.osm --check
Exit status: with --check, 1 if the map has duplicate points, else 0.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter


def _node_xy(node):
    tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
    if "local_x" in tags and "local_y" in tags:
        return float(tags["local_x"]), float(tags["local_y"])
    return float(node.get("lon")), float(node.get("lat"))


def fix_duplicate_points(root, eps=1e-6):
    """Remove the duplicate <nd> of every way in place; return {way id: [removed node ids]}."""
    xy = {node.get("id"): _node_xy(node) for node in root.findall("node")}
    ways = root.findall("way")
    use_count = Counter(nd.get("ref") for way in ways for nd in way.findall("nd"))
    removed = {}
    for way in ways:
        nds = way.findall("nd")
        runs = [[nds[0]]] if nds else []
        for nd in nds[1:]:
            if math.dist(xy[runs[-1][0].get("ref")], xy[nd.get("ref")]) < eps:
                runs[-1].append(nd)
            else:
                runs.append([nd])
        ends = {nds[0], nds[-1]} if nds else set()
        for run in runs:
            if len(run) < 2:
                continue
            keep = next((nd for nd in run if nd in ends), None)
            if keep is None:
                keep = max(run, key=lambda nd: use_count[nd.get("ref")])
            for nd in run:
                if nd is not keep:
                    way.remove(nd)
                    removed.setdefault(way.get("id"), []).append(nd.get("ref"))
    return removed


def lanelets_using(root, way_ids):
    """Return the ids of the lanelets whose left or right bound is one of way_ids."""
    ids = []
    for rel in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        if any(
            m.get("type") == "way"
            and m.get("role") in ("left", "right")
            and m.get("ref") in way_ids
            for m in rel.findall("member")
        ):
            ids.append(rel.get("id"))
    return ids


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("input")
    parser.add_argument("output", nargs="?")
    parser.add_argument("--eps", type=float, default=1e-6, help="same-position distance [m]")
    parser.add_argument("--check", action="store_true", help="only report, write nothing")
    args = parser.parse_args(argv)
    if not args.check and not args.output:
        parser.error("OUT.osm is required unless --check is given")

    tree = ET.parse(args.input)
    root = tree.getroot()
    removed = fix_duplicate_points(root, args.eps)
    lanelets = lanelets_using(root, set(removed))
    print(
        f"{args.input}: {sum(map(len, removed.values()))} duplicate points in {len(removed)} ways, "
        f"{len(lanelets)} lanelets affected"
    )
    if lanelets:
        print("  lanelets: " + " ".join(lanelets))
    if args.check:
        return 1 if removed else 0
    tree.write(args.output, encoding="UTF-8", xml_declaration=True)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
