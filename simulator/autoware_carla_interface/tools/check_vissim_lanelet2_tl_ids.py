#!/usr/bin/env python3

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
Check that the Vissim signal mapping and a Lanelet2 map refer to the same CARLA traffic lights.

Verification helper for docs/Vissim_CARLA_Autoware_信号設置_実装計画_v1.0.md (B1). A CARLA traffic
light is identified by its OpenDRIVE landmark id, which appears in
- vissim_integration/data/signal_mapping_<town>.json: {"<SC>": {"<SG>": [landmark, ...]}}, i.e. which
  Vissim signal group (signal controller SC, signal group SG) drives which CARLA lights, and
- the Lanelet2 map: the id of each traffic_light regulatory element, minus --group-id-offset
  (added by tools/carla_tl_to_lanelet2.py). Regulatory elements sharing their light heads
  (`refers`, one per stop position of a multi-lane light) count as one light with the smallest id.

It reports (NG):
- landmarks found on only one side, and, with --carla-json (the --dump-json output of
  carla_tl_to_lanelet2.py), CARLA lights missing from either side;
- landmarks driven by several (SC, SG): the bridge writes them in turn, so the last one wins;
- with --carla-json files that have "group" (the CARLA intersection of each light): a signal
  controller holding lights of several intersections, or an intersection split over several
  signal controllers;
- with --inpx (the Vissim network): vehicle signal heads farther than --max-distance from every
  CARLA light, signal heads of one landmark in different signal groups, signal heads whose
  signal group differs from signal_mapping.json, mapped landmarks without a vehicle signal head,
  and pedestrian signal heads (on links with isPedArea="true") whose signal controller is not
  the one of the nearest intersection. Pedestrian signal heads are not CARLA lights and are
  left out of the landmark checks.

With --report-stop-offset (needs --inpx and --carla-json) it also prints, for every vehicle
signal head, its distance to the Lanelet2 stop line of its landmark along the direction of
travel (positive = past the stop line, toward the intersection). Vissim vehicles stop at their
signal head, the EGO at the stop line, so this shows how far apart they stop. Values outside
--stop-offset-range are marked with '*' (informational, not NG).

Coordinates: Vissim (x, y) = CARLA (x, -y) = Lanelet2 map (local_x, local_y).

Usage:
    python3 tools/check_vissim_lanelet2_tl_ids.py SIGNAL_MAPPING.json MAP_with_tl.osm \\
        [--carla-json carla_tl.json] [--inpx Town01.inpx] [--group-id-offset N] \\
        [--report-stop-offset]
Exit status: 0 if everything agrees, 1 otherwise.
"""

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET


def _sorted_ids(ids):
    return sorted(ids, key=lambda s: (len(str(s)), str(s)))


# --- inputs -------------------------------------------------------------------------------


def vissim_mapping(json_path):
    """Return {landmark: {(SC, SG)}} of a signal_mapping.json."""
    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)
    landmarks = {}
    for controller, groups in data.items():
        for group, opendrive_ids in groups.items():
            for opendrive_id in opendrive_ids:
                landmarks.setdefault(str(opendrive_id), set()).add((int(controller), int(group)))
    return landmarks


def lanelet2_traffic_lights(osm_path, group_id_offset=0):
    """
    Return ({landmark: [stop line ((x0, y0), (x1, y1)), ...]}, number of regulatory elements).

    Regulatory elements sharing their light heads are one light; its landmark is the smallest
    regulatory-element id minus group_id_offset.
    """
    root = ET.parse(osm_path).getroot()
    nodes = {}
    for node in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            nodes[node.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {
        way.get("id"): [nd.get("ref") for nd in way.findall("nd")] for way in root.findall("way")
    }
    groups = {}  # light heads (refers) -> [(regulatory element id, stop line or None)]
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "traffic_light":
            continue
        regelem_id = int(relation.get("id"))
        heads = frozenset(
            m.get("ref") for m in relation.findall("member") if m.get("role") == "refers"
        )
        line = None
        for member in relation.findall("member"):
            refs = ways.get(member.get("ref"), [])
            if member.get("role") == "ref_line" and len(refs) >= 2:
                if refs[0] in nodes and refs[-1] in nodes:
                    line = (nodes[refs[0]], nodes[refs[-1]])
        groups.setdefault(heads or ("regelem", regelem_id), []).append((regelem_id, line))
    lights = {}
    for elements in groups.values():
        landmark = str(min(regelem_id for regelem_id, _ in elements) - group_id_offset)
        lights[landmark] = [line for _, line in elements if line is not None]
    return lights, sum(len(elements) for elements in groups.values())


def carla_traffic_lights(json_path):
    """Return {landmark: light dict} of a carla_tl_to_lanelet2.py --dump-json file."""
    with open(json_path, encoding="utf-8") as f:
        return {str(light["opendrive_id"]): light for light in json.load(f)["traffic_lights"]}


def _point_at_distance(polyline, distance):
    remaining = distance
    for (x1, y1), (x2, y2) in zip(polyline, polyline[1:]):
        length = math.hypot(x2 - x1, y2 - y1)
        if length == 0:
            continue
        if remaining <= length:
            t = remaining / length
            return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
        remaining -= length
    return polyline[-1]


def vissim_signal_heads(inpx_path):
    """
    Return the signal heads of a Vissim network as dicts: no, link, lane, pos, sc, sg,
    pedestrian (on a link with isPedArea="true") and xy (Vissim = Lanelet2 map coordinates;
    the point `pos` metres along the link's polyline, as util/generate_signal_mapping.py).
    """
    root = ET.parse(inpx_path).getroot()
    links = {}
    for link in root.iter("link"):
        points = [(float(p.get("x")), float(p.get("y"))) for p in link.iter("linkPolyPoint")]
        links[link.get("no")] = (points, link.get("isPedArea") == "true")
    heads = []
    for head in root.iter("signalHead"):
        link_no, lane = head.get("lane").split()
        controller, group = (int(v) for v in head.get("sg").split())
        points, pedestrian = links.get(link_no, ([], False))
        pos = float(head.get("pos"))
        heads.append(
            {
                "no": int(head.get("no")),
                "link": int(link_no),
                "lane": int(lane),
                "pos": pos,
                "sc": controller,
                "sg": group,
                "pedestrian": pedestrian,
                "xy": _point_at_distance(points, pos) if points else None,
            }
        )
    return heads


# --- checks -------------------------------------------------------------------------------


def _carla_xy(light):
    """CARLA light location in Vissim / map coordinates."""
    return (light["location"][0], -light["location"][1])


def nearest_light(xy, carla):
    """Return (landmark, distance) of the CARLA light nearest to a Vissim point."""
    best = min(carla, key=lambda landmark: math.dist(xy, _carla_xy(carla[landmark])))
    return best, math.dist(xy, _carla_xy(carla[best]))


def stop_offset(xy, light, stop_lines):
    """
    Signed distance (m) from the stop line to a point along the direction of travel of the
    light's first CARLA stop position (positive = past the stop line). None without a line.
    """
    if not stop_lines or not light.get("stops"):
        return None
    yaw = math.radians(light["stops"][0]["yaw"])
    direction = (math.cos(yaw), -math.sin(yaw))  # CARLA yaw -> map direction (y flipped)
    best = None
    for (x0, y0), (x1, y1) in stop_lines:
        mid = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        offset = (xy[0] - mid[0]) * direction[0] + (xy[1] - mid[1]) * direction[1]
        if best is None or math.dist(xy, mid) < best[0]:
            best = (math.dist(xy, mid), offset)
    return best[1]


def check(
    mapping_path,
    osm_path,
    carla_json=None,
    inpx=None,
    group_id_offset=0,
    max_distance=10.0,
    report_stop_offset=False,
    stop_offset_range=(0.0, 2.0),
):
    """Print the comparison and return True if everything agrees."""
    vissim = vissim_mapping(mapping_path)
    lanelet2, regelem_count = lanelet2_traffic_lights(osm_path, group_id_offset)
    vissim_ids, lanelet2_ids = set(vissim), set(lanelet2)
    controllers = {sc for pairs in vissim.values() for sc, _ in pairs}
    signal_groups = {pair for pairs in vissim.values() for pair in pairs}
    problems = []

    print(
        f"Vissim: {len(vissim_ids)} landmarks in {len(signal_groups)} signal groups of "
        f"{len(controllers)} signal controllers"
    )
    print(
        f"Lanelet2: {len(lanelet2_ids)} traffic lights in {regelem_count} regulatory elements "
        f"(group id offset {group_id_offset})"
    )
    problems.append(("only in signal_mapping.json", vissim_ids - lanelet2_ids))
    problems.append(("only in Lanelet2 (or not an OpenDRIVE id)", lanelet2_ids - vissim_ids))
    problems.append(
        ("in several signal groups", {lm for lm, pairs in vissim.items() if len(pairs) > 1})
    )

    carla = carla_traffic_lights(carla_json) if carla_json else None
    if carla is not None:
        print(f"CARLA: {len(carla)} traffic lights")
        problems.append(("in CARLA but not in signal_mapping.json", set(carla) - vissim_ids))
        problems.append(("in CARLA but not in Lanelet2", set(carla) - lanelet2_ids))
        if all("group" in light for light in carla.values()):
            intersections = {lm: tuple(map(str, light["group"])) for lm, light in carla.items()}
            print(f"CARLA: {len(set(intersections.values()))} intersections (groups)")
            mixed = {}  # SC -> intersections
            split = {}  # intersection -> SCs
            for landmark, pairs in vissim.items():
                if landmark not in intersections:
                    continue
                for controller, _ in pairs:
                    mixed.setdefault(controller, set()).add(intersections[landmark])
                    split.setdefault(intersections[landmark], set()).add(controller)
            problems.append(
                (
                    "signal controllers with lights of several intersections",
                    {f"SC {sc} ({len(groups)} intersections)" for sc, groups in mixed.items()
                     if len(groups) > 1},
                )
            )
            problems.append(
                (
                    "intersections split over several signal controllers",
                    {f"{'/'.join(group)} (SC {sorted(scs)})" for group, scs in split.items()
                     if len(scs) > 1},
                )
            )
        else:
            print("WARNING: --carla-json has no 'group' (re-dump it with carla_tl_to_lanelet2.py);"
                  " intersection checks skipped")

    stop_rows = []
    if inpx:
        if carla is None:
            print("WARNING: --inpx needs --carla-json; signal head checks skipped")
        else:
            heads = vissim_signal_heads(inpx)
            vehicle = [h for h in heads if not h["pedestrian"]]
            pedestrian = [h for h in heads if h["pedestrian"]]
            print(
                f"Vissim network: {len(vehicle)} vehicle and {len(pedestrian)} pedestrian "
                f"signal heads"
            )
            too_far, missing_xy = set(), set()
            head_groups = {}  # landmark -> {(SC, SG)} of its vehicle signal heads
            for head in vehicle:
                if head["xy"] is None:
                    missing_xy.add(f"head {head['no']} (link {head['link']})")
                    continue
                landmark, distance = nearest_light(head["xy"], carla)
                head["landmark"], head["distance"] = landmark, distance
                if distance > max_distance:
                    too_far.add(f"head {head['no']} ({distance:.1f} m from {landmark})")
                    continue
                head_groups.setdefault(landmark, set()).add((head["sc"], head["sg"]))
                if report_stop_offset:
                    offset = stop_offset(head["xy"], carla[landmark], lanelet2.get(landmark))
                    stop_rows.append((landmark, head, offset))
            problems.append(("signal heads without link geometry", missing_xy))
            problems.append((f"vehicle signal heads farther than {max_distance} m", too_far))
            problems.append(
                (
                    "landmarks whose vehicle signal heads are in different signal groups",
                    {f"{lm} {sorted(g)}" for lm, g in head_groups.items() if len(g) > 1},
                )
            )
            problems.append(
                (
                    "signal heads whose signal group differs from signal_mapping.json",
                    {
                        f"{lm} heads {sorted(g)} mapping {sorted(vissim[lm])}"
                        for lm, g in head_groups.items()
                        if lm in vissim and g != vissim[lm]
                    },
                )
            )
            problems.append(
                ("mapped landmarks without a vehicle signal head", vissim_ids - set(head_groups))
            )
            wrong_controller = set()
            for head in pedestrian:
                if head["xy"] is None:
                    continue
                landmark, distance = nearest_light(head["xy"], carla)
                controllers_here = {sc for sc, _ in vissim.get(landmark, ())}
                note = "OK" if head["sc"] in controllers_here else "NG"
                print(
                    f"  pedestrian head {head['no']} (link {head['link']}): SC {head['sc']} "
                    f"SG {head['sg']}, nearest light {landmark} ({distance:.1f} m, "
                    f"SC {sorted(controllers_here)}) {note}"
                )
                if note == "NG":
                    wrong_controller.add(f"head {head['no']} (SC {head['sc']}, light {landmark})")
            problems.append(
                ("pedestrian signal heads in another intersection's controller", wrong_controller)
            )

    if report_stop_offset:
        if not stop_rows:
            print("--report-stop-offset needs --inpx and --carla-json")
        else:
            low, high = stop_offset_range
            print()
            print(
                "Vehicle signal head vs Lanelet2 stop line (along travel, + = past the line), "
                f"'*' outside {low:g}..{high:g} m:"
            )
            values = []
            for landmark, head, offset in sorted(stop_rows, key=lambda r: (int(r[0]), r[1]["no"])):
                if offset is None:
                    print(f"  {landmark}: head {head['no']} (link {head['link']}) no stop line")
                    continue
                values.append(offset)
                mark = "*" if not low <= offset <= high else " "
                print(
                    f"  {landmark}: head {head['no']:>3} link {head['link']:>5} pos "
                    f"{head['pos']:7.2f}  SC {head['sc']:>2} SG {head['sg']}  "
                    f"{offset:+6.2f} m {mark}"
                )
            if values:
                outside = sum(1 for v in values if not low <= v <= high)
                print(
                    f"  range {min(values):+.2f} .. {max(values):+.2f} m, "
                    f"{outside} of {len(values)} outside {low:g}..{high:g} m"
                )

    ok = True
    for label, ids in problems:
        if ids:
            ok = False
            print(f"NG {label}: {_sorted_ids(ids)}")
    print("OK" if ok else "NG")
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("mapping", help="Vissim signal_mapping.json")
    parser.add_argument("map", help="Lanelet2 map with traffic lights (.osm)")
    parser.add_argument("--carla-json", help="--dump-json output of carla_tl_to_lanelet2.py")
    parser.add_argument("--inpx", help="Vissim network (.inpx) to check the signal heads of")
    parser.add_argument(
        "--group-id-offset",
        type=int,
        default=0,
        help="--group-id-offset given to carla_tl_to_lanelet2.py (default: 0)",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=10.0,
        help="largest signal head to CARLA light distance accepted (m, default: 10)",
    )
    parser.add_argument(
        "--report-stop-offset",
        action="store_true",
        help="print each vehicle signal head's distance to its Lanelet2 stop line",
    )
    parser.add_argument(
        "--stop-offset-range",
        type=float,
        nargs=2,
        default=(0.0, 2.0),
        metavar=("LOW", "HIGH"),
        help="stop offsets outside this range are marked with '*' (m, default: 0 2)",
    )
    args = parser.parse_args(argv)
    ok = check(
        args.mapping,
        args.map,
        args.carla_json,
        args.inpx,
        args.group_id_offset,
        args.max_distance,
        args.report_stop_offset,
        tuple(args.stop_offset_range),
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
