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
Log how the EGO reacts to one traffic light while Autoware drives (full-stack check).

Verification helper for docs/Vissim_CARLA_Autoware_信号状態publish_実装計画_v1.0.md (V8). Only
subscribes (ROS 2); it does not touch CARLA. Prints one line every --period seconds and whenever
something changes:

- the color published for --group (e.g. 372) on /perception/traffic_light_recognition/traffic_signals
- the EGO position (base_link = rear axle) in the map frame and its speed
- the distance from the EGO front to the stop line of regulatory element --group, read from the
  lanelet2 map (--map), along the stop line's normal (positive = before the line)
- whether Autoware reports a 'traffic-signal' velocity factor (/api/planning/velocity_factors)

At the end (Ctrl+C) it prints a summary: the stops near the line with the color at that time, and
whether the EGO front ever crossed the line while the light was red.

Usage (ROS 2 / Autoware sourced):
    python3 tools/monitor_tl_stop.py --map ~/autoware_map/Town01/lanelet2_map_with_tl.osm --group 372
"""

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from autoware_adapi_v1_msgs.msg import VelocityFactorArray
from autoware_perception_msgs.msg import TrafficLightGroupArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

COLOR_NAMES = {1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE", 0: "UNKNOWN"}


def stop_line_of(map_path, group_id):
    """Return ((x0, y0), (x1, y1)) of the ref_line of regulatory element group_id."""
    root = ET.parse(map_path).getroot()
    nodes = {}
    for node in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            nodes[node.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {way.get("id"): [nd.get("ref") for nd in way.findall("nd")] for way in root.findall("way")}
    for rel in root.findall("relation"):
        if rel.get("id") != str(group_id):
            continue
        for member in rel.findall("member"):
            if member.get("role") == "ref_line":
                refs = ways[member.get("ref")]
                return nodes[refs[0]], nodes[refs[-1]]
    raise SystemExit(f"regulatory element {group_id} with a ref_line not found in {map_path}")


class Monitor:
    def __init__(self, node, args, line):
        self.args = args
        (x0, y0), (x1, y1) = line
        self.line_mid = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        # Unit normal of the stop line; its sign is fixed on the first odometry message so that
        # "positive" means "before the line" for the approaching EGO.
        length = math.hypot(x1 - x0, y1 - y0)
        self.normal = (-(y1 - y0) / length, (x1 - x0) / length)
        self.normal_fixed = False
        self.color = None
        self.factor = False
        self.pose = None
        self.speed = 0.0
        self.last_print = 0.0
        self.last_key = None
        self.stops = []
        self.red_crossings = []
        self._stopped_since = None
        self._prev_dist = None
        self.t0 = time.time()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(
            TrafficLightGroupArray, "/perception/traffic_light_recognition/traffic_signals",
            self.on_signals, qos,
        )
        node.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, qos)
        node.create_subscription(
            VelocityFactorArray, "/api/planning/velocity_factors", self.on_factors, qos
        )

    def on_signals(self, msg):
        for group in msg.traffic_light_groups:
            if group.traffic_light_group_id == self.args.group and group.elements:
                self.color = COLOR_NAMES.get(group.elements[0].color, group.elements[0].color)
        self.report()

    def on_factors(self, msg):
        self.factor = any(f.behavior == "traffic-signal" for f in msg.factors)
        self.report()

    def front_distance(self):
        if self.pose is None:
            return None
        x, y, yaw = self.pose
        fx = x + self.args.front_offset * math.cos(yaw)
        fy = y + self.args.front_offset * math.sin(yaw)
        if not self.normal_fixed:
            # Point the normal toward the EGO side (the EGO starts before the line).
            dot = (fx - self.line_mid[0]) * self.normal[0] + (fy - self.line_mid[1]) * self.normal[1]
            if dot < 0:
                self.normal = (-self.normal[0], -self.normal[1])
            self.normal_fixed = True
        return (fx - self.line_mid[0]) * self.normal[0] + (fy - self.line_mid[1]) * self.normal[1]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        dist = self.front_distance()
        now = time.time() - self.t0
        if dist is not None and abs(dist) < 30.0:
            if self.speed < 0.05:
                if self._stopped_since is None:
                    self._stopped_since = now
                    self.stops.append({"t": now, "dist": dist, "color": self.color, "dur": 0.0})
                else:
                    self.stops[-1]["dur"] = now - self._stopped_since
            else:
                self._stopped_since = None
            if self._prev_dist is not None and self._prev_dist > 0.0 >= dist:
                if self.color == "RED":
                    self.red_crossings.append(now)
                print(f"[{now:7.1f}s] EGO front crossed the stop line, light={self.color}", flush=True)
        self._prev_dist = dist
        self.report()

    def report(self):
        now = time.time() - self.t0
        dist = self.front_distance()
        moving = self.speed >= 0.05
        key = (self.color, self.factor, moving)
        if key == self.last_key and now - self.last_print < self.args.period:
            return
        self.last_key = key
        self.last_print = now
        pos = "n/a" if self.pose is None else f"({self.pose[0]:.2f}, {self.pose[1]:.2f})"
        dist_s = "n/a" if dist is None else f"{dist:7.2f} m"
        print(
            f"[{now:7.1f}s] light {self.args.group}={self.color}  traffic-signal factor="
            f"{'yes' if self.factor else 'no '}  speed={self.speed:5.2f} m/s  "
            f"front->stop line={dist_s}  base_link={pos}",
            flush=True,
        )

    def summary(self):
        print()
        print("Stops within 30 m of the stop line (front distance, light at the stop, duration):")
        for s in self.stops:
            print(f"  t={s['t']:.1f}s  {s['dist']:.2f} m  light={s['color']}  {s['dur']:.1f}s")
        if not self.stops:
            print("  none")
        print(f"Crossings of the stop line on RED: {len(self.red_crossings)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument("--map", required=True, help="lanelet2 .osm with the traffic lights")
    parser.add_argument("--group", type=int, default=372, help="traffic light group id")
    parser.add_argument(
        "--front-offset",
        type=float,
        default=3.79,
        help="base_link to vehicle front (m); sample_vehicle: wheel_base 2.79 + front_overhang 1.0",
    )
    parser.add_argument("--period", type=float, default=1.0, help="print period (s)")
    args = parser.parse_args()

    line = stop_line_of(os.path.expanduser(args.map), args.group)
    print(f"stop line of {args.group}: {line[0]} - {line[1]}")
    rclpy.init()
    node = rclpy.create_node("monitor_tl_stop")
    monitor = Monitor(node, args, line)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    monitor.summary()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
