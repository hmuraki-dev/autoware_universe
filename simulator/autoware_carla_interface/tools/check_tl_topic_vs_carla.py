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
Compare /perception/traffic_light_recognition/traffic_signals with the CARLA traffic-light states.

Verification helper for docs/Vissim_CARLA_Autoware_信号状態publish_実装計画_v1.0.md (V7). Run it
while autoware_carla_interface is running with traffic_light.publish:=true. It only reads: it
connects to CARLA as a second client (it never ticks the world nor changes any light) and
subscribes to the topic.

Every CARLA tick it records the state of every traffic light (by OpenDRIVE id). Each received
message is compared with the states of the latest recorded tick ("same tick"), and if that fails,
with the tick before ("1 tick behind"). A group id is turned back into an OpenDRIVE id by
subtracting --group-id-offset (the lanelet2 maps made by carla_tl_to_lanelet2.py use the OpenDRIVE
id + offset as the regulatory element id; without traffic_light.map_path the group id is the
OpenDRIVE id itself).

With --carla-json (the --dump-json output of tools/carla_tl_to_lanelet2.py, with "group") it
also checks every message per CARLA intersection (docs/Vissim_CARLA_Autoware_信号設置_実装計画_v1.0.md,
B6): NG if all lights of an intersection are green at once (crossing roads green together), and
it reports how many messages show differing colors inside an intersection (expected once each
intersection has its own Vissim signal controller with a main-road and a side-road group).

Usage (ROS 2 sourced, Python with the carla package):
    python3 tools/check_tl_topic_vs_carla.py --duration 120
    python3 tools/check_tl_topic_vs_carla.py --carla-json ~/autoware_map/Town01/carla_tl.json
    python3 tools/check_tl_topic_vs_carla.py --frozen-only   # only after the bridge has stopped
                                                             # (ticks CARLA once if needed)

Exit status: 0 if every check passed, 1 otherwise.
"""

import argparse
import collections
import json
import os
import sys
import threading
import time
import types

import carla

# Load modules/traffic_light_publisher.py from the source tree without running the package
# __init__.py files (they import the whole bridge), the same way as test/conftest.py.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
for _name, _rel in (
    ("autoware_carla_interface", "autoware_carla_interface"),
    ("autoware_carla_interface.modules", os.path.join("autoware_carla_interface", "modules")),
):
    _package = types.ModuleType(_name)
    _package.__path__ = [os.path.join(_SRC, _rel)]
    sys.modules[_name] = _package
from autoware_carla_interface.modules.traffic_light_publisher import (  # noqa: E402
    carla_state_to_element,
)

TOPIC = "/perception/traffic_light_recognition/traffic_signals"
COLOR_NAMES = {1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE", 0: "UNKNOWN"}
GREEN = 3
STATUS_NAMES = {1: "SOLID_OFF", 2: "SOLID_ON", 3: "FLASHING", 0: "UNKNOWN"}


def _traffic_lights(world):
    return list(world.get_actors().filter("traffic.traffic_light"))


def check_frozen(world):
    lights = _traffic_lights(world)
    if not lights and world.get_settings().synchronous_mode:
        # The bridge leaves CARLA in synchronous mode: once it has stopped nobody ticks, and a
        # newly connected client sees no actors until the next tick. Tick once (only here, after
        # the bridge has stopped; it advances the world by one fixed_delta_seconds step).
        print("Synchronous world without actors: ticking once to receive them")
        world.tick()
        lights = _traffic_lights(world)
    frozen = sorted(light.get_opendrive_id() for light in lights if light.is_frozen())
    print(f"CARLA traffic lights: {len(lights)}, frozen: {len(frozen)}")
    if not lights:
        # A synchronous world that has never been ticked reports no actors.
        print("  no traffic lights found: wrong map, or the world has not been ticked yet")
        return False
    if frozen:
        print(f"  frozen OpenDRIVE ids: {frozen}")
    return not frozen


class Recorder:
    """Keeps the CARLA states of the last few ticks and the topic comparison results."""

    def __init__(self, lights, intersections=None):
        self._lights = [(light.get_opendrive_id(), light) for light in lights]
        self.intersections = intersections or []  # [[OpenDRIVE id, ...], ...]
        self.all_green = collections.Counter()  # intersection -> messages with every light green
        self.mixed_messages = 0  # messages with differing colors inside some intersection
        self._lock = threading.Lock()
        self.frames = collections.deque(maxlen=20)  # (frame, elapsed, {odid: (color, status)})
        self.tick_count = 0
        self.messages = 0
        self.same_tick = 0
        self.one_behind = 0
        self.mismatches = []
        self.group_counts = collections.Counter()
        self.colors_seen = collections.Counter()
        self.transitions = collections.Counter()
        self.unknown_groups = set()
        self.stamp_offsets = []
        self._last_colors = {}
        self._last_stamp = None
        self.non_increasing_stamps = 0

    def on_tick(self, snapshot):
        states = {}
        for odid, light in self._lights:
            states[odid] = carla_state_to_element(light.state)
        with self._lock:
            self.frames.append((snapshot.frame, snapshot.timestamp.elapsed_seconds, states))
            self.tick_count += 1

    def on_message(self, msg, offset):
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        with self._lock:
            frames = list(self.frames)
        self.messages += 1
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.non_increasing_stamps += 1
        self._last_stamp = stamp
        self.group_counts[len(msg.traffic_light_groups)] += 1
        if not frames:
            return
        published = {}
        for group in msg.traffic_light_groups:
            if not group.elements:
                continue
            element = group.elements[0]
            odid = str(group.traffic_light_group_id - offset)
            published[odid] = (element.color, element.status)
            self.colors_seen[COLOR_NAMES.get(element.color, element.color)] += 1
            previous = self._last_colors.get(odid)
            if previous is not None and previous != element.color:
                self.transitions[
                    f"{COLOR_NAMES.get(previous, previous)}->"
                    f"{COLOR_NAMES.get(element.color, element.color)}"
                ] += 1
            self._last_colors[odid] = element.color
        latest = frames[-1]
        self.stamp_offsets.append(latest[1] - stamp)
        mixed = False
        for group in self.intersections:
            colors = [published[odid][0] for odid in group if odid in published]
            if len(colors) < 2:
                continue
            if all(color == GREEN for color in colors):
                self.all_green["/".join(group)] += 1
            if len(set(colors)) > 1:
                mixed = True
        if mixed:
            self.mixed_messages += 1

        def diff(states):
            out = []
            for odid, value in published.items():
                if odid not in states:
                    self.unknown_groups.add(odid)
                    continue
                if states[odid] != value:
                    out.append((odid, value, states[odid]))
            return out

        if not diff(latest[2]):
            self.same_tick += 1
        elif len(frames) >= 2 and not diff(frames[-2][2]):
            self.one_behind += 1
        else:
            self.mismatches.append((latest[0], diff(latest[2])))


def _fmt(value):
    color, status = value
    return f"{COLOR_NAMES.get(color, color)}/{STATUS_NAMES.get(status, status)}"


def run_compare(world, args):
    import rclpy
    from autoware_perception_msgs.msg import TrafficLightGroupArray
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy

    # A newly connected client may see no actors until it has received a tick from the bridge.
    try:
        world.wait_for_tick(10.0)
    except RuntimeError:
        print("No CARLA tick within 10 s: is the bridge running?")
        return False
    lights = _traffic_lights(world)
    if not lights:
        print("No traffic lights in the CARLA world (has the world been ticked yet?).")
        return False
    intersections = load_intersections(args.carla_json) if args.carla_json else []
    recorder = Recorder(lights, intersections)
    callback_id = world.on_tick(recorder.on_tick)

    rclpy.init()
    node = rclpy.create_node("check_tl_topic_vs_carla")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(
        TrafficLightGroupArray, TOPIC, lambda msg: recorder.on_message(msg, args.group_id_offset), qos
    )
    print(f"Recording {args.duration} s (wall clock) of {TOPIC} and {len(lights)} CARLA lights...")
    end = time.time() + args.duration
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Interrupted, reporting what was recorded so far.")
    publishers = node.count_publishers(TOPIC)
    publisher_nodes = sorted(
        f"{info.node_namespace.rstrip('/')}/{info.node_name}"
        for info in node.get_publishers_info_by_topic(TOPIC)
    )
    world.remove_on_tick(callback_id)
    node.destroy_node()
    rclpy.shutdown()

    ok = True
    print()
    print(f"CARLA ticks seen          : {recorder.tick_count}")
    print(f"messages received         : {recorder.messages}")
    print(f"publishers on the topic   : {publishers} {publisher_nodes}")
    if publishers != 1:
        ok = False
    print(f"groups per message        : {dict(recorder.group_counts)}")
    if recorder.messages == 0:
        print("NG: no message received")
        return False
    compared = recorder.same_tick + recorder.one_behind + len(recorder.mismatches)
    print(
        f"match (same tick)         : {recorder.same_tick} / {compared}"
        f"   1 tick behind: {recorder.one_behind}   mismatch: {len(recorder.mismatches)}"
    )
    for frame, diffs in recorder.mismatches[:10]:
        text = ", ".join(f"{odid}: topic {_fmt(t)} carla {_fmt(c)}" for odid, t, c in diffs[:5])
        print(f"  frame {frame}: {text}")
    if recorder.mismatches:
        ok = False
    if recorder.unknown_groups:
        print(
            f"NG: group ids without a CARLA light (check --group-id-offset): "
            f"{sorted(recorder.unknown_groups)[:10]}"
        )
        ok = False
    if recorder.non_increasing_stamps:
        print(f"NG: stamps did not increase {recorder.non_increasing_stamps} time(s)")
        ok = False
    if recorder.stamp_offsets:
        spread = max(recorder.stamp_offsets) - min(recorder.stamp_offsets)
        print(
            f"stamp vs CARLA elapsed    : offset {recorder.stamp_offsets[0]:.3f} s, "
            f"spread {spread:.3f} s (the bridge stamps with its own game time, so only the "
            f"spread matters; about one tick or less is expected)"
        )
    print(f"colors published (count)  : {dict(recorder.colors_seen)}")
    print(f"color transitions         : {dict(recorder.transitions)}")
    if args.carla_json:
        if not intersections:
            print("NG: --carla-json has no 'group' (re-dump it with carla_tl_to_lanelet2.py)")
            ok = False
        else:
            print(
                f"intersections             : {len(intersections)}, messages with differing "
                f"colors inside an intersection: {recorder.mixed_messages} / {recorder.messages}"
            )
            if recorder.all_green:
                print(
                    "NG: every light of an intersection green at once: "
                    f"{dict(recorder.all_green)}"
                )
                ok = False
    return ok


def load_intersections(json_path):
    """Return the CARLA intersections ("group" of each light) of a --dump-json file."""
    with open(os.path.expanduser(json_path), encoding="utf-8") as f:
        lights = json.load(f)["traffic_lights"]
    if not all("group" in light for light in lights):
        return []
    return sorted({tuple(str(odid) for odid in light["group"]) for light in lights})


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument("--carla-host", default="localhost")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--duration", type=float, default=60.0, help="recording time (s, wall clock)")
    parser.add_argument("--group-id-offset", type=int, default=0)
    parser.add_argument(
        "--carla-json",
        help="carla_tl_to_lanelet2.py --dump-json output with 'group': check each intersection",
    )
    parser.add_argument(
        "--frozen-only", action="store_true", help="only report frozen CARLA lights and exit"
    )
    args = parser.parse_args()

    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(10.0)
    world = client.get_world()
    if args.frozen_only:
        ok = check_frozen(world)
    else:
        ok = run_compare(world, args)
    print("OK" if ok else "NG")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
