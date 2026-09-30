#!/usr/bin/env python3
"""
Step V0 probe for the Vissim warmup / EGO safe spawn plan (see docs/
Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md, Step V0 item 4).

Spawns each given blueprint once in a running CARLA server (at the map's first spawn point,
physics off), prints its bounding box - length (2 x extent.x) and the box center's offset from
the actor origin (bounding_box.location) - then destroys it. The gap check measures clearances
from the actor transform, so a non-zero x offset means the box is not centered on the origin
and the clearance formula has to account for it.

Not installed with the package; run it on the Linux machine with CARLA running:

    python3 tools/carla_bbox_probe.py vehicle.toyota.prius
"""

import argparse
import sys

import carla


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('blueprints', nargs='+', help='blueprint ids, e.g. vehicle.toyota.prius')
    parser.add_argument('--host', default='localhost', help='CARLA host (default: %(default)s)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port (default: %(default)s)')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    library = world.get_blueprint_library()
    spawn_point = world.get_map().get_spawn_points()[0]
    spawn_point.location.z += 2.0

    print('%-40s %8s %8s %8s %10s %10s %10s' %
          ('blueprint', 'length', 'width', 'height', 'offset_x', 'offset_y', 'offset_z'))
    exit_code = 0
    for blueprint_id in args.blueprints:
        matches = library.filter(blueprint_id)
        if not matches:
            print('%-40s unknown blueprint' % blueprint_id)
            exit_code = 1
            continue
        actor = world.try_spawn_actor(matches[0], spawn_point)
        if actor is None:
            print('%-40s spawn failed (spawn point occupied?)' % blueprint_id)
            exit_code = 1
            continue
        try:
            actor.set_simulate_physics(False)
            box = actor.bounding_box
            print('%-40s %8.2f %8.2f %8.2f %10.3f %10.3f %10.3f' %
                  (blueprint_id, 2 * box.extent.x, 2 * box.extent.y, 2 * box.extent.z,
                   box.location.x, box.location.y, box.location.z))
        finally:
            actor.destroy()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
