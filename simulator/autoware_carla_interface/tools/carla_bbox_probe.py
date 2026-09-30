#!/usr/bin/env python3
"""
Step S0 probe for the SUMO warmup / EGO safe spawn plan (see docs/
SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md, Step S0 items 3 and 4).

Spawns each given blueprint once in a running CARLA server (at the map's first spawn point,
physics off), prints its bounding box - length (2 x extent.x) and the box center's offset from
the actor origin (bounding_box.location) - then destroys it. The gap check measures clearances
from the actor transform, so a non-zero x offset means the box is not centered on the origin
and the clearance formula has to account for it.

With --sumo-vtypes, the SUMO vType file (e.g. carlavtypes.rou.xml) is read as well and the SUMO
length of the vType with the same id is printed next to the CARLA length, with the difference.
Pass --all-from-vtypes to probe every vType id in that file that is also a CARLA blueprint.

Not installed with the package; run it on the Linux machine with CARLA running:

    python3 tools/carla_bbox_probe.py vehicle.toyota.prius
    python3 tools/carla_bbox_probe.py --all-from-vtypes \
        --sumo-vtypes /home/divp/CARLA/Co-Simulation/Sumo/examples/carlavtypes.rou.xml
"""

import argparse
import sys
import xml.etree.ElementTree as ET

import carla


def _read_sumo_lengths(path):
    """Returns {vType id: length} from a SUMO route/additional file."""
    lengths = {}
    for vtype in ET.parse(path).getroot().iter('vType'):
        if vtype.get('length') is not None:
            lengths[vtype.get('id')] = float(vtype.get('length'))
    return lengths


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('blueprints', nargs='*', help='blueprint ids, e.g. vehicle.toyota.prius')
    parser.add_argument('--host', default='localhost', help='CARLA host (default: %(default)s)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port (default: %(default)s)')
    parser.add_argument('--sumo-vtypes', metavar='PATH',
                        help='SUMO vType file to compare lengths with (e.g. carlavtypes.rou.xml)')
    parser.add_argument('--all-from-vtypes', action='store_true',
                        help='probe every vType id in --sumo-vtypes that is a CARLA blueprint')
    args = parser.parse_args()
    if args.all_from_vtypes and not args.sumo_vtypes:
        parser.error('--all-from-vtypes requires --sumo-vtypes')

    sumo_lengths = _read_sumo_lengths(args.sumo_vtypes) if args.sumo_vtypes else {}

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    library = world.get_blueprint_library()
    spawn_point = world.get_map().get_spawn_points()[0]
    spawn_point.location.z += 2.0

    blueprint_ids = list(args.blueprints)
    if args.all_from_vtypes:
        library_ids = {bp.id for bp in library}
        blueprint_ids += sorted(i for i in sumo_lengths if i in library_ids and i not in blueprint_ids)
    if not blueprint_ids:
        parser.error('no blueprints given')

    print('%-40s %8s %8s %8s %10s %10s %10s %8s %8s' %
          ('blueprint', 'length', 'width', 'height', 'offset_x', 'offset_y', 'offset_z',
           'sumo_len', 'diff'))
    exit_code = 0
    for blueprint_id in blueprint_ids:
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
            length = 2 * box.extent.x
            if blueprint_id in sumo_lengths:
                sumo_columns = '%8.2f %8.2f' % (sumo_lengths[blueprint_id],
                                                length - sumo_lengths[blueprint_id])
            else:
                sumo_columns = '%8s %8s' % ('-', '-')
            print('%-40s %8.2f %8.2f %8.2f %10.3f %10.3f %10.3f %s' %
                  (blueprint_id, length, 2 * box.extent.y, 2 * box.extent.z,
                   box.location.x, box.location.y, box.location.z, sumo_columns))
        finally:
            actor.destroy()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
