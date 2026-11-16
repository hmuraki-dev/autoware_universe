#!/usr/bin/env python3
"""Measure a CARLA vehicle blueprint's actual mesh extent, not its bounding box.

See docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md (F2). The output is the
`geometry` input of tools/carla_fleet/build_carla_variants.py.

`actor.bounding_box` is not reliable as a body size: measured against known vehicles it is
right for some assets and inflated or clipped for others (in CARLA 0.10, firetruck box 2.901 m
vs mesh 3.875 m wide, fuso box 3.928 m vs mesh 3.181 m). A body that the LiDAR sees is the mesh,
so the mesh is what a blueprint assignment has to be based on.

The body is measured with what a LiDAR sees: semantic ray-cast LiDARs are placed around and
above a parked, physics-disabled actor, and the points whose object index is that actor give its
length, width, height and where the body sits relative to the actor origin (mesh_*_m). The same
actor is also swept with `world.cast_ray` (the original method), kept as castray_*_m for
comparison. In CARLA 0.9.15 the two disagree for some blueprints (measured 2026-11-16):
cast_ray also hits collision shapes a LiDAR does not see (fusorosa 3.893 m vs 3.176 m wide,
volkswagen.t2 2.204 vs 2.058, vespa 2.064 vs 1.810 m long) and misses parts it does see
(mercedes.coupe_2020 mirrors, riders of two-wheelers). Autoware's LiDAR in this co-simulation is
a CARLA ray-cast LiDAR, so the LiDAR extent is what an assignment has to match.

It spawns far below the map, measures, and destroys what it made (the actor and its sensors).
It never moves an actor it did not create and never changes world settings. If the world is in
synchronous mode (as the co-simulation leaves it), it ticks the world itself; otherwise it waits
for ticks.

Ported from SBIR_sim `tools/carla_fleet/measure_blueprint_mesh.py` (3ea1650). Changes:
  - the body extent (mesh_*_m) comes from semantic LiDAR points, not from cast_ray (above),
  - cast_ray keeps the nearest vehicle-labelled hit of each ray, skipping unlabelled (NONE) hits,
  - blueprints to skip are given with --exclude (fnmatch) instead of the hard-coded ego ('j6'),
  - ticks the world in synchronous mode (world.wait_for_tick() never returns there),
  - --out is the output file, not a directory.

Not installed with the package; run it on the Linux machine with CARLA running (no bridge, no
Autoware):

    python3 tools/carla_fleet/measure_blueprint_mesh.py \\
        --out src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import math
from pathlib import Path
import queue
import sys
import time

VEHICLE_LABELS = ('Car', 'Truck', 'Bus', 'Motorcycle', 'Bicycle', 'Rider')
# Heights above the actor origin to sweep with cast_ray. A vehicle is widest somewhere in the
# lower half; sampling only one height can miss a flared body or a mirror.
HEIGHTS_M = (0.3, 0.6, 1.0, 1.5, 2.0, 2.6, 3.2)
# Where the measured actors are parked: far outside and below any CARLA town.
PARK_X_M, PARK_Y_M, PARK_Z_M, PARK_PITCH_M = -4300.0, -4300.0, -60.0, 40.0
# The LiDARs: one above the actor and a ring around it (radius, azimuths, heights above the
# actor origin). Denser than this changed no extent by more than a few cm (2026-11-16).
LIDAR_TOP_M = 14.0
LIDAR_RADIUS_M = 8.0
LIDAR_AZIMUTHS = 16
LIDAR_HEIGHTS_M = (0.5, 1.2, 2.0, 3.0)
LIDAR_ROTATION_HZ = 20.0
LIDAR_ATTRIBUTES = dict(channels='128', range='40', points_per_second='4000000',
                        upper_fov='30', lower_fov='-60')
LIDAR_POINT = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('cos', 'f4'), ('idx', 'u4'), ('tag', 'u4')]


def tick(world):
    """Let the server process spawns/destroys: tick in synchronous mode, otherwise wait for one."""
    if world.get_settings().synchronous_mode:
        world.tick(10.0)
    else:
        world.wait_for_tick(10.0)


def nearest_vehicle_hit(hits, start):
    """The first vehicle-labelled surface a ray from `start` meets, or None.

    Hits labelled NONE are skipped: in CARLA 0.9.15 two-wheelers are wrapped in an unlabelled
    collision shell that every ray meets first, and a LiDAR does not see it (measured
    2026-11-16). Any other non-vehicle hit in front of the vehicle ends the ray (the actor is
    parked alone, so that should not happen). `hits` are carla.LabelledPoint; only `.location`
    and `.label` are used.
    """
    for hit in sorted(hits, key=lambda hit: hit.location.distance(start)):
        label = str(hit.label)
        if label in VEHICLE_LABELS:
            return hit
        if label != 'NONE':
            return None
    return None


def span(world, carla, origin, axis, stations, heights, reach, step):
    """Extreme mesh coordinate along `axis`, sweeping the other axis.

    Each station is probed from both sides and the two near-side surfaces bound the body.
    Only the hit NEAREST to the ray's start counts: in CARLA 0.9.15 `world.cast_ray` returns
    every surface along the segment, including ones a LiDAR does not see (measured 2026-11-16:
    fusorosa 3.893 m wide with all hits, 3.176 m with the nearest hit and with a semantic LiDAR).
    In CARLA 0.10 it returned only the first surface, so this is the same as the original there.
    """
    low, high, rays, hits = None, None, 0, 0
    for offset in stations:
        for height in heights:
            for sign in (-1.0, +1.0):
                if axis == 'y':
                    origin_pt = carla.Location(origin.x + offset, origin.y + sign*reach, origin.z + height)
                    target = carla.Location(origin.x + offset, origin.y, origin.z + height)
                else:
                    origin_pt = carla.Location(origin.x + sign*reach, origin.y + offset, origin.z + height)
                    target = carla.Location(origin.x, origin.y + offset, origin.z + height)
                rays += 1
                nearest = nearest_vehicle_hit(world.cast_ray(origin_pt, target), origin_pt)
                if nearest is None:
                    continue
                value = nearest.location.y if axis == 'y' else nearest.location.x
                hits += 1
                low = value if low is None else min(low, value)
                high = value if high is None else max(high, value)
    if low is None:
        return None
    return dict(min=low, max=high, extent_m=high-low, center=(low+high)/2,
                rays=rays, hits=hits, step_m=step)


def height_span(world, carla, origin, stations_x, stations_y, reach):
    """Roof height above the actor origin.

    Only downward rays are useful: `cast_ray` returns the first surface, and a
    ray from below terminates at the origin plane without reaching the body. So
    each station gives the roof, and the tallest roof over the stations is the
    body height above the actor's ground plane.
    """
    tallest = None
    rays = hits = 0
    for dx in stations_x:
        for dy in stations_y:
            start = carla.Location(origin.x+dx, origin.y+dy, origin.z + reach)
            end = carla.Location(origin.x+dx, origin.y+dy, origin.z)
            rays += 1
            nearest = nearest_vehicle_hit(world.cast_ray(start, end), start)
            if nearest is None:
                continue
            hits += 1
            above = nearest.location.z - origin.z
            tallest = above if tallest is None else max(tallest, above)
    if tallest is None:
        return None
    return dict(roof_above_origin_m=tallest, rays=rays, hits=hits,
                note='actor origin sits at the wheel contact plane')


def lidar_points(world, carla, actor, base):
    """World coordinates (N x 3 numpy array) of the LiDAR points on `actor`, or None.

    Spawns the LiDARs, lets each of them turn at least once, and destroys them again.
    """
    import numpy as np
    sensor_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    for key, value in LIDAR_ATTRIBUTES.items():
        sensor_bp.set_attribute(key, value)
    sensor_bp.set_attribute('rotation_frequency', str(LIDAR_ROTATION_HZ))
    poses = [(0.0, 0.0, LIDAR_TOP_M)] + [
        (LIDAR_RADIUS_M*math.cos(2*math.pi*k/LIDAR_AZIMUTHS),
         LIDAR_RADIUS_M*math.sin(2*math.pi*k/LIDAR_AZIMUTHS), height)
        for k in range(LIDAR_AZIMUTHS) for height in LIDAR_HEIGHTS_M]
    frames, sensors = queue.Queue(), []
    try:
        for x, y, z in poses:
            sensor = world.spawn_actor(sensor_bp, carla.Transform(
                carla.Location(base.x + x, base.y + y, base.z + z)))
            sensors.append(sensor)
            sensor.listen(frames.put)
        delta = world.get_settings().fixed_delta_seconds or 1.0/LIDAR_ROTATION_HZ
        for _ in range(math.ceil(1.0/(LIDAR_ROTATION_HZ*delta)) + 2):
            tick(world)
        time.sleep(0.5)     # the last frames arrive after the tick returns
    finally:
        for sensor in sensors:
            sensor.stop()
        for sensor in sensors:
            sensor.destroy()
    points = []
    while not frames.empty():
        frame = frames.get()
        data = np.frombuffer(frame.raw_data, dtype=np.dtype(LIDAR_POINT))
        data = data[data['idx'] == actor.id]
        if len(data):
            local = np.c_[data['x'], data['y'], data['z'], np.ones(len(data))]
            points.append((local @ np.array(frame.transform.get_matrix()).T)[:, :3])
    return np.vstack(points) if points else None


def measure(world, carla, blueprint, origin, step=0.10, reach=6.0):
    """Body length/width/height of one parked actor (yaw 0, so x is forward), plus its box."""
    blueprint.set_attribute('role_name', 'mesh_measurement')
    actor = world.spawn_actor(blueprint, carla.Transform(origin))
    try:
        tick(world)
        actor.set_simulate_physics(False)
        tick(world)
        box = actor.bounding_box
        half_l, half_w = box.extent.x, box.extent.y
        base = actor.get_transform().location

        # Sweep a little beyond the reported box so an inflated OR a clipped box
        # is still fully covered by the sweep.
        def grid(half):
            count = int((half*1.2)/step)
            return [i*step for i in range(-count, count+1)]

        width = span(world, carla, base, 'y', grid(half_l), HEIGHTS_M, reach, step)
        length = span(world, carla, base, 'x', grid(half_w), HEIGHTS_M, reach, step)
        height = height_span(world, carla, base, grid(half_l)[::3], grid(half_w)[::2], 12.0)
        points = lidar_points(world, carla, actor, base)
        record = dict(blueprint=actor.type_id,
                      base_type=blueprint.get_attribute('base_type').as_str()
                      if blueprint.has_attribute('base_type') else '',
                      box_length_m=2*box.extent.x, box_width_m=2*box.extent.y,
                      box_height_m=2*box.extent.z,
                      box_center_x_m=box.location.x, box_center_y_m=box.location.y,
                      lidar_points=0 if points is None else len(points),
                      castray_width=width, castray_length=length, castray_height=height)
        if points is not None:
            low, high = points.min(axis=0), points.max(axis=0)
            record['mesh_length_m'] = float(high[0] - low[0])
            record['mesh_width_m'] = float(high[1] - low[1])
            record['mesh_height_m'] = float(high[2] - base.z)
            # Where the body sits relative to the actor origin. A body that is not centred on
            # the origin is displaced from where the bridge puts it unless the bridge corrects it.
            record['mesh_center_x_m'] = float((high[0] + low[0])/2 - base.x)
            record['mesh_center_y_m'] = float((high[1] + low[1])/2 - base.y)
            record['width_inflation_m'] = record['box_width_m'] - record['mesh_width_m']
            record['length_inflation_m'] = record['box_length_m'] - record['mesh_length_m']
        if width and length:
            record['castray_width_m'] = width['extent_m']
            record['castray_length_m'] = length['extent_m']
            record['castray_center_x_m'] = length['center'] - base.x
            record['castray_center_y_m'] = width['center'] - base.y
        if height:
            record['castray_height_m'] = height['roof_above_origin_m']
        return record
    finally:
        actor.destroy()
        for _ in range(4):
            tick(world)
        if actor.id in {a.id for a in world.get_actors()}:
            raise RuntimeError(f'Failed to clean measurement actor {actor.id}')


def selected(blueprint_id, only, exclude):
    """Whether to measure this blueprint (--only: exact ids, --exclude: fnmatch patterns)."""
    if only and blueprint_id not in only:
        return False
    return not any(fnmatch.fnmatchcase(blueprint_id, pattern) for pattern in exclude)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', type=Path, required=True, help='output JSON file (must not exist)')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--step', type=float, default=0.10, help='ray spacing [m]')
    parser.add_argument('--only', nargs='*', default=[], help='measure only these blueprint ids')
    parser.add_argument('--exclude', nargs='*', default=[],
                        help='skip blueprints matching these fnmatch patterns')
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error(f'{args.out} already exists')
    args.out.parent.mkdir(parents=True, exist_ok=True)

    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(30)
    world = client.get_world()
    tick(world)
    occupied = [a.get_location() for a in world.get_actors().filter('vehicle.*')]
    records = []
    library = sorted(world.get_blueprint_library().filter('vehicle.*'), key=lambda bp: bp.id)
    for index, blueprint in enumerate(library):
        if not selected(blueprint.id, args.only, args.exclude):
            continue
        origin = carla.Location(x=PARK_X_M + index*PARK_PITCH_M, y=PARK_Y_M, z=PARK_Z_M)
        if any(origin.distance(other) < 20 for other in occupied):
            raise RuntimeError('Measurement parking occupied')
        records.append(measure(world, carla, blueprint, origin, args.step))
        print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v)
                          for k, v in records[-1].items()
                          if k in ('blueprint', 'mesh_width_m', 'mesh_length_m', 'mesh_height_m',
                                   'mesh_center_x_m', 'mesh_center_y_m', 'castray_width_m',
                                   'castray_length_m', 'lidar_points')},
                         ensure_ascii=False), flush=True)
    args.out.write_text(
        # The mesh of a blueprint can change with a CARLA update while this file stays the
        # same. The bridge compares this version with the server it connects to.
        json.dumps(dict(host=args.host, port=args.port, map=world.get_map().name,
                        carla_server_version=client.get_server_version(),
                        carla_client_version=client.get_client_version(), step_m=args.step,
                        heights_m=list(HEIGHTS_M), vehicle_labels=list(VEHICLE_LABELS),
                        lidar=dict(top_m=LIDAR_TOP_M, radius_m=LIDAR_RADIUS_M,
                                   azimuths=LIDAR_AZIMUTHS, heights_m=list(LIDAR_HEIGHTS_M),
                                   rotation_hz=LIDAR_ROTATION_HZ, **LIDAR_ATTRIBUTES),
                        excluded=list(args.exclude),
                        method='mesh_*_m: semantic ray-cast LiDAR points on a parked '
                               'physics-disabled actor; castray_*_m: world.cast_ray sweep, '
                               'nearest vehicle-labelled hit per ray (for comparison only)',
                        limitations=['LiDAR points are discrete: a thin protrusion between beams '
                                     'can be missed (a few cm on passenger cars)',
                                     'Two-wheelers include their rider when the blueprint has one'],
                        measurements=records), ensure_ascii=False, indent=2)+'\n')
    print(f'[measure_blueprint_mesh] {args.out}: {len(records)} blueprint(s)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
