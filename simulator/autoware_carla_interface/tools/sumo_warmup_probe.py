#!/usr/bin/env python3
"""
Step S0 probe for the SUMO warmup / EGO safe spawn plan (see docs/
SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md, Step S0 items 1, 2, 5 and 6).

Starts SUMO headless through TraCI with the same options SumoSimulation uses (no CARLA, no
Autoware) and advances it without any CARLA-side vehicles, exactly what the planned WARMUP phase
will do. Measures:

  1. whether SUMO keeps advancing when nothing is synchronized from CARLA, and whether it keeps
     accepting simulationStep() after every vehicle has arrived (no <end> in the .sumocfg),
  2. the wall-clock time of one step (per-step loop), and of a single simulationStep(target)
     jump over the same simulated time (--mode jump), to pick the warmup implementation,
  5. the vehicle types that depart and their SUMO length/vClass, to check that each one maps to
     a CARLA blueprint (vtypes.json / blueprint id),
  6. how the number of vehicles/persons evolves over simulated time, to help choose
     sumo_warmup_time.

Not installed with the package; run it from the source tree where SUMO runs (needs only SUMO's
traci/sumolib on sys.path, i.e. SUMO_HOME):

    python3 tools/sumo_warmup_probe.py \
        --sumo-cfg /home/divp/CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg \
        --duration 300 --csv sumo_warmup_probe.csv
"""

import argparse
import csv
import os
import statistics
import sys
import time

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

import sumolib  # noqa: E402  pylint: disable=import-error,wrong-import-position
import traci  # noqa: E402  pylint: disable=import-error,wrong-import-position


def _percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


def _suggest_warmup_time(samples, window_s, tolerance):
    """
    Rough hint only: the earliest simulated time from which the vehicle count stays within
    +-tolerance of the mean of the last `window_s` seconds until the end of the run. Returns None
    if the run is too short to tell (shorter than two windows) or never settles.
    """
    if not samples or samples[-1][0] < 2 * window_s:
        return None
    end_time = samples[-1][0]
    tail = [count for t, count in samples if t >= end_time - window_s]
    reference = statistics.mean(tail)
    if reference <= 0:
        return None
    low, high = reference * (1.0 - tolerance), reference * (1.0 + tolerance)
    settled_from = None
    for t, count in samples:
        if low <= count <= high:
            if settled_from is None:
                settled_from = t
        else:
            settled_from = None
    return settled_from


def _start_sumo(args):
    # Same command line as sumo_integration/sumo_simulation.py SumoSimulation.__init__().
    traci.start([
        sumolib.checkBinary('sumo'),
        '--configuration-file', args.sumo_cfg,
        '--step-length', str(args.step_length),
        '--lateral-resolution', '0.25',
        '--collision.check-junctions',
        '--no-step-log', 'true',
    ])


def _run_step_mode(args, total_steps, steps_per_second, report_every):
    """One simulationStep() per co-simulation step, like a plain WARMUP loop."""
    step_times = []
    interval_times = []
    per_second = []  # (sim_time, vehicles, persons, departed_total, pending, expected, wall)
    departed_total = 0
    vtypes = {}  # type_id -> [departed count, length, vClass]
    all_arrived_at = None
    loop_start = time.monotonic()
    for step in range(1, total_steps + 1):
        t0 = time.monotonic()
        traci.simulationStep()
        elapsed = time.monotonic() - t0
        step_times.append(elapsed)
        interval_times.append(elapsed)

        for vehicle_id in traci.simulation.getDepartedIDList():
            departed_total += 1
            type_id = traci.vehicle.getTypeID(vehicle_id)
            if type_id not in vtypes:
                vtypes[type_id] = [0, traci.vehicletype.getLength(type_id),
                                   traci.vehicletype.getVehicleClass(type_id)]
            vtypes[type_id][0] += 1
        if all_arrived_at is None and traci.simulation.getMinExpectedNumber() == 0:
            all_arrived_at = traci.simulation.getTime()

        if step % steps_per_second == 0:
            per_second.append((traci.simulation.getTime(), traci.vehicle.getIDCount(),
                               traci.person.getIDCount(), departed_total,
                               len(traci.simulation.getPendingVehicles()),
                               traci.simulation.getMinExpectedNumber(),
                               time.monotonic() - loop_start))
        if step % report_every == 0:
            sim_time, vehicles, persons, departed, pending, _, wall = per_second[-1]
            print('t=%6.1f s  vehicles=%4d  persons=%4d  departed=%5d  pending=%4d  '
                  'step mean=%.2f ms max=%.2f ms  wall=%.1f s' %
                  (sim_time, vehicles, persons, departed, pending,
                   1000 * statistics.mean(interval_times), 1000 * max(interval_times), wall))
            interval_times = []
    return step_times, per_second, vtypes, all_arrived_at, time.monotonic() - loop_start


def _run_jump_mode(args):
    """A single simulationStep(target) over the whole duration."""
    t0 = time.monotonic()
    traci.simulationStep(float(args.duration))
    wall = time.monotonic() - t0
    return wall, traci.simulation.getTime(), traci.vehicle.getIDCount(), traci.person.getIDCount()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--sumo-cfg', required=True, help='path to the .sumocfg used by the co-sim')
    parser.add_argument('--step-length', type=float, default=0.05,
                        help='fixed_delta_seconds of the co-simulation (default: %(default)s)')
    parser.add_argument('--duration', type=int, default=300,
                        help='simulated seconds to run (default: %(default)s)')
    parser.add_argument('--mode', choices=('step', 'jump'), default='step',
                        help='step: one simulationStep() per step (default); '
                             'jump: a single simulationStep(duration)')
    parser.add_argument('--report-interval', type=float, default=10.0,
                        help='simulated seconds between progress lines (default: %(default)s)')
    parser.add_argument('--csv', metavar='PATH',
                        help='write one row per simulated second to PATH (step mode only)')
    args = parser.parse_args()

    steps_per_second = int(round(1.0 / args.step_length))
    if abs(1.0 / args.step_length - steps_per_second) > 1e-6:
        parser.error('--step-length must be 1/N s')
    if args.duration < 1:
        parser.error('--duration must be >= 1')

    total_steps = args.duration * steps_per_second
    report_every = max(1, int(round(args.report_interval * steps_per_second)))

    print('Starting SUMO (%s, step-length=%s s)...' % (args.sumo_cfg, args.step_length))
    start = time.monotonic()
    _start_sumo(args)
    print('Started in %.1f s.' % (time.monotonic() - start))

    exit_code = 0
    try:
        if args.mode == 'jump':
            wall, sim_time, vehicles, persons = _run_jump_mode(args)
            print('')
            print('== summary (jump) ==')
            print('simulationStep(%d)   : %.3f s wall-clock (%.0fx real time)' %
                  (args.duration, wall, sim_time / wall if wall > 0 else float('inf')))
            print('state after jump     : t=%.2f s vehicles=%d persons=%d' %
                  (sim_time, vehicles, persons))
            return 0

        step_times, per_second, vtypes, all_arrived_at, wall_total = _run_step_mode(
            args, total_steps, steps_per_second, report_every)
    except traci.exceptions.FatalTraCIError as e:
        print('Error: SUMO stopped during the run: %s' % e)
        return 1
    except KeyboardInterrupt:
        print('Interrupted.')
        return 1
    finally:
        try:
            traci.close()
        except Exception:  # pylint: disable=broad-except
            pass

    if args.csv and per_second:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['sim_time_s', 'vehicles', 'persons', 'departed_total', 'pending',
                             'min_expected', 'wall_s'])
            writer.writerows(per_second)
        print('Wrote %s' % args.csv)

    sim_done = len(step_times) / steps_per_second
    ms_per_step = 1000 * wall_total / len(step_times)
    print('')
    print('== summary (step) ==')
    print('steps completed      : %d / %d (%.1f s simulated)' %
          (len(step_times), total_steps, sim_done))
    print('wall-clock           : %.2f s (%.0fx real time)' % (wall_total, sim_done / wall_total))
    print('simulationStep()     : mean %.3f ms, p50 %.3f ms, p95 %.3f ms, max %.3f ms' %
          (1000 * statistics.mean(step_times), 1000 * _percentile(step_times, 0.5),
           1000 * _percentile(step_times, 0.95), 1000 * max(step_times)))
    for warmup_s in (60, 100, 300):
        print('est. warmup %3d s    : %.2f s wall-clock' %
              (warmup_s, warmup_s * steps_per_second * ms_per_step / 1000.0))
    counts = [(row[0], row[1]) for row in per_second]
    print('vehicles at end      : %d (max %d)' % (counts[-1][1], max(c for _, c in counts)))
    print('persons at end       : %d (max %d)' %
          (per_second[-1][2], max(row[2] for row in per_second)))
    if all_arrived_at is None:
        print('all arrived          : no (vehicles still running/expected at the end)')
    else:
        print('all arrived          : at t=%.2f s; SUMO kept stepping to t=%.1f s' %
              (all_arrived_at, per_second[-1][0]))
    hint = _suggest_warmup_time(counts, window_s=60.0, tolerance=0.1)
    if hint is None:
        print('warmup hint          : run too short / count not settled (try a longer --duration)')
    else:
        print('warmup hint          : vehicle count stays within +-10%% of the last 60 s mean '
              'from t=%.0f s (rough hint only - check the CSV)' % hint)
    print('departed vehicle types (%d):' % len(vtypes))
    print('  %-40s %6s %8s  %s' % ('type_id', 'count', 'length', 'vClass'))
    for type_id in sorted(vtypes):
        count, length, vclass = vtypes[type_id]
        print('  %-40s %6d %8.2f  %s' % (type_id, count, length, vclass))
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
