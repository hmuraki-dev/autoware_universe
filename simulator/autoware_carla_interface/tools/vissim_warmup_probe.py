#!/usr/bin/env python3
"""
Step V0 probe for the Vissim warmup / EGO safe spawn plan (see docs/
Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md, Step V0 items 1, 2 and 6).

Talks directly to a running Windows-side vissim adapter (Co-Simulation/PTV-Vissim_windows/
server.py) over ZeroMQ - no CARLA, no Autoware - and sends nothing but *empty* 'tick' requests
(no spawn/update/destroy), exactly what the planned WARMUP phase will send. Measures:

  1. whether Vissim keeps advancing on empty ticks (the tick count and the vehicle count grow),
  2. the round-trip time of one tick, and hence the wall-clock time a warmup would take,
  6. how the number of vissim vehicles/pedestrians evolves over simulated time, to help choose
     vissim_warmup_time.

Not installed with the package; run it from the source tree on the Linux machine (needs only
pyzmq and msgpack, like the co-simulation itself):

    python3 tools/vissim_warmup_probe.py --host 192.168.16.56 --duration 300 --csv probe.csv

Restart the adapter (and Vissim) before each run: the adapter keeps Vissim running with the
first connect's sim_period/sim_res and rejects a later connect with different values. At the end,
'disconnect' is sent, which closes Vissim (pass --no-disconnect to leave it running).
"""

import argparse
import csv
import os
import statistics
import sys
import time

import zmq

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from autoware_carla_interface.vissim_integration import rpc_protocol as rpc  # noqa: E402

# Same value as vissim_integration/constants.py VISSIM_SIM_PERIOD_MARGIN_S (not imported, so this
# probe keeps working even if that module later grows carla-dependent imports).
SIM_PERIOD_MARGIN_S = 10


class AdapterClient(object):
    """Minimal REQ client for the vissim adapter; one request at a time."""

    def __init__(self, endpoint, rpc_timeout_ms):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.RCVTIMEO, rpc_timeout_ms)
        self._socket.connect(endpoint)
        self._rpc_timeout_ms = rpc_timeout_ms
        self._seq = 0

    def request(self, msg_type, payload, timeout_ms=None):
        """Returns the response payload; raises RuntimeError on a timeout or an adapter error."""
        self._seq += 1
        if timeout_ms is not None:
            self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        try:
            self._socket.send(rpc.encode_request(msg_type, self._seq, payload))
            data = self._socket.recv()
        except zmq.error.Again:
            raise RuntimeError('timed out waiting for a %r reply' % msg_type)
        finally:
            if timeout_ms is not None:
                self._socket.setsockopt(zmq.RCVTIMEO, self._rpc_timeout_ms)
        response = rpc.decode_response(data, expected_seq=self._seq)
        if not response['ok']:
            raise RuntimeError('adapter reported an error for %r: %s' %
                               (msg_type, response['error']))
        return response['payload']

    def close(self):
        self._socket.close(linger=0)
        self._context.term()


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


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1', help='vissim adapter IP (default: %(default)s)')
    parser.add_argument('--port', type=int, default=5555, help='vissim adapter port (default: %(default)s)')
    parser.add_argument('--step-length', type=float, default=0.05,
                        help='fixed_delta_seconds of the co-simulation, 1/N s (default: %(default)s)')
    parser.add_argument('--duration', type=int, default=300,
                        help='simulated seconds to run (default: %(default)s)')
    parser.add_argument('--simulator-vehicles', type=int, default=1,
                        help='same as vissim_simulator_vehicles (default: %(default)s)')
    parser.add_argument('--report-interval', type=float, default=10.0,
                        help='simulated seconds between progress lines (default: %(default)s)')
    parser.add_argument('--connect-timeout-ms', type=int, default=60000,
                        help='timeout for the connect request (default: %(default)s)')
    parser.add_argument('--rpc-timeout-ms', type=int, default=2000,
                        help='timeout for each tick request (default: %(default)s)')
    parser.add_argument('--csv', metavar='PATH', help='write one row per simulated second to PATH')
    parser.add_argument('--no-disconnect', action='store_true',
                        help="do not send 'disconnect' at the end (leaves Vissim running)")
    args = parser.parse_args()

    steps_per_second = 1.0 / args.step_length
    sim_res = int(round(steps_per_second))
    if abs(steps_per_second - sim_res) > 1e-6 or not 1 <= sim_res <= 20:
        parser.error('--step-length must be 1/N s with N in [1, 20]')
    if args.duration < 1:
        parser.error('--duration must be >= 1')

    total_ticks = args.duration * sim_res
    report_every = max(1, int(round(args.report_interval * sim_res)))
    endpoint = 'tcp://%s:%d' % (args.host, args.port)
    client = AdapterClient(endpoint, args.rpc_timeout_ms)

    connect_payload = {
        'step_length': args.step_length,
        'simulator_vehicles': args.simulator_vehicles,
        'sim_period': args.duration + SIM_PERIOD_MARGIN_S,
        'sim_res': sim_res,
    }
    print('Connecting to %s (sim_period=%d s, sim_res=%d)...' %
          (endpoint, connect_payload['sim_period'], sim_res))
    start = time.monotonic()
    try:
        client.request(rpc.MSG_CONNECT, connect_payload, timeout_ms=args.connect_timeout_ms)
    except RuntimeError as e:
        print('Error: connect failed: %s' % e)
        client.close()
        return 1
    print('Connected in %.1f s. Sending %d empty ticks (%d s of simulated time)...' %
          (time.monotonic() - start, total_ticks, args.duration))

    rtts = []
    interval_rtts = []
    per_second = []  # (sim_time, vehicles, pedestrians, signals, wall_elapsed)
    exit_code = 0
    empty_tick = {'spawn': [], 'destroy': [], 'update': []}
    loop_start = time.monotonic()
    try:
        for tick in range(1, total_ticks + 1):
            t0 = time.monotonic()
            payload = client.request(rpc.MSG_TICK, empty_tick)
            rtt = time.monotonic() - t0
            rtts.append(rtt)
            interval_rtts.append(rtt)

            if tick % sim_res == 0:
                per_second.append((tick / sim_res, len(payload.get('vehicles', [])),
                                   len(payload.get('pedestrians', [])),
                                   len(payload.get('signals', [])),
                                   time.monotonic() - loop_start))
            if tick % report_every == 0:
                sim_time, vehicles, pedestrians, _, wall = per_second[-1]
                print('t=%6.1f s  vehicles=%4d  pedestrians=%4d  rtt mean=%.1f ms max=%.1f ms  '
                      'wall=%.1f s' % (sim_time, vehicles, pedestrians,
                                       1000 * statistics.mean(interval_rtts),
                                       1000 * max(interval_rtts), wall))
                interval_rtts = []
    except (RuntimeError, rpc.ProtocolError) as e:
        print('Error: tick %d failed: %s' % (len(rtts) + 1, e))
        exit_code = 1
    except KeyboardInterrupt:
        print('Interrupted after %d tick(s).' % len(rtts))
        exit_code = 1

    wall_total = time.monotonic() - loop_start

    if not args.no_disconnect:
        try:
            client.request(rpc.MSG_DISCONNECT, {}, timeout_ms=args.connect_timeout_ms)
            print('Disconnected (Vissim closed).')
        except (RuntimeError, rpc.ProtocolError) as e:
            print('Warning: disconnect failed: %s' % e)
    client.close()

    if args.csv and per_second:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['sim_time_s', 'vehicles', 'pedestrians', 'signals', 'wall_s'])
            writer.writerows(per_second)
        print('Wrote %s' % args.csv)

    if not rtts:
        return exit_code or 1

    sim_done = len(rtts) / sim_res
    ms_per_tick = 1000 * wall_total / len(rtts)
    print('')
    print('== summary ==')
    print('ticks completed      : %d / %d (%.1f s simulated)' % (len(rtts), total_ticks, sim_done))
    print('wall-clock           : %.1f s (%.1fx real time)' % (wall_total, sim_done / wall_total))
    print('tick rtt             : mean %.1f ms, p50 %.1f ms, p95 %.1f ms, max %.1f ms' %
          (1000 * statistics.mean(rtts), 1000 * _percentile(rtts, 0.5),
           1000 * _percentile(rtts, 0.95), 1000 * max(rtts)))
    for warmup_s in (60, 100, 300):
        print('est. warmup %3d s    : %.1f s wall-clock' %
              (warmup_s, warmup_s * sim_res * ms_per_tick / 1000.0))
    if per_second:
        counts = [(t, vehicles) for t, vehicles, _, _, _ in per_second]
        print('vehicles at end      : %d (max %d)' % (counts[-1][1], max(c for _, c in counts)))
        hint = _suggest_warmup_time(counts, window_s=60.0, tolerance=0.1)
        if hint is None:
            print('warmup hint          : run too short / count not settled (try a longer --duration)')
        else:
            print('warmup hint          : vehicle count stays within +-10%% of the last 60 s mean '
                  'from t=%.0f s (rough hint only - check the CSV)' % hint)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
