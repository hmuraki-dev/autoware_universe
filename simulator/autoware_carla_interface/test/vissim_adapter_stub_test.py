#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Adapted from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/util/vissim_adapter_stub_test.py`, `feature/vissim_windows` branch)
# for `autoware_carla_interface`'s vendored `vissim_integration` package. See
# ../src/autoware_carla_interface/vissim_integration/NOTICE.md and
# ../docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md (Step W7) for the deviations
# made when adapting this file:
#   - upstream drives the *real* Windows-side adapter code (`Co-Simulation/PTV-Vissim_windows/
#     server.py` + `vissim_kernel_session.py`, with its ctypes `VISSIM_Veh_Data`/etc. rows faked
#     via monkeypatched `_call_*()` DLL-wrapper methods) against the real `PTVVissimSimulation`
#     client. This repository does not vendor that Windows-side adapter code at all (see
#     docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md section 8), so there is no
#     equivalent adapter module to import here. Instead, this file implements a minimal hand-rolled
#     ZeroMQ REP loop directly in terms of the wire protocol (`rpc_protocol.py` request/response
#     dicts) as the "fake adapter" - this is actually simpler than upstream's version, since the
#     ctypes struct layer (`VISSIM_Veh_Data` etc.) never exists on the Linux/client side of this
#     repository's `vissim_simulation.py` to begin with (it was removed entirely when the file was
#     rewritten as a ZeroMQ client - see Step W2).
#   - imports use `autoware_carla_interface.vissim_integration.*` instead of the upstream
#     `vissim_integration.*`.
"""
Loopback integration test for the Vissim(Windows) <-> CARLA(Linux) remote co-simulation link, from
this repository's (Linux/client) side: runs a minimal hand-rolled ZeroMQ REP loop (standing in for
the real Windows-side adapter, which is not vendored into this repository) in a background thread
bound to an OS-assigned local port, and drives it with the real `PTVVissimSimulation` ZeroMQ REQ
client (`autoware_carla_interface.vissim_integration.vissim_simulation`) - the exact same class
`autoware_carla_interface`'s `InitializeInterface._init_vissim_integration()` instantiates.

Unlike test/vissim_rpc_protocol_test.py (protocol encode/decode only, no sockets), this test
exercises the real network path end-to-end: msgpack encoding, actual TCP sockets via ZeroMQ, and
both sides of the wire protocol talking to each other exactly as they would in production (with a
fake standing in only for the actual Vissim Kernel/DLL access on the Windows side).

Run with:
    python3 test/vissim_adapter_stub_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import os
import sys
import threading
import time
import types

import carla  # pylint: disable=import-error

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from autoware_carla_interface.vissim_integration import rpc_protocol as rpc  # noqa: E402
from autoware_carla_interface.vissim_integration.vissim_simulation import (  # noqa: E402
    PTVVissimSimulation, VissimSignalState)

# ==================================================================================================
# -- fake adapter --------------------------------------------------------------------------------
# ==================================================================================================


class FakeAdapter(object):
    """
    Minimal hand-rolled stand-in for the (not vendored) Windows-side adapter's ZeroMQ REP loop.
    Records the 'tick' commands it receives (for assertions) and replies with whatever
    `next_tick_response` currently holds (mutated by the test between `client.tick()` calls, the
    same way a real Vissim Kernel's state would change from one tick to the next).
    """
    def __init__(self):
        self.next_tick_response = {'vehicles': [], 'pedestrians': [], 'signals': []}
        self.last_tick_request = None
        self.tick_delay_s = 0.0
        self._tick_call_count = 0

    def serve_forever(self, endpoint_holder, stop_event):
        ctx = None
        sock = None
        try:
            import zmq
            ctx = zmq.Context()
            sock = ctx.socket(zmq.REP)
            port = sock.bind_to_random_port('tcp://127.0.0.1')
            endpoint_holder['port'] = port
            sock.setsockopt(zmq.RCVTIMEO, 200)

            while not stop_event.is_set():
                try:
                    data = sock.recv()
                except zmq.error.Again:
                    continue

                req = rpc.decode_request(data)
                if req['type'] == rpc.MSG_CONNECT:
                    sock.send(rpc.encode_response(req['seq'], True, {}))
                elif req['type'] == rpc.MSG_TICK:
                    self._tick_call_count += 1
                    if self._tick_call_count == 1 and self.tick_delay_s:
                        time.sleep(self.tick_delay_s)
                    self.last_tick_request = req['payload']
                    sock.send(rpc.encode_response(req['seq'], True, dict(self.next_tick_response)))
                elif req['type'] == rpc.MSG_DISCONNECT:
                    sock.send(rpc.encode_response(req['seq'], True, {}))
        finally:
            if sock is not None:
                sock.close(linger=0)
            if ctx is not None:
                ctx.term()


def _start_adapter(adapter):
    """
    Starts `adapter.serve_forever()` in a background daemon thread, bound to an OS-assigned
    ephemeral local port. Returns (host, port, stop_event) once the adapter has confirmed it is
    bound and listening.
    """
    endpoint_holder = {}
    stop_event = threading.Event()
    thread = threading.Thread(target=adapter.serve_forever,
                              args=(endpoint_holder, stop_event),
                              daemon=True)
    thread.start()

    deadline = time.time() + 5.0
    while 'port' not in endpoint_holder:
        if time.time() > deadline:
            raise RuntimeError('fake vissim adapter did not start listening within 5s')
        time.sleep(0.01)

    return '127.0.0.1', endpoint_holder['port'], stop_event


def _make_client(host, port, simulator_vehicles=5, rpc_timeout_ms=2000):
    client_args = types.SimpleNamespace(simulator_vehicles=simulator_vehicles,
                                        step_length=0.05,
                                        vissim_adapter_host=host,
                                        vissim_adapter_port=port,
                                        vissim_connect_timeout_ms=2000,
                                        vissim_rpc_timeout_ms=rpc_timeout_ms)
    return PTVVissimSimulation(client_args)


# ==================================================================================================
# -- checks ------------------------------------------------------------------------------------------
# ==================================================================================================


def check_spawn_update_destroy_roundtrip():
    adapter = FakeAdapter()
    host, port, stop_event = _start_adapter(adapter)
    client = _make_client(host, port)

    try:
        # -- spawn: buffered client-side until the next tick() --
        local_id = client.spawn_actor(
            carla.Transform(carla.Location(10.0, 20.0, 0.0), carla.Rotation(0.0, 90.0, 0.0)))
        assert local_id != -1

        # tick 1: adapter echoes back one genuine NPC vehicle (VehicleID unrelated to local_id -
        # the adapter, not vendored here, owns that resolution entirely, see NOTICE.md).
        adapter.next_tick_response = {
            'vehicles': [{
                'id': 999, 'type': 100, 'model_filename': 'npc.v3d', 'color': 0,
                'x': 50.0, 'y': 60.0, 'z': 0.0, 'pitch': 0.0, 'heading': 0.0, 'speed': 8.0,
                'turn_indicator': 0,
            }],
            'pedestrians': [],
            'signals': [],
        }
        client.tick()
        assert adapter.last_tick_request['spawn'][0]['local_id'] == local_id
        assert client.spawned_vehicles == {999}
        assert client.destroyed_vehicles == set()
        npc = client.get_actor(999)
        assert npc.get_transform().location.x == 50.0

        # -- update: buffered client-side until the next tick() --
        assert client.synchronize_vehicle(
            local_id, carla.Transform(carla.Location(11.0, 21.0, 0.0), carla.Rotation(0, 90, 0)),
            carla.Vector3D(5.0, 0.0, 0.0))
        adapter.next_tick_response = {'vehicles': [], 'pedestrians': [], 'signals': []}  # NPC left
        client.tick()
        assert adapter.last_tick_request['update'][0]['local_id'] == local_id
        assert adapter.last_tick_request['update'][0]['x'] == 11.0
        assert client.destroyed_vehicles == {999}

        # -- destroy: buffered client-side until the next tick() --
        assert client.destroy_actor(local_id)
        client.tick()
        assert adapter.last_tick_request['destroy'] == [local_id]
    finally:
        client.close()
        stop_event.set()


def check_capacity_limit_enforced_client_side():
    adapter = FakeAdapter()
    host, port, stop_event = _start_adapter(adapter)
    client = _make_client(host, port, simulator_vehicles=1)

    try:
        transform = carla.Transform(carla.Location(0.0, 0.0, 0.0), carla.Rotation(0.0, 0.0, 0.0))
        first = client.spawn_actor(transform)
        assert first != -1

        from autoware_carla_interface.vissim_integration import constants
        second = client.spawn_actor(transform)
        assert second == constants.INVALID_ACTOR_ID
    finally:
        client.close()
        stop_event.set()


def check_signals_and_pedestrians_pass_through():
    adapter = FakeAdapter()
    host, port, stop_event = _start_adapter(adapter)
    client = _make_client(host, port)

    try:
        adapter.next_tick_response = {
            'vehicles': [],
            'pedestrians': [{
                'id': 7, 'type': 100, 'model_filename': 'man.v3d', 'length': 0.5, 'width': 0.5,
                'height': 1.8, 'x': 1.0, 'y': 2.0, 'z': 0.0, 'pitch': 0.0, 'heading': 0.0,
                'speed': 1.2, 'motion_state': 18,
            }],
            'signals': [{'controller_id': 1, 'signal_group_id': 2, 'state': 3}],
        }
        client.tick()

        assert client.spawned_pedestrians == {7}
        pedestrian = client.get_pedestrian(7)
        assert pedestrian.get_transform().location.x == 1.0
        assert client.get_signal_state((1, 2)) == VissimSignalState.GREEN
    finally:
        client.close()
        stop_event.set()


def check_timeout_triggers_reconnect():
    """
    Makes the adapter's first tick() reply slow (simulating an unresponsive/slow Vissim) so that
    the client's tick() request times out client-side even though the adapter eventually replies.
    Verifies that the client does not raise, recreates its ZeroMQ socket, flags that a reconnect
    is needed, and successfully resends 'connect' before the next tick() succeeds.
    """
    adapter = FakeAdapter()
    adapter.tick_delay_s = 0.5
    host, port, stop_event = _start_adapter(adapter)
    client = _make_client(host, port, rpc_timeout_ms=150)

    try:
        # This tick's request times out client-side (adapter is still sleeping in the slow first
        # call) - must not raise, and must flag that a reconnect is needed before the next tick().
        client.tick()
        assert client._needs_reconnect is True  # pylint: disable=protected-access
        assert client.spawned_vehicles == set()

        # Give the (now-abandoned, from the client's point of view) slow adapter-side handler
        # time to actually finish, so the adapter loop is free to service the reconnect below.
        time.sleep(0.6)

        # The next tick() must resend 'connect' first, then proceed with the tick normally.
        client.tick()
        assert client._needs_reconnect is False  # pylint: disable=protected-access
    finally:
        client.close()
        stop_event.set()


# ==================================================================================================
# -- entry point -------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    check_spawn_update_destroy_roundtrip()
    check_capacity_limit_enforced_client_side()
    check_signals_and_pedestrians_pass_through()
    check_timeout_triggers_reconnect()
    print('All vissim_adapter loopback checks passed.')


if __name__ == '__main__':
    run()
