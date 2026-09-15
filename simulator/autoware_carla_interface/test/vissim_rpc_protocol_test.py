#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Adapted from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/util/rpc_protocol_test.py`, `feature/vissim_windows` branch) for
# `autoware_carla_interface`'s vendored `vissim_integration` package. See
# ../src/autoware_carla_interface/vissim_integration/NOTICE.md and
# ../docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md (Step W7) for the deviations
# made when adapting this file:
#   - imports use `autoware_carla_interface.vissim_integration.rpc_protocol` instead of the
#     upstream `vissim_integration.rpc_protocol`.
#   - `check_windows_vendored_copy_is_identical()` was dropped: unlike upstream, this repository
#     does not vendor a second byte-identical copy of `rpc_protocol.py` for a self-contained
#     Windows-side adapter folder (`Co-Simulation/PTV-Vissim_windows/` stays external, not part of
#     this repository - see docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md
#     section 8), so there is no local second copy to compare against.
"""
Stub/unit test for vissim_integration/rpc_protocol.py (no ZeroMQ socket, no real Vissim/CARLA
needed): exercises encode/decode round-trips for every message type, and the error paths
(protocol version mismatch, unknown message type, response seq mismatch, malformed bytes).

Run with:
    python3 test/vissim_rpc_protocol_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from autoware_carla_interface.vissim_integration import rpc_protocol as rpc  # noqa: E402

# ==================================================================================================
# -- checks ------------------------------------------------------------------------------------------
# ==================================================================================================


def check_connect_roundtrip():
    payload = {'step_length': 0.05, 'simulator_vehicles': 1}
    data = rpc.encode_request(rpc.MSG_CONNECT, seq=1, payload=payload)
    decoded = rpc.decode_request(data)

    assert decoded['v'] == rpc.PROTO_VERSION
    assert decoded['seq'] == 1
    assert decoded['type'] == rpc.MSG_CONNECT
    assert decoded['payload'] == payload

    resp_data = rpc.encode_response(seq=1, ok=True, payload={})
    resp = rpc.decode_response(resp_data, expected_seq=1)
    assert resp['ok'] is True
    assert resp['error'] is None
    assert resp['payload'] == {}


def check_tick_roundtrip():
    payload = {
        'spawn': [{'local_id': 3, 'x': 10.0, 'y': 20.0, 'z': 0.0, 'heading': 1.57, 'pitch': 0.0}],
        'destroy': [5, 7],
        'update': [{
            'local_id': 2,
            'x': 1.0,
            'y': 2.0,
            'z': 0.0,
            'heading': 0.1,
            'pitch': 0.0,
            'speed': 8.3,
        }],
    }
    data = rpc.encode_request(rpc.MSG_TICK, seq=42, payload=payload)
    decoded = rpc.decode_request(data)
    assert decoded['type'] == rpc.MSG_TICK
    assert decoded['payload'] == payload

    response_payload = {
        'vehicles': [{
            'id': 1042,
            'type': 100,
            'model_filename': 'car.v3d',
            'color': 16711680,
            'x': 5.0,
            'y': 6.0,
            'z': 0.0,
            'pitch': 0.0,
            'heading': 0.2,
            'speed': 10.0,
            'turn_indicator': 0,
        }],
        'pedestrians': [],
        'signals': [{'controller_id': 1, 'signal_group_id': 2, 'state': 3}],
    }
    resp_data = rpc.encode_response(seq=42, ok=True, payload=response_payload)
    resp = rpc.decode_response(resp_data, expected_seq=42)
    assert resp['ok'] is True
    assert resp['payload'] == response_payload


def check_disconnect_roundtrip():
    data = rpc.encode_request(rpc.MSG_DISCONNECT, seq=99, payload={})
    decoded = rpc.decode_request(data)
    assert decoded['type'] == rpc.MSG_DISCONNECT

    resp_data = rpc.encode_response(seq=99, ok=True)
    resp = rpc.decode_response(resp_data, expected_seq=99)
    assert resp['ok'] is True
    assert resp['payload'] == {}


def check_error_response():
    resp_data = rpc.encode_response(seq=7, ok=False, error='VISSIM connection failed: bad path')
    resp = rpc.decode_response(resp_data, expected_seq=7)
    assert resp['ok'] is False
    assert resp['error'] == 'VISSIM connection failed: bad path'


def check_unknown_message_type_rejected():
    try:
        rpc.encode_request('not_a_real_type', seq=1, payload={})
        assert False, 'expected ProtocolError for unknown request message type'
    except rpc.ProtocolError:
        pass


def check_version_mismatch_rejected():
    # Manually craft a request with a bogus protocol version, bypassing encode_request() (which
    # always stamps the current PROTO_VERSION), to simulate a client/adapter running mismatched
    # code versions.
    import msgpack
    bad_envelope = {'v': rpc.PROTO_VERSION + 1, 'seq': 1, 'type': rpc.MSG_TICK, 'payload': {}}
    data = msgpack.packb(bad_envelope, use_bin_type=True)

    try:
        rpc.decode_request(data)
        assert False, 'expected ProtocolError for protocol version mismatch'
    except rpc.ProtocolError:
        pass


def check_response_seq_mismatch_rejected():
    resp_data = rpc.encode_response(seq=1, ok=True, payload={})
    try:
        rpc.decode_response(resp_data, expected_seq=2)
        assert False, 'expected ProtocolError for response seq mismatch'
    except rpc.ProtocolError:
        pass


def check_malformed_bytes_rejected():
    try:
        rpc.decode_request(b'\xff\xff\xff not valid msgpack')
        assert False, 'expected ProtocolError for malformed bytes'
    except rpc.ProtocolError:
        pass


# ==================================================================================================
# -- entry point -------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    check_connect_roundtrip()
    check_tick_roundtrip()
    check_disconnect_roundtrip()
    check_error_response()
    check_unknown_message_type_rejected()
    check_version_mismatch_rejected()
    check_response_seq_mismatch_rejected()
    check_malformed_bytes_rejected()
    print('All rpc_protocol stub checks passed.')


if __name__ == '__main__':
    run()
