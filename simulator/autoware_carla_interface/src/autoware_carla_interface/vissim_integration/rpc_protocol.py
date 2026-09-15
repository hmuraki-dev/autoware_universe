#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Vendored from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/vissim_integration/rpc_protocol.py`, `feature/vissim_windows` branch).
# See ../NOTICE.md for the vendoring rationale and the one deviation made (this header comment).
"""
Shared ZeroMQ + msgpack message envelope for the Vissim(Windows) <-> CARLA(Linux) remote
co-simulation link.

This module is intentionally free of any dependency on `carla` or `ctypes`, and must be safely
importable on either side without pulling in platform/simulator-specific packages.

IMPORTANT: this file must stay content-identical to the upstream copies it is vendored from:
  - Co-Simulation/PTV-Vissim/vissim_integration/rpc_protocol.py (upstream Linux orchestrator)
  - Co-Simulation/PTV-Vissim_windows/rpc_protocol.py (upstream self-contained Windows adapter
    folder, meant to be copied wholesale to the Windows machine; not vendored into this repo, see
    docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md section 8)
In particular, PROTO_VERSION must always match what the Windows-side adapter speaks. Whenever the
upstream file changes, re-vendor this copy (see the implementation plan doc's Step W1/W6).

See docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md (section 0.2) for a summary of
the protocol, and the upstream repository's docs/WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md
(section 3) for the full specification.
"""

import msgpack

# ==================================================================================================
# -- protocol constants ------------------------------------------------------------------------------
# ==================================================================================================

# Bumped whenever the wire format (envelope shape or any message payload shape) changes in a
# backwards-incompatible way. Client and adapter must agree on this value - a mismatch is always
# treated as a hard error rather than guessed at.
PROTO_VERSION = 1

MSG_CONNECT = 'connect'
MSG_TICK = 'tick'
MSG_DISCONNECT = 'disconnect'

_VALID_MSG_TYPES = (MSG_CONNECT, MSG_TICK, MSG_DISCONNECT)


class ProtocolError(Exception):
    """
    Raised for any malformed/unexpected message: unknown message type, incompatible
    `PROTO_VERSION`, or a payload that fails basic structural validation. Deliberately a single
    exception type so callers can handle "this message cannot be trusted" uniformly, regardless of
    the specific reason.
    """


# ==================================================================================================
# -- request (client -> adapter) --------------------------------------------------------------------
# ==================================================================================================


def encode_request(msg_type, seq, payload):
    """
    Builds and msgpack-encodes a request envelope.

        :param str msg_type: one of MSG_CONNECT, MSG_TICK, MSG_DISCONNECT.
        :param int seq: monotonically increasing correlation id (see decode_response()).
        :param dict payload: message-type-specific payload.
        :return: bytes ready to send over the wire.
    """
    if msg_type not in _VALID_MSG_TYPES:
        raise ProtocolError('Unknown request message type: %r' % (msg_type, ))

    envelope = {
        'v': PROTO_VERSION,
        'seq': seq,
        'type': msg_type,
        'payload': payload,
    }
    return msgpack.packb(envelope, use_bin_type=True)


def decode_request(data):
    """
    Decodes and validates a request envelope received over the wire.

        :param bytes data: raw bytes as received from the socket.
        :return: dict with keys 'v', 'seq', 'type', 'payload'.
        :raises ProtocolError: if the message cannot be decoded, has an incompatible protocol
            version, or is missing/mis-typing any required envelope field.
    """
    envelope = _unpack(data)
    _check_common_envelope_fields(envelope)

    if envelope['type'] not in _VALID_MSG_TYPES:
        raise ProtocolError('Unknown request message type: %r' % (envelope.get('type'), ))

    return envelope


# ==================================================================================================
# -- response (adapter -> client) ---------------------------------------------------------------------
# ==================================================================================================


def encode_response(seq, ok, payload=None, error=None):
    """
    Builds and msgpack-encodes a response envelope.

        :param int seq: must echo the seq of the request being answered (see decode_response()).
        :param bool ok: whether the request was processed successfully.
        :param dict payload: message-type-specific response payload. Ignored/empty when ok=False.
        :param str error: human-readable error message. Only meaningful when ok=False.
        :return: bytes ready to send over the wire.
    """
    envelope = {
        'v': PROTO_VERSION,
        'seq': seq,
        'ok': bool(ok),
        'error': error,
        'payload': payload if payload is not None else {},
    }
    return msgpack.packb(envelope, use_bin_type=True)


def decode_response(data, expected_seq=None):
    """
    Decodes and validates a response envelope received over the wire.

        :param bytes data: raw bytes as received from the socket.
        :param int expected_seq: if given, the response's 'seq' must match this value - guards
            against a stale response from a previous (timed out) request being mistaken for the
            answer to the current one (see the implementation plan doc's Step W2 on ZeroMQ REQ
            socket recreation after a timeout).
        :return: dict with keys 'v', 'seq', 'ok', 'error', 'payload'.
        :raises ProtocolError: if the message cannot be decoded, has an incompatible protocol
            version, is missing/mis-typing any required envelope field, or does not match
            expected_seq.
    """
    envelope = _unpack(data)
    _check_common_envelope_fields(envelope)

    if 'ok' not in envelope:
        raise ProtocolError("Response envelope missing 'ok' field: %r" % (envelope, ))

    if expected_seq is not None and envelope['seq'] != expected_seq:
        raise ProtocolError('Response seq mismatch: expected %r, got %r' %
                            (expected_seq, envelope['seq']))

    return envelope


# ==================================================================================================
# -- shared helpers ----------------------------------------------------------------------------------
# ==================================================================================================


def _unpack(data):
    try:
        envelope = msgpack.unpackb(data, raw=False)
    except (msgpack.exceptions.UnpackException, ValueError) as e:
        raise ProtocolError('Could not decode message: %s' % e)

    if not isinstance(envelope, dict):
        raise ProtocolError('Decoded message is not a dict/map: %r' % (envelope, ))

    return envelope


def _check_common_envelope_fields(envelope):
    for field in ('v', 'seq', 'payload'):
        if field not in envelope:
            raise ProtocolError('Envelope missing required field %r: %r' % (field, envelope))

    if envelope['v'] != PROTO_VERSION:
        raise ProtocolError(
            'Protocol version mismatch: this side is PROTO_VERSION=%d, message declares v=%r. '
            'Client and adapter must be upgraded together.' % (PROTO_VERSION, envelope['v']))
