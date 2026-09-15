#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Vendored from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/vissim_integration/vissim_simulation.py`, `feature/vissim_windows`
# branch). See ../NOTICE.md for the deviations made when vendoring this file.
"""
This module is responsible for the management of the ptv-vissim simulation.

`PTVVissimSimulation` is a ZeroMQ REQ client talking to a Windows-side Vissim adapter
(`Co-Simulation/PTV-Vissim_windows/server.py` in the upstream CARLA repository - self-contained,
not vendored into this repository, see
docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md section 8), running next to the
actual PTV-Vissim Kernel/GUI/console instance and `DrivingSimulatorProxy.dll`. All DLL-specific
logic (ctypes structs, `Create`/`CreateID` pending/active state machine, DLL loading,
`VISSIM_Connect`/`VISSIM_ConnectToConsole`) now lives entirely on the Windows side.

`PTVVissimSimulation`'s public interface is unchanged from the previous in-process (ctypes)
implementation (`__init__(args)`, `tick()`, `spawned_vehicles`, `destroyed_vehicles`,
`spawned_pedestrians`, `destroyed_pedestrians`, `spawn_actor(transform)`,
`destroy_actor(actor_id)`, `synchronize_vehicle(vehicle_id, transform, velocity)`,
`get_actor(actor_id)`, `get_pedestrian(pedestrian_id)`, `get_signal_state(signal_id)`,
`signal_ids`, `tick_count`, `close()`), so `SimulationSynchronization`
(vissim_integration/simulation_synchronization.py) requires no changes.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import enum
import logging
import math

import carla  # pylint: disable=import-error
import zmq

from . import constants
from . import rpc_protocol as rpc

# ==================================================================================================
# -- vissim enums ------------------------------------------------------------------------------------
# ==================================================================================================


class VissimPedestrianMotionState(enum.Enum):
    """
    VissimPedestrianMotionState contains the different vissim pedestrian motion states.

    Values match enum Pedestrian_Motion_State_Type in DrivingSimulatorProxy.h (PTV Vissim Kernel
    for Linux 2026.00-10).
    """
    APPROACHING_PT_VEHICLE = 1
    ALIGHTING_FROM_PT_VEHICLE = 2
    WAITING_FOR_PT_VEHICLE = 3
    WALKING_UP_ON_ESCALATOR = 4
    WALKING_DOWN_ON_ESCALATOR = 5
    STANDING_ON_ESCALATOR = 6
    WALKING_ON_MOVING_WALKWAY = 7
    STANDING_ON_MOVING_WALKWAY = 8
    WAITING_AT_QUEUE_HEAD = 9
    WAITING_IN_QUEUE = 10
    WALKING_UPSTAIRS = 11
    WALKING_DOWNSTAIRS = 12
    APPROACHING_ELEVATOR = 13
    ALIGHTING_FROM_ELEVATOR = 14
    WAITING_FOR_ELEVATOR = 15
    RIDING_ELEVATOR = 16
    WAITING = 17
    WALKING_ON_LEVEL = 18
    END = 19


class VissimPedestrianConstructionElementType(enum.Enum):
    """
    VissimPedestrianConstructionElementType contains the different types of construction element a
    vissim pedestrian can currently be on.

    Values match enum Pedestrian_Construction_Element_Type in DrivingSimulatorProxy.h (PTV Vissim
    Kernel for Linux 2026.00-10).
    """
    NONE = 0
    AREA = 1
    RAMP = 2
    ELEVATOR_GROUP = 3
    PED_LINK = 4


class VissimLightState(enum.Enum):
    """
    VissimLightState contains the different vissim indicator states.
    """
    LEFT = 1
    NONE = 0
    RIGHT = -1


class VissimSignalState(enum.Enum):
    """
    VissimSignalState contains the different vissim signal group states.

    Values match enum SignalStateType in DrivingSimulatorProxy.h (PTV Vissim Kernel for Linux
    2026.00-10).
    """
    RED = 1
    RED_AMBER = 2
    GREEN = 3
    AMBER = 4
    OFF = 5
    UNDEFINED = 6
    FLASHING_AMBER = 7
    FLASHING_RED = 8
    FLASHING_GREEN = 9
    ALTERNATING_RED_GREEN = 10
    GREEN_AMBER = 11


class VissimVehicle(object):
    """
    VissimVehicle holds the data relative to traffic vehicles in vissim.
    """
    def __init__(self,
                 vehicle_id,
                 type_id,
                 model_filename,
                 color,
                 location,
                 rotation,
                 velocity,
                 lights_state=VissimLightState.NONE):
        # Static parameters.
        self.id = vehicle_id
        self.type = type_id
        self.model_filename = model_filename
        self.color = color

        # Dynamic attributes.
        loc = carla.Location(location[0], location[1], location[2])
        rot = carla.Rotation(math.degrees(rotation[0]), math.degrees(rotation[1]),
                             math.degrees(rotation[2]))
        self._transform = carla.Transform(loc, rot)
        self._velocity = carla.Vector3D(
            velocity * math.cos(math.radians(rot.yaw)) * math.cos(math.radians(rot.pitch)),
            velocity * math.sin(math.radians(rot.yaw)) * math.cos(math.radians(rot.pitch)),
            velocity * math.sin(math.radians(rot.pitch)))
        self._lights_state = lights_state

    def get_velocity(self):
        """
        Returns the vehicle's velocity.
        """
        return self._velocity

    def get_transform(self):
        """
        Returns carla transform.
        """
        return self._transform


class VissimPedestrian(object):
    """
    VissimPedestrian holds the data relative to traffic pedestrians in vissim.

    vissim -> carla direction only (see PEDESTRIAN_TODO.md): there is no carla -> vissim
    equivalent for pedestrians, so unlike VissimVehicle, no 'own actor' bookkeeping is needed.
    """
    def __init__(self,
                 pedestrian_id,
                 type_id,
                 model_filename,
                 extent,
                 location,
                 rotation,
                 velocity,
                 motion_state=None):
        # Static parameters.
        self.id = pedestrian_id
        self.type = type_id
        self.model_filename = model_filename
        self.extent = extent  # (length, width, height) in m, as reported by vissim.

        # Dynamic attributes.
        loc = carla.Location(location[0], location[1], location[2])
        rot = carla.Rotation(math.degrees(rotation[0]), math.degrees(rotation[1]),
                             math.degrees(rotation[2]))
        self._transform = carla.Transform(loc, rot)
        self._velocity = carla.Vector3D(
            velocity * math.cos(math.radians(rot.yaw)) * math.cos(math.radians(rot.pitch)),
            velocity * math.sin(math.radians(rot.yaw)) * math.cos(math.radians(rot.pitch)),
            velocity * math.sin(math.radians(rot.pitch)))
        self.motion_state = motion_state

    def get_velocity(self):
        """
        Returns the pedestrian's velocity.
        """
        return self._velocity

    def get_transform(self):
        """
        Returns carla transform.
        """
        return self._transform


# ==================================================================================================
# -- vissim simulation -----------------------------------------------------------------------------
# ==================================================================================================


class PTVVissimSimulation(object):
    """
    PTVVissimSimulation is responsible for the management of the vissim simulation.

    This is a ZeroMQ REQ client for the Windows-side Vissim adapter (see the module docstring
    above) - it holds no direct connection to Vissim or any DLL. `local_id`s used to identify
    carla-origin (Driving-Simulator) vehicles are allocated here and are otherwise opaque to the
    adapter; the adapter never needs to be told the real vissim VehicleID resolution, and this
    class never needs to know it either (see
    docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md section 0.2 for the full
    rationale of this simplification).
    """
    def __init__(self, args):
        self._max_simulator_vehicles = args.simulator_vehicles
        self._step_length = args.step_length

        self._endpoint = 'tcp://%s:%d' % (args.vissim_adapter_host, args.vissim_adapter_port)
        # Two distinct timeouts: 'connect' (sent once at startup, and again by _reconnect() after
        # a timeout) may need to wait much longer than a regular per-tick request - launching a
        # real Vissim GUI instance and loading its network can easily take tens of seconds, while
        # a normal 'tick' round trip is expected to be fast (see
        # docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md section 4, Step W2).
        self._connect_timeout_ms = args.vissim_connect_timeout_ms
        self._tick_timeout_ms = args.vissim_rpc_timeout_ms
        self._zmq_context = zmq.Context()
        self._socket = None
        self._seq = 0
        self._connect_socket()

        # Cached so a lost connection can be re-established (resending 'connect') without
        # needing the original args object again - see _reconnect() below.
        self._connect_payload = {
            'step_length': args.step_length,
            'simulator_vehicles': args.simulator_vehicles,
        }
        # Set whenever a request times out (see _request()): the next tick() must resend
        # 'connect' before anything else, since the adapter (and/or Vissim itself) may have been
        # restarted while we were disconnected (the adapter is expected to simply reset its own
        # state on a fresh 'connect'; we do not attempt to resynchronize previously-known
        # vehicles/pedestrians/signals here).
        self._needs_reconnect = False

        logging.info('Connecting to vissim adapter at %s...', self._endpoint)
        response = self._request(rpc.MSG_CONNECT, self._connect_payload,
                                 timeout_ms=self._connect_timeout_ms)
        if response is None:
            raise RuntimeError('Timed out connecting to the vissim adapter (%s) after %d ms' %
                              (self._endpoint, self._connect_timeout_ms))
        if not response['ok']:
            raise RuntimeError('There was an error connecting to the vissim adapter (%s): %s' %
                              (self._endpoint, response['error']))

        # Structures to keep track of the simulation state at each time step.
        # Real vissim VehicleID -> VissimVehicle, only for genuine vissim traffic (the adapter
        # already filters out ControlledByVissim == False rows).
        self._vissim_vehicles = {}

        # Real vissim PedestrianID -> VissimPedestrian. vissim -> carla direction only (see
        # PEDESTRIAN_TODO.md).
        self._vissim_pedestrians = {}

        # (ControllerID, SignalGroupID) -> VissimSignalState, refreshed every tick. Empty until
        # the first tick() (and remains empty if the network has no signal controllers).
        self._signal_states = {}

        self.spawned_vehicles = set()
        self.destroyed_vehicles = set()
        self.spawned_pedestrians = set()
        self.destroyed_pedestrians = set()
        self._tick_count = 0

        # Our own local_id (int, allocated here, opaque to the adapter beyond correlating
        # spawn/destroy/update commands) -> currently outstanding (spawned, not yet destroyed)
        # carla-origin vehicles. Used only for the client-side capacity check below - the actual
        # pending/active/CreateID state machine lives entirely on the adapter now.
        self._active_local_ids = set()
        self._next_local_id = 1

        # Commands buffered by spawn_actor()/destroy_actor()/synchronize_vehicle() since the last
        # tick(), to be sent together in the next 'tick' request (one round trip per simulation
        # step).
        self._pending_spawn = []  # list of {'local_id', 'x', 'y', 'z', 'heading', 'pitch'}
        self._pending_destroy = []  # list of local_id
        self._pending_update = {}  # local_id -> {'x', 'y', 'z', 'heading', 'pitch', 'speed'}

    def _connect_socket(self):
        """
        Creates a fresh ZeroMQ REQ socket, with `RCVTIMEO` set to the regular per-tick timeout
        (`self._tick_timeout_ms`) - the longer connect-specific timeout is applied temporarily by
        `_request(..., timeout_ms=...)` only for the duration of a 'connect' request (see below).

        Split out into its own method because a REQ socket that has timed out waiting for a reply
        must be closed and recreated before another request can be sent.
        """
        if self._socket is not None:
            self._socket.close(linger=0)
        self._socket = self._zmq_context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, self._tick_timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(self._endpoint)

    def _request(self, msg_type, payload, timeout_ms=None):
        """
        Sends a request to the vissim adapter and returns the decoded response envelope (a dict
        with keys 'v', 'seq', 'ok', 'error', 'payload'), or None if the request timed out.

            :param timeout_ms: if given, temporarily overrides the socket's `RCVTIMEO` for this
                request only (restored to `self._tick_timeout_ms` afterwards). Used for 'connect'
                requests (see __init__()/_reconnect()), which may need to wait much longer than a
                regular per-tick request - launching a real Vissim GUI instance can easily take
                tens of seconds.

        On a timeout (`zmq.error.Again`, raised once `RCVTIMEO` elapses), the REQ socket is closed
        and recreated before returning: a standard ZeroMQ REQ socket enforces a strict alternating
        send/recv state machine, and calling send() again on a socket that is still "waiting for a
        reply" raises `EFSM` - so the only way to safely try again after a timeout is a fresh
        socket. This also sets `self._needs_reconnect = True`, so the next tick() knows to resend
        'connect' first.
        """
        self._seq += 1
        seq = self._seq
        if timeout_ms is not None:
            self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        try:
            self._socket.send(rpc.encode_request(msg_type, seq, payload))
            data = self._socket.recv()
        except zmq.error.Again:
            logging.error(
                'Timed out waiting for a reply from the vissim adapter (%s) after %d ms '
                '(request type=%r) - recreating the ZeroMQ socket.', self._endpoint,
                timeout_ms if timeout_ms is not None else self._tick_timeout_ms, msg_type)
            self._connect_socket()
            self._needs_reconnect = True
            return None
        finally:
            if timeout_ms is not None and self._socket is not None:
                self._socket.setsockopt(zmq.RCVTIMEO, self._tick_timeout_ms)
        return rpc.decode_response(data, expected_seq=seq)

    def _reconnect(self):
        """
        Resends 'connect' (using the longer connect-specific timeout, see _request()) after a
        timeout-triggered socket recreation.

            :return: True if the adapter accepted the reconnection, False otherwise (in which
                case self._needs_reconnect is left set to True, so the next tick() tries again).
        """
        logging.info('Attempting to reconnect to vissim adapter at %s...', self._endpoint)
        response = self._request(rpc.MSG_CONNECT, self._connect_payload,
                                 timeout_ms=self._connect_timeout_ms)
        if response is None:
            return False
        if not response['ok']:
            logging.error('Reconnection to vissim adapter (%s) was rejected: %s', self._endpoint,
                         response['error'])
            return False
        logging.info('Reconnected to vissim adapter (%s).', self._endpoint)
        self._needs_reconnect = False
        return True

    # ----------------------------------------------------------------------------------------------
    # -- carla-origin vehicle commands (spawn/destroy/update) -----------------------------------------
    # ----------------------------------------------------------------------------------------------

    def spawn_actor(self, transform):
        """
        Requests the creation of a new Driving-Simulator (CARLA) vehicle in vissim. The actual
        request is only sent with the next tick() call - this method only allocates a local_id
        and buffers the command, returning immediately (mirrors the previous in-process
        implementation's behavior of deferring the real send until the next simulation step).

        Warning: When the maximum number of simulator vehicles being tracked at the same time is
        reached, no new vehicle is spawned.

            :param carla.Transform transform: vissim-frame transform of the new vehicle (as
                produced by BridgeHelper.get_vissim_transform()).
            :return: the allocated local_id, or constants.INVALID_ACTOR_ID if the maximum number
                of simulator vehicles has been reached.
        """
        if len(self._active_local_ids) >= self._max_simulator_vehicles:
            logging.warning(
                'Maximum number of simulator vehicles reached. No vehicle will be spawned.')
            return constants.INVALID_ACTOR_ID

        local_id = self._next_local_id
        self._next_local_id += 1
        self._active_local_ids.add(local_id)

        self._pending_spawn.append({
            'local_id': local_id,
            'x': transform.location.x,
            'y': transform.location.y,
            'z': transform.location.z,
            'heading': math.radians(transform.rotation.yaw),
            'pitch': math.radians(transform.rotation.pitch),
        })
        return local_id

    def destroy_actor(self, actor_id):
        """
        Requests the removal of the given actor from vissim (sent with the next tick() call).

            :param actor_id: id of the vehicle to be destroyed (as returned by spawn_actor()).
            :return: True if successfully requested. Otherwise, False.
        """
        if actor_id not in self._active_local_ids:
            return False
        self._active_local_ids.discard(actor_id)
        self._pending_destroy.append(actor_id)
        return True

    def synchronize_vehicle(self, vehicle_id, transform, velocity):
        """
        Updates vehicle state (sent with the next tick() call).

            :param int vehicle_id: id of the vehicle to be updated (as returned by spawn_actor).
            :param carla.Transform transform: new vehicle transform (i.e., position and rotation).
            :param carla.Vector3D velocity: new vehicle velocity.
            :return: True if successfully updated. Otherwise, False.
        """
        if vehicle_id not in self._active_local_ids:
            return False

        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self._pending_update[vehicle_id] = {
            'x': transform.location.x,
            'y': transform.location.y,
            'z': transform.location.z,
            'heading': math.radians(transform.rotation.yaw),
            'pitch': math.radians(transform.rotation.pitch),
            'speed': speed,
        }
        return True

    # ----------------------------------------------------------------------------------------------
    # -- accessors -------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------

    def get_actor(self, actor_id):
        """
        Accessor for vissim actor.
        """
        return self._vissim_vehicles[actor_id]

    def get_pedestrian(self, pedestrian_id):
        """
        Accessor for vissim pedestrian.
        """
        return self._vissim_pedestrians[pedestrian_id]

    @property
    def signal_ids(self):
        """
        Returns the set of (ControllerID, SignalGroupID) pairs seen in the last tick().
        """
        return set(self._signal_states.keys())

    def get_signal_state(self, signal_id):
        """
        Returns the current VissimSignalState for the given (ControllerID, SignalGroupID) pair, as
        retrieved in the last tick(). Returns None if the pair was not reported by vissim (e.g.,
        unknown signal group, or no signal controllers in the network).
        """
        return self._signal_states.get(signal_id)

    @property
    def tick_count(self):
        """
        Returns the number of tick() calls processed so far. Useful (together with --step-length)
        to correlate vissim-side signal state changes against carla's own frame counter when
        cross-checking the two systems for a suspected synchronization delay.
        """
        return self._tick_count

    # ----------------------------------------------------------------------------------------------
    # -- tick ------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------

    def _clear_diff_sets(self):
        """
        Resets spawned/destroyed vehicle/pedestrian sets to empty, without touching
        self._vissim_vehicles/self._vissim_pedestrians/self._signal_states. Used when a tick()
        call could not be completed (timeout or adapter-side error) so that this tick reports "no
        changes" rather than a stale or incorrect diff.
        """
        self.spawned_vehicles = set()
        self.destroyed_vehicles = set()
        self.spawned_pedestrians = set()
        self.destroyed_pedestrians = set()

    def tick(self):
        """
        Tick to vissim simulation: sends any commands buffered by spawn_actor()/destroy_actor()/
        synchronize_vehicle() since the last call, then rebuilds VissimVehicle/VissimPedestrian
        objects and the signal state map from the adapter's response.

        If the previous tick's request timed out, this first attempts to resend 'connect' (see
        _reconnect()) before doing anything else; if that also fails, this tick is a no-op (empty
        diff sets, previous state preserved) so that the caller's carla.tick() is never blocked
        waiting on a vissim adapter that is not responding.
        """
        if self._needs_reconnect:
            if not self._reconnect():
                self._clear_diff_sets()
                return

        response = self._request(
            rpc.MSG_TICK, {
                'spawn': self._pending_spawn,
                'destroy': self._pending_destroy,
                'update': [
                    dict(local_id=local_id, **fields)
                    for local_id, fields in self._pending_update.items()
                ],
            })
        self._pending_spawn = []
        self._pending_destroy = []
        self._pending_update = {}

        if response is None:
            # Timed out - _request() already recreated the socket and set self._needs_reconnect;
            # the next tick() call will attempt to reconnect before sending another 'tick'.
            logging.error('vissim adapter tick request timed out; will attempt to reconnect on '
                         'the next tick()')
            self._clear_diff_sets()
            return

        if not response['ok']:
            # Leave state as-is (previous tick's vehicles/pedestrians/signals) and report no
            # spawns/destructions this tick, so that carla.tick() downstream is never blocked by
            # a transient adapter error.
            logging.error('vissim adapter tick request failed: %s', response['error'])
            self._clear_diff_sets()
            return

        payload = response['payload']

        # -- vehicles --
        vehicles = {}
        for row in payload.get('vehicles', []):
            vehicles[row['id']] = VissimVehicle(
                row['id'], row['type'], row['model_filename'], row['color'],
                [row['x'], row['y'], row['z']], [row['pitch'], row['heading'], 0.0], row['speed'],
                row['turn_indicator'])

        active_vehicles = set(self._vissim_vehicles.keys())
        current_vehicles = set(vehicles.keys())
        self.spawned_vehicles = current_vehicles.difference(active_vehicles)
        self.destroyed_vehicles = active_vehicles.difference(current_vehicles)
        self._vissim_vehicles = vehicles

        # -- pedestrians --
        pedestrians = {}
        for row in payload.get('pedestrians', []):
            try:
                motion_state = VissimPedestrianMotionState(row['motion_state'])
            except ValueError:
                logging.warning('[vissim] unknown MotionState value %r for PedestrianID=%s',
                                row['motion_state'], row['id'])
                motion_state = None

            pedestrians[row['id']] = VissimPedestrian(
                row['id'], row['type'], row['model_filename'],
                [row['length'], row['width'], row['height']], [row['x'], row['y'], row['z']],
                [row['pitch'], row['heading'], 0.0], row['speed'], motion_state)

        active_pedestrians = set(self._vissim_pedestrians.keys())
        current_pedestrians = set(pedestrians.keys())
        self.spawned_pedestrians = current_pedestrians.difference(active_pedestrians)
        self.destroyed_pedestrians = active_pedestrians.difference(current_pedestrians)
        self._vissim_pedestrians = pedestrians

        # -- signal groups --
        signal_states = {}
        for row in payload.get('signals', []):
            try:
                state = VissimSignalState(row['state'])
            except ValueError:
                logging.warning(
                    '[vissim] unknown SignalState value %r for ControllerID=%s '
                    'SignalGroupID=%s', row['state'], row['controller_id'],
                    row['signal_group_id'])
                state = VissimSignalState.UNDEFINED
            signal_states[(row['controller_id'], row['signal_group_id'])] = state
        self._signal_states = signal_states

        self._tick_count += 1
        if self._tick_count % 20 == 0 and pedestrians:
            sample_id, sample_pedestrian = next(iter(pedestrians.items()))
            sample_transform = sample_pedestrian.get_transform()
            logging.debug(
                '[vissim] sample pedestrian PedestrianID=%s pos=(%.2f, %.2f, %.2f) motion_state=%s '
                '(%d pedestrian(s) total)', sample_id, sample_transform.location.x,
                sample_transform.location.y, sample_transform.location.z,
                sample_pedestrian.motion_state.name if sample_pedestrian.motion_state else None,
                len(pedestrians))
        if self._tick_count % 20 == 0 and vehicles:
            sample_id, sample_vehicle = next(iter(vehicles.items()))
            sample_transform = sample_vehicle.get_transform()
            logging.debug(
                '[vissim] sample NPC VehicleID=%s pos=(%.2f, %.2f, %.2f) (%d NPC(s) total, for '
                'comparison against the EGO coordinates logged above)', sample_id,
                sample_transform.location.x, sample_transform.location.y,
                sample_transform.location.z, len(vehicles))
        if self._tick_count % 20 == 0 and signal_states:
            sample_id, sample_state = next(iter(signal_states.items()))
            logging.debug(
                '[vissim] sample signal (ControllerID, SignalGroupID)=%s state=%s (%d signal(s) '
                'total)', sample_id, sample_state.name, len(signal_states))

    def close(self):
        """
        Disconnects from the vissim adapter (which in turn calls VISSIM_Disconnect() on the
        Windows side) and releases the ZeroMQ socket/context.
        """
        try:
            response = self._request(rpc.MSG_DISCONNECT, {})
            if response is None:
                logging.error('vissim adapter disconnect request timed out')
            elif not response['ok']:
                logging.error('vissim adapter disconnect request failed: %s', response['error'])
        except zmq.ZMQError:
            logging.exception('Error sending disconnect request to vissim adapter')
        finally:
            self._socket.close(linger=0)
            self._zmq_context.term()
