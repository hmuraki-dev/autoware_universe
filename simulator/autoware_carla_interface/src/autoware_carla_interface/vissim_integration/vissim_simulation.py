#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Vendored from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/vissim_integration/vissim_simulation.py`). See ../NOTICE.md for the
# one deviation made when vendoring this file (removal of a dead, import-time debug block).
""" This module is responsible for the management of the ptv-vissim simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import enum
import logging
import math
import os

import carla  # pylint: disable=import-error
from ctypes import *

from . import constants

# ==================================================================================================
# -- vissim definitions ----------------------------------------------------------------------------
# ==================================================================================================


class Simulator_Veh_Data(Structure):
    """
    Structure to hold the data sent to vissim about the status of the simulator vehicles (i.e.,
    carla vehicles).

    Field layout matches struct Simulator_Veh_Data in DrivingSimulatorProxy.h (PTV Vissim Kernel
    for Linux 2026.00-10). All integer fields are C `int` (always 32-bit) - NOT `long`, which is
    64-bit on Linux/LP64 and would silently corrupt this struct's memory layout.
    """
    _fields_ = [
        ('VehicleID', c_int),  # vehicle number in Vissim, irrelevant for new vehicles
        ('VehicleType', c_int),  # vehicle type number in Vissim
        ('Position_X', c_double),  # front center of the vehicle in m
        ('Position_Y', c_double),  # front center of the vehicle in m
        ('Position_Z', c_double),  # front center of the vehicle in m
        ('Orient_Heading', c_double),  # in radians, eastbound = zero, northbound = +Pi/2
        ('Orient_Pitch', c_double),  # in radians, uphill = positive
        ('Speed', c_double),  # in m/s
        ('Create', c_bool),  # new vehicle to be placed in the network
        ('CreateID', c_int),  # unique positive ID, echoed back in VISSIM_Veh_Data.CreateID
        ('Delete', c_bool),  # vehicle to be removed from the network
        ('ControlledByVissim', c_bool),  # affects next time step
        ('RoutingDecisionNo', c_int),  # used once if ControlledByVissim changed from false to true
        ('RouteNo', c_int)  # used once if ControlledByVissim changed from false to true
    ]


class VISSIM_Veh_Data(Structure):
    """
    Structure to hold the data received from vissim about the status of the traffic vehicles (i.e.,
    vissim vehicles).

    Field layout matches struct VISSIM_Veh_Data in DrivingSimulatorProxy.h (PTV Vissim Kernel for
    Linux 2026.00-10), including the trailing CreateID/ControlledByVissim fields used to detect our
    own driving-simulator (CARLA) vehicles being echoed back in the traffic vehicle list.
    """
    _fields_ = [
        ('VehicleID', c_int),
        ('VehicleType', c_int),  # vehicle type number from Vissim
        ('ModelFileName', c_char * constants.NAME_MAX_LENGTH),  # .v3d (utf-8)
        ('color', c_int),  # RGB
        ('Position_X', c_double),  # front center of the vehicle in m
        ('Position_Y', c_double),  # front center of the vehicle in m
        ('Position_Z', c_double),  # front center of the vehicle in m
        ('Orient_Heading', c_double),  # in radians, eastbound = zero, northbound = +Pi/2
        ('Orient_Pitch', c_double),  # in radians, uphill = positive
        ('Speed', c_double),  # in m/s
        ('LeadingVehicleID', c_int),  # relevant vehicle in front
        ('TrailingVehicleID', c_int),  # next vehicle back on the same lane
        ('LinkID', c_int),  # Vissim link attribute "Number"
        ('LinkName', c_char * constants.NAME_MAX_LENGTH),  # empty if "Name" not set in Vissim (utf-8)
        ('LinkCoordinate', c_double),  # in m
        ('LaneIndex', c_int),  # 0 = rightmost
        ('TurningIndicator', c_int),  # 1 = left, 0 = none, -1 = right
        ('PreviousIndex', c_int),  # index in the previous Vissim time step, < 0 = new in visibility area
        ('NumUDAs', c_int),  # the number of UDA values in the following array
        ('UDA', c_double * constants.MAX_UDA),  # the first MAX_UDA user-defined numeric vehicle attributes
        ('CreateID', c_int),  # unique ID as passed from the simulator for the new vehicle, else zero
        ('ControlledByVissim', c_bool)  # false for vehicles controlled by the Driving Simulator (CARLA)
    ]


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


class VISSIM_Ped_Data(Structure):
    """
    Structure to hold the data received from vissim about traffic pedestrians (i.e., vissim
    pedestrians).

    Field layout matches struct VISSIM_Ped_Data in DrivingSimulatorProxy.h (PTV Vissim Kernel for
    Linux 2026.00-10). As with VISSIM_Veh_Data, all integer fields are C `int` (always 32-bit) -
    NOT `long`, which is 64-bit on Linux/LP64 and would silently corrupt this struct's memory
    layout.

    Note: unlike VISSIM_Veh_Data, there is no `ControlledByVissim` field here - per
    DrivingSimulatorProxy.h, VISSIM_GetTrafficPedestrians() already excludes our own simulator
    pedestrian(s) from the result, so every row returned is genuine vissim-controlled traffic (see
    PEDESTRIAN_TODO.md task 1). This co-simulation only implements the vissim -> carla direction
    for pedestrians (carla -> vissim / Simulator_Ped_Data is out of scope, see PEDESTRIAN_TODO.md).
    """
    _fields_ = [
        ('PedestrianID', c_int),
        ('PedestrianType', c_int),  # pedestrian type number from Vissim
        ('ModelFileName', c_char * constants.NAME_MAX_LENGTH),  # .v3d (utf-8)
        ('Length', c_double),  # in m
        ('Width', c_double),  # in m
        ('Height', c_double),  # in m
        ('Position_X', c_double),  # in m
        ('Position_Y', c_double),  # in m
        ('Position_Z', c_double),  # in m
        ('Orient_Heading', c_double),  # in radians
        ('Orient_Pitch', c_double),  # in radians
        ('DistanceSinceBirth', c_double),  # in m
        ('Speed', c_double),  # in m/s
        ('MotionState', c_int),  # Pedestrian_Motion_State_Type enum, see VissimPedestrianMotionState above
        ('ConstructionElementType', c_int),  # Pedestrian_Construction_Element_Type enum, see VissimPedestrianConstructionElementType above
        ('ConstructionElementID', c_int),  # the construction element the pedestrian is currently on
        ('ConstructionElementName', c_char * constants.NAME_MAX_LENGTH),  # empty if not set in Vissim (utf-8)
        ('PreviousIndex', c_int),  # index in the previous Vissim time step, < 0 = new in visibility area
    ]


class VissimLightState(enum.Enum):
    """
    VissimLightState contains the different vissim indicator states.
    """
    LEFT = 1
    NONE = 0
    RIGHT = -1


class VISSIM_Sig_Data(Structure):
    """
    Structure to hold the data received from vissim about the state of a signal group.

    Field layout matches struct VISSIM_Sig_Data in DrivingSimulatorProxy.h (PTV Vissim Kernel for
    Linux 2026.00-10). As with VISSIM_Veh_Data, all integer fields (including the SignalState
    enum, whose C declaration is `enum SignalStateType : int`) are C `int` (always 32-bit) - NOT
    `long`, which is 64-bit on Linux/LP64 and would silently corrupt this struct's memory layout.

    Note: a signal group is uniquely identified by the (ControllerID, SignalGroupID) pair, not by
    SignalGroupID alone (signal group numbers are only unique within a given signal controller).
    """
    _fields_ = [
        ('ControllerID', c_int),
        ('SignalGroupID', c_int),
        ('SignalState', c_int),  # SignalStateType enum, see VissimSignalState below
    ]


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

    This targets PTV Vissim Kernel for Linux and connects through libDrivingSimulatorProxy.so
    using VISSIM_ConnectToKernel(), which (unlike the Windows-only VISSIM_Connect()) takes no
    Vissim version number argument.
    """
    def __init__(self, args):
        # Maximum number of simulator vehicles to be tracked by the driving simulator interface.
        self._max_simulator_vehicles = args.simulator_vehicles

        # Loading driving simulator proxy library. Relies on LD_LIBRARY_PATH/RUNPATH unless an
        # explicit path is given via --vissim-lib-path.
        lib_path = args.vissim_lib_path or 'libDrivingSimulatorProxy.so'
        logging.info('Loading DrivingSimulatorProxy library from %s...', lib_path)
        self.ds_proxy = cdll.LoadLibrary(lib_path)
        self._declare_prototypes()

        # Connection to the Vissim Kernel.
        logging.info('Establishing a connection with PTV-Vissim Kernel...')
        result = self.ds_proxy.VISSIM_ConnectToKernel(
            os.path.abspath(args.vissim_network), c_ushort(int(1. / args.step_length)),
            c_double(constants.VISSIM_VISIBILITY_RADIUS),
            c_ushort(constants.VISSIM_MAX_SIMULATOR_VEH),
            c_ushort(constants.VISSIM_MAX_SIMULATOR_PED),
            c_ushort(constants.VISSIM_MAX_SIMULATOR_DET), c_ushort(constants.VISSIM_MAX_VISSIM_VEH),
            c_ushort(constants.VISSIM_MAX_VISSIM_PED), c_ushort(constants.VISSIM_MAX_VISSIM_SIGGRP))

        if not result:
            error_message = self.ds_proxy.VISSIM_GetLastErrorMessage()
            raise RuntimeError(
                'There was an error connecting to PTV-Vissim Kernel: %s' % error_message)

        # Structures to keep track of the simulation state at each time step.
        # Real vissim VehicleID -> VissimVehicle, only for genuine vissim traffic (i.e., rows with
        # ControlledByVissim == True). Excludes our own simulator vehicles echoed back by vissim.
        self._vissim_vehicles = {}

        # Real vissim PedestrianID -> VissimPedestrian. vissim -> carla direction only (see
        # PEDESTRIAN_TODO.md): VISSIM_GetTrafficPedestrians() already excludes any simulator
        # pedestrian(s), so no ControlledByVissim-style filtering is needed here, unlike vehicles.
        self._vissim_pedestrians = {}

        # (ControllerID, SignalGroupID) -> VissimSignalState, refreshed every tick. Empty until
        # the first tick() (and remains empty if the network has no signal controllers).
        self._signal_states = {}

        # Our own actor_id (stable for the whole vehicle lifetime) -> record dict with keys:
        # 'state' ('pending' | 'active'), 'create_sent', 'delete_requested', 'data'.
        self._simulator_vehicles = {}

        self.spawned_vehicles = set()
        self.destroyed_vehicles = set()
        self.spawned_pedestrians = set()
        self.destroyed_pedestrians = set()
        self._tick_count = 0

        # Unique, monotonically increasing CreateID generator (not reused across retries).
        self._next_create_id = 1

        # How many ticks to wait for vissim's CreateID confirmation before giving up and
        # re-issuing the Create request with a fresh CreateID. Observed empirically: vissim can
        # report VISSIM_SetDriverVehicles success while silently failing to actually place the
        # DS vehicle (e.g., a transient position conflict with background traffic), without ever
        # reporting an error and without ever echoing the CreateID back.
        self._pending_retry_ticks = max(1, int(2.0 / args.step_length))

    def _declare_prototypes(self):
        """
        Declares argument/return types for the subset of the DS Interface used here, matching
        DrivingSimulatorProxy.h. Explicit prototypes avoid relying on ctypes' default int/pointer
        guesses, which cannot be trusted across platforms and calling conventions.
        """
        self.ds_proxy.VISSIM_ConnectToKernel.argtypes = [
            c_wchar_p, c_ushort, c_double, c_ushort, c_ushort, c_ushort, c_ushort, c_ushort,
            c_ushort
        ]
        self.ds_proxy.VISSIM_ConnectToKernel.restype = c_bool

        self.ds_proxy.VISSIM_Disconnect.argtypes = []
        self.ds_proxy.VISSIM_Disconnect.restype = c_bool

        self.ds_proxy.VISSIM_SetDriverVehicles.argtypes = [c_int, POINTER(Simulator_Veh_Data)]
        self.ds_proxy.VISSIM_SetDriverVehicles.restype = c_bool

        self.ds_proxy.VISSIM_GetTrafficVehicles.argtypes = [
            POINTER(c_int), POINTER(POINTER(VISSIM_Veh_Data))
        ]
        self.ds_proxy.VISSIM_GetTrafficVehicles.restype = None

        self.ds_proxy.VISSIM_GetTrafficPedestrians.argtypes = [
            POINTER(c_int), POINTER(POINTER(VISSIM_Ped_Data))
        ]
        self.ds_proxy.VISSIM_GetTrafficPedestrians.restype = None

        self.ds_proxy.VISSIM_GetSignalStates.argtypes = [
            POINTER(c_int), POINTER(POINTER(VISSIM_Sig_Data))
        ]
        self.ds_proxy.VISSIM_GetSignalStates.restype = None

        self.ds_proxy.VISSIM_GetLastErrorMessage.argtypes = []
        self.ds_proxy.VISSIM_GetLastErrorMessage.restype = c_wchar_p

    def _get_next_actor_id(self):
        """
        Returns an available actor id. Otherwise, returns INVALID_ACTOR_ID.
        """
        all_ids = set(range(1, self._max_simulator_vehicles + 1))
        used_ids = set(self._simulator_vehicles.keys())
        available_ids = all_ids - used_ids
        if len(available_ids):
            return available_ids.pop()
        else:
            return constants.INVALID_ACTOR_ID

    def _allocate_create_id(self):
        """
        Returns a fresh, unique CreateID for a new (or retried) creation request.
        """
        create_id = self._next_create_id
        self._next_create_id += 1
        return create_id

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

    def spawn_actor(self, transform):
        """
        Requests the creation of a new Driving-Simulator (CARLA) vehicle in vissim.

        The vehicle is not created immediately: vissim confirms the request (and assigns the real
        VehicleID) only in a following simulation step, reported back through VISSIM_Veh_Data's
        CreateID field. Until then, the actor is tracked internally as 'pending'.

        Warning: When the maximum number of simulator vehicles being tracked at the same time is
        reached, no new vehicles are spawned.
        """
        if len(self._simulator_vehicles) >= self._max_simulator_vehicles:
            logging.warning(
                'Maximum number of simulator vehicles reached. No vehicle will be spawned.')
            return constants.INVALID_ACTOR_ID

        actor_id = self._get_next_actor_id()
        create_id = self._allocate_create_id()
        veh_data = Simulator_Veh_Data(0, constants.VISSIM_DEFAULT_VEHICLE_TYPE,
                                      transform.location.x, transform.location.y,
                                      transform.location.z, math.radians(transform.rotation.yaw),
                                      math.radians(transform.rotation.pitch), 0.0, True, create_id,
                                      False, False, 0, 0)
        self._simulator_vehicles[actor_id] = {
            'state': 'pending',
            'create_sent': False,
            'delete_requested': False,
            'ticks_waiting': 0,
            'data': veh_data,
        }
        return actor_id

    def destroy_actor(self, actor_id):
        """
        Requests the removal of the given actor from vissim.

        If the create request for this actor has not been confirmed by vissim yet, the deletion is
        deferred until the real VehicleID is known (or dropped locally if the create request was
        never even sent to vissim).

            :param actor_id: id of the vehicle to be destroyed.
            :return: True if successfully requested. Otherwise, False.
        """
        if actor_id in self._simulator_vehicles:
            self._simulator_vehicles[actor_id]['delete_requested'] = True
            return True
        return False

    def synchronize_vehicle(self, vehicle_id, transform, velocity):
        """
        Updates vehicle state.

            :param int vehicle_id: id of the vehicle to be updated (as returned by spawn_actor).
            :param carla.Transform transform: new vehicle transform (i.e., position and rotation).
            :param carla.Vector3D velocity: new vehicle velocity.
            :return: True if successfully updated. Otherwise, False.
        """
        if vehicle_id not in self._simulator_vehicles:
            return False

        record = self._simulator_vehicles[vehicle_id]
        heading = math.radians(transform.rotation.yaw)
        pitch = math.radians(transform.rotation.pitch)
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        if record['state'] == 'pending':
            # Always refresh the pose (whether or not the create request has already been sent),
            # so that a retry - if one becomes necessary - uses the most recent known position.
            record['data'] = Simulator_Veh_Data(0, constants.VISSIM_DEFAULT_VEHICLE_TYPE,
                                                transform.location.x, transform.location.y,
                                                transform.location.z, heading, pitch, speed,
                                                True, record['data'].CreateID, False, False, 0, 0)
        else:
            record['data'] = Simulator_Veh_Data(record['data'].VehicleID,
                                                constants.VISSIM_DEFAULT_VEHICLE_TYPE,
                                                transform.location.x, transform.location.y,
                                                transform.location.z, heading, pitch, speed, False,
                                                0, False, False, 0, 0)
        return True

    def _get_simulator_veh_data(self):
        """
        Returns the list of Simulator_Veh_Data structures ready to be sent to the driving simulator
        interface for this time step, plus the local actor ids to drop from bookkeeping right after
        sending (vehicles whose deletion was just requested and sent, or cancelled before ever
        being sent to vissim).

        Pending (unconfirmed) creations are sent with Create=True exactly once; while waiting for
        vissim to report back the assigned VehicleID they are omitted from the array entirely
        (repeating Create=True every tick could spawn duplicate vehicles, and referencing an
        unknown VehicleID=0 for a non-pending update could corrupt an unrelated vissim vehicle).
        """
        data = []
        actor_ids_to_drop = []

        for actor_id, record in self._simulator_vehicles.items():
            if record['state'] == 'pending':
                if record['delete_requested'] and not record['create_sent']:
                    # Cancelled before vissim ever heard about it: nothing to send.
                    actor_ids_to_drop.append(actor_id)
                    continue
                if not record['create_sent']:
                    logging.debug(
                        '[vissim] sending Create actor_id=%s CreateID=%d pos=(%.2f, %.2f, %.2f) '
                        'heading=%.3f', actor_id, record['data'].CreateID, record['data'].Position_X,
                        record['data'].Position_Y, record['data'].Position_Z,
                        record['data'].Orient_Heading)
                    data.append(record['data'])
                    record['create_sent'] = True
                    record['ticks_waiting'] = 0
                elif not record['delete_requested']:
                    # Still waiting for vissim to confirm the CreateID -> VehicleID mapping. If
                    # this drags on too long, vissim likely silently failed to actually create the
                    # vehicle (observed in practice, e.g. due to a transient position conflict with
                    # background traffic) - retry with a fresh CreateID and the latest known pose.
                    record['ticks_waiting'] += 1
                    if record['ticks_waiting'] >= self._pending_retry_ticks:
                        old_data = record['data']
                        new_create_id = self._allocate_create_id()
                        record['data'] = Simulator_Veh_Data(
                            0, constants.VISSIM_DEFAULT_VEHICLE_TYPE, old_data.Position_X,
                            old_data.Position_Y, old_data.Position_Z, old_data.Orient_Heading,
                            old_data.Orient_Pitch, old_data.Speed, True, new_create_id, False,
                            False, 0, 0)
                        logging.debug(
                            '[vissim] retrying Create actor_id=%s (no confirmation for %d ticks) '
                            'old_CreateID=%d new_CreateID=%d pos=(%.2f, %.2f, %.2f) heading=%.3f',
                            actor_id, record['ticks_waiting'], old_data.CreateID, new_create_id,
                            old_data.Position_X, old_data.Position_Y, old_data.Position_Z,
                            old_data.Orient_Heading)
                        data.append(record['data'])
                        record['ticks_waiting'] = 0
                # else: waiting for confirmation but already marked for deletion once resolved -
                # do not retry a create that will be deleted anyway.
                continue

            veh_data = record['data']
            if record['delete_requested']:
                veh_data = Simulator_Veh_Data(veh_data.VehicleID,
                                              constants.VISSIM_DEFAULT_VEHICLE_TYPE,
                                              veh_data.Position_X, veh_data.Position_Y,
                                              veh_data.Position_Z, veh_data.Orient_Heading,
                                              veh_data.Orient_Pitch, veh_data.Speed, False, 0, True,
                                              False, 0, 0)
                actor_ids_to_drop.append(actor_id)
            else:
                logging.debug(
                    '[vissim] sending update actor_id=%s VehicleID=%d pos=(%.2f, %.2f, %.2f) '
                    'heading=%.3f speed=%.2f', actor_id, veh_data.VehicleID, veh_data.Position_X,
                    veh_data.Position_Y, veh_data.Position_Z, veh_data.Orient_Heading,
                    veh_data.Speed)

            data.append(veh_data)

        return data, actor_ids_to_drop

    def tick(self):
        """
        Tick to vissim simulation.
        """
        # Updating simulator vehicles data.
        veh_data, actor_ids_to_drop = self._get_simulator_veh_data()
        num_simulator_vehicles = len(veh_data)
        if num_simulator_vehicles:
            arr = (Simulator_Veh_Data * num_simulator_vehicles)(*veh_data)
            result = self.ds_proxy.VISSIM_SetDriverVehicles(num_simulator_vehicles, arr)
            logging.debug('[vissim] VISSIM_SetDriverVehicles(%d) returned %s', num_simulator_vehicles,
                          bool(result))
        else:
            result = self.ds_proxy.VISSIM_SetDriverVehicles(0, None)
            if not result:
                logging.debug('[vissim] VISSIM_SetDriverVehicles(0, None) returned False')

        for actor_id in actor_ids_to_drop:
            del self._simulator_vehicles[actor_id]

        # Retrieving vissim traffic data (this also includes our own simulator vehicles, echoed
        # back with ControlledByVissim == False).
        num_vehicles = c_int(0)
        traffic_data = POINTER(VISSIM_Veh_Data)()
        self.ds_proxy.VISSIM_GetTrafficVehicles(byref(num_vehicles), byref(traffic_data))

        vehicles = {}
        for i in range(num_vehicles.value):
            vehicle_data = traffic_data[i]

            if not vehicle_data.ControlledByVissim:
                # One of our own pending simulator vehicles reported back for the first time:
                # resolve its real VehicleID via the CreateID we sent, then keep it out of the
                # vissim-traffic bookkeeping (it is not a new NPC to spawn in carla).
                logging.debug(
                    '[vissim] saw ControlledByVissim=False row: VehicleID=%d CreateID=%d '
                    'pos=(%.2f, %.2f, %.2f) LinkID=%d', vehicle_data.VehicleID,
                    vehicle_data.CreateID, vehicle_data.Position_X, vehicle_data.Position_Y,
                    vehicle_data.Position_Z, vehicle_data.LinkID)
                for record in self._simulator_vehicles.values():
                    if (record['state'] == 'pending' and record['create_sent']
                            and vehicle_data.CreateID == record['data'].CreateID):
                        record['state'] = 'active'
                        record['data'] = Simulator_Veh_Data(
                            vehicle_data.VehicleID, constants.VISSIM_DEFAULT_VEHICLE_TYPE,
                            vehicle_data.Position_X, vehicle_data.Position_Y,
                            vehicle_data.Position_Z, vehicle_data.Orient_Heading,
                            vehicle_data.Orient_Pitch, vehicle_data.Speed, False, 0, False, False,
                            0, 0)
                        logging.debug(
                            '[vissim] resolved CreateID=%d -> real VehicleID=%d pos=(%.2f, %.2f, '
                            '%.2f) LinkID=%d LaneIndex=%d', vehicle_data.CreateID,
                            vehicle_data.VehicleID, vehicle_data.Position_X,
                            vehicle_data.Position_Y, vehicle_data.Position_Z, vehicle_data.LinkID,
                            vehicle_data.LaneIndex)
                        break
                continue

            vehicles[vehicle_data.VehicleID] = VissimVehicle(
                vehicle_data.VehicleID, vehicle_data.VehicleType, vehicle_data.ModelFileName,
                vehicle_data.color,
                [vehicle_data.Position_X, vehicle_data.Position_Y, vehicle_data.Position_Z],
                [vehicle_data.Orient_Pitch, vehicle_data.Orient_Heading, 0.0], vehicle_data.Speed,
                vehicle_data.TurningIndicator)

        # Update data structures for the current time step.
        active_vehicles = set(self._vissim_vehicles.keys())
        current_vehicles = set(vehicles.keys())

        self.spawned_vehicles = current_vehicles.difference(active_vehicles)
        self.destroyed_vehicles = active_vehicles.difference(current_vehicles)

        self._vissim_vehicles = vehicles

        # Retrieving vissim traffic pedestrian data (read-only: vissim -> carla direction only,
        # see PEDESTRIAN_TODO.md - carla -> vissim / Simulator_Ped_Data is out of scope). Unlike
        # VISSIM_GetTrafficVehicles, VISSIM_GetTrafficPedestrians already excludes any simulator
        # pedestrian(s), so every row here is genuine vissim-controlled traffic.
        num_pedestrians = c_int(0)
        pedestrian_data = POINTER(VISSIM_Ped_Data)()
        self.ds_proxy.VISSIM_GetTrafficPedestrians(byref(num_pedestrians), byref(pedestrian_data))

        pedestrians = {}
        for i in range(num_pedestrians.value):
            ped_data = pedestrian_data[i]
            try:
                motion_state = VissimPedestrianMotionState(ped_data.MotionState)
            except ValueError:
                logging.warning(
                    '[vissim] unknown MotionState value %d for PedestrianID=%d',
                    ped_data.MotionState, ped_data.PedestrianID)
                motion_state = None

            pedestrians[ped_data.PedestrianID] = VissimPedestrian(
                ped_data.PedestrianID, ped_data.PedestrianType, ped_data.ModelFileName,
                [ped_data.Length, ped_data.Width, ped_data.Height],
                [ped_data.Position_X, ped_data.Position_Y, ped_data.Position_Z],
                [ped_data.Orient_Pitch, ped_data.Orient_Heading, 0.0], ped_data.Speed, motion_state)

        # Update data structures for the current time step.
        active_pedestrians = set(self._vissim_pedestrians.keys())
        current_pedestrians = set(pedestrians.keys())

        self.spawned_pedestrians = current_pedestrians.difference(active_pedestrians)
        self.destroyed_pedestrians = active_pedestrians.difference(current_pedestrians)

        self._vissim_pedestrians = pedestrians

        # Retrieving vissim signal group states (read-only: vissim -> carla direction only, the
        # DS Interface has no equivalent "set signal state" function).
        num_signals = c_int(0)
        signal_data = POINTER(VISSIM_Sig_Data)()
        self.ds_proxy.VISSIM_GetSignalStates(byref(num_signals), byref(signal_data))

        signal_states = {}
        for i in range(num_signals.value):
            sig = signal_data[i]
            try:
                state = VissimSignalState(sig.SignalState)
            except ValueError:
                logging.warning(
                    '[vissim] unknown SignalState value %d for ControllerID=%d SignalGroupID=%d',
                    sig.SignalState, sig.ControllerID, sig.SignalGroupID)
                state = VissimSignalState.UNDEFINED
            signal_states[(sig.ControllerID, sig.SignalGroupID)] = state

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
        self.ds_proxy.VISSIM_Disconnect()
