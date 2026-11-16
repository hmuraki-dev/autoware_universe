#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Vendored from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/vissim_integration/bridge_helper.py`). See ../NOTICE.md for the
# deviations (blueprint/color/driver choice is derived from the vissim id instead of the global
# random generator).
""" This module provides a helper for the co-simulation between vissim and carla. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import logging
import math
import random

import carla  # pylint: disable=import-error

from .vissim_simulation import VissimSignalState

# ==================================================================================================
# -- Bridge helper (VISSIM <=> CARLA) --------------------------------------------------------------
# ==================================================================================================

# Vissim -> carla traffic light state mapping. Carla's carla.TrafficLightState only distinguishes
# Red/Yellow/Green/Off/Unknown, so several of vissim's richer SignalStateType values collapse onto
# the same carla state. Choices favor the safer/more conservative reading (e.g., states that still
# show red map to Red, not Green).
_VISSIM_TO_CARLA_SIGNAL_STATE = {
    VissimSignalState.RED: carla.TrafficLightState.Red,
    VissimSignalState.RED_AMBER: carla.TrafficLightState.Red,  # red+amber: still must stop
    VissimSignalState.GREEN: carla.TrafficLightState.Green,
    VissimSignalState.AMBER: carla.TrafficLightState.Yellow,
    VissimSignalState.OFF: carla.TrafficLightState.Off,
    VissimSignalState.UNDEFINED: carla.TrafficLightState.Unknown,
    VissimSignalState.FLASHING_AMBER: carla.TrafficLightState.Yellow,
    VissimSignalState.FLASHING_RED: carla.TrafficLightState.Red,
    VissimSignalState.FLASHING_GREEN: carla.TrafficLightState.Green,
    # Lane-control style signal, not a normal junction light; no safe single-state mapping.
    VissimSignalState.ALTERNATING_RED_GREEN: carla.TrafficLightState.Unknown,
    # Still green, amber is only an overlay warning that it is about to change.
    VissimSignalState.GREEN_AMBER: carla.TrafficLightState.Green,
}


def body_origins(front, yaw_deg, pitch_deg, actors, mesh):
    """
    Actor origins (CARLA coordinates) of the CARLA bodies that represent one vissim vehicle.

    Not in upstream; see docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md 2.4.
    Pure function (no carla), so that it can be tested on its own.

        :param front: (x, y, z) of the vehicle's front-center in CARLA coordinates (vissim reports
            the front-center of the vehicle, see VISSIM_Veh_Data.Position_X).
        :param yaw_deg: / :param pitch_deg: the vehicle's CARLA rotation.
        :param actors: [{blueprint, front_offset_m}, ...] from the carla_variants (FleetPlan).
        :param mesh: {blueprint: {length, width, center_x, center_y}}: the body as a LiDAR sees it,
            and where it sits relative to the actor origin (FleetPlan.mesh).
        :return: [(x, y, z), ...], one per actor, in the same order.

    Each body's front is `front_offset_m` behind the vehicle's front, so its center is
    `front_offset_m + length/2` behind it; the actor origin is then that center minus the mesh
    center offset, rotated by the vehicle's yaw. Height follows the pitch the same way as
    get_carla_transform() does with the bounding box (only the along-track distance changes).
    """
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
    forward = (math.cos(yaw), math.sin(yaw))
    right = (-math.sin(yaw), math.cos(yaw))  # CARLA's y axis points to the right of x
    origins = []
    for actor in actors:
        body = mesh[actor['blueprint']]
        back = float(actor['front_offset_m']) + body['length'] / 2.0 + body.get('center_x', 0.0)
        side = body.get('center_y', 0.0)
        origins.append((front[0] - forward[0] * back - right[0] * side,
                        front[1] - forward[1] * back - right[1] * side,
                        front[2] - math.sin(pitch) * back))
    return origins


class BridgeHelper(object):
    """
    BridgeHelper provides methos to ease the co-simulation between vissim and carla.
    """

    blueprint_library = []
    vtypes = {}

    # vissim pedestrianType number (str) -> [carla walker blueprint id, ...]. Loaded from
    # data/ptypes.json, mirroring vtypes above. vissim -> carla direction only (see
    # PEDESTRIAN_TODO.md).
    ptypes = {}

    @staticmethod
    def get_carla_traffic_light_state(vissim_signal_state):
        """
        Returns the carla.TrafficLightState corresponding to the given vissim VissimSignalState.
        """
        carla_state = _VISSIM_TO_CARLA_SIGNAL_STATE.get(vissim_signal_state)
        if carla_state is None:
            logging.warning('vissim signal state %s not supported, defaulting to Unknown',
                            vissim_signal_state)
            return carla.TrafficLightState.Unknown
        return carla_state

    @staticmethod
    def get_carla_transform(in_vissim_transform, extent=None):
        """
        Returns carla transform based on vissim transform.
        """
        in_location = in_vissim_transform.location
        in_rotation = in_vissim_transform.rotation

        # From front-center-bumper to center (vissim reference system).
        if extent is not None:
            out_location = (in_location.x - math.cos(math.radians(in_rotation.yaw)) * extent.x,
                            in_location.y - math.sin(math.radians(in_rotation.yaw)) * extent.x,
                            in_location.z - math.sin(math.radians(in_rotation.pitch)) * extent.x)
        else:
            out_location = (in_location.x, in_location.y, in_location.z)
        out_rotation = (in_rotation.pitch, in_rotation.yaw, in_rotation.roll)

        # Transform to carla reference system (left-handed system).
        out_transform = carla.Transform(
            carla.Location(out_location[0], -out_location[1], out_location[2]),
            carla.Rotation(out_rotation[0], -out_rotation[1], out_rotation[2]))

        return out_transform

    @staticmethod
    def get_carla_pedestrian_transform(vissim_pedestrian):
        """
        Returns the carla transform for the given vissim pedestrian, corrected for CARLA's walker
        actor origin convention.

        Unlike vehicle actors (origin at ground level, corrected for via the `extent` argument of
        get_carla_transform() above), a carla Walker's actor transform origin sits at the
        vertical *center* of its bounding box - confirmed empirically on a live carla server:
        spawning a walker.pedestrian.* blueprint reports `bounding_box.location == (0, 0, 0)` and
        `bounding_box.extent.z ~= 0.93` (about half of an average adult's height), i.e. the box
        spans symmetrically above and below the actor's own transform. VISSIM_Ped_Data.Position_Z
        (vissim_pedestrian's transform.location.z), on the other hand, is ground level (the
        pedestrian's feet). Without correction, walkers therefore appear to sink into the ground
        by roughly half their height.

        Raising the transform by half of vissim's own reported pedestrian height
        (VISSIM_Ped_Data.Height, i.e. vissim_pedestrian.extent[2]) lines the feet back up with
        the ground, using only data already provided by vissim (no extra carla actor lookup
        needed, and it automatically adapts to e.g. shorter child/wheelchair pedestrian types).
        """
        carla_transform = BridgeHelper.get_carla_transform(vissim_pedestrian.get_transform())
        carla_transform.location.z += vissim_pedestrian.extent[2] / 2.0
        return carla_transform

    @staticmethod
    def get_vissim_transform(in_carla_transform, extent=None):
        """
        Returns vissim transform based on carla transform.
        """
        in_location = in_carla_transform.location
        in_rotation = in_carla_transform.rotation

        # From center to front-center-bumper (carla reference system).
        if extent is not None:
            yaw = -1 * in_rotation.yaw
            pitch = in_rotation.pitch
            out_location = (in_location.x + math.cos(math.radians(yaw)) * extent.x,
                            in_location.y - math.sin(math.radians(yaw)) * extent.x,
                            in_location.z - math.sin(math.radians(pitch)) * extent.x)
        else:
            out_location = (in_location.x, in_location.y, in_location.z)
        out_rotation = (in_rotation.pitch, in_rotation.yaw, in_rotation.roll)

        # Transform to vissim reference system (right-handed system).
        out_transform = carla.Transform(
            carla.Location(out_location[0], -out_location[1], out_location[2]),
            carla.Rotation(out_rotation[0], -out_rotation[1], out_rotation[2]))

        return out_transform

    @staticmethod
    def _flip_y(in_vector):
        """
        Flips y coordinate of the given vector.
        """
        return carla.Vector3D(in_vector.x, -in_vector.y, in_vector.z)

    @staticmethod
    def get_carla_velocity(in_vissim_velocity):
        """
        Returns carla velocity based on vissim velocity.
        """
        return BridgeHelper._flip_y(in_vissim_velocity)

    @staticmethod
    def get_vissim_velocity(in_carla_velocity):
        """
        Returns vissim velocity based on carla velocity.
        """
        return BridgeHelper._flip_y(in_carla_velocity)

    @staticmethod
    def _get_recommended_carla_blueprint(vissim_actor):
        """
        Returns an appropriate blueprint based on the given vissim actor.
        """
        blueprint = BridgeHelper.blueprint_library.filter('vehicle.seat.leon')[0]
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
        return blueprint

    @staticmethod
    def get_carla_blueprint(vissim_actor):
        """
        Returns an appropriate blueprint based on the received vissim actor.
        """
        type_id = str(vissim_actor.type)
        # Seeded by the vissim id so that the same vissim vehicle always gets the same model
        # (and thus the same length) regardless of run or spawn order.
        rng = random.Random(int(vissim_actor.id))

        if type_id in BridgeHelper.vtypes:
            candidates = BridgeHelper.vtypes[type_id]
            if candidates:
                blueprint_id = rng.choice(candidates)
            else:
                logging.error(
                    'vissim type %s not supported. No vehicle will be spawned in carla', type_id)
                return None

            blueprint = BridgeHelper.blueprint_library.filter(blueprint_id)
            if not blueprint:
                logging.error(
                    'carla blueprint %s unknown. No vehicle will be spawned', blueprint_id)
                return None
            blueprint = blueprint[0]

            if blueprint.has_attribute('color'):
                color = rng.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)

            if blueprint.has_attribute('driver_id'):
                driver_id = rng.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)

            blueprint.set_attribute('role_name', 'vissim_driver')
            return blueprint

        else:
            logging.error(
                'vissim type %s unknown. No vehicle will be spawned in carla', type_id)
            return None

    @staticmethod
    def get_fleet_blueprints(fleet_plan, vissim_actor):
        """
        Returns the blueprints of the CARLA bodies for the received vissim vehicle, from the
        carla_variants (see docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md
        F5). Not in upstream.

        Unlike get_carla_blueprint(), the model is not chosen at random: it is the one
        FleetPlan assigns to the vehicle's (VehicleType, ModelFileName). Only the color and the
        driver are chosen, seeded by the vissim id as in get_carla_blueprint(); every body of the
        same vehicle gets the same color.

            :raises carla_fleet.VariantError: if the vehicle's model is unknown or has no
                assignment (FleetPlan.actors_for()); never falls back to another blueprint.
            :return: [(blueprint, actor spec), ...], front to rear.
        """
        rng = random.Random(int(vissim_actor.id))
        color = driver_id = None
        result = []
        for spec in fleet_plan.actors_for(vissim_actor.type, vissim_actor.model_filename):
            blueprint = BridgeHelper.blueprint_library.find(spec['blueprint'])
            if blueprint.has_attribute('color'):
                if color is None:
                    color = rng.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                if driver_id is None:
                    driver_id = rng.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'vissim_driver')
            result.append((blueprint, spec))
        return result

    @staticmethod
    def get_fleet_transforms(fleet_plan, in_vissim_transform, actors):
        """
        Returns the carla transforms of the given bodies of one vissim vehicle, placed by their
        measured mesh (see body_origins()) instead of by the bounding box. Not in upstream.
        """
        front = BridgeHelper.get_carla_transform(in_vissim_transform)
        origins = body_origins((front.location.x, front.location.y, front.location.z),
                               front.rotation.yaw, front.rotation.pitch, actors, fleet_plan.mesh)
        return [carla.Transform(carla.Location(x, y, z), front.rotation) for x, y, z in origins]

    @staticmethod
    def get_carla_pedestrian_blueprint(vissim_pedestrian):
        """
        Returns an appropriate walker blueprint based on the received vissim pedestrian.

        vissim -> carla direction only (see PEDESTRIAN_TODO.md). Unlike get_carla_blueprint()
        above, walker blueprints have no 'color'/'driver_id' attributes to randomize.
        """
        type_id = str(vissim_pedestrian.type)
        # Seeded by the vissim id, as in get_carla_blueprint().
        rng = random.Random(int(vissim_pedestrian.id))

        if type_id in BridgeHelper.ptypes:
            candidates = BridgeHelper.ptypes[type_id]
            if candidates:
                blueprint_id = rng.choice(candidates)
            else:
                logging.error(
                    'vissim pedestrian type %s not supported. No pedestrian will be spawned in '
                    'carla', type_id)
                return None

            blueprint = BridgeHelper.blueprint_library.filter(blueprint_id)
            if not blueprint:
                logging.error(
                    'carla blueprint %s unknown. No pedestrian will be spawned', blueprint_id)
                return None
            blueprint = blueprint[0]

            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute('role_name', 'vissim_pedestrian')
            return blueprint

        else:
            logging.error(
                'vissim pedestrian type %s unknown. No pedestrian will be spawned in carla',
                type_id)
            return None
