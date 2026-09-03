#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Vendored from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/vissim_integration/carla_simulation.py`). See ../NOTICE.md for the
# deviation made when vendoring this file (CARLA client/world injection, see __init__ below).
""" This module is responsible for the management of the carla simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import logging

import carla  # pylint: disable=import-error

from .constants import INVALID_ACTOR_ID, CARLA_SPAWN_OFFSET_Z

# ==================================================================================================
# -- carla simulation ------------------------------------------------------------------------------
# ==================================================================================================


class CarlaSimulation(object):
    """
    CarlaSimulation is responsible for the management of the carla simulation.
    """
    def __init__(self, client, world):
        """
        Unlike the upstream implementation (which created its own `carla.Client`/`carla.World`
        from `args.carla_host`/`args.carla_port`), this takes an already connected `client`/
        `world`, injected by `autoware_carla_interface` (`InitializeInterface.load_world()`), so
        that CARLA connection management (including synchronous mode / fixed_delta_seconds) stays
        centralized in a single place. See docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md
        (section 2.5).
        """
        self.client = client
        self.world = world
        self.blueprint_library = self.world.get_blueprint_library()

        # The following sets contain updated information for the current frame.
        self._active_actors = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()

        # Same as above, but for walkers (vissim pedestrians mirrored into carla). Kept separate
        # from the vehicle sets above since walker ids share the same carla actor id space as
        # vehicles but are never controlled/tracked as vehicles (vissim->carla direction only, see
        # PEDESTRIAN_TODO.md - there is no carla->vissim equivalent to mix in here).
        self._active_walkers = set()
        self.spawned_walkers = set()
        self.destroyed_walkers = set()

        # Set traffic lights. {opendrive_id (str): traffic_light actor}. Note this is keyed by the
        # OpenDRIVE signal id (as returned by carla.TrafficLight.get_opendrive_id()), matching the
        # id space used in data/signal_mapping.json - not the carla actor id.
        self._traffic_lights = {}
        for traffic_light in self.world.get_actors().filter('traffic.traffic_light*'):
            opendrive_id = traffic_light.get_opendrive_id()
            if opendrive_id:
                self._traffic_lights[opendrive_id] = traffic_light
            else:
                logging.warning('carla traffic light actor %d has no OpenDRIVE id',
                                traffic_light.id)

    def get_actor(self, actor_id):
        """
        Accessor for carla actor.
        """
        return self.world.get_actor(actor_id)

    @property
    def traffic_light_ids(self):
        """
        Returns the set of known carla traffic light OpenDRIVE ids.
        """
        return set(self._traffic_lights.keys())

    def get_traffic_light_state(self, opendrive_id):
        """
        Accessor for traffic light state.

        If the traffic light does not exist, returns None.
        """
        if opendrive_id not in self._traffic_lights:
            return None
        return self._traffic_lights[opendrive_id].state

    def switch_off_traffic_lights(self, opendrive_ids=None):
        """
        Freezes the given carla traffic lights and forces them to green, so that carla's own
        autonomous signal program stops overriding them - required before an external simulator
        (here, vissim) can reliably drive their actual state via synchronize_traffic_light().

        (Green, not 'off', is used because the 'off' visual state actually reports as Red in
        carla.TrafficLightState, which would otherwise mask our own explicit set_state() calls.)

            :param opendrive_ids: iterable of OpenDRIVE ids to switch off. If None, every known
                traffic light is switched off (matches sumo_integration's behavior).
        """
        if opendrive_ids is None:
            traffic_lights = self._traffic_lights.values()
        else:
            traffic_lights = [
                self._traffic_lights[opendrive_id] for opendrive_id in opendrive_ids
                if opendrive_id in self._traffic_lights
            ]

        for traffic_light in traffic_lights:
            traffic_light.freeze(True)
            traffic_light.set_state(carla.TrafficLightState.Green)

    def unfreeze_traffic_lights(self, opendrive_ids=None):
        """
        Unfreezes the given carla traffic lights, handing control back to carla's own autonomous
        signal program. Intended to undo switch_off_traffic_lights() when co-simulation ends, so
        that a long-running/shared carla server is not left with permanently frozen traffic
        lights after this script exits.

            :param opendrive_ids: iterable of OpenDRIVE ids to unfreeze. If None, every known
                traffic light is unfrozen.
        """
        if opendrive_ids is None:
            traffic_lights = self._traffic_lights.values()
        else:
            traffic_lights = [
                self._traffic_lights[opendrive_id] for opendrive_id in opendrive_ids
                if opendrive_id in self._traffic_lights
            ]

        for traffic_light in traffic_lights:
            traffic_light.freeze(False)

    def synchronize_traffic_light(self, opendrive_id, state):
        """
        Updates traffic light state.

            :param opendrive_id: OpenDRIVE id of the traffic light to be updated.
            :param state: new carla.TrafficLightState.
            :return: True if successfully updated. Otherwise, False.
        """
        if opendrive_id not in self._traffic_lights:
            logging.warning('carla traffic light with OpenDRIVE id %s not found', opendrive_id)
            return False

        self._traffic_lights[opendrive_id].set_state(state)
        return True

    def spawn_actor(self, blueprint, transform):
        """
        Spawns a new actor.

            :param blueprint: blueprint of the actor to be spawned.
            :param transform: transform where the actor will be spawned.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        transform = carla.Transform(transform.location + carla.Location(0, 0, CARLA_SPAWN_OFFSET_Z),
                                    transform.rotation)

        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False))
        ]
        response = self.client.apply_batch_sync(batch, False)[0]
        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return INVALID_ACTOR_ID

        return response.actor_id

    def destroy_actor(self, actor_id):
        """
        Destroys the given actor.
        """
        actor = self.world.get_actor(actor_id)
        if actor is not None:
            return actor.destroy()
        return False

    def synchronize_vehicle(self, vehicle_id, transform, velocity, lights=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param lights: new vehicle light state.
            :return: True if successfully updated. Otherwise, False.
        """
        vehicle = self.world.get_actor(vehicle_id)
        if vehicle is None:
            return False

        vehicle.set_transform(transform)
        if velocity is not None:
            vehicle.set_target_velocity(velocity)

        if lights is not None:
            vehicle.set_light_state(carla.VehicleLightState(lights))
        return True

    def synchronize_pedestrian(self, walker_id, transform, velocity=None):
        """
        Updates pedestrian (walker) state.

        vissim -> carla direction only (see PEDESTRIAN_TODO.md): the walker is driven purely
        kinematically via set_transform(), the same approach used by synchronize_vehicle() above,
        rather than via carla.WalkerControl - chosen for consistency/simplicity with the existing
        vehicle sync, at the cost of the walker's own walking animation not necessarily matching
        its actual speed.

            :param walker_id: id of the actor to be updated.
            :param transform: new pedestrian transform (i.e., position and rotation).
            :param velocity: new pedestrian velocity.
            :return: True if successfully updated. Otherwise, False.
        """
        walker = self.world.get_actor(walker_id)
        if walker is None:
            return False

        walker.set_transform(transform)
        if velocity is not None:
            walker.set_target_velocity(velocity)
        return True

    def tick(self):
        """
        Tick to carla simulation.

        Kept for standalone/backward-compatibility use (e.g. a future non-`autoware_carla_interface`
        caller); the wired-in main loop calls `world.tick()` itself (exactly once per loop) and
        then `update_actor_diff()` directly instead, so CARLA tick stays centralized in exactly one
        place. See docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md Step 4 / section 3.2.
        """
        self.world.tick()
        self.update_actor_diff()

    def update_actor_diff(self):
        """
        Updates `spawned_actors`/`destroyed_actors`/the internal active-actor set for the current
        frame, without ticking the world. Must be called once per frame, after `world.tick()` has
        already advanced the simulation elsewhere.
        """
        current_actors = set(
            [vehicle.id for vehicle in self.world.get_actors().filter('vehicle.*')])
        self.spawned_actors = current_actors.difference(self._active_actors)
        self.destroyed_actors = self._active_actors.difference(current_actors)
        self._active_actors = current_actors

        # Same as above, but for walkers (vissim pedestrians mirrored into carla).
        current_walkers = set(
            [walker.id for walker in self.world.get_actors().filter('walker.pedestrian.*')])
        self.spawned_walkers = current_walkers.difference(self._active_walkers)
        self.destroyed_walkers = self._active_walkers.difference(current_walkers)
        self._active_walkers = current_walkers
