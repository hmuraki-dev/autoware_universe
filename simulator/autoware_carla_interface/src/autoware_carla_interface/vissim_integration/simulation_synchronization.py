#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Extracted from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/run_synchronization.py`), keeping only the `SimulationSynchronization`
# class definition. See ../NOTICE.md for the deviations made when vendoring this file (the CLI
# entry point / standalone loop are intentionally not vendored, and the duplicate CARLA
# synchronous-mode configuration was removed from __init__).
"""
Script to co-simulate CARLA and PTV-Vissim.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import json
import logging
import os

import carla

from .bridge_helper import BridgeHelper
from .constants import INVALID_ACTOR_ID

# Catch-up spawn after the Vissim warmup (see spawn_all_vissim_actors_in_carla()): an actor whose
# spawn fails (e.g. "collision at spawn position" with a neighbor in a queue, both lifted by the
# same CARLA_SPAWN_OFFSET_Z) is spawned again this much higher each time, up to this many times.
# CARLA only checks collisions at spawn time, and the actor is moved to its actual position right
# after, so the extra height never shows. Specific to this repository (not in upstream).
CATCH_UP_SPAWN_RETRIES = 10
CATCH_UP_SPAWN_LIFT_STEP_M = 4.0

# A vissim vehicle represented by several CARLA bodies (carla_variants, see docs/
# Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md F6): the 2nd and later bodies
# overlap the 1st one, so spawning them at their position fails ("collision at spawn position").
# They are spawned this far below the vehicle instead (CarlaSimulation.spawn_actor() adds
# CARLA_SPAWN_OFFSET_Z), one EXTRA_BODY_PARK_STEP_M lower per body and per retry, and moved to
# their position in the same frame, like every other mirrored vehicle. Not in upstream.
EXTRA_BODY_PARK_Z_M = -200.0
EXTRA_BODY_PARK_STEP_M = 10.0
EXTRA_BODY_SPAWN_RETRIES = 3

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of ptv-vissim and carla
    simulations.
    """
    # FleetPlan (fleet_plan.py) when the CARLA bodies of vissim vehicles come from a carla_variants
    # file (see docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md F5), else None:
    # the upstream behavior (data/vtypes.json, a random model per vehicle type). A class attribute
    # so that instances built without __init__ (tests) get the upstream behavior.
    fleet = None

    def __init__(self, vissim_simulation, carla_simulation, args):
        self.args = args

        self.vissim = vissim_simulation
        self.carla = carla_simulation
        self.fleet = getattr(args, 'fleet_plan', None)

        # NOTE: weather is no longer hardcoded here. autoware_carla_interface
        # (InitializeInterface.load_world()) already applies the `weather` ROS parameter to the
        # world before this class is constructed (via _init_vissim_integration()); setting it
        # again here would silently override that configurable value back to ClearNoon.

        # Mapped actor ids.
        self.vissim2carla_ids = {}  # Contains only actors controlled by vissim.
        self.carla2vissim_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(dir_path, 'data', 'vtypes.json')) as f:
            BridgeHelper.vtypes = json.load(f)

        # Pedestrian synchronization is vissim -> carla only: Simulator_Ped_Data has no
        # id/create/delete fields to push a carla-controlled pedestrian back into vissim (see
        # PEDESTRIAN_TODO.md). Unlike traffic light sync, this is always enabled - same as vehicle
        # synchronization above, there is no CLI flag to opt out of it.

        # Mapped pedestrian ids. vissim PedestrianID -> carla walker actor id. Kept separate from
        # vissim2carla_ids (vehicles above) since the two use unrelated id spaces/lifecycles, and
        # there is no carla2vissim equivalent for pedestrians.
        self.vissim2carla_ped_ids = {}

        BridgeHelper.ptypes = self._load_ptypes(os.path.join(dir_path, 'data', 'ptypes.json'))
        if BridgeHelper.ptypes:
            logging.info('Vissim pedestrian type(s) mapped : %s', sorted(BridgeHelper.ptypes.keys()))
        else:
            logging.warning(
                'No usable ptypes.json was found - no pedestrians will be synchronized.')

        # Signal (traffic light) synchronization is vissim -> carla only: the DS Interface has no
        # function to push a state back into vissim (see TRAFFIC_SIGNAL_TODO.md task 1), so there
        # is no direction to choose, only whether it is enabled at all.
        self.sync_traffic_lights = args.sync_traffic_lights

        # {(controller_id, signal_group_id): [opendrive_id (str), ...]}. Empty if disabled, the
        # mapping file is missing, or it fails to load - in all of those cases signal sync is a
        # no-op in tick(). The file is chosen by the vissim_signal_mapping launch argument
        # (signal_mapping.py; auto = data/signal_mapping_<CARLA map>.json) and checked to exist
        # by autoware_carla_interface before Vissim is started; args.signal_mapping_path holds it.
        self.signal_mapping = {}
        self.signal_mapping_path = getattr(args, 'signal_mapping_path', None)
        if self.sync_traffic_lights and self.signal_mapping_path:
            logging.info('Vissim signal mapping file : %s', self.signal_mapping_path)
            self.signal_mapping = self._load_signal_mapping(self.signal_mapping_path)

        # OpenDRIVE ids frozen via switch_off_traffic_lights() below, remembered so close() can
        # hand them back to carla's own autonomous signal program (unfreeze_traffic_lights()).
        self._frozen_opendrive_ids = set()

        # Last vissim VissimSignalState seen per (controller_id, signal_group_id), used only to
        # log state *transitions* (not every tick) for cross-checking against vissim's own timing
        # when diagnosing a suspected synchronization delay - see TRAFFIC_SIGNAL_TODO.md task 9.
        self._last_signal_states = {}

        if self.signal_mapping:
            all_mapped_opendrive_ids = {
                opendrive_id
                for opendrive_ids in self.signal_mapping.values() for opendrive_id in opendrive_ids
            }
            known_opendrive_ids = all_mapped_opendrive_ids & self.carla.traffic_light_ids
            missing_opendrive_ids = all_mapped_opendrive_ids - self.carla.traffic_light_ids

            logging.info('Vissim signal group(s) mapped : %s', sorted(self.signal_mapping.keys()))
            logging.info('Carla traffic light(s) mapped : %s', sorted(known_opendrive_ids))
            if missing_opendrive_ids:
                logging.warning(
                    '%s references %d carla OpenDRIVE id(s) not found in the current map: %s',
                    os.path.basename(self.signal_mapping_path), len(missing_opendrive_ids),
                    sorted(missing_opendrive_ids))

            # Freeze carla's own signal program on exactly the mapped traffic lights, so that our
            # own per-tick synchronize_traffic_light() calls are not immediately overridden.
            self.carla.switch_off_traffic_lights(known_opendrive_ids)
            self._frozen_opendrive_ids = known_opendrive_ids
        elif self.sync_traffic_lights:
            logging.warning(
                'Traffic light synchronization was requested (--sync-traffic-lights) but no '
                'usable signal mapping file was found (%s) - no traffic lights will be '
                'synchronized.', self.signal_mapping_path)
        else:
            # Logged explicitly (not just silence) so it is obvious from the log alone - without
            # this, carla's traffic lights keep running their own autonomous program, completely
            # disconnected from vissim, which looks identical to a broken/laggy synchronization if
            # you are only watching the vissim-side '[vissim] sample signal' log.
            logging.info(
                'Traffic light synchronization is disabled (pass --sync-traffic-lights to '
                'enable it). Carla traffic lights will keep running their own autonomous program.'
            )

        # NOTE: unlike the upstream run_synchronization.py, this __init__ intentionally does NOT
        # configure carla's synchronous mode / fixed_delta_seconds here. autoware_carla_interface
        # (InitializeInterface.load_world()) already does this once, centrally; duplicating it
        # here would conflict with that single source of truth. See docs/
        # Vissim_CARLA_Autoware_統合_実装計画_v1.0.md section 2.6.

    @staticmethod
    def _load_signal_mapping(path):
        """
        Loads a signal mapping file (data/signal_mapping_<town>.json) and returns {(controller_id, signal_group_id):
        [opendrive_id (str), ...]}, or {} if the file is missing/invalid (logged as a warning, not
        a fatal error, so that vehicle-only co-simulation keeps working without this file).
        """
        try:
            with open(path) as f:
                raw_mapping = json.load(f)
        except (IOError, OSError, ValueError) as e:
            logging.warning('Could not load signal mapping file %s: %s', path, e)
            return {}

        mapping = {}
        for controller_id_str, signal_groups in raw_mapping.items():
            for signal_group_id_str, opendrive_ids in signal_groups.items():
                key = (int(controller_id_str), int(signal_group_id_str))
                mapping[key] = [str(opendrive_id) for opendrive_id in opendrive_ids]
        return mapping

    @staticmethod
    def _load_ptypes(path):
        """
        Loads data/ptypes.json and returns {pedestrianType (str): [carla walker blueprint id,
        ...]}, or {} if the file is missing/invalid (logged as a warning, not a fatal error, so
        that vehicle-only co-simulation keeps working without this file).
        """
        try:
            with open(path) as f:
                return json.load(f)
        except (IOError, OSError, ValueError) as e:
            logging.warning('Could not load pedestrian type mapping file %s: %s', path, e)
            return {}

    def tick(self):
        """
        Tick to simulation synchronization

        Kept for standalone/backward-compatibility use; the wired-in main loop calls
        `sync_vissim_to_carla()`, then ticks CARLA exactly once itself, then calls
        `sync_carla_to_vissim()` instead, so that CARLA tick stays centralized in exactly one
        place. See docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md Step 4 / section 3.2.
        """
        self.sync_vissim_to_carla()
        self.carla.tick()
        self.sync_carla_to_vissim()

    def sync_vissim_to_carla(self):
        """
        Vissim -> CARLA sync: ticks vissim (push of the previous frame's CARLA-origin vehicle
        state + pull of vissim's NPC/signal state, see section 3.2-1), then reflects vissim's NPC
        state (spawn/destroy/position) and, if enabled, its traffic light state into CARLA.

        Does NOT tick CARLA itself; must be called before the (single, external) CARLA tick.
        """
        self.vissim.tick()

        # Spawning vissim controlled vehicles in carla.
        vissim_spawned_actors = self.vissim.spawned_vehicles - set(self.carla2vissim_ids.values())
        for vissim_actor_id in vissim_spawned_actors:
            vissim_actor = self.vissim.get_actor(vissim_actor_id)

            carla_blueprint, carla_transform = self._vehicle_spawn(vissim_actor)
            if carla_blueprint is not None:
                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)

                if (carla_actor_id != INVALID_ACTOR_ID and
                        self._spawn_extra_bodies(vissim_actor, carla_actor_id)):
                    self.vissim2carla_ids[vissim_actor_id] = carla_actor_id
                    self._record_vehicle_spawn(vissim_actor, carla_blueprint)

        # Destroying vissim controlled vehicles in carla.
        for vissim_actor_id in self.vissim.destroyed_vehicles:
            if vissim_actor_id in self.vissim2carla_ids:
                self.carla.destroy_actor(self.vissim2carla_ids.pop(vissim_actor_id))
                self._destroy_extra_bodies(vissim_actor_id)

        # Updating vissim controlled vehicles in carla.
        for vissim_actor_id in self.vissim2carla_ids:
            carla_actor_id = self.vissim2carla_ids[vissim_actor_id]

            vissim_actor = self.vissim.get_actor(vissim_actor_id)

            carla_transform = self._vehicle_transform(vissim_actor, carla_actor_id)
            carla_velocity = BridgeHelper.get_carla_velocity(vissim_actor.get_velocity())
            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_velocity)
            self._synchronize_extra_bodies(vissim_actor_id, vissim_actor, carla_velocity)

        # -------------------------
        # vissim-->carla pedestrian sync
        # -------------------------
        # Spawning vissim pedestrians in carla. Unlike vehicles, no subtraction against a
        # carla->vissim mapping is needed here: VISSIM_GetTrafficPedestrians already excludes
        # any simulator pedestrian(s), and there are none anyway since carla -> vissim
        # pedestrian sync is out of scope (see PEDESTRIAN_TODO.md).
        for vissim_pedestrian_id in self.vissim.spawned_pedestrians:
            vissim_pedestrian = self.vissim.get_pedestrian(vissim_pedestrian_id)

            carla_blueprint = BridgeHelper.get_carla_pedestrian_blueprint(vissim_pedestrian)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_pedestrian_transform(vissim_pedestrian)
                carla_walker_id = self.carla.spawn_actor(carla_blueprint, carla_transform)

                if carla_walker_id != INVALID_ACTOR_ID:
                    self.vissim2carla_ped_ids[vissim_pedestrian_id] = carla_walker_id
                    logging.debug(
                        '[sync] pedestrian %s: spawned carla walker %s (blueprint=%s) at '
                        'pos=(%.2f, %.2f, %.2f)', vissim_pedestrian_id, carla_walker_id,
                        carla_blueprint.id, carla_transform.location.x,
                        carla_transform.location.y, carla_transform.location.z)
                else:
                    logging.debug(
                        '[sync] pedestrian %s: carla.spawn_actor() failed (blueprint=%s), '
                        'see the "Spawn carla actor failed" error above', vissim_pedestrian_id,
                        carla_blueprint.id)
            else:
                logging.debug(
                    '[sync] pedestrian %s: no carla blueprint resolved (pedestrianType=%s), '
                    'see the error above - not spawned', vissim_pedestrian_id,
                    vissim_pedestrian.type)

        # Destroying vissim pedestrians in carla.
        for vissim_pedestrian_id in self.vissim.destroyed_pedestrians:
            if vissim_pedestrian_id in self.vissim2carla_ped_ids:
                self.carla.destroy_actor(self.vissim2carla_ped_ids.pop(vissim_pedestrian_id))

        # Updating vissim pedestrians in carla.
        for vissim_pedestrian_id in self.vissim2carla_ped_ids:
            carla_walker_id = self.vissim2carla_ped_ids[vissim_pedestrian_id]

            vissim_pedestrian = self.vissim.get_pedestrian(vissim_pedestrian_id)

            carla_transform = BridgeHelper.get_carla_pedestrian_transform(vissim_pedestrian)
            carla_velocity = BridgeHelper.get_carla_velocity(vissim_pedestrian.get_velocity())
            self.carla.synchronize_pedestrian(carla_walker_id, carla_transform, carla_velocity)

        # Periodic sample log (same cadence/style as vissim_simulation.py's '[vissim] sample
        # ...' logs), so that --debug alone is enough to tell whether any carla walker is
        # currently mirroring a vissim pedestrian.
        if self.vissim.tick_count % 20 == 0:
            logging.debug(
                '[sync] pedestrian sync: %d carla walker(s) currently mapped (vissim '
                'pedestrian id -> carla actor id): %s', len(self.vissim2carla_ped_ids),
                self.vissim2carla_ped_ids)

        # -------------------------
        # vissim-->carla signal sync
        # -------------------------
        if self.sync_traffic_lights:
            for signal_id, opendrive_ids in self.signal_mapping.items():
                vissim_state = self.vissim.get_signal_state(signal_id)
                if vissim_state is None:
                    # Not (yet) reported by VISSIM_GetSignalStates this tick - leave the carla
                    # traffic light(s) at their last known state rather than guessing.
                    continue

                if vissim_state != self._last_signal_states.get(signal_id):
                    # Transition-triggered (not per-tick) log, so it stays readable across a long
                    # run while still pinpointing exactly when/where a change was applied. Compare
                    # 'carla frame (pre-tick)' + 1 (i.e., the frame about to be rendered by the
                    # carla.tick() call further below in this same loop iteration) against vissim's
                    # own timestamp/log for the same vissim_tick to check for synchronization lag.
                    logging.debug(
                        '[sync] signal %s: %s -> %s (carla state=%s) at vissim_tick=%d, carla '
                        'frame (pre-tick)=%d', signal_id,
                        self._last_signal_states.get(signal_id), vissim_state.name,
                        BridgeHelper.get_carla_traffic_light_state(vissim_state).name,
                        self.vissim.tick_count, self.carla.world.get_snapshot().frame)
                    self._last_signal_states[signal_id] = vissim_state

                carla_state = BridgeHelper.get_carla_traffic_light_state(vissim_state)
                for opendrive_id in opendrive_ids:
                    self.carla.synchronize_traffic_light(opendrive_id, carla_state)

    def spawn_all_vissim_actors_in_carla(self):
        """
        Spawns in CARLA every vissim vehicle and pedestrian currently in the vissim network that is
        not mirrored in CARLA yet, then returns how many were spawned.

        Used once after the Vissim warmup (see docs/
        Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md section 2.4): during
        the warmup only vissim is ticked, and sync_vissim_to_carla() only spawns the actors that
        appeared in the latest vissim tick, so everything that entered the network during the
        warmup would otherwise never show up in CARLA. Uses the same steps as
        sync_vissim_to_carla()'s spawn blocks (blueprint, transform, spawn), with one difference:
        all the actors accumulated during the warmup are spawned at once, so neighbors in a queue
        (e.g. at a red light) can collide with each other at their spawn positions; a failed spawn
        is therefore retried higher up (CATCH_UP_SPAWN_RETRIES / CATCH_UP_SPAWN_LIFT_STEP_M).
        Actors that still cannot be spawned (unknown type, or every retry failed) are left
        without a CARLA counterpart, as in sync_vissim_to_carla(). Does not tick CARLA.

        Like sync_vissim_to_carla(), the actors just spawned are then moved to their actual
        position right away, before CARLA is ticked: they are spawned lifted by
        CARLA_SPAWN_OFFSET_Z (plus the retries' extra height), and vehicles at vissim's
        front-center-bumper position, which only becomes the actor's center once corrected with
        the actor's own bounding box.

            :return: dict with 'vehicles' / 'pedestrians' (spawned now),
                'vehicles_not_spawned' / 'pedestrians_not_spawned' (sets of vissim ids left
                without a CARLA counterpart) and 'spawn_retries' (number of retried spawns).
        """
        spawn_retries = 0
        vehicles_to_spawn = (self.vissim.vehicle_ids - set(self.vissim2carla_ids.keys()) -
                             set(self.carla2vissim_ids.values()))
        vehicles_not_spawned = set()
        vehicles_spawned = []
        for vissim_actor_id in sorted(vehicles_to_spawn):
            vissim_actor = self.vissim.get_actor(vissim_actor_id)

            carla_blueprint, carla_transform = self._vehicle_spawn(vissim_actor)
            carla_actor_id = INVALID_ACTOR_ID
            if carla_blueprint is not None:
                carla_actor_id, retries = self._spawn_retrying_higher(carla_blueprint,
                                                                     carla_transform)
                spawn_retries += retries

            if (carla_actor_id != INVALID_ACTOR_ID and
                    self._spawn_extra_bodies(vissim_actor, carla_actor_id)):
                self.vissim2carla_ids[vissim_actor_id] = carla_actor_id
                self._record_vehicle_spawn(vissim_actor, carla_blueprint)
                vehicles_spawned.append(vissim_actor_id)
            else:
                vehicles_not_spawned.add(vissim_actor_id)

        # Same update as sync_vissim_to_carla()'s "Updating vissim controlled vehicles in carla",
        # for the vehicles just spawned.
        for vissim_actor_id in vehicles_spawned:
            carla_actor_id = self.vissim2carla_ids[vissim_actor_id]

            vissim_actor = self.vissim.get_actor(vissim_actor_id)

            carla_transform = self._vehicle_transform(vissim_actor, carla_actor_id)
            carla_velocity = BridgeHelper.get_carla_velocity(vissim_actor.get_velocity())
            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_velocity)
            self._synchronize_extra_bodies(vissim_actor_id, vissim_actor, carla_velocity)

        pedestrians_to_spawn = self.vissim.pedestrian_ids - set(self.vissim2carla_ped_ids.keys())
        pedestrians_not_spawned = set()
        pedestrians_spawned = []
        for vissim_pedestrian_id in sorted(pedestrians_to_spawn):
            vissim_pedestrian = self.vissim.get_pedestrian(vissim_pedestrian_id)

            carla_blueprint = BridgeHelper.get_carla_pedestrian_blueprint(vissim_pedestrian)
            carla_walker_id = INVALID_ACTOR_ID
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_pedestrian_transform(vissim_pedestrian)
                carla_walker_id, retries = self._spawn_retrying_higher(carla_blueprint,
                                                                      carla_transform)
                spawn_retries += retries

            if carla_walker_id != INVALID_ACTOR_ID:
                self.vissim2carla_ped_ids[vissim_pedestrian_id] = carla_walker_id
                pedestrians_spawned.append(vissim_pedestrian_id)
            else:
                pedestrians_not_spawned.add(vissim_pedestrian_id)

        # Same update as sync_vissim_to_carla()'s "Updating vissim pedestrians in carla", for the
        # pedestrians just spawned.
        for vissim_pedestrian_id in pedestrians_spawned:
            carla_walker_id = self.vissim2carla_ped_ids[vissim_pedestrian_id]

            vissim_pedestrian = self.vissim.get_pedestrian(vissim_pedestrian_id)

            carla_transform = BridgeHelper.get_carla_pedestrian_transform(vissim_pedestrian)
            carla_velocity = BridgeHelper.get_carla_velocity(vissim_pedestrian.get_velocity())
            self.carla.synchronize_pedestrian(carla_walker_id, carla_transform, carla_velocity)

        return {
            'vehicles': len(vehicles_to_spawn) - len(vehicles_not_spawned),
            'vehicles_not_spawned': vehicles_not_spawned,
            'pedestrians': len(pedestrians_to_spawn) - len(pedestrians_not_spawned),
            'pedestrians_not_spawned': pedestrians_not_spawned,
            'spawn_retries': spawn_retries,
        }

    def _vehicle_spawn(self, vissim_actor):
        """
        Returns (blueprint, spawn transform) of the CARLA body for the given vissim vehicle, or
        (None, None) if the upstream path finds no blueprint (unknown vehicle type).

        With a FleetPlan (see docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md
        F5), the blueprint is the one assigned to the vehicle's model and the transform is already
        the body's final position (by its measured mesh); an unknown or unassigned model stops the
        co-simulation (carla_fleet.VariantError) instead of skipping the vehicle. This is the
        first body of the assignment, kept in vissim2carla_ids; the others (if any) are spawned by
        _spawn_extra_bodies() and kept in vissim2carla_extra_ids (F6). Without a FleetPlan,
        the upstream behavior: a random model of the vehicle type (data/vtypes.json), spawned at
        vissim's front-center-bumper position.
        """
        if self.fleet is None:
            blueprint = BridgeHelper.get_carla_blueprint(vissim_actor)
            if blueprint is None:
                return None, None
            return blueprint, BridgeHelper.get_carla_transform(vissim_actor.get_transform())
        blueprint, spec = BridgeHelper.get_fleet_blueprints(self.fleet, vissim_actor)[0]
        transform, = BridgeHelper.get_fleet_transforms(self.fleet, vissim_actor.get_transform(),
                                                       [spec])
        return blueprint, transform

    def _vehicle_transform(self, vissim_actor, carla_actor_id):
        """
        Returns the carla transform of the CARLA body mirroring the given vissim vehicle: by the
        measured mesh with a FleetPlan, else (upstream) by the actor's own bounding box.
        """
        if self.fleet is None:
            carla_actor = self.carla.get_actor(carla_actor_id)
            return BridgeHelper.get_carla_transform(vissim_actor.get_transform(),
                                                    carla_actor.bounding_box.extent)
        spec = self.fleet.actors_for(vissim_actor.type, vissim_actor.model_filename)[0]
        transform, = BridgeHelper.get_fleet_transforms(self.fleet, vissim_actor.get_transform(),
                                                       [spec])
        return transform

    @property
    def vissim2carla_extra_ids(self):
        """
        vissim vehicle id -> [carla actor id, ...] of its 2nd and later CARLA bodies (F6). Empty
        without a FleetPlan, and for vehicles represented by one body. A property (created on
        first use) so that instances built without __init__ (tests) have it too.
        """
        return self.__dict__.setdefault('_vissim2carla_extra_ids', {})

    def mirrored_carla_ids(self):
        """Every CARLA actor id that mirrors a vissim vehicle (all bodies)."""
        ids = set(self.vissim2carla_ids.values())
        for extra_ids in self.vissim2carla_extra_ids.values():
            ids.update(extra_ids)
        return ids

    def _spawn_extra_bodies(self, vissim_actor, first_carla_actor_id):
        """
        Spawns the 2nd and later CARLA bodies of the given vissim vehicle (F6), below it (see
        EXTRA_BODY_PARK_Z_M); _synchronize_extra_bodies() moves them to their position in the
        same frame. Returns True if the vehicle is complete (or has a single body). If a body cannot be
        spawned, destroys every body of the vehicle already spawned (the first one included),
        logs an error and returns False: the vehicle is then left without a CARLA counterpart, as
        when the first body fails to spawn.
        """
        if self.fleet is None:
            return True
        bodies = BridgeHelper.get_fleet_blueprints(self.fleet, vissim_actor)[1:]
        if not bodies:
            return True
        front = BridgeHelper.get_carla_transform(vissim_actor.get_transform())
        extra_ids = []
        for index, (blueprint, _) in enumerate(bodies, start=1):
            carla_actor_id = INVALID_ACTOR_ID
            for attempt in range(EXTRA_BODY_SPAWN_RETRIES + 1):
                depth = EXTRA_BODY_PARK_STEP_M * (index + attempt * len(bodies))
                parking = carla.Transform(
                    carla.Location(front.location.x, front.location.y,
                                   EXTRA_BODY_PARK_Z_M - depth),
                    front.rotation)
                carla_actor_id = self.carla.spawn_actor(blueprint, parking)
                if carla_actor_id != INVALID_ACTOR_ID:
                    break
            if carla_actor_id == INVALID_ACTOR_ID:
                logging.error('[sync] vehicle %s: CARLA body %d of %d (%s) could not be spawned; '
                              'no CARLA counterpart for this vehicle', vissim_actor.id, index + 1,
                              len(bodies) + 1, blueprint.id)
                for spawned_id in [first_carla_actor_id] + extra_ids:
                    self.carla.destroy_actor(spawned_id)
                return False
            extra_ids.append(carla_actor_id)
        self.vissim2carla_extra_ids[vissim_actor.id] = extra_ids
        self.fleet.record_spawn([blueprint.id for blueprint, _ in bodies])
        return True

    def _synchronize_extra_bodies(self, vissim_actor_id, vissim_actor, carla_velocity):
        """Moves the 2nd and later CARLA bodies of a vissim vehicle (F6), if it has any."""
        extra_ids = self.vissim2carla_extra_ids.get(vissim_actor_id)
        if not extra_ids:
            return
        actors = self.fleet.actors_for(vissim_actor.type, vissim_actor.model_filename)
        transforms = BridgeHelper.get_fleet_transforms(self.fleet, vissim_actor.get_transform(),
                                                       actors)
        for carla_actor_id, carla_transform in zip(extra_ids, transforms[1:]):
            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_velocity)

    def _destroy_extra_bodies(self, vissim_actor_id):
        for carla_actor_id in self.vissim2carla_extra_ids.pop(vissim_actor_id, []):
            self.carla.destroy_actor(carla_actor_id)

    def _record_vehicle_spawn(self, vissim_actor, blueprint):
        """Counts the spawn for the end-of-run summary and logs the body/model residual."""
        if self.fleet is None:
            return
        self.fleet.record_spawn([blueprint.id])
        spec = self.fleet.spec_for(vissim_actor.type, vissim_actor.model_filename)
        logging.debug('[sync] vehicle %s (type %s, %r): %s, dL=%+.3f dW=%+.3f', vissim_actor.id,
                      vissim_actor.type, vissim_actor.model_filename,
                      '+'.join(actor['blueprint'] for actor in spec['actors']), spec['dL'],
                      spec['dW'])

    def _spawn_retrying_higher(self, blueprint, transform):
        """
        Spawns an actor for spawn_all_vissim_actors_in_carla(); if the spawn fails, tries again
        CATCH_UP_SPAWN_LIFT_STEP_M higher each time, up to CATCH_UP_SPAWN_RETRIES times.

            :return: (carla actor id or INVALID_ACTOR_ID, number of retries made).
        """
        carla_actor_id = self.carla.spawn_actor(blueprint, transform)
        retries = 0
        while carla_actor_id == INVALID_ACTOR_ID and retries < CATCH_UP_SPAWN_RETRIES:
            retries += 1
            lifted = carla.Transform(
                carla.Location(transform.location.x, transform.location.y,
                               transform.location.z + retries * CATCH_UP_SPAWN_LIFT_STEP_M),
                transform.rotation)
            carla_actor_id = self.carla.spawn_actor(blueprint, lifted)
        if retries:
            logging.warning(
                '[sync] catch-up spawn of %s needed %d retr%s (%s)', blueprint.id, retries,
                'y' if retries == 1 else 'ies',
                'spawned %.1f m higher' % (retries * CATCH_UP_SPAWN_LIFT_STEP_M)
                if carla_actor_id != INVALID_ACTOR_ID else 'gave up')
        return carla_actor_id, retries

    def sync_carla_to_vissim(self):
        """
        CARLA -> Vissim sync: refreshes CARLA's spawned/destroyed actor diff (without ticking
        CARLA, see `CarlaSimulation.update_actor_diff()`), then reflects CARLA's vehicle state
        (spawn request/destroy request/position, EGO included via the auto-adopt mechanism) into
        vissim.

        Must be called after the (single, external) CARLA tick.
        """
        self.carla.update_actor_diff()

        # Spawning carla controlled vehicles in vissim. This also takes into account carla vehicles
        # that could not be spawned in vissim in previous time steps.
        # Every body of a vissim vehicle (the 2nd and later ones too, F6) is a vissim actor, never
        # a CARLA-origin vehicle to adopt into vissim.
        carla_spawned_actors = self.carla.spawned_actors - self.mirrored_carla_ids()
        carla_spawned_actors.update(
            [c_id for c_id, v_id in self.carla2vissim_ids.items() if v_id == INVALID_ACTOR_ID])
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            vissim_transform = BridgeHelper.get_vissim_transform(carla_actor.get_transform())
            vissim_actor_id = self.vissim.spawn_actor(vissim_transform)

            # Add the vissim_actor_id even if it was not possible to spawn it (INVALID_ACTOR_ID) to
            # try to spawn it again in next time steps.
            self.carla2vissim_ids[carla_actor_id] = vissim_actor_id

        # Destroying carla controlled vehicles in vissim.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2vissim_ids:
                self.vissim.destroy_actor(self.carla2vissim_ids.pop(carla_actor_id))

        # Updating carla controlled vehicles in vissim.
        for carla_actor_id in self.carla2vissim_ids:
            vissim_actor_id = self.carla2vissim_ids[carla_actor_id]
            if vissim_actor_id != INVALID_ACTOR_ID:
                carla_actor = self.carla.get_actor(carla_actor_id)

                vissim_transform = BridgeHelper.get_vissim_transform(
                    carla_actor.get_transform(), carla_actor.bounding_box.extent)
                vissim_velocity = BridgeHelper.get_vissim_velocity(carla_actor.get_velocity())
                self.vissim.synchronize_vehicle(vissim_actor_id, vissim_transform, vissim_velocity)

    def close(self):
        """
        Cleans synchronization.
        """
        # Hand the previously frozen traffic lights back to carla's own autonomous signal program
        # (undoes the switch_off_traffic_lights() call from __init__, if any). Must happen before
        # disabling synchronous_mode below, while world.freeze()/set_state() calls still behave
        # predictably under a fixed, ticked timestep.
        if self._frozen_opendrive_ids:
            self.carla.unfreeze_traffic_lights(self._frozen_opendrive_ids)

        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors (every body of each vissim vehicle).
        for carla_actor_id in self.mirrored_carla_ids():
            self.carla.destroy_actor(carla_actor_id)

        # Destroying synchronized pedestrians (walkers).
        for carla_walker_id in self.vissim2carla_ped_ids.values():
            self.carla.destroy_actor(carla_walker_id)

        # Closing PTV-Vissim connection (sends 'disconnect' to the Windows-side vissim adapter,
        # which in turn calls VISSIM_Disconnect()). Note: signal state is polled as part of every
        # 'tick' request (unlike SUMO's traci subscriptions) - there is no signal-related
        # subscription to cancel here, the disconnect is sufficient on its own.
        self.vissim.close()
