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

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of ptv-vissim and carla
    simulations.
    """
    def __init__(self, vissim_simulation, carla_simulation, args):
        self.args = args

        self.vissim = vissim_simulation
        self.carla = carla_simulation

        # weather setting
        self.carla.world.set_weather(carla.WeatherParameters.ClearNoon)

        # Mapped actor ids.
        self.vissim2carla_ids = {}  # Contains only actors controlled by vissim.
        self.carla2vissim_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(dir_path, 'data', 'vtypes.json')) as f:
            BridgeHelper.vtypes = json.load(f)

        # Signal (traffic light) synchronization is vissim -> carla only: the DS Interface has no
        # function to push a state back into vissim (see TRAFFIC_SIGNAL_TODO.md task 1), so there
        # is no direction to choose, only whether it is enabled at all.
        self.sync_traffic_lights = args.sync_traffic_lights

        # {(controller_id, signal_group_id): [opendrive_id (str), ...]}. Empty if disabled, the
        # mapping file is missing, or it fails to load - in all of those cases signal sync is a
        # no-op in tick().
        self.signal_mapping = {}
        if self.sync_traffic_lights:
            self.signal_mapping = self._load_signal_mapping(
                os.path.join(dir_path, 'data', 'signal_mapping.json'))

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
                    'signal_mapping.json references %d carla OpenDRIVE id(s) not found in the '
                    'current map: %s', len(missing_opendrive_ids), sorted(missing_opendrive_ids))

            # Freeze carla's own signal program on exactly the mapped traffic lights, so that our
            # own per-tick synchronize_traffic_light() calls are not immediately overridden.
            self.carla.switch_off_traffic_lights(known_opendrive_ids)
            self._frozen_opendrive_ids = known_opendrive_ids
        elif self.sync_traffic_lights:
            logging.warning(
                'Traffic light synchronization was requested (--sync-traffic-lights) but no '
                'usable signal_mapping.json was found - no traffic lights will be synchronized.')
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
        Loads data/signal_mapping.json and returns {(controller_id, signal_group_id):
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

            carla_blueprint = BridgeHelper.get_carla_blueprint(vissim_actor)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(vissim_actor.get_transform())
                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)

                if carla_actor_id != INVALID_ACTOR_ID:
                    self.vissim2carla_ids[vissim_actor_id] = carla_actor_id

        # Destroying vissim controlled vehicles in carla.
        for vissim_actor_id in self.vissim.destroyed_vehicles:
            if vissim_actor_id in self.vissim2carla_ids:
                self.vissim.destroy_actor(self.vissim2carla_ids.pop(vissim_actor_id))

        # Updating vissim controlled vehicles in carla.
        for vissim_actor_id in self.vissim2carla_ids:
            carla_actor_id = self.vissim2carla_ids[vissim_actor_id]

            vissim_actor = self.vissim.get_actor(vissim_actor_id)
            carla_actor = self.carla.get_actor(carla_actor_id)

            carla_transform = BridgeHelper.get_carla_transform(vissim_actor.get_transform(),
                                                               carla_actor.bounding_box.extent)
            carla_velocity = BridgeHelper.get_carla_velocity(vissim_actor.get_velocity())
            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_velocity)

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
        carla_spawned_actors = self.carla.spawned_actors - set(self.vissim2carla_ids.values())
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

        # Destroying synchronized actors.
        for carla_actor_id in self.vissim2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        # Closing PTV-Vissim connection. Note: signal state is read via VISSIM_GetSignalStates,
        # which is a plain poll (unlike SUMO's traci subscriptions) - there is no signal-related
        # subscription to cancel here, VISSIM_Disconnect() is sufficient on its own.
        self.vissim.close()
