#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import random
import signal
import time

import carla

from .carla_ros import carla_ros2_interface
from .modules.carla_data_provider import CarlaDataProvider
from .modules.carla_data_provider import GameTime
from .modules.carla_wrapper import SensorReceivedNoData
from .modules.carla_wrapper import SensorWrapper


class SensorLoop(object):

    def __init__(self):
        self.start_game_time = None
        self.start_system_time = None
        self.sensor = None
        self.ego_actor = None
        self.running = False
        self.timestamp_last_run = 0.0
        self.timeout = 20.0
        # Step 4 (v0.5 3.1-3.7): vendored SimulationSynchronization instance,
        # or None when SUMO co-simulation is disabled (default). When set,
        # its sync_sumo_to_carla()/sync_carla_to_sumo() halves are called
        # around the single CARLA world.tick() below.
        self.sumo_sync = None

        # Main-loop period measurement
        self._main_loop_measure_wall_start = None
        self._main_loop_measure_sim_start = None
        self._main_loop_measure_count = 0
        self._main_loop_measure_interval = 10

    def _stop_loop(self):
        self.running = False

    def _measure_main_loop_period(self, current_sim_time):
        """Measure and log the average main-loop period."""

        current_wall_time = time.monotonic()

        # First call: initialize the measurement origin.
        if self._main_loop_measure_wall_start is None:
            self._main_loop_measure_wall_start = current_wall_time
            self._main_loop_measure_sim_start = current_sim_time
            self._main_loop_measure_count = 0
            return

        self._main_loop_measure_count += 1

        # Log only after the configured number of loops.
        if self._main_loop_measure_count < self._main_loop_measure_interval:
            return

        wall_elapsed = current_wall_time - self._main_loop_measure_wall_start
        sim_elapsed = current_sim_time - self._main_loop_measure_sim_start

        wall_period = wall_elapsed / self._main_loop_measure_count
        sim_period = sim_elapsed / self._main_loop_measure_count

        wall_frequency = 1.0 / wall_period if wall_period > 0.0 else 0.0
        sim_frequency = 1.0 / sim_period if sim_period > 0.0 else 0.0

        print(
            "[MAIN_LOOP_PERIOD] "
            f"samples={self._main_loop_measure_count}, "
            f"wall_period={wall_period:.6f} sec, "
            f"wall_frequency={wall_frequency:.2f} Hz, "
            f"sim_period={sim_period:.6f} sec, "
            f"sim_frequency={sim_frequency:.2f} Hz",
            flush=True,
        )

        # Reset the measurement window.
        self._main_loop_measure_wall_start = current_wall_time
        self._main_loop_measure_sim_start = current_sim_time
        self._main_loop_measure_count = 0

    def _tick_sensor(self, timestamp):
        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            # Measurement-only processing
            self._measure_main_loop_period(timestamp.elapsed_seconds)
        
            self.timestamp_last_run = timestamp.elapsed_seconds
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            try:
                ego_action = self.sensor()
            except SensorReceivedNoData as e:
                raise RuntimeError(e)
            self.ego_actor.apply_control(ego_action)
            if self.sumo_sync is not None:
                # SUMO Tick + "SUMO→CARLA同期" (v0.5 3.3/3.4). Does not touch
                # CARLA's world.tick(); that stays the single call below.
                self.sumo_sync.sync_sumo_to_carla()
        if self.running:
            CarlaDataProvider.get_world().tick()
            if self.sumo_sync is not None:
                # "CARLA→SUMO同期" (v0.5 3.7). Refreshes CARLA's actor diff
                # for the frame that was just ticked, then reflects CARLA
                # (EGO included, via auto-adapt - v0.5 2.10) into SUMO.
                self.sumo_sync.sync_carla_to_sumo()


class InitializeInterface(object):

    def __init__(self):
        self.interface = carla_ros2_interface()
        self.param_ = self.interface.get_param()
        self.world = None
        self.sensor_wrapper = None
        self.ego_actor = None
        self.prev_tick_wall_time = 0.0

        # Parameter for Initializing Carla World
        self.local_host = self.param_["host"]
        self.port = self.param_["port"]
        self.timeout = self.param_["timeout"]
        self.sync_mode = self.param_["sync_mode"]
        self.fixed_delta_seconds = self.param_["fixed_delta_seconds"]
        self.carla_map = self.param_["carla_map"]
        self.agent_role_name = self.param_["ego_vehicle_role_name"]
        self.vehicle_type = self.param_["vehicle_type"]
        self.spawn_point = self.param_["spawn_point"]
        self.use_traffic_manager = self.param_["use_traffic_manager"]
        self.max_real_delta_seconds = self.param_["max_real_delta_seconds"]

        # SUMO co-simulation parameters (see docs/SUMO_CARLA_Autoware_統合_実装ステップ計画_v1.1.md)
        self.use_sumo = self.param_["use_sumo"]
        self.sumo_cfg_file = self.param_["sumo_cfg_file"]
        self.sumo_gui = self.param_["sumo_gui"]
        self.sumo_host = self.param_["sumo_host"]
        self.sumo_port = self.param_["sumo_port"]
        self.sumo_client_order = self.param_["sumo_client_order"]
        self.sync_vehicle_lights = self.param_["sync_vehicle_lights"]
        self.sync_vehicle_color = self.param_["sync_vehicle_color"]
        self.tls_manager = self.param_["tls_manager"]
        self.sumo_carla_sim = None
        self.sumo_sim = None
        self.sumo_sync = None

        self._check_sumo_traffic_manager_exclusivity()

    def _check_sumo_traffic_manager_exclusivity(self):
        """
        Refuse to start if `use_sumo` and `use_traffic_manager` are both enabled.

        Step 5 (v0.5 section 2.11): the CARLA<->SUMO auto-adapt mechanism
        (v0.5 section 2.10) registers every `vehicle.*` CARLA actor into SUMO,
        with no way to distinguish EGO from Traffic-Manager-driven NPCs. If
        `use_traffic_manager` is also enabled, its randomly spawned NPCs would
        get unintentionally duplicated into SUMO alongside SUMO's own NPCs.
        This is a required precondition for the auto-adapt mechanism to work
        correctly, not just a recommendation, so it is enforced as a hard
        error rather than a warning.
        """
        if self.use_sumo and self.use_traffic_manager:
            raise ValueError(
                "use_sumo and use_traffic_manager cannot both be True. "
                "CARLA's Traffic Manager would spawn random NPCs that the "
                "SUMO auto-adapt mechanism (v0.5 section 2.10) would then "
                "duplicate into SUMO, alongside SUMO's own NPCs. Disable "
                "use_traffic_manager when use_sumo is enabled (see "
                "docs/SUMO_CARLA_Autoware_統合修正項目_v0.5.md section 2.11)."
            )

    def _parse_spawn_point(self):
        """Parse spawn point string and return transform with randomize flag."""
        spawn_point = carla.Transform()
        point_items = self.spawn_point.split(",")
        randomize = False
        if len(point_items) == 6:
            spawn_point.location.x = float(point_items[0])
            spawn_point.location.y = float(point_items[1])
            spawn_point.location.z = (
                float(point_items[2]) + 2
            )  # +2 is used so the car did not stuck on the road when spawned.
            spawn_point.rotation.roll = float(point_items[3])
            spawn_point.rotation.pitch = float(point_items[4])
            spawn_point.rotation.yaw = float(point_items[5])
        else:
            randomize = True
        return spawn_point, randomize

    def _setup_traffic_manager(self, client):
        """Configure traffic manager with NPC vehicles."""
        traffic_manager = client.get_trafficmanager()  # cspell:ignore trafficmanager
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(0)
        random.seed(0)
        spawn_points_tm = self.world.get_map().get_spawn_points()
        for i, spawn_point in enumerate(spawn_points_tm):
            self.world.debug.draw_string(spawn_point.location, str(i), life_time=10)
        models = [
            "dodge",
            "audi",
            "model3",
            "mini",
            "mustang",
            "lincoln",
            "prius",
            "nissan",
            "crown",
            "impala",
        ]
        blueprints = []
        for vehicle in self.world.get_blueprint_library().filter("*vehicle*"):
            if any(model in vehicle.id for model in models):
                blueprints.append(vehicle)
        max_vehicles = 30
        max_vehicles = min([max_vehicles, len(spawn_points_tm)])
        vehicles = []
        for i, spawn_point in enumerate(random.sample(spawn_points_tm, max_vehicles)):
            temp = self.world.try_spawn_actor(random.choice(blueprints), spawn_point)
            if temp is not None:
                vehicles.append(temp)

        for vehicle in vehicles:
            vehicle.set_autopilot(True)

    def _init_sumo_integration(self, client):
        """
        Start SUMO/TraCI and construct the (vendored) SimulationSynchronization.

        See docs/SUMO_CARLA_Autoware_統合_実装ステップ計画_v1.1.md: this
        establishes the CARLA/SUMO connections and builds the synchronization
        engine (ID maps + coordinate transforms initialized in its
        constructor). The resulting `self.sumo_sync` is later handed to
        `SensorLoop` in `run_bridge()`, which calls its
        `sync_sumo_to_carla()`/`sync_carla_to_sumo()` halves around the main
        loop's single `world.tick()` (Step 4). When `use_sumo` is False
        (default), this is a no-op and behavior is unchanged from CARLA-only
        operation.
        """
        if not self.use_sumo:
            return

        # Deferred import: `sumo_simulation.py` requires `traci`/`sumolib`
        # (only available when SUMO_HOME is configured on sys.path), so these
        # must not be imported when SUMO integration is disabled.
        from .sumo_integration.carla_simulation import CarlaSimulation as SumoCarlaSimulation
        from .sumo_integration.simulation_synchronization import SimulationSynchronization
        from .sumo_integration.sumo_simulation import SumoSimulation

        sumo_host = None if self.sumo_host == "None" else self.sumo_host
        sumo_port = None if self.sumo_port == "None" else int(self.sumo_port)

        self.sumo_carla_sim = SumoCarlaSimulation(client, self.world, self.fixed_delta_seconds)
        self.sumo_sim = SumoSimulation(
            self.sumo_cfg_file,
            self.fixed_delta_seconds,
            host=sumo_host,
            port=sumo_port,
            sumo_gui=self.sumo_gui,
            client_order=self.sumo_client_order,
        )
        self.sumo_sync = SimulationSynchronization(
            self.sumo_sim,
            self.sumo_carla_sim,
            tls_manager=self.tls_manager,
            sync_vehicle_color=self.sync_vehicle_color,
            sync_vehicle_lights=self.sync_vehicle_lights,
        )
        self.interface.logger.info(
            "SUMO co-simulation connected (sync engine ready; will be ticked each loop iteration)."
        )

    def load_world(self):
        client = carla.Client(self.local_host, self.port)
        client.set_timeout(self.timeout)
        client.load_world_if_different(self.carla_map)

        # Wait for the world to be fully loaded
        # This is critical for non-default maps that need time to load
        time.sleep(2.0)

        self.world = client.get_world()

        # Verify world is ready by attempting to tick it
        # This ensures the world is fully initialized before accessing settings
        try:
            self.world.tick()
        except RuntimeError:
            # If synchronous mode is not enabled yet, tick() may fail
            # In this case, just wait a bit more
            time.sleep(1.0)

        settings = self.world.get_settings()
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        settings.synchronous_mode = self.sync_mode
        self.world.apply_settings(settings)
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(client)

        # Step 3: connect to SUMO / build the sync engine before spawning EGO,
        # per the confirmed initialization order (v0.5 section 2.0). No-op
        # when `use_sumo` is False.
        self._init_sumo_integration(client)

        spawn_point, randomize = self._parse_spawn_point()
        self.ego_actor = CarlaDataProvider.request_new_actor(
            self.vehicle_type, spawn_point, self.agent_role_name, random_location=randomize
        )
        self.interface.ego_actor = self.ego_actor  # TODO improve design
        self.interface.physics_control = self.ego_actor.get_physics_control()

        self.sensor_wrapper = SensorWrapper(self.interface)
        self.sensor_wrapper.setup_sensors(self.ego_actor, False)

        if self.use_traffic_manager:
            self._setup_traffic_manager(client)

    def run_bridge(self):
        self.bridge_loop = SensorLoop()
        self.bridge_loop.sensor = self.sensor_wrapper
        self.bridge_loop.ego_actor = self.ego_actor
        self.bridge_loop.sumo_sync = self.sumo_sync
        self.bridge_loop.start_system_time = time.time()
        self.bridge_loop.start_game_time = GameTime.get_time()
        self.bridge_loop.running = True
        while self.bridge_loop.running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                delta_step = time.time() - self.prev_tick_wall_time
                if delta_step <= self.max_real_delta_seconds:
                    # Add a wait to match the max_real_delta_seconds
                    time.sleep(self.max_real_delta_seconds - delta_step)
                self.prev_tick_wall_time = time.time()
                self.bridge_loop._tick_sensor(timestamp)

    def _stop_loop(self, sign, frame):
        self.bridge_loop._stop_loop()

    def _cleanup(self):
        """
        Clean up all CARLA resources in reverse initialization order.

        Ensures cleanup happens even if individual steps fail.

        """
        self._cleanup_sensors()
        self._cleanup_ros_interface()
        self._cleanup_ego_actor()
        self._cleanup_sumo()
        self._cleanup_carla_provider()

    def _cleanup_sensors(self):
        """Clean up sensor wrapper, continuing on error."""
        if not self.sensor_wrapper:
            return
        try:
            self.sensor_wrapper.cleanup()
        except Exception as e:
            print(f"Warning: Sensor cleanup failed: {e}")

    def _cleanup_ros_interface(self):
        """Clean up ROS interface, continuing on error."""
        if not self.interface:
            return
        try:
            self.interface.shutdown()
            self.interface = None
        except Exception as e:
            print(f"Warning: ROS interface shutdown failed: {e}")

    def _cleanup_ego_actor(self):
        """Destroy ego vehicle, continuing on error."""
        if not self.ego_actor:
            return
        try:
            self.ego_actor.destroy()
            self.ego_actor = None
        except Exception as e:
            print(f"Warning: Ego actor destruction failed: {e}")

    def _cleanup_carla_provider(self):
        """Clean up CARLA data provider, continuing on error."""
        try:
            CarlaDataProvider.cleanup()
        except Exception as e:
            print(f"Warning: CARLA data provider cleanup failed: {e}")

    def _cleanup_sumo(self):
        """
        Tear down the SUMO co-simulation, continuing on error at every step.

        Step 7 (v0.5 section 2.13/3.11/3.12): integrates
        `SimulationSynchronization`'s actor-destruction + TraCI-disconnect
        behavior into `autoware_carla_interface`'s own cleanup path, instead
        of calling `SimulationSynchronization.close()` directly. That method
        has no per-step error handling: if destroying any single
        synchronized actor raises, it would also skip the final
        `self.sumo.close()` (`traci.close()`) call, leaking the TraCI
        connection/process. Each step here is isolated so one failure cannot
        prevent the others - in particular, closing the TraCI connection -
        from running.

        Deliberately does NOT reset CARLA's `synchronous_mode`/
        `fixed_delta_seconds` settings back to async, unlike upstream
        `SimulationSynchronization.close()` would: `autoware_carla_interface`'s
        own shutdown path has never restored these settings even without
        SUMO, so this keeps shutdown behavior consistent regardless of
        whether `use_sumo` is enabled.
        """
        if self.sumo_sync is None:
            return

        # Destroy SUMO-origin actors mirrored into CARLA (background traffic
        # spawned via sync_sumo_to_carla()).
        for carla_actor_id in list(self.sumo_sync.sumo2carla_ids.values()):
            try:
                self.sumo_carla_sim.destroy_actor(carla_actor_id)
            except Exception as e:
                print(f"Warning: failed to destroy SUMO-origin CARLA actor {carla_actor_id}: {e}")

        # Destroy CARLA-origin actors mirrored into SUMO (this only removes
        # the SUMO-side shadow vehicle; the real CARLA actor - e.g. EGO - is
        # untouched here and is destroyed separately by _cleanup_ego_actor()).
        for sumo_actor_id in list(self.sumo_sync.carla2sumo_ids.values()):
            try:
                self.sumo_sim.destroy_actor(sumo_actor_id)
            except Exception as e:
                print(f"Warning: failed to destroy CARLA-origin SUMO actor {sumo_actor_id}: {e}")

        # Un-freeze any traffic lights CarlaSimulation may have frozen
        # (switch_off_traffic_lights(), used when tls_manager == 'sumo').
        try:
            self.sumo_carla_sim.close()
        except Exception as e:
            print(f"Warning: CarlaSimulation cleanup failed: {e}")

        # Always attempt to close the TraCI connection, even if the above failed.
        try:
            self.sumo_sim.close()
        except Exception as e:
            print(f"Warning: SUMO/TraCI cleanup failed: {e}")

        self.sumo_sync = None
        self.sumo_sim = None
        self.sumo_carla_sim = None


def main():
    """Run the CARLA-Autoware bridge with proper cleanup on all exit paths."""
    carla_bridge = InitializeInterface()
    carla_bridge.load_world()

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, carla_bridge._stop_loop)
    signal.signal(signal.SIGTERM, carla_bridge._stop_loop)

    try:
        carla_bridge.run_bridge()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"\nError during bridge operation: {e}")
        raise
    finally:
        # Ensure cleanup always happens, even on exception or signal
        print("Cleaning up CARLA resources...")
        carla_bridge._cleanup()
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
