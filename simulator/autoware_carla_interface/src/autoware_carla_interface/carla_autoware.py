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

import random
import signal
import time
from types import SimpleNamespace

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
        # Vissim-CARLA co-simulation synchronizer (see docs/
        # Vissim_CARLA_Autoware_統合_実装計画_v1.0.md). None (default) keeps _tick_sensor()'s
        # behavior identical to CARLA-only operation.
        self.vissim_sync = None

        # Main-loop period measurement
        self._main_loop_measure_wall_start = None
        self._main_loop_measure_sim_start = None
        self._main_loop_measure_count = 0
        self._main_loop_measure_interval = 10

        # ROS interface for reading block timing measured inside carla_ros.py
        self.ros_interface = None

        # Block processing time measurement
        self._block_time_interval = 10
        self._block_time_count = 0

        self._block_time_sum = {
            "sensor": 0.0,
            "light": 0.0,
            "light_cpu": 0.0,
            "light_lock_wait": 0.0,
            "ego_status": 0.0,
            "control": 0.0,
            "vissim_tick": 0.0,
            "vissim_to_carla": 0.0,
            "carla_tick": 0.0,
            "carla_to_vissim": 0.0,
        }

    def _record_block_times(self, block_times, current_sim_time):
        """Accumulate block processing times and print averages every N steps."""

        for key in self._block_time_sum:
            self._block_time_sum[key] += block_times.get(key, 0.0)

        self._block_time_count += 1

        if self._block_time_count < self._block_time_interval:
            return

        count = self._block_time_count

        avg_ms = {
            key: (value / count) * 1000.0
            for key, value in self._block_time_sum.items()
        }

        total_ms = sum(avg_ms.values())

        if self.vissim_sync is not None:
            vissim_vehicle_count = len(self.vissim_sync.vissim2carla_ids)
        else:
            vissim_vehicle_count = 0

        print(
            "[BLOCK_TIME] "
            f"sim_time={current_sim_time:.3f} sec, "
            f"vissim_vehicle_count={vissim_vehicle_count}, "
            f"samples={count}, "
            f"sensor={avg_ms['sensor']:.3f} ms, "
            f"light={avg_ms['light']:.3f} ms, "
            f"light_cpu={avg_ms['light_cpu']:.3f} ms, "
            f"light_lock_wait={avg_ms['light_lock_wait']:.3f} ms, "
            f"ego_status={avg_ms['ego_status']:.3f} ms, "
            f"control={avg_ms['control']:.3f} ms, "
            f"vissim_tick={avg_ms['vissim_tick']:.3f} ms, "
            f"vissim_to_carla={avg_ms['vissim_to_carla']:.3f} ms, "
            f"carla_tick={avg_ms['carla_tick']:.3f} ms, "
            f"carla_to_vissim={avg_ms['carla_to_vissim']:.3f} ms, "
            f"total={total_ms:.3f} ms",
            flush=True,
        )

        # Reset accumulation window
        for key in self._block_time_sum:
            self._block_time_sum[key] = 0.0

        self._block_time_count = 0

    def _stop_loop(self):
        self.running = False

        self._main_loop_measure_interval = 10

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

        if self._main_loop_measure_count < self._main_loop_measure_interval:
            return

        wall_elapsed = current_wall_time - self._main_loop_measure_wall_start
        sim_elapsed = current_sim_time - self._main_loop_measure_sim_start

        wall_period = wall_elapsed / self._main_loop_measure_count
        sim_period = sim_elapsed / self._main_loop_measure_count

        wall_frequency = 1.0 / wall_period if wall_period > 0.0 else 0.0
        sim_frequency = 1.0 / sim_period if sim_period > 0.0 else 0.0

        # Number of Vissim-controlled vehicles currently synchronized to CARLA.
        if self.vissim_sync is not None:
            vissim_vehicle_count = len(self.vissim_sync.vissim2carla_ids)
        else:
            vissim_vehicle_count = 0
            
        print(
            "[MAIN_LOOP_PERIOD] "
            f"sim_time={current_sim_time:.3f} sec, "
            f"vissim_vehicle_count={vissim_vehicle_count}, "
            f"samples={self._main_loop_measure_count}, "
            f"wall_period={wall_period:.6f} sec, "
            f"wall_frequency={wall_frequency:.2f} Hz, "
            f"sim_period={sim_period:.6f} sec, "
            f"sim_frequency={sim_frequency:.2f} Hz",
            flush=True,
        )

        self._main_loop_measure_wall_start = current_wall_time
        self._main_loop_measure_sim_start = current_sim_time
        self._main_loop_measure_count = 0

    def _stop_loop(self):
        self.running = False

    def _tick_sensor(self, timestamp):
        block_times = {
            "sensor": 0.0,
            "light": 0.0,
            "light_cpu": 0.0,
            "light_lock_wait": 0.0,
            "ego_status": 0.0,
            "control": 0.0,
            "vissim_tick": 0.0,
            "vissim_to_carla": 0.0,
            "carla_tick": 0.0,
            "carla_to_vissim": 0.0,
        }

        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds

            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            # --------------------------------------------------------------
            # 1-3. Sensor / light / EGO status
            # Measured inside carla_ros.py
            # --------------------------------------------------------------
            try:
                ego_action = self.sensor()
            except SensorReceivedNoData as e:
                raise RuntimeError(e)

            if self.ros_interface is not None:
                block_times["sensor"] = self.ros_interface._perf_last["sensor"]
                block_times["light"] = self.ros_interface._perf_last["light"]
                block_times["light_cpu"] = self.ros_interface._perf_last["light_cpu"]
                block_times["light_lock_wait"] = self.ros_interface._perf_last["light_lock_wait"]
                block_times["ego_status"] = self.ros_interface._perf_last["ego_status"]

            # --------------------------------------------------------------
            # 4. Control command -> EGO
            # --------------------------------------------------------------
            t0 = time.monotonic()
            self.ego_actor.apply_control(ego_action)
            block_times["control"] = time.monotonic() - t0

            if self.vissim_sync is not None:

                # ----------------------------------------------------------
                # 5. Vissim Tick
                # ----------------------------------------------------------
                t0 = time.monotonic()
                self.vissim_sync.vissim.tick()
                block_times["vissim_tick"] = time.monotonic() - t0

                # ----------------------------------------------------------
                # 6. Vissim -> CARLA synchronization
                # tick_vissim=False because it was already executed above.
                # ----------------------------------------------------------
                t0 = time.monotonic()
                self.vissim_sync.sync_vissim_to_carla(
                    tick_vissim=False
                )
                block_times["vissim_to_carla"] = time.monotonic() - t0

        if self.running:

            # --------------------------------------------------------------
            # 7. CARLA Tick
            # --------------------------------------------------------------
            t0 = time.monotonic()
            CarlaDataProvider.get_world().tick()
            block_times["carla_tick"] = time.monotonic() - t0

            if self.vissim_sync is not None:

                # ----------------------------------------------------------
                # 8. CARLA -> Vissim synchronization
                # ----------------------------------------------------------
                t0 = time.monotonic()
                self.vissim_sync.sync_carla_to_vissim()
                block_times["carla_to_vissim"] = time.monotonic() - t0

            self._record_block_times(
                block_times,
                timestamp.elapsed_seconds,
            )

            # Existing main-loop-period measurement
            self._measure_main_loop_period(
                timestamp.elapsed_seconds
            )

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

        # Vissim-CARLA co-simulation parameters (see docs/
        # Vissim_CARLA_Autoware_統合_実装計画_v1.0.md). use_vissim=False (default) keeps every
        # code path below a complete no-op.
        self.use_vissim = self.param_["use_vissim"]
        self.vissim_network = self.param_["vissim_network"]
        self.vissim_lib_path = self.param_["vissim_lib_path"]
        self.vissim_simulator_vehicles = self.param_["vissim_simulator_vehicles"]
        self.sync_traffic_lights = self.param_["sync_traffic_lights"]
        self.vissim_carla_sim = None
        self.vissim_sim = None
        self.vissim_sync = None

        self._check_vissim_traffic_manager_exclusivity()

    def _check_vissim_traffic_manager_exclusivity(self):
        """
        Rejects `use_vissim=True` combined with `use_traffic_manager=True` at startup.

        The Vissim auto-adopt mechanism (see docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md
        section 2.11) registers *every* `vehicle.*` CARLA actor into Vissim, up to the
        `vissim_simulator_vehicles` cap - with no distinction between the ego vehicle and Traffic
        Manager NPCs. Unlike a plain "everything gets registered and breaks" failure mode, this
        cap makes the failure silent: some NPCs get registered (competing with the ego for the
        same limited slot pool) while the rest are dropped with a warning log only, which is
        harder to notice than an outright error. Reject the combination outright instead.

            :raises ValueError: if both `use_vissim` and `use_traffic_manager` are True.
        """
        if self.use_vissim and self.use_traffic_manager:
            raise ValueError(
                "use_vissim=True and use_traffic_manager=True cannot be combined: the Vissim "
                "auto-adopt mechanism would register Traffic Manager NPCs into Vissim "
                "indiscriminately alongside the ego vehicle, silently competing for the limited "
                "vissim_simulator_vehicles slot pool. Disable one of the two."
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

    def _init_vissim_integration(self, client):
        """
        Initializes Vissim-CARLA co-simulation if enabled (see docs/
        Vissim_CARLA_Autoware_統合_実装計画_v1.0.md). Complete no-op when `use_vissim` is False
        (the default), so existing CARLA-only behavior is unaffected.
        """
        if not self.use_vissim:
            return

        from .vissim_integration.carla_simulation import CarlaSimulation
        from .vissim_integration.simulation_synchronization import SimulationSynchronization
        from .vissim_integration.vissim_simulation import PTVVissimSimulation

        # The vendored classes were written against an argparse.Namespace; a plain namespace with
        # the same attribute names lets us reuse them unmodified. step_length is deliberately
        # reused from fixed_delta_seconds (not a separate parameter) so the CARLA/Vissim step
        # time cannot drift apart - see docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md
        # sections 0.3/2.2 (a step-time mismatch was the confirmed root cause of a CreateID
        # handshake failure in the upstream bridge).
        vissim_args = SimpleNamespace(
            simulator_vehicles=self.vissim_simulator_vehicles,
            vissim_lib_path=self.vissim_lib_path or None,
            vissim_network=self.vissim_network,
            step_length=self.fixed_delta_seconds,
            sync_traffic_lights=self.sync_traffic_lights,
        )

        self.vissim_carla_sim = CarlaSimulation(client, self.world)
        self.vissim_sim = PTVVissimSimulation(vissim_args)
        self.vissim_sync = SimulationSynchronization(
            self.vissim_sim, self.vissim_carla_sim, vissim_args
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

        self._init_vissim_integration(client)

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
        self.bridge_loop.vissim_sync = self.vissim_sync
        self.bridge_loop.ros_interface = self.interface
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
        self._cleanup_vissim()
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

    def _cleanup_vissim(self):
        """
        Tear down the Vissim-CARLA co-simulation, continuing past individual failures.

        Reimplements what `SimulationSynchronization.close()` does, but with each step isolated
        in its own try/except so that one failing step (e.g. destroying a single synchronized
        actor) cannot prevent the later steps - in particular `PTVVissimSimulation.close()`
        (`VISSIM_Disconnect()`) - from running. See docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md
        Step 7.

        Order:
          1. Destroy CARLA actors mirrored from vissim (`vissim2carla_ids`), one at a time.
          2. Destroy CARLA walkers mirrored from vissim pedestrians (`vissim2carla_ped_ids`), one
             at a time. Pedestrian synchronization is vissim -> carla only (see docs/
             Vissim_CARLA_Autoware_歩行者同期_実装計画_v1.0.md), so there is no vissim-side
             pedestrian counterpart to destroy here, unlike vehicles below.
          3. Destroy vissim-side vehicles mirrored from CARLA (`carla2vissim_ids`, EGO included).
             The real CARLA EGO actor itself is left untouched here (`_cleanup_ego_actor()`
             handles it separately).
          4. Unfreeze traffic lights that were frozen for signal sync.
          5. Disconnect from the Vissim Kernel (`VISSIM_Disconnect()`) - always attempted last,
             regardless of whether the steps above succeeded.

        Deliberately does NOT restore CARLA's world settings to asynchronous mode (unlike
        `SimulationSynchronization.close()`), so that `use_vissim`'s value does not change
        shutdown behavior compared to CARLA-only operation (which has never restored settings
        either).
        """
        if not self.vissim_sync:
            return

        for carla_actor_id in list(self.vissim_sync.vissim2carla_ids.values()):
            try:
                self.vissim_carla_sim.destroy_actor(carla_actor_id)
            except Exception as e:
                print(f"Warning: Failed to destroy vissim-mirrored CARLA actor "
                      f"{carla_actor_id}: {e}")

        for carla_walker_id in list(self.vissim_sync.vissim2carla_ped_ids.values()):
            try:
                self.vissim_carla_sim.destroy_actor(carla_walker_id)
            except Exception as e:
                print(f"Warning: Failed to destroy vissim-mirrored CARLA walker "
                      f"{carla_walker_id}: {e}")

        for vissim_actor_id in list(self.vissim_sync.carla2vissim_ids.values()):
            try:
                self.vissim_sim.destroy_actor(vissim_actor_id)
            except Exception as e:
                print(f"Warning: Failed to destroy CARLA-mirrored vissim actor "
                      f"{vissim_actor_id}: {e}")

        try:
            if self.vissim_sync._frozen_opendrive_ids:
                self.vissim_carla_sim.unfreeze_traffic_lights(
                    self.vissim_sync._frozen_opendrive_ids
                )
        except Exception as e:
            print(f"Warning: Failed to unfreeze traffic lights: {e}")

        try:
            self.vissim_sim.close()
        except Exception as e:
            print(f"Warning: Vissim disconnect failed: {e}")

    def _cleanup_carla_provider(self):
        """Clean up CARLA data provider, continuing on error."""
        try:
            CarlaDataProvider.cleanup()
        except Exception as e:
            print(f"Warning: CARLA data provider cleanup failed: {e}")


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
