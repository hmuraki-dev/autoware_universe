# Vendored code notice

The files in this directory (`constants.py`, `vissim_simulation.py`, `bridge_helper.py`,
`carla_simulation.py`, `simulation_synchronization.py`, `data/vtypes.json`,
`data/signal_mapping.json`, `data/ptypes.json`) are vendored from CARLA's official Vissim-CARLA
co-simulation bridge:

- Upstream location (this workspace's reference checkout):
  `/home/divp/CARLA/Co-Simulation/PTV-Vissim/vissim_integration/` and
  `/home/divp/CARLA/Co-Simulation/PTV-Vissim/data/`
- Upstream license: MIT (`Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat
  Autonoma de Barcelona (UAB)`, see the header of each vendored file)

See `docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md` (Step 0 / Step 1) for the rationale behind
vendoring these files instead of referencing them via an external path.

## Deviations from the upstream files

- `carla_simulation.py`: `CarlaSimulation.__init__` no longer creates its own `carla.Client`/
  `carla.World` (previously via `args.carla_host`/`args.carla_port`). It now takes an already
  connected `client`/`world` injected by `autoware_carla_interface`, so that CARLA connection
  management stays centralized in `InitializeInterface.load_world()`. `tick()` was also split:
  it is kept for standalone/backward-compatible use, but a new `update_actor_diff()` method
  performs the actor-diff bookkeeping *without* calling `world.tick()`, so that CARLA tick stays
  centralized in exactly one place (`autoware_carla_interface`'s own main loop calls `world.tick()`
  once, then `update_actor_diff()` directly; see plan doc Step 4). All other methods are
  unmodified. The pedestrian (walker) additions - `_active_walkers`/`spawned_walkers`/
  `destroyed_walkers` tracking and `synchronize_pedestrian()` - were vendored with the same
  content as upstream, except that the walker actor-diff computation (upstream: inside `tick()`)
  was placed inside `update_actor_diff()` instead, consistent with the tick()/update_actor_diff()
  split above - see docs/Vissim_CARLA_Autoware_歩行者同期_実装計画_v1.0.md Step P2.
- `vissim_simulation.py`: the upstream file contained a module-level debug block (`dsi =
  ctypes.CDLL("/opt/vissim_kernel_2026.00-10/lib/libDrivingSimulatorProxy.so")` and
  `print_vissim_last_error()`) that eagerly loads the DS Interface library from a hardcoded
  absolute path at *import* time, and is not referenced anywhere else in the file. This was
  removed when vendoring: left in place, it would make importing this module fail unconditionally
  in any environment where that exact path does not exist, independently of the (correct, already
  lazy/configurable) library loading that `PTVVissimSimulation.__init__` performs via
  `args.vissim_lib_path`. Everything else in the file is unmodified, including the pedestrian
  synchronization additions (`VissimPedestrianMotionState`, `VissimPedestrianConstructionElementType`,
  `VISSIM_Ped_Data`, `VissimPedestrian`, `get_pedestrian()`, and the `VISSIM_GetTrafficPedestrians`
  fetching logic in `tick()`), which were vendored byte-for-byte identical to upstream - see docs/
  Vissim_CARLA_Autoware_歩行者同期_実装計画_v1.0.md Step P1.
- `simulation_synchronization.py`: extracted from the upstream `run_synchronization.py`, keeping
  only the `SimulationSynchronization` class definition (the CLI entry point / standalone
  `while True:` loop / pacing logic in `run_synchronization.py` are intentionally not vendored,
  since `autoware_carla_interface` provides its own main loop). The duplicate CARLA synchronous
  mode configuration block (`world.apply_settings()` with `synchronous_mode`/
  `fixed_delta_seconds`) was removed from `__init__`, since `autoware_carla_interface`
  (`InitializeInterface.load_world()`) already configures this. `tick()` was also split into
  `sync_vissim_to_carla()` (former "vissim-->carla sync" + signal sync half, does not tick CARLA)
  and `sync_carla_to_vissim()` (former "carla-->vissim sync" half, starts with
  `CarlaSimulation.update_actor_diff()` instead of ticking CARLA); `tick()` itself is kept as a
  thin `sync_vissim_to_carla() -> self.carla.tick() -> sync_carla_to_vissim()` wrapper for
  standalone/backward-compatible use, but the wired-in main loop calls the two halves directly
  around its own single `world.tick()` call (see plan doc Step 4).
- `bridge_helper.py`: vendored with the `ptypes = {}` class attribute and the
  `get_carla_pedestrian_blueprint()`/`get_carla_pedestrian_transform()` methods added, both
  byte-for-byte identical to upstream. All pre-existing methods (`get_carla_transform()`,
  `get_carla_velocity()`, `get_carla_blueprint()`, etc.) are unmodified.
- `constants.py`, `data/vtypes.json`, `data/signal_mapping.json`, `data/ptypes.json`: vendored
  without modification (only this provenance header was added to `constants.py`). `data/
  ptypes.json` maps vissim pedestrianType (100=Man, 200=Woman, 300=Wheelchair User) to CARLA
  `walker.pedestrian.*` blueprint ids; type 300 has an empty candidate list (no CARLA wheelchair
  walker exists), mirroring `vtypes.json`'s unsupported-type convention.

Not vendored (see plan doc Step 0 ④):

- `run_synchronization.py`'s CLI entry point / standalone loop / real-time pacing
- `CarlaSimulation.__init__`'s original independent `carla.Client` creation
- `SimulationSynchronization.__init__`'s duplicate synchronous-mode configuration
- `CarlaSimulation.tick()`'s `world.tick()` call (CARLA tick stays centralized in
  `autoware_carla_interface`; see plan doc Step 4)
- `test_carla_spawn_autopilot.py` (multi-process test helper, superseded by the single-process
  integration)
