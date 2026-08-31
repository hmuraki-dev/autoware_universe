# Vendored code notice

The files in this directory (`constants.py`, `vissim_simulation.py`, `bridge_helper.py`,
`carla_simulation.py`, `simulation_synchronization.py`, `data/vtypes.json`,
`data/signal_mapping.json`) are vendored from CARLA's official Vissim-CARLA co-simulation bridge:

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
  management stays centralized in `InitializeInterface.load_world()`. All other methods are
  unmodified.
- `vissim_simulation.py`: the upstream file contained a module-level debug block (`dsi =
  ctypes.CDLL("/opt/vissim_kernel_2026.00-10/lib/libDrivingSimulatorProxy.so")` and
  `print_vissim_last_error()`) that eagerly loads the DS Interface library from a hardcoded
  absolute path at *import* time, and is not referenced anywhere else in the file. This was
  removed when vendoring: left in place, it would make importing this module fail unconditionally
  in any environment where that exact path does not exist, independently of the (correct, already
  lazy/configurable) library loading that `PTVVissimSimulation.__init__` performs via
  `args.vissim_lib_path`. Everything else in the file is unmodified.
- `simulation_synchronization.py`: extracted from the upstream `run_synchronization.py`, keeping
  only the `SimulationSynchronization` class definition (the CLI entry point / standalone
  `while True:` loop / pacing logic in `run_synchronization.py` are intentionally not vendored,
  since `autoware_carla_interface` provides its own main loop). The duplicate CARLA synchronous
  mode configuration block (`world.apply_settings()` with `synchronous_mode`/
  `fixed_delta_seconds`) was removed from `__init__`, since `autoware_carla_interface`
  (`InitializeInterface.load_world()`) already configures this.
- `constants.py`, `bridge_helper.py`, `data/vtypes.json`, `data/signal_mapping.json`: vendored
  without modification (only this provenance header was added to `constants.py`/`bridge_helper.py`).

Not vendored (see plan doc Step 0 ④):

- `run_synchronization.py`'s CLI entry point / standalone loop / real-time pacing
- `CarlaSimulation.__init__`'s original independent `carla.Client` creation
- `SimulationSynchronization.__init__`'s duplicate synchronous-mode configuration
- `CarlaSimulation.tick()`'s `world.tick()` call (CARLA tick stays centralized in
  `autoware_carla_interface`; see plan doc Step 4)
- `test_carla_spawn_autopilot.py` (multi-process test helper, superseded by the single-process
  integration)
