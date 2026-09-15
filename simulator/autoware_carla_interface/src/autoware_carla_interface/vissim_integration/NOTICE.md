# Vendored code notice

The files in this directory (`constants.py`, `vissim_simulation.py`, `rpc_protocol.py`,
`bridge_helper.py`, `carla_simulation.py`, `simulation_synchronization.py`, `data/vtypes.json`,
`data/signal_mapping.json`, `data/ptypes.json`) are vendored from CARLA's official Vissim-CARLA
co-simulation bridge:

- Upstream location (this workspace's reference checkout):
  `/home/divp/CARLA/Co-Simulation/PTV-Vissim/vissim_integration/` and
  `/home/divp/CARLA/Co-Simulation/PTV-Vissim/data/`
- `rpc_protocol.py` specifically is vendored from the `feature/vissim_windows` branch of that
  same upstream repository (not `main`), which introduces the ZeroMQ+msgpack remote
  Vissim(Windows)<->CARLA(Linux) co-simulation link. See
  `docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md` for the integration plan.
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
- `vissim_simulation.py`: as of `feature/vissim_windows_co-sim`, this file is re-vendored from the
  upstream `feature/vissim_windows` branch (previously it was vendored from `main`, using ctypes
  to talk to `libDrivingSimulatorProxy.so` directly in-process). `PTVVissimSimulation` is now a
  ZeroMQ REQ client for a Windows-side Vissim adapter instead - see
  `docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md` Step W2 for the full rationale.
  All DLL-specific logic (the `Simulator_Veh_Data`/`VISSIM_Veh_Data`/`VISSIM_Ped_Data`/
  `VISSIM_Sig_Data` ctypes structs, DLL loading, the `Create`/`CreateID` pending/active state
  machine) was removed from this file entirely (it now lives on the Windows side, not vendored
  into this repository - see section 8 of that plan doc). `PTVVissimSimulation`'s public
  interface (`__init__(args)`, `tick()`, `spawn_actor()`, `destroy_actor()`,
  `synchronize_vehicle()`, `get_actor()`, `get_pedestrian()`, `get_signal_state()`, `signal_ids`,
  `tick_count`, `spawned_vehicles`/`destroyed_vehicles`/`spawned_pedestrians`/
  `destroyed_pedestrians`, `close()`) is unchanged, so `simulation_synchronization.py` requires no
  changes. The pedestrian synchronization additions (`VissimPedestrianMotionState`,
  `VissimPedestrianConstructionElementType`, `VissimPedestrian`, `get_pedestrian()`), signal
  synchronization (`VissimSignalState`, `get_signal_state()`), and turn-indicator propagation
  (`VissimLightState`) are all preserved with equivalent behavior (the upstream
  `feature/vissim_windows` branch already had the same features, sourced from the same
  DrivingSimulatorProxy.h fields, just delivered over the wire as msgpack dict payloads instead of
  ctypes struct fields - confirmed field-for-field identical during the Step W0 comparison).
  `args.vissim_lib_path`/`args.vissim_network` were replaced by `args.vissim_adapter_host`/
  `args.vissim_adapter_port`/`args.vissim_connect_timeout_ms`/`args.vissim_rpc_timeout_ms` (see
  plan doc section 3).
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
  around its own single `world.tick()` call (see plan doc Step 4). Pedestrian synchronization
  (`vissim2carla_ped_ids` mapping, `_load_ptypes()`, and the "vissim-->carla pedestrian sync"
  spawn/destroy/update block) was ported byte-for-byte identical to upstream, placed in
  `sync_vissim_to_carla()` (former `tick()`'s single "vissim-->carla" half) since pedestrian sync
  is vissim->carla only - see docs/Vissim_CARLA_Autoware_歩行者同期_実装計画_v1.0.md Step P3. The
  pedestrian actor cleanup loop in `close()` was ported identically as well.
- `bridge_helper.py`: vendored with the `ptypes = {}` class attribute and the
  `get_carla_pedestrian_blueprint()`/`get_carla_pedestrian_transform()` methods added, both
  byte-for-byte identical to upstream. All pre-existing methods (`get_carla_transform()`,
  `get_carla_velocity()`, `get_carla_blueprint()`, etc.) are unmodified.
- `constants.py`, `data/vtypes.json`, `data/signal_mapping.json`, `data/ptypes.json`: vendored
  without modification (only this provenance header was added to `constants.py`). `data/
  ptypes.json` maps vissim pedestrianType (100=Man, 200=Woman, 300=Wheelchair User) to CARLA
  `walker.pedestrian.*` blueprint ids; type 300 has an empty candidate list (no CARLA wheelchair
  walker exists), mirroring `vtypes.json`'s unsupported-type convention.
- `rpc_protocol.py`: vendored functionally unmodified from the upstream `feature/vissim_windows`
  branch (every constant, function, and the `ProtocolError` class are byte-for-byte identical to
  upstream) - only the module-level header comment/docstring was adapted to reference this repo's
  own documentation paths instead of the upstream repository's `docs/WINDOWS_VISSIM_REMOTE_TODO.md`/
  `docs/WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md` (which do not exist in this repo), and to
  clarify that the upstream `Co-Simulation/PTV-Vissim_windows/rpc_protocol.py` copy is not vendored
  here (see `docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md` section 8). This module
  is new to this repository (see plan doc Step W1); the previous ctypes-based `vissim_simulation.py`
  had no equivalent.

Not vendored (see plan doc Step 0 ④):

- `run_synchronization.py`'s CLI entry point / standalone loop / real-time pacing
- `CarlaSimulation.__init__`'s original independent `carla.Client` creation
- `SimulationSynchronization.__init__`'s duplicate synchronous-mode configuration
- `CarlaSimulation.tick()`'s `world.tick()` call (CARLA tick stays centralized in
  `autoware_carla_interface`; see plan doc Step 4)
- `test_carla_spawn_autopilot.py` (multi-process test helper, superseded by the single-process
  integration)
