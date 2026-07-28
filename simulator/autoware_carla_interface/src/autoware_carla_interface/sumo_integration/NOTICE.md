# Vendored `sumo_integration` module (Step 1)

Source: CARLA's official SUMO co-simulation bridge
(`Co-Simulation/Sumo/sumo_integration/` in the CARLA repository), MIT License,
Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
Barcelona (UAB). See <https://opensource.org/licenses/MIT>.

Vendored locally from `/home/divp/CARLA/Co-Simulation/Sumo/` per
`docs/SUMO_CARLA_Autoware_統合修正項目_v0.5.md` (2.4/2.5/2.6) and
`docs/SUMO_CARLA_Autoware_統合_実装ステップ計画_v1.1.md` (Step 0 / Step 1).

## Files vendored as-is (no logic changes)

- `constants.py`
- `sumo_simulation.py`
- `bridge_helper.py` (only the `vtypes.json` path lookup was adjusted, since
  the data file was moved from `../data/vtypes.json` to `./data/vtypes.json`
  relative to this package)
- `data/vtypes.json`

## Files vendored with modifications

- `carla_simulation.py`: `CarlaSimulation.__init__` no longer creates its own
  `carla.Client`/`carla.World`. It now accepts an already-connected `client`
  and `world`, injected by `autoware_carla_interface` (v0.5 section 2.5).
- `simulation_synchronization.py`: New module. Contains the
  `SimulationSynchronization` class extracted from the original
  `run_synchronization.py` (the standalone CLI entry point and
  `synchronization_loop()` were intentionally **not** vendored, per the Step 0
  decision). The duplicate CARLA sync-mode configuration
  (`world.apply_settings()` / `traffic_manager.set_synchronous_mode()`) that
  used to run inside `__init__` has been removed, since
  `autoware_carla_interface`'s `InitializeInterface.load_world()` already
  manages this centrally (v0.5 section 2.6).

## Intentionally not vendored (v0.5 section 0.5 item 4 / Step 0 decision)

- `run_synchronization.py`'s CLI entry point / argument parsing
- The standalone `synchronization_loop()` while-loop and its real-time pacing
  (duplicates `autoware_carla_interface`'s `max_real_delta_seconds` pacing)
- Any code that creates its own `carla.Client`
- Any code that sets CARLA synchronous mode / `fixed_delta_seconds`
- Any code that calls `world.tick()` directly (CARLA Tick will be unified in
  a later step; see v0.5 section 3.2)

## Known limitation carried over from Step 1

`SimulationSynchronization.tick()` still internally calls
`self.carla.tick()` (which calls `world.tick()`) and
`SimulationSynchronization.close()` still resets CARLA world settings. Both
will be revisited in later steps (CARLA Tick unification and shutdown
integration) — they are out of scope for Step 1, which only covers the
constructor injection and removal of duplicate sync-mode setup.
