#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Adapted from CARLA's official Vissim-CARLA co-simulation bridge
# (`Co-Simulation/PTV-Vissim/util/pedestrian_sync_stub_test.py`) for
# `autoware_carla_interface`'s vendored `vissim_integration` package. See
# ../src/autoware_carla_interface/vissim_integration/NOTICE.md and
# ../docs/Vissim_CARLA_Autoware_歩行者同期_実装計画_v1.0.md (Step P5) for the deviations made
# when adapting this file:
#   - imports use `autoware_carla_interface.vissim_integration.*` instead of the upstream
#     `vissim_integration.*` / `run_synchronization` (this repo extracted the
#     `SimulationSynchronization` class into `vissim_integration/simulation_synchronization.py`,
#     see plan doc Step 1).
#   - `FakeCarlaSimulation` gained an `update_actor_diff()` no-op method (in addition to `tick()`),
#     since this repo's `SimulationSynchronization.tick()` calls
#     `self.carla.tick()` then `self.carla.update_actor_diff()` separately (see plan doc Step 1 /
#     Step 4), instead of upstream's single combined `CarlaSimulation.tick()`.
#   - `data/ptypes.json` is loaded relative to the vendored package directory instead of upstream's
#     `Co-Simulation/PTV-Vissim/data/`.
"""
Stub/mock verification for vissim -> carla pedestrian synchronization, without needing a real
PTV-Vissim Kernel connection or a real carla server.

Exercises the actual production code (`SimulationSynchronization` from
`autoware_carla_interface.vissim_integration.simulation_synchronization`, and `BridgeHelper`'s
pedestrian blueprint/transform/velocity conversion) against fake vissim/carla objects that only
implement the subset of the interface those code paths touch. This is safe to run anywhere: it
never opens a real DrivingSimulatorProxy connection and never calls
carla.Client()/world.apply_settings() on a real server.

Also loads the real vendored data/ptypes.json (not a fake copy), so this doubles as a regression
check for that file's contents.

Run with:
    python3 test/vissim_pedestrian_sync_stub_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import fnmatch
import json
import os
import sys
import types

import carla  # pylint: disable=import-error

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from autoware_carla_interface.vissim_integration.bridge_helper import BridgeHelper  # noqa: E402
from autoware_carla_interface.vissim_integration.vissim_simulation import (  # noqa: E402
    VissimPedestrian, VissimPedestrianMotionState)
from autoware_carla_interface.vissim_integration.simulation_synchronization import (  # noqa: E402
    SimulationSynchronization)

# ==================================================================================================
# -- fakes -------------------------------------------------------------------------------------------
# ==================================================================================================


class FakeVissimSimulation(object):
    """
    Minimal stand-in for PTVVissimSimulation, covering only what SimulationSynchronization.tick()
    touches: no simulator/traffic vehicles ever, and pedestrians settable by the test via
    spawned_pedestrians/destroyed_pedestrians/get_pedestrian().
    """
    def __init__(self):
        self.spawned_vehicles = set()
        self.destroyed_vehicles = set()
        self.spawned_pedestrians = set()
        self.destroyed_pedestrians = set()
        self._pedestrians = {}  # {pedestrian_id: VissimPedestrian}
        self.tick_count = 0

    def tick(self):
        self.tick_count += 1

    def get_pedestrian(self, pedestrian_id):
        return self._pedestrians[pedestrian_id]

    def get_actor(self, actor_id):
        raise AssertionError('unexpected call: no vehicles are used in this stub test')

    def get_signal_state(self, signal_id):
        raise AssertionError('unexpected call: signal sync is disabled in this stub test')

    def close(self):
        pass


class FakeWalkerBlueprint(object):
    """
    Minimal stand-in for a carla.ActorBlueprint of a walker, covering only what
    BridgeHelper.get_carla_pedestrian_blueprint() touches.
    """
    def __init__(self, blueprint_id):
        self.id = blueprint_id
        self._attributes = {'role_name': 'walker'}

    def has_attribute(self, name):
        return name in self._attributes

    def set_attribute(self, name, value):
        self._attributes[name] = value


class FakeBlueprintLibrary(object):
    """
    Minimal stand-in for carla.BlueprintLibrary: only .filter(pattern), matching blueprint ids
    with shell-style wildcards the same way the real carla API does.
    """
    def __init__(self, blueprints):
        self._blueprints = list(blueprints)

    def filter(self, pattern):
        return [bp for bp in self._blueprints if fnmatch.fnmatch(bp.id, pattern)]


class FakeWorld(object):
    """
    Minimal stand-in for carla.World: settings are stored locally and never touch a real server.
    """
    def __init__(self, blueprint_library):
        self._settings = carla.WorldSettings()
        self.frame = 0
        self._blueprint_library = blueprint_library

    def get_blueprint_library(self):
        return self._blueprint_library

    def get_settings(self):
        return self._settings

    def get_snapshot(self):
        return types.SimpleNamespace(frame=self.frame)

    def apply_settings(self, settings):
        self._settings = settings

    def set_weather(self, weather):
        pass


class FakeCarlaSimulation(object):
    """
    Minimal stand-in for CarlaSimulation, covering only what SimulationSynchronization touches for
    pedestrian sync: no vehicles/traffic lights ever, and real bookkeeping of spawned walkers (by
    a locally assigned actor id) so spawn/update/destroy can be asserted on directly, instead of
    relying on a real carla server's actor list.
    """
    def __init__(self, walker_blueprint_ids):
        self.world = FakeWorld(
            FakeBlueprintLibrary([FakeWalkerBlueprint(bp_id) for bp_id in walker_blueprint_ids]))
        self.spawned_actors = set()
        self.destroyed_actors = set()
        self._next_actor_id = 1
        self.walkers = {}  # {actor_id: {'blueprint_id': str, 'transform':..., 'velocity':...}}

    def get_actor(self, actor_id):
        raise AssertionError('unexpected call: no vehicles are used in this stub test')

    def spawn_actor(self, blueprint, transform):
        actor_id = self._next_actor_id
        self._next_actor_id += 1
        self.walkers[actor_id] = {
            'blueprint_id': blueprint.id,
            'transform': transform,
            'velocity': None,
        }
        return actor_id

    def destroy_actor(self, actor_id):
        if actor_id in self.walkers:
            del self.walkers[actor_id]
            return True
        return False

    def synchronize_pedestrian(self, walker_id, transform, velocity=None):
        if walker_id not in self.walkers:
            return False
        self.walkers[walker_id]['transform'] = transform
        self.walkers[walker_id]['velocity'] = velocity
        return True

    def tick(self):
        self.world.frame += 1

    def update_actor_diff(self):
        # No vehicles are ever tracked in this stub test (spawned_actors/destroyed_actors stay
        # empty), so there is nothing to recompute here. Present only because this repo's
        # `SimulationSynchronization.sync_carla_to_vissim()` calls it unconditionally - see plan
        # doc Step P5 deviation notes above.
        pass


# ==================================================================================================
# -- test ----------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    dir_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src',
        'autoware_carla_interface', 'vissim_integration')
    with open(os.path.join(dir_path, 'data', 'ptypes.json')) as f:
        ptypes = json.load(f)

    # pedestrianType "100" (Man) must have at least one candidate blueprint - used below for the
    # main spawn/update/destroy checks. "300" (Wheelchair User) is expected to have zero
    # candidates - used below to check the "unsupported type" path.
    assert ptypes.get('100'), 'data/ptypes.json must map pedestrianType "100" to at least one id'
    assert ptypes.get('300') == [], 'data/ptypes.json "300" (Wheelchair User) should stay empty'

    all_blueprint_ids = {bp_id for candidates in ptypes.values() for bp_id in candidates}

    fake_vissim = FakeVissimSimulation()
    fake_carla = FakeCarlaSimulation(all_blueprint_ids)

    args = types.SimpleNamespace(step_length=0.05, sync_traffic_lights=False)
    sync = SimulationSynchronization(fake_vissim, fake_carla, args)

    assert BridgeHelper.ptypes == ptypes, 'BridgeHelper.ptypes should be loaded from data/ptypes.json'
    assert sync.vissim2carla_ped_ids == {}, 'no pedestrians should be mapped yet'

    # Tick 1: one new supported pedestrian (type 100) and one new unsupported pedestrian
    # (type 300, no candidate blueprints) both appear in vissim at the same time.
    ped1 = VissimPedestrian(1, 100, b'man.v3d', [0.5, 0.5, 1.8], [100.0, 20.0, 0.0],
                            [0.0, 0.0, 0.0], 1.0, VissimPedestrianMotionState.WALKING_ON_LEVEL)
    ped2 = VissimPedestrian(2, 300, b'wheelchair.v3d', [0.5, 0.5, 1.4], [0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0], 0.5, VissimPedestrianMotionState.WALKING_ON_LEVEL)
    fake_vissim._pedestrians = {1: ped1, 2: ped2}
    fake_vissim.spawned_pedestrians = {1, 2}
    sync.tick()
    fake_vissim.spawned_pedestrians = set()

    assert 1 in sync.vissim2carla_ped_ids, 'supported pedestrian (type 100) should be spawned'
    assert 2 not in sync.vissim2carla_ped_ids, \
        'unsupported pedestrian (type 300, no candidate blueprints) should not be spawned'
    assert len(fake_carla.walkers) == 1, 'exactly one walker should have been spawned'

    walker_id = sync.vissim2carla_ped_ids[1]
    walker = fake_carla.walkers[walker_id]
    assert walker['blueprint_id'] in ptypes['100'], \
        'spawned walker blueprint should be one of ptypes.json["100"]'
    assert walker['transform'].location.x == 100.0
    assert walker['transform'].location.y == -20.0, 'vissim -> carla must flip the y coordinate'
    assert abs(walker['transform'].location.z - 0.9) < 1e-4, \
        'carla transform must be raised by half of vissim Height (1.8 / 2 = 0.9) to compensate ' \
        'for the walker actor origin being at its vertical center, not its feet'

    # Tick 2: move the pedestrian, verify the update is reflected via synchronize_pedestrian().
    ped1_moved = VissimPedestrian(1, 100, b'man.v3d', [0.5, 0.5, 1.8], [105.0, 25.0, 0.0],
                                  [0.0, 0.0, 0.0], 1.0, VissimPedestrianMotionState.WALKING_ON_LEVEL)
    fake_vissim._pedestrians[1] = ped1_moved
    sync.tick()

    walker = fake_carla.walkers[walker_id]
    assert walker['transform'].location.x == 105.0
    assert walker['transform'].location.y == -25.0, 'position update should also flip y'
    assert walker['velocity'] is not None, 'velocity should be passed to synchronize_pedestrian()'

    # Tick 3: pedestrian leaves vissim -> the mirrored walker must actually be destroyed (not just
    # forgotten), unlike world.get_actor(id) against a real server which can keep returning a
    # stale Actor object right after destroy().
    fake_vissim.destroyed_pedestrians = {1}
    del fake_vissim._pedestrians[1]
    sync.tick()
    fake_vissim.destroyed_pedestrians = set()

    assert 1 not in sync.vissim2carla_ped_ids, 'destroyed pedestrian must be unmapped'
    assert walker_id not in fake_carla.walkers, 'the mirrored walker must actually be destroyed'

    sync.close()
    print('All pedestrian synchronization stub checks passed.')


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.parse_args()

    run()
