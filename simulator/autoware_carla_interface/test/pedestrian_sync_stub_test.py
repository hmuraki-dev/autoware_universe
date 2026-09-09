#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Adapted from the upstream fork's `util/pedestrian_sync_stub_test.py`
# (`/home/divp/CARLA/Co-Simulation/Sumo/util/pedestrian_sync_stub_test.py`) for
# this repository's vendored `sumo_integration` package. See
# docs/SUMO_CARLA_Autoware_歩行者同期_実装計画_v1.0.md (task 8).
"""
Stub/mock verification for sumo -> carla pedestrian synchronization, without needing a real
SUMO/traci connection or a real carla server.

Exercises the actual production code (`SimulationSynchronization` from
`autoware_carla_interface.sumo_integration.simulation_synchronization`, and `BridgeHelper`'s
pedestrian blueprint/transform conversion) against fake sumo/carla objects that only implement the
subset of the interface those code paths touch. This is safe to run anywhere: it never calls
`traci.init()`/`traci.start()` and never calls `carla.Client()`/`world.apply_settings()` on a real
server.

Also loads the real `data/vtypes.json` (not a fake copy), so this doubles as a regression check
for the `walker.pedestrian.*` / `vClass:"pedestrian"` entries added in task 4.

Not wired into colcon/ament (no pytest infra exists for this package yet); run manually with:
    python3 test/pedestrian_sync_stub_test.py

Exits with a non-zero status and an AssertionError if any check fails.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import fnmatch
import json
import os
import sys
import types

import carla  # pylint: disable=import-error

_PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
sys.path.insert(0, _PACKAGE_SRC)

from autoware_carla_interface.sumo_integration.simulation_synchronization import (  # noqa: E402  pylint: disable=wrong-import-position
    SimulationSynchronization,
)
from autoware_carla_interface.sumo_integration.sumo_simulation import (  # noqa: E402  pylint: disable=wrong-import-position
    SumoActor,
    SumoActorClass,
)

# ==================================================================================================
# -- fakes -------------------------------------------------------------------------------------------
# ==================================================================================================


class FakeSumoSimulation(object):
    """
    Minimal stand-in for SumoSimulation, covering only what SimulationSynchronization's
    sync_sumo_to_carla() touches: no sumo-controlled vehicles ever, and pedestrians settable by
    the test via spawned_persons/destroyed_persons/get_person().
    """
    def __init__(self):
        self.traffic_light_ids = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()
        self.spawned_persons = set()
        self.destroyed_persons = set()
        self._persons = {}  # {person_id: SumoActor}
        self.subscribed_person_ids = set()

    def get_net_offset(self):
        return (0, 0)

    def subscribe_person(self, person_id):
        self.subscribed_person_ids.add(person_id)

    def unsubscribe_person(self, person_id):
        self.subscribed_person_ids.discard(person_id)

    def get_person(self, person_id):
        return self._persons[person_id]

    def get_actor(self, actor_id):
        raise AssertionError('unexpected call: no vehicles are used in this stub test')

    def destroy_actor(self, actor_id):
        raise AssertionError('unexpected call: no vehicles are used in this stub test')

    def get_traffic_light_state(self, landmark_id):
        raise AssertionError('unexpected call: no traffic lights are used in this stub test')

    def tick(self):
        pass

    def close(self):
        pass


class FakeWalkerBlueprint(object):
    """
    Minimal stand-in for a carla.ActorBlueprint of a walker, covering only what
    BridgeHelper.get_carla_blueprint() touches. Real walker.pedestrian.* blueprints have no
    'color'/'driver_id' attributes but do have 'role_name' (confirmed live against a real CARLA
    server, see the upstream fork's docs/PEDESTRIAN_TODO.md tasks 4/5).
    """
    def __init__(self, blueprint_id):
        self.id = blueprint_id
        self._attributes = {'role_name': 'pedestrian'}

    def has_attribute(self, name):
        return name in self._attributes

    def get_attribute(self, name):
        return types.SimpleNamespace(recommended_values=[self._attributes[name]])

    def set_attribute(self, name, value):
        self._attributes[name] = value


class FakeBlueprintLibrary(object):
    """
    Minimal stand-in for carla.BlueprintLibrary: iteration and .filter(pattern), matching
    blueprint ids the same way the real carla API does.
    """
    def __init__(self, blueprints):
        self._blueprints = list(blueprints)

    def __iter__(self):
        return iter(self._blueprints)

    def filter(self, pattern):
        return [bp for bp in self._blueprints if fnmatch.fnmatch(bp.id, pattern)]


class FakeWorld(object):
    """
    Minimal stand-in for carla.World: settings are stored locally and never touch a real server.
    """
    def __init__(self, blueprint_library):
        self._settings = carla.WorldSettings()
        self._blueprint_library = blueprint_library

    def get_blueprint_library(self):
        return self._blueprint_library

    def get_settings(self):
        return self._settings

    def apply_settings(self, settings):
        self._settings = settings


class FakeCarlaSimulation(object):
    """
    Minimal stand-in for CarlaSimulation, covering only what SimulationSynchronization touches for
    pedestrian sync: no vehicles/traffic lights ever, and real bookkeeping of spawned walkers (by
    a locally assigned actor id) so spawn/update/destroy can be asserted on directly, instead of
    relying on a real carla server's actor list.

    Unlike the upstream fake, `client`/`traffic_manager` stand-ins are omitted: this repository's
    `SimulationSynchronization.__init__` no longer configures CARLA's synchronous mode itself
    (that duplicate configuration was removed - see simulation_synchronization.py), so nothing
    ever calls `self.carla.client.get_trafficmanager()`.
    """
    def __init__(self, walker_blueprint_ids):
        self.world = FakeWorld(
            FakeBlueprintLibrary([FakeWalkerBlueprint(bp_id) for bp_id in walker_blueprint_ids]))
        self.client = None
        self.step_length = 0.05
        self.spawned_actors = set()
        self.destroyed_actors = set()
        self.traffic_light_ids = set()
        self._next_actor_id = 1
        self.walkers = {}  # {actor_id: {'blueprint_id': str, 'transform': carla.Transform}}

    def get_actor(self, actor_id):
        raise AssertionError('unexpected call: no vehicles are used in this stub test')

    def spawn_actor(self, blueprint, transform):
        actor_id = self._next_actor_id
        self._next_actor_id += 1
        self.walkers[actor_id] = {'blueprint_id': blueprint.id, 'transform': transform}
        return actor_id

    def destroy_actor(self, actor_id):
        if actor_id in self.walkers:
            del self.walkers[actor_id]
            return True
        return False

    def synchronize_pedestrian(self, walker_id, transform):
        if walker_id not in self.walkers:
            return False
        self.walkers[walker_id]['transform'] = transform
        return True

    def update_actor_diff(self):
        # No vehicles are used in this stub test, so there is never anything to diff.
        pass

    def tick(self):
        self.update_actor_diff()

    def close(self):
        pass


# ==================================================================================================
# -- test ----------------------------------------------------------------------------------------
# ==================================================================================================


def run():
    vtypes_path = os.path.join(_PACKAGE_SRC, 'autoware_carla_interface', 'sumo_integration',
                               'data', 'vtypes.json')
    with open(vtypes_path) as f:
        vtypes = json.load(f)['carla_blueprints']

    # data/vtypes.json must have walker.pedestrian.* entries flatly tagged vClass:"pedestrian"
    # (task 4) - used below to build this test's fake blueprint library.
    pedestrian_blueprint_ids = sorted(
        bp_id for bp_id, spec in vtypes.items() if spec.get('vClass') == 'pedestrian')
    assert pedestrian_blueprint_ids, \
        'data/vtypes.json must have at least one walker.pedestrian.* entry (see task 4)'
    assert all(bp_id.startswith('walker.pedestrian.') for bp_id in pedestrian_blueprint_ids)

    fake_sumo = FakeSumoSimulation()
    fake_carla = FakeCarlaSimulation(pedestrian_blueprint_ids)

    sync = SimulationSynchronization(fake_sumo, fake_carla)

    assert sync.sumo2carla_ped_ids == {}, 'no pedestrians should be mapped yet'

    # Tick 1: one new supported pedestrian (DEFAULT_PEDTYPE, vClass "pedestrian") and one new
    # unsupported pedestrian (vClass "bicycle", which this fake world's blueprint library has no
    # candidates for, since it only contains pedestrian blueprints) both appear in sumo at the
    # same time.
    ped1 = SumoActor(
        type_id='DEFAULT_PEDTYPE',
        vclass=SumoActorClass.PEDESTRIAN,
        transform=carla.Transform(carla.Location(100.0, 20.0, 0.0), carla.Rotation(0.0, 0.0, 0.0)),
        signals=None,
        extent=carla.Vector3D(0.3, 0.3, 0.9),
        color=(-1, -1, -1),
    )
    ped2 = SumoActor(
        type_id='DEFAULT_BIKETYPE',
        vclass=SumoActorClass.BICYCLE,
        transform=carla.Transform(carla.Location(0.0, 0.0, 0.0), carla.Rotation(0.0, 0.0, 0.0)),
        signals=None,
        extent=carla.Vector3D(0.9, 0.3, 0.9),
        color=(-1, -1, -1),
    )
    fake_sumo._persons = {'p1': ped1, 'p2': ped2}
    fake_sumo.spawned_persons = {'p1', 'p2'}
    sync.tick()
    fake_sumo.spawned_persons = set()

    assert 'p1' in sync.sumo2carla_ped_ids, 'supported pedestrian should be spawned'
    assert 'p2' not in sync.sumo2carla_ped_ids, \
        'unsupported vclass (no candidate blueprints in this fake world) should not be spawned'
    assert len(fake_carla.walkers) == 1, 'exactly one walker should have been spawned'

    walker_id = sync.sumo2carla_ped_ids['p1']
    walker = fake_carla.walkers[walker_id]
    assert walker['blueprint_id'] in pedestrian_blueprint_ids, \
        'spawned walker blueprint should be one of the walker.pedestrian.* ids in vtypes.json'
    assert walker['transform'].location.x == 100.0
    assert walker['transform'].location.y == -20.0, 'sumo -> carla must flip the y coordinate'
    assert abs(walker['transform'].location.z - 0.9) < 1e-4, \
        'carla transform must be raised by half of sumo VAR_HEIGHT (extent.z = 0.9) to ' \
        'compensate for the walker actor origin being at its vertical center, not its feet'

    # Tick 2: move the pedestrian, verify the update is reflected via synchronize_pedestrian().
    ped1_moved = ped1._replace(
        transform=carla.Transform(carla.Location(105.0, 25.0, 0.0), carla.Rotation(0.0, 0.0, 0.0)))
    fake_sumo._persons['p1'] = ped1_moved
    sync.tick()

    walker = fake_carla.walkers[walker_id]
    assert walker['transform'].location.x == 105.0
    assert walker['transform'].location.y == -25.0, 'position update should also flip y'
    assert abs(walker['transform'].location.z - 0.9) < 1e-4

    # Tick 3: pedestrian leaves sumo -> the mirrored walker must actually be destroyed (not just
    # forgotten).
    fake_sumo.destroyed_persons = {'p1'}
    del fake_sumo._persons['p1']
    sync.tick()
    fake_sumo.destroyed_persons = set()

    assert 'p1' not in sync.sumo2carla_ped_ids, 'destroyed pedestrian must be unmapped'
    assert walker_id not in fake_carla.walkers, 'the mirrored walker must actually be destroyed'

    sync.close()
    print('All pedestrian synchronization stub checks passed.')


if __name__ == '__main__':
    run()
