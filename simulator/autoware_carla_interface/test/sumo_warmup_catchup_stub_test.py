#!/usr/bin/env python3
"""
Step S3 stub checks for the SUMO warmup / EGO safe spawn plan (see
docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md, sections 2.3/2.4 and Step S3).

1. `SimulationSynchronization.spawn_all_sumo_actors_in_carla()` (the catch-up after the warmup),
   run as the production code against fake SUMO/CARLA objects:
   - every SUMO vehicle/person not mirrored in CARLA yet is spawned there and mapped,
   - vehicles already mirrored, and CARLA-origin vehicles (EGO's shadow in SUMO), are skipped,
   - a vehicle without a matching blueprint is unsubscribed and counted as "not in carla",
   - calling it twice does not spawn anything twice.
2. `EgoSpawnGate.run()`: SUMO alone for exactly sumo_warmup_time / step_length steps with no CARLA
   tick, then the catch-up, one synchronization step, and the EGO spawn - in that order; and a
   stop request during the warmup returns False without catch-up or EGO spawn.

Never calls traci.start()/traci.init() or carla.Client(). If the `carla` / `lxml` Python modules
are not installed (e.g. on a Windows dev PC), minimal stand-ins are injected so the vendored
modules can be imported; on the Linux machine the real modules are used. `traci`/`sumolib` must
be importable (SUMO_HOME). Not wired into colcon/ament; run manually with:
    python3 test/sumo_warmup_catchup_stub_test.py

Exits with a non-zero status and an AssertionError if any check fails.
"""

import fnmatch
import os
import sys
import types

_PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
sys.path.insert(0, _PACKAGE_SRC)
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))


def _install_stand_ins():
    """Minimal carla / lxml stand-ins, only if the real modules are missing."""
    try:
        import carla  # noqa: F401  pylint: disable=import-outside-toplevel,unused-import
    except ImportError:
        carla = types.ModuleType('carla')

        class _Vec(object):
            def __init__(self, x=0.0, y=0.0, z=0.0):
                self.x, self.y, self.z = x, y, z

        class Rotation(object):
            def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
                self.pitch, self.yaw, self.roll = pitch, yaw, roll

        class Transform(object):
            def __init__(self, location=None, rotation=None):
                self.location = location if location is not None else _Vec()
                self.rotation = rotation if rotation is not None else Rotation()

        carla.Location = carla.Vector3D = _Vec
        carla.Rotation = Rotation
        carla.Transform = Transform
        carla.VehicleLightState = types.SimpleNamespace(NONE=0)
        carla.TrafficLightState = types.SimpleNamespace(Red=0, Yellow=1, Green=2, Off=3,
                                                        Unknown=4)
        sys.modules['carla'] = carla
    try:
        import lxml.etree  # noqa: F401  pylint: disable=import-outside-toplevel,unused-import
    except ImportError:
        # sumo_simulation.py only needs the module to exist at import time; aliasing the standard
        # library's ElementTree keeps sumolib (which prefers lxml when importable) working.
        import xml.etree.ElementTree  # pylint: disable=import-outside-toplevel
        lxml = types.ModuleType('lxml')
        lxml.etree = xml.etree.ElementTree
        sys.modules['lxml'] = lxml
        sys.modules['lxml.etree'] = lxml.etree


_install_stand_ins()

import carla  # noqa: E402  pylint: disable=wrong-import-position,import-error

from autoware_carla_interface.sumo_integration.ego_spawn_gate import (  # noqa: E402  pylint: disable=wrong-import-position
    EgoSpawnGate,
)
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


def _sumo_actor(type_id, vclass, x, y):
    return SumoActor(
        type_id=type_id,
        vclass=vclass,
        transform=carla.Transform(carla.Location(x, y, 0.0), carla.Rotation(0.0, 90.0, 0.0)),
        signals=0,
        extent=carla.Vector3D(2.25, 1.0, 0.75),
        color=(255, 0, 0, 255),
    )


class FakeSumoSimulation(object):
    """Stand-in for SumoSimulation: vehicles/persons currently in the simulation are settable."""

    def __init__(self, vehicles, persons):
        self.traffic_light_ids = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()
        self.spawned_persons = set()
        self.destroyed_persons = set()
        self.vehicles = dict(vehicles)  # {vehicle_id: SumoActor}
        self.persons = dict(persons)  # {person_id: SumoActor}
        self.subscribed = set()
        self.subscribed_persons = set()
        self.ticks = 0
        self.calls = []  # shared call log, set by the test

    def get_net_offset(self):
        return (0, 0)

    def get_vehicle_ids(self):
        return tuple(self.vehicles)

    def get_person_ids(self):
        return tuple(self.persons)

    def get_time(self):
        return self.ticks * 0.05

    def subscribe(self, actor_id):
        self.subscribed.add(actor_id)

    def unsubscribe(self, actor_id):
        self.subscribed.discard(actor_id)

    def subscribe_person(self, person_id):
        self.subscribed_persons.add(person_id)

    def unsubscribe_person(self, person_id):
        self.subscribed_persons.discard(person_id)

    def get_actor(self, actor_id):
        assert actor_id in self.subscribed, 'get_actor() before subscribe() for %s' % actor_id
        return self.vehicles[actor_id]

    def get_person(self, person_id):
        assert person_id in self.subscribed_persons, \
            'get_person() before subscribe_person() for %s' % person_id
        return self.persons[person_id]

    def synchronize_vehicle(self, vehicle_id, transform, signals=None):
        return True

    def tick(self):
        self.ticks += 1
        self.calls.append('sumo.tick')


class FakeBlueprint(object):
    def __init__(self, blueprint_id):
        self.id = blueprint_id
        self._attributes = {'role_name': 'npc', 'color': '0,0,0'}

    def has_attribute(self, name):
        return name in self._attributes

    def get_attribute(self, name):
        return types.SimpleNamespace(recommended_values=[self._attributes[name]])

    def set_attribute(self, name, value):
        self._attributes[name] = value


class FakeBlueprintLibrary(object):
    def __init__(self, blueprint_ids):
        self._blueprints = [FakeBlueprint(bp_id) for bp_id in blueprint_ids]

    def __iter__(self):
        return iter(self._blueprints)

    def filter(self, pattern):
        return [bp for bp in self._blueprints if fnmatch.fnmatch(bp.id, pattern)]


class FakeWorld(object):
    def __init__(self, blueprint_ids, calls):
        self._library = FakeBlueprintLibrary(blueprint_ids)
        self.calls = calls

    def get_blueprint_library(self):
        return self._library

    def tick(self):
        self.calls.append('world.tick')


class FakeCarlaSimulation(object):
    """Stand-in for CarlaSimulation: spawned actors are kept locally by a sequential id."""

    def __init__(self, world):
        self.world = world
        self.traffic_light_ids = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()
        self.actors = {}  # {actor_id: blueprint_id}
        self._next_id = 100

    def spawn_actor(self, blueprint, transform):
        actor_id = self._next_id
        self._next_id += 1
        self.actors[actor_id] = blueprint.id
        return actor_id

    def get_actor(self, actor_id):
        return types.SimpleNamespace(get_light_state=lambda: 0)

    def synchronize_vehicle(self, vehicle_id, transform, lights=None):
        return True

    def synchronize_pedestrian(self, walker_id, transform):
        return True

    def update_actor_diff(self):
        self.world.calls.append('carla.update_actor_diff')


def _make_sync(vehicles, persons, calls):
    fake_sumo = FakeSumoSimulation(vehicles, persons)
    fake_sumo.calls = calls
    blueprint_ids = ['vehicle.toyota.prius', 'vehicle.mitsubishi.fusorosa',
                     'walker.pedestrian.0001']
    fake_carla = FakeCarlaSimulation(FakeWorld(blueprint_ids, calls))
    return SimulationSynchronization(fake_sumo, fake_carla), fake_sumo, fake_carla


# ==================================================================================================
# -- tests -------------------------------------------------------------------------------------------
# ==================================================================================================


def test_catch_up():
    vehicles = {
        'in1_A.0': _sumo_actor('vehicle.toyota.prius', SumoActorClass.EVEHICLE, 10.0, 0.0),
        'in1_D.0': _sumo_actor('vehicle.mitsubishi.fusorosa', SumoActorClass.BUS, 30.0, 0.0),
        # no blueprint with this id and no 'rail' vClass candidates in vtypes.json
        'tram.0': _sumo_actor('custom_tram', SumoActorClass.RAIL, 50.0, 0.0),
        'already.0': _sumo_actor('vehicle.toyota.prius', SumoActorClass.EVEHICLE, 70.0, 0.0),
        'carla0': _sumo_actor('vehicle.toyota.prius', SumoActorClass.EVEHICLE, 90.0, 0.0),
    }
    persons = {'ped.0': _sumo_actor('DEFAULT_PEDTYPE', SumoActorClass.PEDESTRIAN, 5.0, 5.0)}
    sync, fake_sumo, fake_carla = _make_sync(vehicles, persons, [])
    sync.sumo2carla_ids['already.0'] = 1  # mirrored before (e.g. departed in a synced step)
    sync.carla2sumo_ids[7] = 'carla0'  # CARLA-origin vehicle (EGO's shadow in SUMO)

    counts = sync.spawn_all_sumo_actors_in_carla()

    assert counts == {'vehicles': 2, 'vehicles_not_in_carla': 1,
                      'pedestrians': 1, 'pedestrians_not_in_carla': 0}, counts
    assert set(sync.sumo2carla_ids) == {'already.0', 'in1_A.0', 'in1_D.0'}
    assert sync.sumo2carla_ids['already.0'] == 1, 'an existing mapping must not be replaced'
    assert fake_carla.actors[sync.sumo2carla_ids['in1_A.0']] == 'vehicle.toyota.prius'
    assert fake_carla.actors[sync.sumo2carla_ids['in1_D.0']] == 'vehicle.mitsubishi.fusorosa', \
        'a type id that is a CARLA blueprint id is used as is, even if not in vtypes.json'
    assert 'carla0' not in sync.sumo2carla_ids, 'CARLA-origin vehicles must not be mirrored back'
    assert 'tram.0' not in sync.sumo2carla_ids and 'tram.0' not in fake_sumo.subscribed, \
        'a vehicle without a blueprint must be unsubscribed (as in sync_sumo_to_carla())'
    assert set(sync.sumo2carla_ped_ids) == {'ped.0'}
    assert fake_carla.actors[sync.sumo2carla_ped_ids['ped.0']].startswith('walker.pedestrian.')

    spawned = len(fake_carla.actors)
    counts = sync.spawn_all_sumo_actors_in_carla()
    assert len(fake_carla.actors) == spawned, 'a second catch-up must not spawn anything again'
    assert counts['vehicles'] == 0 and counts['pedestrians'] == 0, counts
    assert counts['vehicles_not_in_carla'] == 1, 'the vehicle without a blueprint is retried'


def test_gate_order():
    calls = []
    vehicles = {'in1_A.0': _sumo_actor('vehicle.toyota.prius', SumoActorClass.EVEHICLE, 10.0, 0.0)}
    sync, fake_sumo, fake_carla = _make_sync(vehicles, {}, calls)
    logs = []
    gate = EgoSpawnGate(sync, fake_carla.world, warmup_time=100, step_length=0.05,
                        spawn_ego=lambda: calls.append('spawn_ego'),
                        stop_requested=lambda: False, log_info=logs.append)

    assert gate.run() is True
    warmup_steps = 2000
    assert calls[:warmup_steps] == ['sumo.tick'] * warmup_steps, \
        'the warmup must tick SUMO alone, without any CARLA tick'
    assert calls[warmup_steps:] == ['sumo.tick', 'world.tick', 'carla.update_actor_diff',
                                    'spawn_ego'], \
        'after the warmup: one synchronization step (sumo->carla, CARLA tick, carla->sumo), ' \
        'then EGO spawn; got %r' % calls[warmup_steps:]
    assert 'in1_A.0' in sync.sumo2carla_ids, 'the catch-up must run before the EGO spawn'
    progress = [line for line in logs if line.startswith('[SUMO WARMUP] t=')]
    assert len(progress) == 9, 'one progress line every 10 s, except at the end: %r' % progress
    assert any(line.startswith('[SUMO WARMUP] completed: t=100.0 s') for line in logs), logs
    assert any(line.startswith('[SUMO WARMUP] caught up: carla_spawned=1 ') for line in logs), logs
    assert any(line.startswith('[EGO SPAWN] ') for line in logs), logs


def test_gate_stop_during_warmup():
    calls = []
    sync, fake_sumo, fake_carla = _make_sync({}, {}, calls)
    gate = EgoSpawnGate(sync, fake_carla.world, warmup_time=100, step_length=0.05,
                        spawn_ego=lambda: calls.append('spawn_ego'),
                        stop_requested=lambda: fake_sumo.ticks >= 30, log_info=lambda line: None)

    assert gate.run() is False
    assert calls == ['sumo.tick'] * 30, \
        'a stop request must end the warmup without catch-up, CARLA tick or EGO spawn'


def main():
    tests = [test_catch_up, test_gate_order, test_gate_stop_during_warmup]
    for test in tests:
        test()
        print('PASS %s' % test.__name__)
    print('All %d checks passed.' % len(tests))


if __name__ == '__main__':
    main()
