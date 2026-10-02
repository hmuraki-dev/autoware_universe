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
3. (Step S6) WAIT_FOR_SAFE_GAP: one synchronization step per unsafe check until the check is safe,
   EgoSpawnGateError after ego_spawn_wait_timeout or when EGO cannot be spawned, a stop request
   during the wait, and the [EGO SPAWN CHECK] log throttling.
4. (Step S6) `collect_gap_actors()` / `ego_footprint()`: which positions and sizes the gap check
   is fed with (CARLA actor vs. SUMO-only vehicle vs. pedestrian; EGO footprint fallbacks).

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


# carla / lxml stand-ins for machines without them (shared with the other stub tests).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stand_ins import install_stand_ins  # noqa: E402  pylint: disable=wrong-import-position

install_stand_ins()

import carla  # noqa: E402  pylint: disable=wrong-import-position,import-error

from autoware_carla_interface.sumo_integration.ego_spawn_gate import (  # noqa: E402  pylint: disable=wrong-import-position
    EGO_FOOTPRINT_DEFAULT,
    EgoSpawnGate,
    EgoSpawnGateError,
    GapActor,
    SpawnSpot,
    collect_gap_actors,
    ego_footprint,
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

    def get_vehicle_footprint(self, actor_id):
        a = self.vehicles[actor_id]
        return (a.transform.location.x, a.transform.location.y, a.transform.rotation.yaw,
                2.0 * a.extent.x, 2.0 * a.extent.y)

    def get_person_footprint(self, person_id):
        a = self.persons[person_id]
        return (a.transform.location.x, a.transform.location.y, a.transform.rotation.yaw,
                2.0 * a.extent.x, 2.0 * a.extent.y)

    def get_vtype_size(self, type_id):
        return {'vehicle.toyota.prius': (4.54, 2.0)}.get(type_id)

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
        self.poses = {}  # {actor_id: carla.Transform}
        self._next_id = 100

    def spawn_actor(self, blueprint, transform):
        actor_id = self._next_id
        self._next_id += 1
        self.actors[actor_id] = blueprint.id
        self.poses[actor_id] = transform
        return actor_id

    def get_actor(self, actor_id):
        if actor_id not in self.actors:
            return None  # like carla.World.get_actor() for an unknown id
        return types.SimpleNamespace(
            get_light_state=lambda: 0,
            get_transform=lambda: self.poses[actor_id],
            bounding_box=types.SimpleNamespace(extent=carla.Vector3D(2.5, 1.1, 0.8)))

    def synchronize_vehicle(self, vehicle_id, transform, lights=None):
        self.poses[vehicle_id] = transform
        return True

    def synchronize_pedestrian(self, walker_id, transform):
        self.poses[walker_id] = transform
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


_SPOT = SpawnSpot(0.0, 0.0, 0.0, 4.54, 2.0, 3.5)
_BLOCKER = GapActor('sumo:blocker', 3.0, 0.0, 0.0, 4.0, 2.0, 'carla')  # overlaps the spot


def _gate(sync, world, calls, logs, collect=lambda: [], stop=lambda: False, spawn=None,
          wait_timeout=60):
    return EgoSpawnGate(sync, world, warmup_time=100, step_length=0.05, spot=_SPOT,
                        front_margin=20.0, rear_margin=20.0, wait_timeout=wait_timeout,
                        collect_actors=collect,
                        spawn_ego=spawn or (lambda: calls.append('spawn_ego')),
                        stop_requested=stop, log_info=logs.append)


_SYNC_STEP = ['sumo.tick', 'world.tick', 'carla.update_actor_diff']
_WARMUP_STEPS = 2000


def test_gate_order():
    calls = []
    vehicles = {'in1_A.0': _sumo_actor('vehicle.toyota.prius', SumoActorClass.EVEHICLE, 10.0, 0.0)}
    sync, fake_sumo, fake_carla = _make_sync(vehicles, {}, calls)
    logs = []
    gate = _gate(sync, fake_carla.world, calls, logs)

    assert gate.run() is True
    assert calls[:_WARMUP_STEPS] == ['sumo.tick'] * _WARMUP_STEPS, \
        'the warmup must tick SUMO alone, without any CARLA tick'
    assert calls[_WARMUP_STEPS:] == _SYNC_STEP + ['spawn_ego'], \
        'after the warmup: one synchronization step (sumo->carla, CARLA tick, carla->sumo), ' \
        'then (gap already safe) EGO spawn; got %r' % calls[_WARMUP_STEPS:]
    assert 'in1_A.0' in sync.sumo2carla_ids, 'the catch-up must run before the EGO spawn'
    progress = [line for line in logs if line.startswith('[SUMO WARMUP] t=')]
    assert len(progress) == 9, 'one progress line every 10 s, except at the end: %r' % progress
    assert any(line.startswith('[SUMO WARMUP] completed: t=100.0 s') for line in logs), logs
    assert any(line.startswith('[SUMO WARMUP] caught up: carla_spawned=1 ') for line in logs), logs
    assert any(line.startswith('[EGO SPAWN CHECK] t=100.05 front=- rear=- overlap=none '
                               'result=SAFE') for line in logs), logs
    assert any(line.startswith('[EGO SPAWN] t=100.05 s waited=0.00 s') for line in logs), logs


def test_gate_stop_during_warmup():
    calls = []
    sync, fake_sumo, fake_carla = _make_sync({}, {}, calls)
    gate = _gate(sync, fake_carla.world, calls, [], stop=lambda: fake_sumo.ticks >= 30)

    assert gate.run() is False
    assert calls == ['sumo.tick'] * 30, \
        'a stop request must end the warmup without catch-up, CARLA tick or EGO spawn'


def test_gate_waits_for_gap():
    calls = []
    sync, fake_sumo, fake_carla = _make_sync({}, {}, calls)
    checks = []

    def collect():
        checks.append(fake_sumo.ticks)
        return [_BLOCKER] if len(checks) <= 3 else []

    logs = []
    assert _gate(sync, fake_carla.world, calls, logs, collect=collect).run() is True
    assert calls[_WARMUP_STEPS:] == _SYNC_STEP * 4 + ['spawn_ego'], \
        'three unsafe checks -> three more synchronization steps, then EGO spawn: %r' % (
            calls[_WARMUP_STEPS:],)
    assert checks == [2001, 2002, 2003, 2004], 'one check after every synchronization step'
    assert any(line.startswith('[EGO SPAWN] t=100.20 s waited=0.15 s') for line in logs), logs
    waits = [line for line in logs if 'result=WAIT' in line]
    assert len(waits) == 1 and 'overlap=sumo:blocker' in waits[0], \
        'an unchanged WAIT is logged once per second only: %r' % waits


def test_gate_timeout():
    calls = []
    sync, fake_sumo, fake_carla = _make_sync({}, {}, calls)
    try:
        _gate(sync, fake_carla.world, calls, [], collect=lambda: [_BLOCKER], wait_timeout=1).run()
    except EgoSpawnGateError as e:
        assert 'no safe gap found within ego_spawn_wait_timeout=1 s' in str(e), e
        assert 'overlap=sumo:blocker result=WAIT' in str(e), e
    else:
        raise AssertionError('expected EgoSpawnGateError after the wait timeout')
    assert 'spawn_ego' not in calls, 'EGO must not be spawned without a safe gap'
    assert calls[_WARMUP_STEPS:] == _SYNC_STEP * 21, \
        '1 s = 20 waiting steps (21 checks) after the first synchronization step'


def test_gate_stop_during_wait():
    calls = []
    sync, fake_sumo, fake_carla = _make_sync({}, {}, calls)
    gate = _gate(sync, fake_carla.world, calls, [], collect=lambda: [_BLOCKER],
                 stop=lambda: fake_sumo.ticks >= _WARMUP_STEPS + 5)
    assert gate.run() is False
    assert 'spawn_ego' not in calls and calls[_WARMUP_STEPS:] == _SYNC_STEP * 5


def test_gate_spawn_failure():
    def spawn():
        raise RuntimeError('failed to spawn EGO (vehicle.toyota.prius) at spawn_point=...')

    sync, fake_sumo, fake_carla = _make_sync({}, {}, [])
    try:
        _gate(sync, fake_carla.world, [], [], spawn=spawn).run()
    except EgoSpawnGateError as e:
        assert 'EGO could not be spawned after a safe gap was found' in str(e), e
    else:
        raise AssertionError('expected EgoSpawnGateError when EGO cannot be spawned')


def test_check_log_throttle():
    sync, fake_sumo, fake_carla = _make_sync({}, {}, [])
    checks = []

    def collect():
        checks.append(None)
        return [_BLOCKER] if len(checks) <= 40 else []

    logs = []
    _gate(sync, fake_carla.world, [], logs, collect=collect).run()
    lines = [line for line in logs if line.startswith('[EGO SPAWN CHECK] t=')]
    assert [line.split()[-1] for line in lines] == ['result=WAIT', 'result=WAIT', 'result=SAFE'], \
        '2 s of the same WAIT: logged at 0 s and 1 s, then the SAFE: %r' % lines


def test_collect_gap_actors():
    vehicles = {
        'mirrored': _sumo_actor('vehicle.toyota.prius', SumoActorClass.EVEHICLE, 10.0, 5.0),
        'tram.0': _sumo_actor('custom_tram', SumoActorClass.RAIL, 50.0, 5.0),
        'carla0': _sumo_actor('vehicle.toyota.prius', SumoActorClass.EVEHICLE, 90.0, 5.0),
    }
    persons = {'ped.0': _sumo_actor('DEFAULT_PEDTYPE', SumoActorClass.PEDESTRIAN, 5.0, 5.0)}
    sync, fake_sumo, fake_carla = _make_sync(vehicles, persons, [])
    sync.carla2sumo_ids[7] = 'carla0'
    sync.spawn_all_sumo_actors_in_carla()
    mirrored_id = sync.sumo2carla_ids['mirrored']
    # the CARLA actor has moved away from where SUMO put it: CARLA is the reference (2.6.2)
    fake_carla.poses[mirrored_id] = carla.Transform(carla.Location(11.0, -6.0, 0.0),
                                                    carla.Rotation(0.0, 3.0, 0.0))

    actors = {a.actor_id: a for a in collect_gap_actors(sync)}
    walker_id = sync.sumo2carla_ped_ids['ped.0']
    assert set(actors) == {'sumo:mirrored/carla:%d' % mirrored_id, 'sumo:tram.0',
                           'sumo:ped.0/carla:%d' % walker_id}, actors
    mirrored = actors['sumo:mirrored/carla:%d' % mirrored_id]
    assert (mirrored.x, mirrored.y, mirrored.yaw) == (11.0, -6.0, 3.0)
    assert (mirrored.length, mirrored.width, mirrored.source) == (5.0, 2.2, 'carla'), \
        'size from the CARLA bounding box'
    tram = actors['sumo:tram.0']
    # SUMO front bumper (50, 5) heading east (SUMO angle 90), 4.5 m long -> CARLA center
    # (50 - 2.25, -5) heading CARLA yaw 0, SUMO size
    assert abs(tram.x - 47.75) < 1e-9 and abs(tram.y + 5.0) < 1e-9 and abs(tram.yaw) < 1e-9, tram
    assert (tram.length, tram.width, tram.source) == (4.5, 2.0, 'sumo_only')
    assert actors['sumo:ped.0/carla:%d' % walker_id].source == 'pedestrian'


def test_ego_footprint():
    sumo = FakeSumoSimulation({}, {})
    assert ego_footprint(sumo, 'vehicle.toyota.prius')[:2] == (4.54, 2.0), 'SUMO vType first'

    class NoVtypes(FakeSumoSimulation):
        def get_vtype_size(self, type_id):
            return None

    sumo = NoVtypes({}, {})
    assert ego_footprint(sumo, 'vehicle.toyota.prius')[:2] == (4.51, 2.01), 'fallback table'
    length, width, origin = ego_footprint(sumo, 'vehicle.unknown.model')
    assert (length, width) == EGO_FOOTPRINT_DEFAULT and 'unknown vehicle_type' in origin


def main():
    tests = [
        test_catch_up,
        test_gate_order,
        test_gate_stop_during_warmup,
        test_gate_waits_for_gap,
        test_gate_timeout,
        test_gate_stop_during_wait,
        test_gate_spawn_failure,
        test_check_log_throttle,
        test_collect_gap_actors,
        test_ego_footprint,
    ]
    for test in tests:
        test()
        print('PASS %s' % test.__name__)
    print('All %d checks passed.' % len(tests))


if __name__ == '__main__':
    main()
