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
"""Vissim ブリッジへのブループリント割り当ての組み込み(FleetPlan、配置の計算、同期処理)。

計画: docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md F5

CARLA サーバ・Vissim・ROS は使わない(carla モジュールは Transform などの値の型にだけ使う)。
生成物を使わないとき(vtypes.json と乱数)の動作は、既存の vissim_ego_spawn_gate_test.py などが確かめる。

Run with:
    python3 -m pytest test/test_vissim_fleet_plan.py
"""
import math
from pathlib import Path
import shutil
import types
from unittest import mock

import pytest
import yaml

carla = pytest.importorskip('carla')

from autoware_carla_interface import carla_fleet  # noqa: E402
from autoware_carla_interface.carla_fleet import VariantError  # noqa: E402
from autoware_carla_interface.vissim_integration import bridge_helper as bh  # noqa: E402
from autoware_carla_interface.vissim_integration import fleet_plan as fp  # noqa: E402
from autoware_carla_interface.vissim_integration import simulation_synchronization as ss  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE / 'src' / 'autoware_carla_interface'
TOWN01 = SRC / 'vissim_integration' / 'data' / 'carla_variants_town01.yaml'
MESH = SRC / 'data' / 'carla_fleet' / 'mesh_measurements_carla0.9.15.json'
RULES = SRC / 'data' / 'carla_fleet' / 'carla_variant_rules.yaml'


def all_blueprints():
    return set(carla_fleet.load_mesh(MESH)[0])


@pytest.fixture(scope='module')
def plan():
    return fp.FleetPlan(str(TOWN01), all_blueprints(), '0.9.15')


def copy_variants(tmp_path, edit=None):
    """Town01 の生成物と、その生成元(実メッシュ・規則)を同じ相対位置に写す。edit で中身を変える。"""
    data = tmp_path / 'data' / 'carla_fleet'
    out = tmp_path / 'vissim_integration' / 'data'
    data.mkdir(parents=True)
    out.mkdir(parents=True)
    shutil.copy(MESH, data / MESH.name)
    shutil.copy(RULES, data / RULES.name)
    variants = yaml.safe_load(TOWN01.read_text(encoding='utf-8'))
    if edit:
        edit(variants)
    path = out / TOWN01.name
    path.write_text(yaml.safe_dump(variants, allow_unicode=True), encoding='utf-8')
    return str(path)


# ---------- FleetPlan ----------

def test_town01_plan_loads(plan):
    assert plan.name == 'carla_variants_town01'
    assert len(plan.assignment) == 14
    assert plan.version_note is None


@pytest.mark.parametrize('vehicle_type, model_filename, model', [
    (100, b'Car - Volkswagen Golf (2007)', '3'),           # DSI の形(bytes、F0 で確かめた)
    ('100', 'Car - Fiat 500 (2018)', '1'),
    (100, '#3dmodels#Vehicles\\Road\\Car - Fiat 500 (2018).u3dm', '1'),
    (210, b'HGV - Delivery DAF LF (2014)', '23'),
    (220, b'HGV - Semi-Tractor Volvo E (2019)', '22'),
    (300, b'Bus - C2 Standard 2-doors', '31'),
    (700, b'Bike - Motorbike Yamaha MT 07 (2022)', '70'),
])
def test_model_lookup(plan, vehicle_type, model_filename, model):
    assert plan.model_for(vehicle_type, model_filename) == model


def test_actors_of_a_stacked_model(plan):
    actors = plan.actors_for(300, b'Bus - C2 Standard 2-doors')
    assert [a['blueprint'] for a in actors] == ['vehicle.carlamotors.carlacola'] * 3
    assert [a['front_offset_m'] for a in actors] == pytest.approx([0.0, 3.513071, 7.026142])
    assert plan.spec_for(300, b'Bus - C2 Standard 2-doors')['dL'] == pytest.approx(0.0)


@pytest.mark.parametrize('vehicle_type, model_filename', [
    (100, b'Car - Unknown (2030)'),                          # inpx が変わった
    (300, b'Car - Volkswagen Golf (2007)'),                  # 車種とファイルの組が表に無い
    (101, b'Car - Volkswagen Golf (2007)'),                  # 自車の車種(Vissim は返さないはず)
])
def test_unknown_model_stops(plan, vehicle_type, model_filename):
    with pytest.raises(VariantError) as caught:
        plan.model_for(vehicle_type, model_filename)
    assert caught.value.code == 'npc_model_unknown'


def test_zero_share_model_stops_during_run(plan):
    with pytest.raises(VariantError) as caught:
        plan.actors_for(610, b'Bike - Cycle Man 02')
    assert caught.value.code == 'variant_model_empty'
    assert caught.value.evidence['models'] == ['61']
    assert caught.value.evidence['during'] == 'run'


def test_startup_checks(tmp_path):
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(str(tmp_path / 'missing.yaml'), all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_config_missing'
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(str(TOWN01), all_blueprints() - {'vehicle.carlamotors.carlacola'}, '0.9.15')
    assert caught.value.code == 'variant_blueprint_missing'
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(str(TOWN01), all_blueprints(), '0.9.16')
    assert caught.value.code == 'variant_mesh_carla_version_changed'


def test_changed_rules_stop(tmp_path):
    path = copy_variants(tmp_path)
    fp.FleetPlan(path, all_blueprints(), '0.9.15')
    rules = tmp_path / 'data' / 'carla_fleet' / RULES.name
    rules.write_text(rules.read_text(encoding='utf-8') + '\n', encoding='utf-8')
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(path, all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_source_changed'


def test_ambiguous_model_stops(tmp_path):
    def duplicate(variants):
        variants['assignment']['99'] = dict(variants['assignment']['3'])
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(copy_variants(tmp_path, duplicate), all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_model_ambiguous'
    assert caught.value.evidence['models'] == ['3', '99']


def test_model_identity_is_required(tmp_path):
    def strip(variants):
        del variants['assignment']['3']['file']
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(copy_variants(tmp_path, strip), all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_model_identity_missing'


def test_follows_path_stops(tmp_path):
    def articulate(variants):
        variants['assignment']['31']['actors'][1]['follows_path'] = True
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(copy_variants(tmp_path, articulate), all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_articulated_unsupported'


def test_resolve_variants_path():
    assert fp.resolve_variants_path('carla_variants_town01.yaml') == str(TOWN01.resolve())
    assert fp.resolve_variants_path('/abs/v.yaml') == '/abs/v.yaml'


def test_summary_counts_spawns(tmp_path):
    plan = fp.FleetPlan(copy_variants(tmp_path), all_blueprints(), '0.9.15')
    plan.record_spawn(['vehicle.audi.tt', 'vehicle.audi.tt', 'vehicle.carlamotors.carlacola'])
    summary = plan.summary()
    assert list(summary)[0] == 'vehicle.audi.tt'
    assert summary['vehicle.audi.tt']['spawned'] == 2
    assert summary['vehicle.audi.tt']['share'] == pytest.approx(2 / 3)
    assert summary['vehicle.carlamotors.carlacola']['expected_share'] == pytest.approx(70.0 / 140.0)    # 体数の合計は 140.0
    assert summary['vehicle.nissan.patrol']['spawned'] == 0


# ---------- 配置の計算(計画書 2.4) ----------

# 配置の計算そのもの(carla_fleet.body_origins)のテストは test_carla_fleet.py にある。


def test_fleet_transforms_match_the_upstream_front_convention(plan):
    """前端の位置・向きは get_carla_transform() と同じ(y と yaw の反転)。"""
    vissim_tf = carla.Transform(carla.Location(100.0, 50.0, 0.0), carla.Rotation(0.0, 90.0, 0.0))
    spec, = plan.actors_for(100, b'Car - Fiat 500 (2018)')
    transform, = bh.BridgeHelper.get_fleet_transforms(plan, vissim_tf, [spec])
    body = plan.body('vehicle.nissan.micra')
    # vissim の向き 90 度(北) = CARLA の -90 度(-y)。体は前端から +y 側(後ろ)にある
    assert transform.rotation.yaw == pytest.approx(-90.0)
    assert transform.location.x == pytest.approx(100.0 - body['center_y'], abs=1e-4)
    assert transform.location.y == pytest.approx(-50.0 + body['length'] / 2 + body['center_x'], abs=1e-4)


# ---------- 同期処理(sync_vissim_to_carla / spawn_all_vissim_actors_in_carla) ----------

class FakeBlueprint(object):
    def __init__(self, blueprint_id):
        self.id = blueprint_id
        self.attributes = {}

    def has_attribute(self, name):
        return name in ('color', 'driver_id', 'role_name')

    def get_attribute(self, name):
        values = ['%d,0,0' % i for i in range(5)] if name == 'color' else ['0', '1', '2']
        return types.SimpleNamespace(recommended_values=values)

    def set_attribute(self, name, value):
        self.attributes[name] = value


class FakeLibrary(object):
    def find(self, blueprint_id):
        return FakeBlueprint(blueprint_id)


def vissim_vehicle(vehicle_id, vehicle_type, model_filename, x=100.0):
    return types.SimpleNamespace(
        id=vehicle_id, type=vehicle_type, model_filename=model_filename,
        get_transform=lambda: carla.Transform(carla.Location(x, 20.0, 0.0), carla.Rotation(0.0, 0.0, 0.0)),
        get_velocity=lambda: carla.Vector3D(5.0, 0.0, 0.0))


def make_sync(plan, vehicles, spawned=(), destroyed=()):
    sync = ss.SimulationSynchronization.__new__(ss.SimulationSynchronization)
    sync.fleet = plan
    sync.vissim = mock.MagicMock()
    sync.vissim.spawned_vehicles = set(spawned)
    sync.vissim.destroyed_vehicles = set(destroyed)
    sync.vissim.vehicle_ids = set(vehicles)
    sync.vissim.spawned_pedestrians = set()
    sync.vissim.destroyed_pedestrians = set()
    sync.vissim.pedestrian_ids = set()
    sync.vissim.tick_count = 1
    sync.vissim.get_actor.side_effect = lambda i: vehicles[i]
    sync.carla = mock.MagicMock()
    sync.carla.spawn_actor.side_effect = lambda blueprint, transform: 1000 + len(sync.carla.spawn_actor.mock_calls)
    sync.vissim2carla_ids = {}
    sync.carla2vissim_ids = {}
    sync.vissim2carla_ped_ids = {}
    sync.sync_traffic_lights = False
    return sync


@pytest.fixture
def fresh_plan(tmp_path):
    return fp.FleetPlan(copy_variants(tmp_path), all_blueprints(), '0.9.15')


def test_sync_spawns_the_assigned_body_at_its_mesh_position(fresh_plan):
    vehicles = {7: vissim_vehicle(7, 100, b'Car - Fiat 500 (2018)'),
                8: vissim_vehicle(8, 300, b'Bus - C2 Standard 2-doors', x=200.0)}
    sync = make_sync(fresh_plan, vehicles, spawned={7, 8})
    with mock.patch.object(bh.BridgeHelper, 'blueprint_library', FakeLibrary()):
        sync.sync_vissim_to_carla()
    spawned = {c.args[0].id: c.args for c in sync.carla.spawn_actor.call_args_list}
    assert set(spawned) == {'vehicle.nissan.micra', 'vehicle.carlamotors.carlacola'}
    micra_bp, micra_tf = spawned['vehicle.nissan.micra']
    assert micra_bp.attributes['role_name'] == 'vissim_driver'
    body = fresh_plan.body('vehicle.nissan.micra')
    # vissim の前端 (100, 20) -> CARLA (100, -20)、向き 0 度。原点は前端から L/2 + center_x 後ろ
    assert micra_tf.location.x == pytest.approx(100.0 - body['length'] / 2 - body['center_x'], abs=1e-4)
    assert micra_tf.location.y == pytest.approx(-20.0 - body['center_y'], abs=1e-4)
    # バスは carlacola 3 体(F6)。1 体目は自分の位置に、2・3 体目は重なるので車の真下の地下に spawn する
    calls = sync.carla.spawn_actor.call_args_list
    assert len(calls) == 4
    bus = [c.args for c in calls if c.args[0].id == 'vehicle.carlamotors.carlacola']
    assert len(bus) == 3
    assert bus[0][1].location.z == pytest.approx(0.0)
    assert [tf.location.z for _, tf in bus[1:]] == pytest.approx(
        [ss.EXTRA_BODY_PARK_Z_M - ss.EXTRA_BODY_PARK_STEP_M * k for k in (1, 2)])
    assert all(tf.location.x == pytest.approx(200.0) for _, tf in bus[1:])     # 車の前端の真下
    assert set(sync.vissim2carla_ids) == {7, 8}
    assert set(sync.vissim2carla_extra_ids) == {8}
    assert len(sync.vissim2carla_extra_ids[8]) == 2
    assert sync.mirrored_carla_ids() == {sync.vissim2carla_ids[7], sync.vissim2carla_ids[8],
                                         *sync.vissim2carla_extra_ids[8]}
    # 同じ tick のうちに、同じ置き方で更新される(箱の寸法は使わない)。バスの 3 体は前端から順に並ぶ
    updated = {c.args[0]: c.args[1] for c in sync.carla.synchronize_vehicle.call_args_list}
    assert updated[sync.vissim2carla_ids[7]].location.x == pytest.approx(micra_tf.location.x, abs=1e-4)
    cola = fresh_plan.body('vehicle.carlamotors.carlacola')
    bus_ids = [sync.vissim2carla_ids[8]] + sync.vissim2carla_extra_ids[8]
    assert [updated[i].location.x for i in bus_ids] == pytest.approx(
        [200.0 - offset - cola['length'] / 2 - cola['center_x'] for offset in (0.0, 3.513071, 7.026142)],
        abs=1e-4)
    assert all(updated[i].location.z == pytest.approx(0.0) for i in bus_ids)
    # 体ごとの速度も同じ
    velocities = {c.args[0]: c.args[2] for c in sync.carla.synchronize_vehicle.call_args_list}
    assert len({(velocities[i].x, velocities[i].y) for i in bus_ids}) == 1
    sync.carla.get_actor.assert_not_called()
    assert fresh_plan.spawn_counts == {'vehicle.nissan.micra': 1, 'vehicle.carlamotors.carlacola': 3}
    # 同じ車の体は同じ色
    assert len({bp.attributes['color'] for bp, _ in bus}) == 1


def test_sync_color_depends_on_the_vissim_id_only(fresh_plan):
    def color_of(vehicle_id):
        vehicles = {vehicle_id: vissim_vehicle(vehicle_id, 100, b'Car - Fiat 500 (2018)')}
        sync = make_sync(fresh_plan, vehicles, spawned={vehicle_id})
        with mock.patch.object(bh.BridgeHelper, 'blueprint_library', FakeLibrary()):
            sync.sync_vissim_to_carla()
        return sync.carla.spawn_actor.call_args.args[0].attributes['color']
    assert color_of(11) == color_of(11)
    assert len({color_of(i) for i in range(1, 30)}) > 1


def test_sync_stops_on_an_unknown_model(fresh_plan):
    vehicles = {7: vissim_vehicle(7, 100, b'Car - Unknown (2030)')}
    sync = make_sync(fresh_plan, vehicles, spawned={7})
    with mock.patch.object(bh.BridgeHelper, 'blueprint_library', FakeLibrary()), \
            pytest.raises(VariantError) as caught:
        sync.sync_vissim_to_carla()
    assert caught.value.code == 'npc_model_unknown'
    sync.carla.spawn_actor.assert_not_called()


def test_catch_up_spawn_uses_the_plan(fresh_plan):
    vehicles = {3: vissim_vehicle(3, 210, b'HGV - Delivery DAF LF (2014)'),
                4: vissim_vehicle(4, 700, b'Bike - Motorbike Yamaha MT 07 (2022)', x=150.0)}
    sync = make_sync(fresh_plan, vehicles)
    with mock.patch.object(bh.BridgeHelper, 'blueprint_library', FakeLibrary()):
        result = sync.spawn_all_vissim_actors_in_carla()
    assert result['vehicles'] == 2 and result['vehicles_not_spawned'] == set()
    # トラック(DAF)は carlacola 2 体
    assert sorted(c.args[0].id for c in sync.carla.spawn_actor.call_args_list) == [
        'vehicle.carlamotors.carlacola', 'vehicle.carlamotors.carlacola', 'vehicle.kawasaki.ninja']
    assert len(sync.vissim2carla_extra_ids[3]) == 1
    assert len(sync.carla.synchronize_vehicle.call_args_list) == 3
    sync.carla.get_actor.assert_not_called()


# ---------- 重ね置き(F6) ----------

def test_failed_extra_body_leaves_the_vehicle_without_counterpart(fresh_plan):
    """2 体目以降が spawn できなければ(地下で 1 + 再試行 3 回)、その車の体をすべて消し、写さない。"""
    vehicles = {8: vissim_vehicle(8, 300, b'Bus - C2 Standard 2-doors')}
    sync = make_sync(fresh_plan, vehicles, spawned={8})
    calls = []

    def spawn(blueprint, transform):
        calls.append(transform.location.z)
        return 500 if len(calls) == 1 else (501 if len(calls) == 2 else ss.INVALID_ACTOR_ID)
    sync.carla.spawn_actor.side_effect = spawn
    with mock.patch.object(bh.BridgeHelper, 'blueprint_library', FakeLibrary()), \
            mock.patch.object(ss.logging, 'error') as error:
        sync.sync_vissim_to_carla()
    # 1 体目、2 体目、3 体目(1 + 3 回、毎回もっと下で)
    assert len(calls) == 2 + 1 + ss.EXTRA_BODY_SPAWN_RETRIES
    assert calls[2:] == sorted(calls[2:], reverse=True)
    assert sorted(c.args[0] for c in sync.carla.destroy_actor.call_args_list) == [500, 501]
    assert sync.vissim2carla_ids == {} and sync.vissim2carla_extra_ids == {}
    error.assert_called_once()
    sync.carla.synchronize_vehicle.assert_not_called()
    assert fresh_plan.spawn_counts == {}


def test_destroyed_vehicle_takes_every_body(fresh_plan):
    vehicles = {8: vissim_vehicle(8, 300, b'Bus - C2 Standard 2-doors')}
    sync = make_sync(fresh_plan, vehicles, destroyed={8})
    sync.vissim2carla_ids = {8: 600}
    sync.vissim2carla_extra_ids[8] = [601, 602]
    sync.sync_vissim_to_carla()
    assert sorted(c.args[0] for c in sync.carla.destroy_actor.call_args_list) == [600, 601, 602]
    assert sync.vissim2carla_ids == {} and sync.vissim2carla_extra_ids == {}


def test_extra_bodies_are_never_adopted_into_vissim(fresh_plan):
    """CARLA 側に現れた 2 体目以降を、CARLA 由来の車(EGO など)として Vissim に送らない。"""
    sync = make_sync(fresh_plan, {})
    sync.vissim2carla_ids = {8: 600}
    sync.vissim2carla_extra_ids[8] = [601, 602]
    sync.carla.spawned_actors = {600, 601, 602, 900}           # 900: EGO
    sync.carla.destroyed_actors = set()
    sync.carla.get_actor.return_value = types.SimpleNamespace(
        get_transform=lambda: carla.Transform(), get_velocity=lambda: carla.Vector3D(),
        bounding_box=types.SimpleNamespace(extent=carla.Vector3D(2.0, 1.0, 0.7)))
    sync.vissim.spawn_actor.return_value = 77
    sync.sync_carla_to_vissim()
    assert sync.carla2vissim_ids == {900: 77}
    sync.vissim.spawn_actor.assert_called_once()


def test_close_destroys_every_body(fresh_plan):
    sync = make_sync(fresh_plan, {})
    sync._frozen_opendrive_ids = set()
    sync.vissim2carla_ids = {8: 600, 9: 700}
    sync.vissim2carla_extra_ids[8] = [601, 602]
    sync.close()
    assert sorted(c.args[0] for c in sync.carla.destroy_actor.call_args_list) == [600, 601, 602, 700]


def test_gap_check_sees_the_whole_stacked_vehicle(fresh_plan):
    """EGO の安全スポーンの隙間は、重ね置きの車を前端から最後の体の後端までの外形で見る。"""
    from autoware_carla_interface.vissim_integration import ego_spawn_gate as egsg
    bus = vissim_vehicle(8, 300, b'Bus - C2 Standard 2-doors', x=200.0)
    car = vissim_vehicle(7, 100, b'Car - Fiat 500 (2018)', x=100.0)
    sync = make_sync(fresh_plan, {7: car, 8: bus})
    sync.vissim2carla_ids = {7: 500, 8: 600}
    sync.vissim2carla_extra_ids[8] = [601, 602]
    first_box = types.SimpleNamespace(
        get_transform=lambda: carla.Transform(carla.Location(195.0, -20.0, 0.0), carla.Rotation()),
        bounding_box=types.SimpleNamespace(location=types.SimpleNamespace(x=0.0, y=0.0),
                                           extent=types.SimpleNamespace(x=2.6, y=1.3)))
    sync.carla.get_actor.return_value = first_box
    gate = egsg.EgoSpawnGate(sync, 0, 0.05, 3, lambda: False, log=lambda line: None)
    vehicles = {v.label.split('/')[0]: v for v in gate.collect_gap_vehicles()}
    whole = vehicles['vissim:8']
    cola = fresh_plan.body('vehicle.carlamotors.carlacola')
    length = 7.026142 + cola['length']
    assert whole.label == 'vissim:8/carla:600+2'
    assert whole.length == pytest.approx(length, abs=1e-4)      # バスの全長 12.135 m
    assert whole.length == pytest.approx(12.135, abs=1e-3)
    assert whole.width == pytest.approx(cola['width'])
    assert whole.x == pytest.approx(200.0 - length / 2, abs=1e-4)   # 前端 200 から全長の半分
    assert whole.y == pytest.approx(-20.0)
    # 1 体の車は今までどおり CARLA の箱
    assert vehicles['vissim:7'].length == pytest.approx(5.2)


def test_legacy_mode_without_a_plan():
    """fleet が無ければ今までどおり(BridgeHelper.get_carla_blueprint と箱の寸法)。"""
    vehicles = {7: vissim_vehicle(7, 100, b'Car - Fiat 500 (2018)')}
    sync = make_sync(None, vehicles, spawned={7})
    blueprint = FakeBlueprint('vehicle.legacy')
    sync.carla.get_actor.return_value = types.SimpleNamespace(
        bounding_box=types.SimpleNamespace(extent=carla.Vector3D(2.0, 1.0, 0.7)))
    with mock.patch.object(bh.BridgeHelper, 'get_carla_blueprint', return_value=blueprint) as legacy:
        sync.sync_vissim_to_carla()
    legacy.assert_called_once()
    assert sync.carla.spawn_actor.call_args.args[0] is blueprint
    updated = sync.carla.synchronize_vehicle.call_args.args[1]
    assert updated.location.x == pytest.approx(98.0)         # 前端 100 から箱の半分(2.0)
