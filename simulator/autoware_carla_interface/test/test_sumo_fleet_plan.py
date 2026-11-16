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
"""SUMO ブリッジへのブループリント割り当ての組み込み(FleetPlan、配置の計算、同期処理、隙間の検査)。

計画: docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md S4
(Vissim 構成の test_vissim_fleet_plan.py と同じ項目を、SUMO の形で)

CARLA サーバ・SUMO・ROS は使わない(carla モジュールは Transform などの値の型にだけ使う)。
.sumocfg の route-files を照合するテストは、~/CARLA(main)の Town01 があるときだけ動く。
生成物を使わないとき(vType = ブループリント)の動作は、既存の sumo_*_test.py が確かめる。

Run with:
    python3 -m pytest test/test_sumo_fleet_plan.py
"""
import os
from pathlib import Path
import shutil
import sys
import types
from unittest import mock

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stand_ins import install_stand_ins  # noqa: E402  pylint: disable=wrong-import-position

install_stand_ins()

import carla  # noqa: E402  pylint: disable=wrong-import-position,import-error

from autoware_carla_interface import carla_fleet  # noqa: E402
from autoware_carla_interface.carla_fleet import VariantError  # noqa: E402
from autoware_carla_interface.sumo_integration import bridge_helper as bh  # noqa: E402
from autoware_carla_interface.sumo_integration import fleet_plan as fp  # noqa: E402
from autoware_carla_interface.sumo_integration import simulation_synchronization as ss  # noqa: E402
from autoware_carla_interface.sumo_integration.ego_spawn_gate import collect_gap_actors  # noqa: E402
from autoware_carla_interface.sumo_integration.sumo_simulation import SumoActor  # noqa: E402
from autoware_carla_interface.sumo_integration.sumo_simulation import SumoActorClass  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE / 'src' / 'autoware_carla_interface'
TOWN01 = SRC / 'sumo_integration' / 'data' / 'carla_variants_sumo_town01.yaml'
MESH = SRC / 'data' / 'carla_fleet' / 'mesh_measurements_carla0.9.15.json'
RULES = SRC / 'data' / 'carla_fleet' / 'carla_variant_rules.yaml'
SUMO_EXAMPLES = Path.home() / 'CARLA' / 'Co-Simulation' / 'Sumo' / 'examples'
SUMOCFG = SUMO_EXAMPLES / 'Town01.sumocfg'
needs_sumo_town01 = pytest.mark.skipif(not SUMOCFG.is_file(), reason='~/CARLA (main) の Town01 が無い')

FIAT = 'vissim.1.fiat_500'
BUS = 'vissim.31.c2_standard'
DAF = 'vissim.23.delivery_daf_lf'
YAMAHA = 'vissim.70.motorbike_yamaha_mt_07'


def all_blueprints():
    return set(carla_fleet.load_mesh(MESH)[0])


@pytest.fixture(scope='module')
def plan():
    return fp.FleetPlan(str(TOWN01), all_blueprints(), '0.9.15')


def copy_variants(tmp_path, edit=None):
    """Town01 の生成物と、その生成元(実メッシュ・規則)を同じ相対位置に写す。edit で中身を変える。"""
    data = tmp_path / 'data' / 'carla_fleet'
    out = tmp_path / 'sumo_integration' / 'data'
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


def copy_sumocfg(tmp_path):
    """~/CARLA の Town01.sumocfg と route-files を、同じ相対位置に写す。"""
    out = tmp_path / 'sumo'
    (out / 'rou').mkdir(parents=True)
    for name in ('Town01.sumocfg', 'carlavtypes.rou.xml', 'vissim_fleet_vtypes.rou.xml', 'rou/Town01.rou.xml'):
        shutil.copy(SUMO_EXAMPLES / name, out / name)
    return out / 'Town01.sumocfg'


def size_of(plan, vtype):
    spec = plan.assignment[plan.model_for(vtype)]
    return carla.Vector3D(spec['L'] / 2.0, spec['W'] / 2.0, 0.75)


# ---------- FleetPlan ----------

def test_town01_plan_loads(plan):
    assert plan.name == 'carla_variants_sumo_town01'
    assert len(plan.assignment) == 14
    assert plan.version_note is None


@pytest.mark.parametrize('vtype, model', [
    (FIAT, 'L3.561_W1.883'),
    (BUS, 'L12.135_W2.550'),
    (YAMAHA, None),
])
def test_model_lookup(plan, vtype, model):
    found = plan.model_for(vtype)
    assert vtype in plan.assignment[found]['vtypes']
    if model is not None:
        assert found == model


def test_actors_of_a_stacked_model(plan):
    actors = plan.actors_for(BUS)
    assert [a['blueprint'] for a in actors] == ['vehicle.carlamotors.carlacola'] * 3
    assert [a['front_offset_m'] for a in actors] == pytest.approx([0.0, 3.513071, 7.026142])
    assert plan.spec_for(BUS)['dL'] == pytest.approx(0.0)


def test_same_assignment_as_vissim(plan):
    """Vissim の 14 モデルと同じ寸法の vType は、Vissim 構成と同じ体になる(S3 で確かめた結果を固定する)。"""
    assert [a['blueprint'] for a in plan.actors_for(FIAT)] == ['vehicle.nissan.micra']
    assert [a['blueprint'] for a in plan.actors_for(DAF)] == ['vehicle.carlamotors.carlacola'] * 2
    assert [a['blueprint'] for a in plan.actors_for(YAMAHA)] == ['vehicle.kawasaki.ninja']


@pytest.mark.parametrize('vtype', ['vissim.99.unknown', 'A_car'])
def test_unknown_vtype_stops(plan, vtype):
    """表に無い vType(.rou.xml が変わった)、分布の ID(TraCI はメンバーの ID を返すはず)。"""
    with pytest.raises(VariantError) as caught:
        plan.model_for(vtype)
    assert caught.value.code == 'npc_model_unknown'


def test_zero_share_vtype_stops_during_run(plan):
    """carlavtypes.rou.xml の vType(割合 0、skipped)が走行中に出てきた。"""
    with pytest.raises(VariantError) as caught:
        plan.actors_for('vehicle.audi.tt')
    assert caught.value.code == 'variant_model_empty'
    assert caught.value.evidence['during'] == 'run'


def test_vehicle_size_is_checked(plan):
    plan.model_for(FIAT, size_of(plan, FIAT))
    with pytest.raises(VariantError) as caught:
        plan.model_for(FIAT, carla.Vector3D(2.5, 0.9, 0.75))
    assert caught.value.code == 'variant_source_changed'
    assert caught.value.evidence['source'] == 'vehicle_dimensions'


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


def test_ambiguous_vtype_stops(tmp_path):
    def duplicate(variants):
        variants['assignment']['L99.000_W2.000'] = dict(variants['assignment']['L3.561_W1.883'])
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(copy_variants(tmp_path, duplicate), all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_model_ambiguous'
    assert caught.value.evidence['vtype'] == FIAT


def test_model_identity_is_required(tmp_path):
    def strip(variants):
        del variants['assignment']['L3.561_W1.883']['vtypes']
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(copy_variants(tmp_path, strip), all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_model_identity_missing'


def test_follows_path_stops(tmp_path):
    def articulate(variants):
        variants['assignment']['L12.135_W2.550']['actors'][1]['follows_path'] = True
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(copy_variants(tmp_path, articulate), all_blueprints(), '0.9.15')
    assert caught.value.code == 'variant_articulated_unsupported'


@needs_sumo_town01
def test_route_files_are_checked(tmp_path):
    """渡された .sumocfg の route-files が生成時と同じなら通り、1 つでも変われば止まる。"""
    sumocfg = copy_sumocfg(tmp_path)
    fp.FleetPlan(str(TOWN01), all_blueprints(), '0.9.15', str(sumocfg))
    rou = tmp_path / 'sumo' / 'rou' / 'Town01.rou.xml'
    rou.write_bytes(rou.read_bytes() + b'\n')
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(str(TOWN01), all_blueprints(), '0.9.15', str(sumocfg))
    assert caught.value.code == 'variant_source_changed'
    assert caught.value.evidence['source'] == 'route_files'


@needs_sumo_town01
def test_route_file_list_is_checked(tmp_path):
    """route-files の並びが変わった(vissim_fleet_vtypes.rou.xml を外した)ときも止まる。"""
    sumocfg = copy_sumocfg(tmp_path)
    text = sumocfg.read_bytes().replace(b'vissim_fleet_vtypes.rou.xml,', b'')
    sumocfg.write_bytes(text)
    with pytest.raises(VariantError) as caught:
        fp.FleetPlan(str(TOWN01), all_blueprints(), '0.9.15', str(sumocfg))
    assert caught.value.code == 'variant_source_changed'


def test_resolve_variants_path():
    path, _ = fp.resolve_variants_path('auto', '/x/examples/Town01.sumocfg')
    assert path == str(TOWN01.resolve())
    path, reason = fp.resolve_variants_path('auto', '/x/examples/Town05.sumocfg')
    assert path is None and 'carla_variants_sumo_town05.yaml' in reason
    assert fp.resolve_variants_path('legacy_vtypes', '/x/Town01.sumocfg')[0] is None
    assert fp.resolve_variants_path('carla_variants_sumo_town01.yaml', '')[0] == str(TOWN01.resolve())
    assert fp.resolve_variants_path('/abs/v.yaml', '')[0] == '/abs/v.yaml'


def test_summary_counts_spawns(tmp_path):
    plan = fp.FleetPlan(copy_variants(tmp_path), all_blueprints(), '0.9.15')
    plan.record_spawn(['vehicle.audi.tt', 'vehicle.audi.tt', 'vehicle.carlamotors.carlacola'])
    summary = plan.summary()
    assert list(summary)[0] == 'vehicle.audi.tt'
    assert summary['vehicle.audi.tt']['spawned'] == 2
    assert summary['vehicle.audi.tt']['share'] == pytest.approx(2 / 3)
    assert summary['vehicle.nissan.patrol']['spawned'] == 0


# ---------- 配置の計算 ----------

# 配置の計算そのもの(carla_fleet.body_origins)のテストは test_carla_fleet.py にある。

def test_fleet_transforms_match_the_upstream_front_convention(plan):
    """前端の位置・向きは get_carla_transform(extent = 0)と同じ(SUMO の角度、y の反転、net offset)。"""
    sumo_tf = carla.Transform(carla.Location(100.0, 50.0, 0.0), carla.Rotation(0.0, 0.0, 0.0))
    spec, = plan.actors_for(FIAT)
    with mock.patch.object(bh.BridgeHelper, 'offset', (0.0, 0.0)):
        transform, = bh.BridgeHelper.get_fleet_transforms(plan, sumo_tf, [spec])
    body = plan.body('vehicle.nissan.micra')
    # SUMO の角度 0 度(北、+y) = CARLA の -90 度(-y)。体は前端から +y 側(後ろ)にある
    assert transform.rotation.yaw == pytest.approx(-90.0)
    assert transform.location.x == pytest.approx(100.0 - body['center_y'], abs=1e-4)
    assert transform.location.y == pytest.approx(-50.0 + body['length'] / 2 + body['center_x'], abs=1e-4)


# ---------- 同期処理(sync_sumo_to_carla / spawn_all_sumo_actors_in_carla) ----------

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


def sumo_vehicle(plan, vtype, y=100.0, extent=None):
    """SUMO の前端 (50, y)、角度 90 度(東、+x) = CARLA の前端 (50, -y)、向き 0 度。"""
    return SumoActor(vtype, SumoActorClass.PASSENGER,
                     carla.Transform(carla.Location(50.0, y, 0.0), carla.Rotation(0.0, 90.0, 0.0)),
                     0, extent if extent is not None else size_of(plan, vtype), (10, 20, 30, 255))


def make_sync(plan, vehicles, spawned=(), destroyed=()):
    sync = ss.SimulationSynchronization.__new__(ss.SimulationSynchronization)
    sync.fleet = plan
    sync.sync_vehicle_color = False
    sync.sync_vehicle_lights = False
    sync.tls_manager = 'none'
    sync.ego_role_name = None
    sync._ped_tick_count = 0
    sync.sumo = mock.MagicMock()
    sync.sumo.spawned_actors = set(spawned)
    sync.sumo.destroyed_actors = set(destroyed)
    sync.sumo.spawned_persons = set()
    sync.sumo.destroyed_persons = set()
    sync.sumo.get_vehicle_ids.return_value = list(vehicles)
    sync.sumo.get_person_ids.return_value = []
    sync.sumo.get_actor.side_effect = lambda i: vehicles[i] if i in vehicles else mock.MagicMock()
    sync.carla = mock.MagicMock()
    sync.carla.spawn_actor.side_effect = lambda blueprint, transform: 1000 + len(sync.carla.spawn_actor.mock_calls)
    sync.sumo2carla_ids = {}
    sync.carla2sumo_ids = {}
    sync.sumo2carla_ped_ids = {}
    sync.sumo2carla_extra_ids = {}
    return sync


@pytest.fixture
def fresh_plan(tmp_path):
    return fp.FleetPlan(copy_variants(tmp_path), all_blueprints(), '0.9.15')


@pytest.fixture(autouse=True)
def no_net_offset():
    with mock.patch.object(bh.BridgeHelper, 'offset', (0.0, 0.0)), \
            mock.patch.object(bh.BridgeHelper, 'blueprint_library', FakeLibrary()):
        yield


def test_sync_spawns_the_assigned_body_at_its_mesh_position(fresh_plan):
    vehicles = {'car.0': sumo_vehicle(fresh_plan, FIAT, y=20.0),
                'bus.0': sumo_vehicle(fresh_plan, BUS, y=40.0)}
    sync = make_sync(fresh_plan, vehicles, spawned=set(vehicles))
    sync.sync_sumo_to_carla()
    calls = sync.carla.spawn_actor.call_args_list
    spawned = {c.args[0].id: c.args for c in calls}
    assert set(spawned) == {'vehicle.nissan.micra', 'vehicle.carlamotors.carlacola'}
    micra_bp, micra_tf = spawned['vehicle.nissan.micra']
    assert micra_bp.attributes['role_name'] == 'sumo_driver'
    body = fresh_plan.body('vehicle.nissan.micra')
    # 前端 (50, -20)、向き 0 度。原点は前端から L/2 + center_x 後ろ(SUMO の長さは使わない)
    assert micra_tf.rotation.yaw == pytest.approx(0.0, abs=1e-6)
    assert micra_tf.location.x == pytest.approx(50.0 - body['length'] / 2 - body['center_x'], abs=1e-4)
    assert micra_tf.location.y == pytest.approx(-20.0 - body['center_y'], abs=1e-4)
    # バスは carlacola 3 体。1 体目は自分の位置に、2・3 体目は重なるので車の真下の地下に spawn する
    assert len(calls) == 4
    bus = [c.args for c in calls if c.args[0].id == 'vehicle.carlamotors.carlacola']
    assert bus[0][1].location.z == pytest.approx(0.0)
    assert [tf.location.z for _, tf in bus[1:]] == pytest.approx(
        [ss.EXTRA_BODY_PARK_Z_M - ss.EXTRA_BODY_PARK_STEP_M * k for k in (1, 2)])
    assert all(tf.location.x == pytest.approx(50.0) for _, tf in bus[1:])     # 車の前端の真下
    assert set(sync.sumo2carla_ids) == {'car.0', 'bus.0'}
    assert set(sync.sumo2carla_extra_ids) == {'bus.0'}
    assert len(sync.sumo2carla_extra_ids['bus.0']) == 2
    assert sync.mirrored_carla_ids() == {sync.sumo2carla_ids['car.0'], sync.sumo2carla_ids['bus.0'],
                                         *sync.sumo2carla_extra_ids['bus.0']}
    # 同じ tick のうちに、同じ置き方で更新される。バスの 3 体は前端から順に並ぶ
    updated = {c.args[0]: c.args[1] for c in sync.carla.synchronize_vehicle.call_args_list}
    assert updated[sync.sumo2carla_ids['car.0']].location.x == pytest.approx(micra_tf.location.x, abs=1e-4)
    cola = fresh_plan.body('vehicle.carlamotors.carlacola')
    bus_ids = [sync.sumo2carla_ids['bus.0']] + sync.sumo2carla_extra_ids['bus.0']
    assert [updated[i].location.x for i in bus_ids] == pytest.approx(
        [50.0 - offset - cola['length'] / 2 - cola['center_x'] for offset in (0.0, 3.513071, 7.026142)],
        abs=1e-4)
    assert all(updated[i].location.z == pytest.approx(0.0) for i in bus_ids)
    assert fresh_plan.spawn_counts == {'vehicle.nissan.micra': 1, 'vehicle.carlamotors.carlacola': 3}
    # 同じ車の体は同じ色
    assert len({bp.attributes['color'] for bp, _ in bus}) == 1


def test_sync_color_depends_on_the_sumo_id_only(fresh_plan):
    def color_of(vehicle_id):
        vehicles = {vehicle_id: sumo_vehicle(fresh_plan, FIAT)}
        sync = make_sync(fresh_plan, vehicles, spawned={vehicle_id})
        sync.sync_sumo_to_carla()
        return sync.carla.spawn_actor.call_args.args[0].attributes['color']
    assert color_of('A_car.11') == color_of('A_car.11')
    assert len({color_of('A_car.%d' % i) for i in range(30)}) > 1


def test_sync_vehicle_color_uses_the_sumo_color(fresh_plan):
    vehicles = {'bus.0': sumo_vehicle(fresh_plan, BUS)}
    sync = make_sync(fresh_plan, vehicles, spawned={'bus.0'})
    sync.sync_vehicle_color = True
    sync.sync_sumo_to_carla()
    assert {c.args[0].attributes['color'] for c in sync.carla.spawn_actor.call_args_list} == {'10,20,30'}


def test_sync_stops_on_an_unknown_vtype(fresh_plan):
    vehicles = {'x.0': sumo_vehicle(fresh_plan, FIAT)._replace(type_id='vissim.99.unknown')}
    sync = make_sync(fresh_plan, vehicles, spawned={'x.0'})
    with pytest.raises(VariantError) as caught:
        sync.sync_sumo_to_carla()
    assert caught.value.code == 'npc_model_unknown'
    sync.carla.spawn_actor.assert_not_called()


def test_sync_stops_on_a_vehicle_of_another_size(fresh_plan):
    vehicles = {'x.0': sumo_vehicle(fresh_plan, FIAT, extent=carla.Vector3D(2.5, 0.9, 0.75))}
    sync = make_sync(fresh_plan, vehicles, spawned={'x.0'})
    with pytest.raises(VariantError) as caught:
        sync.sync_sumo_to_carla()
    assert caught.value.code == 'variant_source_changed'
    sync.carla.spawn_actor.assert_not_called()


def test_catch_up_spawn_uses_the_plan(fresh_plan):
    vehicles = {'truck.0': sumo_vehicle(fresh_plan, DAF),
                'bike.0': sumo_vehicle(fresh_plan, YAMAHA, y=150.0)}
    sync = make_sync(fresh_plan, vehicles)
    result = sync.spawn_all_sumo_actors_in_carla()
    assert result['vehicles'] == 2 and result['vehicles_not_in_carla'] == 0
    # トラック(DAF)は carlacola 2 体
    assert sorted(c.args[0].id for c in sync.carla.spawn_actor.call_args_list) == [
        'vehicle.carlamotors.carlacola', 'vehicle.carlamotors.carlacola', 'vehicle.kawasaki.ninja']
    assert len(sync.sumo2carla_extra_ids['truck.0']) == 1


# ---------- 重ね置き ----------

def test_failed_extra_body_leaves_the_vehicle_without_counterpart(fresh_plan):
    """2 体目以降が spawn できなければ(地下で 1 + 再試行 3 回)、その車の体をすべて消し、写さない。"""
    vehicles = {'bus.0': sumo_vehicle(fresh_plan, BUS)}
    sync = make_sync(fresh_plan, vehicles, spawned={'bus.0'})
    calls = []

    def spawn(blueprint, transform):
        calls.append(transform.location.z)
        return 500 if len(calls) == 1 else (501 if len(calls) == 2 else ss.INVALID_ACTOR_ID)
    sync.carla.spawn_actor.side_effect = spawn
    with mock.patch.object(ss.logging, 'error') as error:
        sync.sync_sumo_to_carla()
    assert len(calls) == 2 + 1 + ss.EXTRA_BODY_SPAWN_RETRIES
    assert calls[2:] == sorted(calls[2:], reverse=True)
    assert sorted(c.args[0] for c in sync.carla.destroy_actor.call_args_list) == [500, 501]
    assert sync.sumo2carla_ids == {} and sync.sumo2carla_extra_ids == {}
    error.assert_called_once()
    sync.carla.synchronize_vehicle.assert_not_called()
    assert fresh_plan.spawn_counts == {}


def test_destroyed_vehicle_takes_every_body(fresh_plan):
    sync = make_sync(fresh_plan, {}, destroyed={'bus.0'})
    sync.sumo2carla_ids = {'bus.0': 600}
    sync.sumo2carla_extra_ids['bus.0'] = [601, 602]
    sync.sync_sumo_to_carla()
    assert sorted(c.args[0] for c in sync.carla.destroy_actor.call_args_list) == [600, 601, 602]
    assert sync.sumo2carla_ids == {} and sync.sumo2carla_extra_ids == {}


def test_extra_bodies_are_never_adopted_into_sumo(fresh_plan):
    """CARLA 側に現れた 2 体目以降を、CARLA 由来の車(EGO など)として SUMO に送らない。"""
    sync = make_sync(fresh_plan, {})
    sync.sumo2carla_ids = {'bus.0': 600}
    sync.sumo2carla_extra_ids['bus.0'] = [601, 602]
    sync.carla.spawned_actors = {600, 601, 602, 900}           # 900: EGO
    sync.carla.destroyed_actors = set()
    sync.carla.get_actor.return_value = types.SimpleNamespace(
        attributes={'role_name': 'ego_vehicle'}, get_transform=lambda: carla.Transform(),
        bounding_box=types.SimpleNamespace(extent=carla.Vector3D(2.0, 1.0, 0.7)))
    sync.sumo.spawn_actor.return_value = 'carla0'
    with mock.patch.object(bh.BridgeHelper, 'get_sumo_vtype', return_value='vehicle.toyota.prius'), \
            mock.patch.object(bh.BridgeHelper, 'get_sumo_transform', return_value=carla.Transform()):
        sync.sync_carla_to_sumo()
    assert sync.carla2sumo_ids == {900: 'carla0'}
    sync.sumo.spawn_actor.assert_called_once()


def test_close_destroys_every_body(fresh_plan):
    sync = make_sync(fresh_plan, {})
    sync.sumo2carla_ids = {'bus.0': 600, 'car.0': 700}
    sync.sumo2carla_extra_ids['bus.0'] = [601, 602]
    sync.close()
    assert sorted(c.args[0] for c in sync.carla.destroy_actor.call_args_list) == [600, 601, 602, 700]


def test_gap_check_sees_the_whole_stacked_vehicle(fresh_plan):
    """EGO の安全スポーンの隙間は、重ね置きの車を前端から最後の体の後端までの外形で見る。"""
    vehicles = {'bus.0': sumo_vehicle(fresh_plan, BUS, y=20.0),
                'car.0': sumo_vehicle(fresh_plan, FIAT, y=40.0)}
    sync = make_sync(fresh_plan, vehicles)
    sync.sumo2carla_ids = {'bus.0': 600, 'car.0': 700}
    sync.sumo2carla_extra_ids['bus.0'] = [601, 602]
    sync.sumo.get_vehicle_footprint.return_value = (50.0, 20.0, 90.0, 12.135, 2.55)
    first_box = types.SimpleNamespace(
        get_transform=lambda: carla.Transform(carla.Location(44.0, -40.0, 0.0), carla.Rotation()),
        bounding_box=types.SimpleNamespace(extent=carla.Vector3D(1.8, 0.85, 0.7)))
    sync.carla.get_actor.return_value = first_box
    actors = {a.actor_id.split('/')[0]: a for a in collect_gap_actors(sync)}
    whole = actors['sumo:bus.0']
    cola = fresh_plan.body('vehicle.carlamotors.carlacola')
    length = 7.026142 + cola['length']
    assert whole.actor_id == 'sumo:bus.0/carla:600+2'
    assert whole.length == pytest.approx(length, abs=1e-4)
    assert whole.length == pytest.approx(12.135, abs=1e-3)      # バスの全長
    assert whole.width == pytest.approx(cola['width'])
    assert whole.x == pytest.approx(50.0 - length / 2, abs=1e-4)   # 前端 50 から全長の半分
    assert whole.y == pytest.approx(-20.0, abs=1e-4)
    # 1 体の車は今までどおり CARLA の箱
    assert actors['sumo:car.0'].length == pytest.approx(3.6)


def test_legacy_mode_without_a_plan(fresh_plan):
    """fleet が無ければ今までどおり(BridgeHelper.get_carla_blueprint と SUMO の長さ)。"""
    vehicles = {'car.0': sumo_vehicle(fresh_plan, FIAT, extent=carla.Vector3D(2.0, 0.9, 0.75))}
    sync = make_sync(None, vehicles, spawned={'car.0'})
    blueprint = FakeBlueprint('vehicle.legacy')
    with mock.patch.object(bh.BridgeHelper, 'get_carla_blueprint', return_value=blueprint) as legacy:
        sync.sync_sumo_to_carla()
    legacy.assert_called_once()
    assert sync.carla.spawn_actor.call_args.args[0] is blueprint
    updated = sync.carla.synchronize_vehicle.call_args.args[1]
    assert updated.location.x == pytest.approx(48.0)         # 前端 50 から SUMO の長さの半分(2.0)
    assert sync.sumo2carla_extra_ids == {}
