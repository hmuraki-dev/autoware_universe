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
"""SUMO の .sumocfg・.rou.xml からのモデルの読み込み(sumo_integration/sumo_fleet.py)と、
生成ツールの --sumocfg(tools/carla_fleet/build_carla_variants.py)。SUMO・CARLA・ROS は使わない。

計画: docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md S3

Run with:
    python3 -m pytest test/test_sumo_fleet.py

コミットした Town01 の生成物を、sumo/ の .sumocfg から作り直して比べるテストは、sumo/ の route-files が
生成物に記録した sha256 と同じときだけ動く(Town01 を変えて、表を作り直していないときは動かない)。
"""
import importlib.util
import os
from pathlib import Path

import pytest

from autoware_carla_interface import carla_fleet
from autoware_carla_interface.carla_fleet import VariantError
from autoware_carla_interface.sumo_integration import sumo_fleet

PACKAGE = Path(__file__).resolve().parents[1]
DATA = PACKAGE / 'src' / 'autoware_carla_interface' / 'data' / 'carla_fleet'
RULES = DATA / 'carla_variant_rules.yaml'
MESH = DATA / 'mesh_measurements_carla0.9.15.json'
SUMO_TOWN01 = (PACKAGE / 'src' / 'autoware_carla_interface' / 'sumo_integration' / 'data'
               / 'carla_variants_sumo_town01.yaml')
CARLA_TOWN01_SUMOCFG = PACKAGE / 'sumo' / 'Town01.sumocfg'

_spec = importlib.util.spec_from_file_location(
    'build_carla_variants', PACKAGE / 'tools' / 'carla_fleet' / 'build_carla_variants.py')
builder = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(builder)

SUMOCFG = """<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="net/x.net.xml"/>
        <route-files value="vtypes.rou.xml, rou/demand.rou.xml"/>
    </input>
</configuration>
"""
# 寸法の同じ car_a・car_b は 1 つのモデルになる。long_van は passenger でも 6 m を超えるので HGV。歩行者は除く
VTYPES = """<routes>
  <vType id="car_a" length="4.0" width="1.8" vClass="passenger"/>
  <vType id="car_b" length="4.0" width="1.8" vClass="evehicle"/>
  <vType id="long_van" length="7.0" width="2.3" vClass="passenger"/>
  <vType id="bike" length="2.1" width="0.9" vClass="motorcycle"/>
  <vType id="unused" length="5.5" width="2.0" vClass="passenger"/>
  <vType id="walker" length="0.3" width="0.5" vClass="pedestrian"/>
</routes>
"""
# G: car_a 2・car_b 1・long_van 1 の割合、H: 中に vType を持つ分布
DEMAND = """<routes>
  <vTypeDistribution id="G" vTypes="car_a car_b long_van" probabilities="2 1 1"/>
  <vTypeDistribution id="H">
    <vType id="inner_bus" length="12.0" width="2.5" vClass="bus" probability="1"/>
  </vTypeDistribution>
  <flow id="f1" type="G" begin="0" end="100" period="exp(0.4)"/>
  <flow id="f2" type="bike" begin="0" end="100" vehsPerHour="360"/>
  <flow id="f3" type="H" begin="0" end="100" number="5"/>
  <vehicle id="v1" type="long_van" depart="1"/>
  <vehicle id="v2" depart="2"/>
  <personFlow id="p" type="walker" begin="0" end="100" period="10"/>
</routes>
"""
# 合計: G 40 台(car 30・long_van 10)+ long_van 1 + bike 10 + inner_bus 5 + 既定 1 = 57 台


@pytest.fixture
def sumocfg(tmp_path):
    (tmp_path / 'rou').mkdir()
    (tmp_path / 'vtypes.rou.xml').write_text(VTYPES, encoding='utf-8')
    (tmp_path / 'rou' / 'demand.rou.xml').write_text(DEMAND, encoding='utf-8')
    path = tmp_path / 'x.sumocfg'
    path.write_text(SUMOCFG, encoding='utf-8')
    return path


# ---------- 読み込み ----------

def test_route_files_resolve_from_the_sumocfg(sumocfg):
    names = sumo_fleet.route_file_names(sumocfg)
    assert [name for name, _ in names] == ['vtypes.rou.xml', 'rou/demand.rou.xml']
    assert [Path(path) for _, path in names] == [sumocfg.parent / 'vtypes.rou.xml',
                                                  sumocfg.parent / 'rou' / 'demand.rou.xml']


def test_sumocfg_without_route_files_stops(tmp_path):
    path = tmp_path / 'x.sumocfg'
    path.write_text('<configuration><input><net-file value="n.xml"/></input></configuration>', encoding='utf-8')
    with pytest.raises(VariantError) as caught:
        sumo_fleet.route_file_names(path)
    assert caught.value.code == 'variant_rou_unreadable'


def test_read_models_across_files(sumocfg):
    models, model_of = sumo_fleet.read_models(sumo_fleet.route_files(sumocfg))
    car, van, bike = models['L4.000_W1.800'], models['L7.000_W2.300'], models['L2.100_W0.900']
    bus, unused, default = models['L12.000_W2.500'], models['L5.500_W2.000'], models['L5.000_W1.800']
    # 寸法でまとめる。歩行者は入らない
    assert car['vtypes'] == ['car_a', 'car_b']
    assert model_of['car_b'] == 'L4.000_W1.800'
    assert 'walker' not in model_of
    assert 'G' not in model_of and 'H' not in model_of
    # 割合: 台数(flow の到着率 × 時間、vehsPerHour、number、vehicle)× 分布の確率 / 歩行者を除く台数
    total = 57.0
    assert car['share'] == pytest.approx(30 / total)
    assert van['share'] == pytest.approx(11 / total)
    assert bike['share'] == pytest.approx(10 / total)
    assert bus['share'] == pytest.approx(5 / total)
    assert default['share'] == pytest.approx(1 / total)
    assert unused['share'] == 0.0
    assert sum(m['share'] for m in models.values()) == pytest.approx(1.0)
    # カテゴリ: vClass から。passenger でも 6 m を超えると HGV
    assert car['categories'] == {'CAR'} and van['categories'] == {'HGV'}
    assert bike['categories'] == {'BIKE'} and bus['categories'] == {'BUS'}
    # 分布の名前を残す(生成物のまとめ方に使う)
    assert car['distributions'] == {'G'} and bus['distributions'] == {'H'} and bike['distributions'] == set()
    # 寸法は丸めない。セグメントは 1
    assert car['L'] == 4.0 and car['segments'] == 1
    assert default['dimension_defaulted'] and not car['dimension_defaulted']


def test_undefined_vtype_stops(tmp_path):
    rou = tmp_path / 'r.rou.xml'
    rou.write_text('<routes><flow id="f" type="nope" begin="0" end="10" number="1"/></routes>', encoding='utf-8')
    with pytest.raises(VariantError) as caught:
        sumo_fleet.read_models([rou])
    assert caught.value.code == 'variant_rou_unreadable'
    assert caught.value.evidence['vtype'] == 'nope'


def test_flow_without_rate_stops(tmp_path):
    rou = tmp_path / 'r.rou.xml'
    rou.write_text('<routes><vType id="a" length="4" width="2"/><flow id="f" type="a" begin="0" end="10"/></routes>',
                   encoding='utf-8')
    with pytest.raises(VariantError) as caught:
        sumo_fleet.read_models([rou])
    assert caught.value.code == 'variant_rou_unreadable'


def test_model_key():
    assert sumo_fleet.model_key(12.135, 2.55) == 'L12.135_W2.550'


# ---------- 生成(--sumocfg) ----------

def test_main_writes_sumo_variants(tmp_path, sumocfg):
    out = tmp_path / 'pkg' / 'sumo_integration' / 'data' / 'carla_variants_sumo_x.yaml'
    assert builder.main(['--sumocfg', str(sumocfg), '--sumocfg-ref', 'synthetic',
                         '--geometry', str(MESH), '--rules', str(RULES), '--out', str(out),
                         '--generated-at', '2026-11-16T00:00:00+09:00']) == 0
    variants = carla_fleet.load_variants(out)
    assert variants['sources']['sumocfg'] == dict(ref='synthetic', sha256=carla_fleet.digest(sumocfg))
    assert variants['sources']['route_files'] == {
        'vtypes.rou.xml': carla_fleet.digest(sumocfg.parent / 'vtypes.rou.xml'),
        'rou/demand.rou.xml': carla_fleet.digest(sumocfg.parent / 'rou' / 'demand.rou.xml')}
    assert 'inpx' not in variants['sources']
    carla_fleet.check_sources(variants)
    car = variants['assignment']['L4.000_W1.800']
    assert car['vtypes'] == ['car_a', 'car_b'] and car['segments'] == 1
    assert 'file' not in car and 'types' not in car
    assert variants['skipped']['L5.500_W2.000']['vtypes'] == ['unused']
    # 12 m のバスは carlacola の重ね置き
    assert variants['assignment']['L12.000_W2.500']['method'] == 'fallback_stack'


def test_main_records_the_sumocfg_path_relative_to_the_variants(tmp_path, sumocfg, monkeypatch):
    # 絶対パスで渡しても、生成物には生成物のディレクトリからの相対パスを書く(置き場所の違う PC でも同じ生成物)
    out = tmp_path / 'pkg' / 'sumo_integration' / 'data' / 'carla_variants_sumo_x.yaml'
    monkeypatch.chdir(tmp_path)
    assert builder.main(['--sumocfg', str(sumocfg.resolve()),
                         '--geometry', str(MESH), '--rules', str(RULES), '--out', str(out),
                         '--generated-at', '2026-11-16T00:00:00+09:00']) == 0
    variants = carla_fleet.load_variants(out)
    recorded = variants['sources']['sumocfg']['path']
    assert not os.path.isabs(recorded)
    assert carla_fleet.resolve_source(recorded, out.parent).resolve() == sumocfg.resolve()
    assert variants['sources']['sumocfg']['sha256'] == carla_fleet.digest(sumocfg)
    # 作り方のコメントも、今のディレクトリの下のファイルは相対パスで書く
    text = out.read_text(encoding='utf-8')
    assert str(tmp_path) not in text
    assert f'--sumocfg {os.path.relpath(sumocfg.resolve(), tmp_path)} ' in text


def test_main_needs_exactly_one_input(tmp_path, sumocfg):
    with pytest.raises(SystemExit):
        builder.main(['--geometry', str(MESH), '--rules', str(RULES), '--out', str(tmp_path / 'v.yaml')])
    with pytest.raises(SystemExit):
        builder.main(['--inpx', 'a.inpx', '--sumocfg', str(sumocfg), '--geometry', str(MESH),
                      '--rules', str(RULES), '--out', str(tmp_path / 'v.yaml')])


# ---------- コミットした Town01 の生成物 ----------

# Vissim の Town01 と同じ割り当て(Vissim の carla_variants_town01.yaml と同じ。Vissim のモデル番号 -> 体)
TOWN01_EXPECTED = {
    '1': ['vehicle.nissan.micra'], '2': ['vehicle.audi.tt'], '3': ['vehicle.audi.tt'],
    '4': ['vehicle.mini.cooper_s_2021'], '5': ['vehicle.tesla.model3'], '6': ['vehicle.tesla.model3'],
    '7': ['vehicle.dodge.charger_2020'], '8': ['vehicle.mini.cooper_s_2021'], '9': ['vehicle.mini.cooper_s_2021'],
    '10': ['vehicle.nissan.patrol'],
    '22': ['vehicle.carlamotors.carlacola'] * 2, '23': ['vehicle.carlamotors.carlacola'] * 2,
    '31': ['vehicle.carlamotors.carlacola'] * 3, '70': ['vehicle.kawasaki.ninja'],
}


def test_town01_sumo_variants_match_the_vissim_assignment():
    variants = carla_fleet.load_variants(SUMO_TOWN01)
    carla_fleet.check_sources(variants)
    mesh, meta = carla_fleet.mesh_for(variants)
    carla_fleet.check_blueprints(variants, mesh)
    carla_fleet.check_no_follows_path(variants)
    assert carla_fleet.check_carla_version(meta, '0.9.15') is None
    assert len(variants['assignment']) == 14
    seen = {}
    for key, spec in variants['assignment'].items():
        (vtype,) = spec['vtypes']
        assert vtype.startswith('vissim.'), vtype
        seen[vtype.split('.')[1]] = [a['blueprint'] for a in spec['actors']]
    assert seen == TOWN01_EXPECTED
    # 自車の vType(carlavtypes.rou.xml の prius)は割り当てられず、skipped にある
    assert any('vehicle.toyota.prius' in s['vtypes'] for s in variants['skipped'].values())
    assert list(variants['sources']['route_files']) == [
        'carlavtypes.rou.xml', 'vissim_fleet_vtypes.rou.xml', 'rou/Town01.rou.xml']


def _carla_town01_matches_shipped():
    if not CARLA_TOWN01_SUMOCFG.is_file() or not SUMO_TOWN01.is_file():
        return False
    recorded = carla_fleet.load_variants(SUMO_TOWN01)['sources']['route_files']
    names = sumo_fleet.route_file_names(CARLA_TOWN01_SUMOCFG)
    return [n for n, _ in names] == list(recorded) and all(
        os.path.isfile(p) and carla_fleet.digest(p) == recorded[n] for n, p in names)


@pytest.mark.skipif(not _carla_town01_matches_shipped(), reason='sumo/ の Town01 が生成物を作ったときと違う(または無い)')
def test_town01_sumo_variants_are_what_the_generator_writes():
    shipped = carla_fleet.load_variants(SUMO_TOWN01)
    out = SUMO_TOWN01.parent / f'_rebuilt_{os.getpid()}.yaml'      # sources の相対パスが同じになる場所
    try:
        # 表に出どころのメモ(--sumocfg-ref)があればそれを、なければパスを記録する(起動手順書 2.2.3 はメモなし)
        ref = shipped['sources']['sumocfg'].get('ref')
        assert builder.main(['--sumocfg', str(CARLA_TOWN01_SUMOCFG), *(['--sumocfg-ref', ref] if ref else []),
                             '--geometry', str(MESH), '--rules', str(RULES), '--out', str(out),
                             '--generated-at', shipped['generated_at'], '--name', shipped['name']]) == 0
        rebuilt = carla_fleet.load_variants(out)
    finally:
        out.unlink(missing_ok=True)
    for key in ('assignment', 'skipped', 'expected_bodies_per_100_vehicles', 'rules', 'sources'):
        assert rebuilt[key] == shipped[key], key
