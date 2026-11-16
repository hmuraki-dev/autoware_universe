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
"""inpx のモデルの読み込み(vissim_integration/inpx_models.py)と、carla_variants の生成
(tools/carla_fleet/build_carla_variants.py)。CARLA・Vissim・ROS は使わない。

計画: docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md F4

Run with:
    python3 -m pytest test/test_build_carla_variants.py

コミットした Town01 の生成物を、inpx から作り直して比べるテストは、inpx のパスを環境変数
CARLA_TOWN01_INPX で渡したときだけ動く(inpx は CARLA リポジトリ feature/vissim_windows にある)。
"""
import copy
import importlib.util
import json
import os
from pathlib import Path

import pytest
import yaml

from autoware_carla_interface import carla_fleet
from autoware_carla_interface.carla_fleet import VariantError
from autoware_carla_interface.vissim_integration import inpx_models
from autoware_carla_interface.vissim_integration.inpx_models import InpxModels

PACKAGE = Path(__file__).resolve().parents[1]
DATA = PACKAGE / 'src' / 'autoware_carla_interface' / 'data' / 'carla_fleet'
RULES = DATA / 'carla_variant_rules.yaml'
MESH = DATA / 'mesh_measurements_carla0.9.15.json'
TOWN01 = PACKAGE / 'src' / 'autoware_carla_interface' / 'vissim_integration' / 'data' / 'carla_variants_town01.yaml'

_spec = importlib.util.spec_from_file_location(
    'build_carla_variants', PACKAGE / 'tools' / 'carla_fleet' / 'build_carla_variants.py')
builder = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(builder)

# 合成の inpx。乗用車 2 モデル(車種 100)、2 セグメントの連結バス(車種 300)、割合 0 の自転車(車種 610)、
# 自車(車種 101 = drivSimVehType、モデル 1 を使う)、歩行者(車種 901)。車両構成 2 は需要が使わない。
INPX = """<?xml version="1.0" encoding="UTF-8"?>
<network>
  <netPara drivSimVehType="101"/>
  <models2D3D>
    <model2D3D no="1" name="Car - Small"><model2D3DSegs>
      <model2D3DSegment file3D="#3dmodels#Vehicles\\Road\\Car - Small (2018).u3dm" length="3.6" width="1.85" height="1.5"/>
    </model2D3DSegs></model2D3D>
    <model2D3D no="2" name="Car - Large"><model2D3DSegs>
      <model2D3DSegment file3D="#3dmodels#Vehicles\\Road\\Car - Large (2021).fbx" length="4.75" width="2.03" height="1.45"/>
    </model2D3DSegs></model2D3D>
    <model2D3D no="35" name="Bus - Articulated"><model2D3DSegs>
      <model2D3DSegment file3D="#3dmodels#Vehicles\\Road\\Bus - Articulated - front.fbx" length="10.0" width="2.55" height="3.1" jointFront="0" jointRear="9.0"/>
      <model2D3DSegment file3D="#3dmodels#Vehicles\\Road\\Bus - Articulated - rear.fbx" length="9.0" width="2.55" height="3.1" jointFront="1.0" jointRear="0"/>
    </model2D3DSegs></model2D3D>
    <model2D3D no="61" name="Bike - Cycle"><model2D3DSegs>
      <model2D3DSegment file3D="#3dmodels#Vehicles\\Road\\Bike - Cycle.fbx" length="1.8" width="0.6" height="1.7"/>
    </model2D3DSegs></model2D3D>
    <model2D3D no="101" name="Ped - Man"><model2D3DSegs>
      <model2D3DSegment file3D="#3dmodels#Pedestrians\\Ped - Man.fbx" length="0.3" width="0.5" height="1.8"/>
    </model2D3DSegs></model2D3D>
  </models2D3D>
  <model2D3DDistributions>
    <model2D3DDistribution no="10"><model2D3DDistributionElements>
      <model2D3DDistributionElement model2D3D="1" share="1"/>
      <model2D3DDistributionElement model2D3D="2" share="3"/>
    </model2D3DDistributionElements></model2D3DDistribution>
    <model2D3DDistribution no="35"><model2D3DDistributionElements>
      <model2D3DDistributionElement model2D3D="35" share="1"/>
    </model2D3DDistributionElements></model2D3DDistribution>
    <model2D3DDistribution no="61"><model2D3DDistributionElements>
      <model2D3DDistributionElement model2D3D="61" share="1"/>
    </model2D3DDistributionElements></model2D3DDistribution>
    <model2D3DDistribution no="100"><model2D3DDistributionElements>
      <model2D3DDistributionElement model2D3D="101" share="1"/>
    </model2D3DDistributionElements></model2D3DDistribution>
  </model2D3DDistributions>
  <vehicleTypes>
    <vehicleType no="100" name="Car" category="CAR" model2D3DDistr="10"/>
    <vehicleType no="101" name="EGO" category="CAR" model2D3DDistr="10"/>
    <vehicleType no="300" name="Bus" category="BUS" model2D3DDistr="35"/>
    <vehicleType no="610" name="Bike" category="BIKE" model2D3DDistr="61"/>
    <vehicleType no="901" name="Ped" category="PEDESTRIAN" model2D3DDistr="100"/>
  </vehicleTypes>
  <vehicleCompositions>
    <vehicleComposition no="1"><vehCompRelFlows>
      <vehicleCompositionRelativeFlow vehType="100" relFlow="0.8"/>
      <vehicleCompositionRelativeFlow vehType="300" relFlow="0.2"/>
      <vehicleCompositionRelativeFlow vehType="610" relFlow="0"/>
    </vehCompRelFlows></vehicleComposition>
    <vehicleComposition no="2"><vehCompRelFlows>
      <vehicleCompositionRelativeFlow vehType="610" relFlow="1"/>
    </vehCompRelFlows></vehicleComposition>
  </vehicleCompositions>
  <vehicleInputs>
    <vehicleInput no="1"><timeIntVehVols><timeIntervalVehVolume vehComp="1" volume="300"/></timeIntVehVols></vehicleInput>
  </vehicleInputs>
</network>
"""

# 合成の実メッシュ(長さ・幅だけ)
MESH_ROWS = {
    'vehicle.small.car': (3.65, 1.80), 'vehicle.large.car': (4.70, 2.05),
    'vehicle.box.truck': (5.10, 2.56), 'vehicle.ego.car': (4.75, 2.03),
}
SYNTH_RULES = dict(length_tolerance_m=0.75, width_over_tolerance_m=0.35, select='nearest_length',
                   width_under_tolerance_m=dict(CAR=0.35, HGV=0.10, BUS=0.10, TRAM=0.10, BIKE=0.35),
                   fallback=dict(blueprint='vehicle.box.truck', kind='stack'),
                   excluded={'vehicle.ego.*': '自車'})


@pytest.fixture
def inpx_path(tmp_path):
    path = tmp_path / 'net.inpx'
    path.write_text(INPX, encoding='utf-8')
    return path


@pytest.fixture
def sources(tmp_path):
    """合成の実メッシュと規則のファイル(生成物の sources が指すもの)。"""
    mesh = tmp_path / 'data' / 'mesh.json'
    mesh.parent.mkdir()
    rows = [dict(blueprint=name, mesh_length_m=length, mesh_width_m=width)
            for name, (length, width) in MESH_ROWS.items()]
    mesh.write_text(json.dumps(dict(carla_server_version='0.9.15', measurements=rows)), encoding='utf-8')
    rules = tmp_path / 'data' / 'rules.yaml'
    rules.write_text(yaml.safe_dump(SYNTH_RULES, allow_unicode=True), encoding='utf-8')
    return mesh, rules


def synth_mesh():
    return {name: dict(length=length, width=width, center_x=0.0, center_y=0.0)
            for name, (length, width) in MESH_ROWS.items()}


# ---------- inpx の読み込み ----------

@pytest.mark.parametrize('raw', [
    '#3dmodels#Vehicles\\Road\\Car - Fiat 500 (2018).u3dm', 'Car - Fiat 500 (2018).fbx',
    'Car - Fiat 500 (2018).V3D', b'Car - Fiat 500 (2018)', bytearray(b'Car - Fiat 500 (2018)'),
    'C:/models/Car - Fiat 500 (2018).u3dm',
])
def test_model_key_matches_inpx_and_dsi_forms(raw):
    """inpx の file3D と DSI の ModelFileName(bytes。F0 で確かめた形)が同じ鍵になる。"""
    assert inpx_models.model_key(raw) == 'Car - Fiat 500 (2018)'


def test_model_key_of_nothing():
    assert inpx_models.model_key(None) == ''
    assert inpx_models.model_key(b'') == ''


def test_articulated_body_spans_all_segments():
    """2 体目の前端 = 1 体目の jointRear − 2 体目の jointFront。全長 = 8.0 + 9.0。"""
    segments = [dict(length='10.0', width='2.55', height='3.1', jointFront='0', jointRear='9.0'),
                dict(length='9.0', width='2.50', height='3.2', jointFront='1.0', jointRear='0')]
    assert inpx_models.model_body(segments) == pytest.approx((17.0, 2.55, 3.2))


def test_inpx_models_and_shares(inpx_path):
    inpx = InpxModels.from_file(inpx_path)
    assert inpx.driving_sim_vehicle_type == '101'
    assert inpx.models['1']['file'] == 'Car - Small (2018)'
    assert inpx.models['35']['file'] == 'Bus - Articulated - front'
    assert inpx.models['35']['segments'] == 2
    assert inpx.models['35']['length'] == pytest.approx(17.0)    # 8.0 + 9.0
    assert inpx.active_compositions == ['1']
    shares = inpx.model_shares()
    # 自車(101)と歩行者(901)の車種は入らない。使われない車両構成 2 も入らない
    assert set(shares) == {'1', '2', '35', '61'}
    assert shares['1'] == pytest.approx(0.8 * 0.25)
    assert shares['2'] == pytest.approx(0.8 * 0.75)
    assert shares['35'] == pytest.approx(0.2)
    assert shares['61'] == pytest.approx(0.0)
    assert not inpx.is_traffic_vehicle_type('101')
    assert not inpx.is_traffic_vehicle_type('901')
    assert not inpx.is_traffic_vehicle_type('999')
    assert inpx.is_traffic_vehicle_type(100)


def test_network_models(inpx_path):
    models = builder.network_models(InpxModels.from_file(inpx_path))
    assert set(models) == {'1', '2', '35', '61'}
    assert models['2']['file'] == 'Car - Large (2021)'
    assert models['2']['types'] == {'100': pytest.approx(0.75)}     # 自車の車種 101 は入らない
    assert models['2']['categories'] == {'CAR'}
    assert models['35']['segments'] == 2


# ---------- 生成(build) ----------

def test_build_assigns_skips_share_zero_and_is_deterministic(inpx_path):
    models = builder.network_models(InpxModels.from_file(inpx_path))
    first = builder.build(models, SYNTH_RULES, synth_mesh())
    assert first == builder.build(copy.deepcopy(models), SYNTH_RULES, synth_mesh())
    assignment, skipped = first
    assert [a['blueprint'] for a in assignment['1']['actors']] == ['vehicle.small.car']
    # 除外した自車のブループリント(寸法はぴったり)ではなく、次に近いもの
    assert [a['blueprint'] for a in assignment['2']['actors']] == ['vehicle.large.car']
    assert assignment['35']['method'] == 'fallback_stack'
    assert len(assignment['35']['actors']) == 4                    # ceil(17.0 / 5.10)
    assert skipped == {'61': dict(L=1.8, W=0.6, reason='share_zero')}


def test_build_stops_when_a_used_model_has_no_body(inpx_path):
    models = builder.network_models(InpxModels.from_file(inpx_path))
    models['61']['share'] = 0.01
    with pytest.raises(VariantError) as caught:
        builder.build(models, SYNTH_RULES, synth_mesh())
    assert caught.value.code == 'variant_model_empty'
    assert [row['model'] for row in caught.value.evidence['models']] == ['61']


def test_expected_bodies_count_stacked_bodies(inpx_path):
    models = builder.network_models(InpxModels.from_file(inpx_path))
    assignment, _ = builder.build(models, SYNTH_RULES, synth_mesh())
    bodies = builder.expected_bodies(models, assignment)
    assert bodies['vehicle.box.truck'] == pytest.approx(100 * 0.2 * 4)
    assert bodies['vehicle.large.car'] == pytest.approx(100 * 0.6)
    assert list(bodies)[0] == 'vehicle.box.truck'                   # 多い順


# ---------- 書き出しと、生成物の起動時検査 ----------

def run_main(inpx_path, sources, out, **extra):
    mesh, rules = sources
    argv = ['--inpx', str(inpx_path), '--geometry', str(mesh), '--rules', str(rules), '--out', str(out),
            '--generated-at', '2026-11-16T00:00:00+09:00']
    for key, value in extra.items():
        argv += [f"--{key.replace('_', '-')}", value]
    return builder.main(argv)


def test_main_writes_variants_the_bridge_can_check(tmp_path, inpx_path, sources):
    out = tmp_path / 'pkg' / 'vissim_integration' / 'data' / 'carla_variants_synth.yaml'
    assert run_main(inpx_path, sources, out, inpx_ref='synthetic network') == 0
    variants = carla_fleet.load_variants(out)
    assert variants['name'] == 'carla_variants_synth'
    # --inpx-ref があればパスの代わりに出どころを記録する(渡した inpx が一時ファイルのときのため)
    assert variants['sources']['inpx'] == dict(ref='synthetic network', sha256=carla_fleet.digest(inpx_path))
    assert not Path(variants['sources']['geometry']['path']).is_absolute()
    assert variants['sources']['geometry']['carla_server_version'] == '0.9.15'
    carla_fleet.check_sources(variants)
    carla_fleet.check_blueprints(variants, MESH_ROWS)
    # モデルを DSI から引くための項目
    car = variants['assignment']['2']
    assert car['file'] == 'Car - Large (2021)'
    assert car['types'] == {'100': pytest.approx(0.75)}
    assert car['segments'] == 1
    assert variants['skipped']['61']['file'] == 'Bike - Cycle'
    assert variants['skipped']['61']['types'] == {'610': pytest.approx(1.0)}
    # 連結バス(2 セグメント)は follows_path を持つので、扱わないブリッジは止める
    assert variants['assignment']['35']['segments'] == 2
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_no_follows_path(variants)
    assert caught.value.evidence['models'] == ['35']


def test_main_records_the_inpx_path_without_ref(tmp_path, inpx_path, sources):
    out = tmp_path / 'v.yaml'
    assert run_main(inpx_path, sources, out) == 0
    assert carla_fleet.load_variants(out)['sources']['inpx'] == dict(
        path=str(inpx_path), sha256=carla_fleet.digest(inpx_path))


def test_main_is_reproducible(tmp_path, inpx_path, sources):
    first, second = tmp_path / 'a' / 'v.yaml', tmp_path / 'b' / 'v.yaml'
    assert run_main(inpx_path, sources, first) == 0
    assert run_main(inpx_path, sources, second) == 0
    assert first.read_text(encoding='utf-8') == second.read_text(encoding='utf-8').replace(
        '/b/v.yaml', '/a/v.yaml')


def test_main_stops_without_writing(tmp_path, inpx_path, sources):
    mesh, _ = sources
    mesh.write_text(json.dumps(dict(measurements=[
        dict(blueprint='vehicle.box.truck', mesh_length_m=5.1, mesh_width_m=2.56)])), encoding='utf-8')
    out = tmp_path / 'v.yaml'
    assert run_main(inpx_path, sources, out) == 1
    assert not out.exists()


# ---------- コミットした Town01 の生成物 ----------

def test_town01_variants_pass_the_startup_checks():
    variants = carla_fleet.load_variants(TOWN01)
    carla_fleet.check_sources(variants)
    mesh, meta = carla_fleet.mesh_for(variants)
    carla_fleet.check_blueprints(variants, mesh)
    carla_fleet.check_no_follows_path(variants)
    assert carla_fleet.check_carla_version(meta, '0.9.15') is None
    assert set(variants['assignment']) == {'1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '22', '23', '31', '70'}
    for model, spec in variants['assignment'].items():
        assert spec['file'] and spec['types'], model
    assert variants['assignment']['3']['file'] == 'Car - Volkswagen Golf (2007)'
    assert variants['assignment']['31']['file'] == 'Bus - C2 Standard 2-doors'
    assert list(variants['assignment']['70']['types']) == ['700']


@pytest.mark.skipif(not os.environ.get('CARLA_TOWN01_INPX'), reason='CARLA_TOWN01_INPX が無い')
def test_town01_variants_are_what_the_generator_writes(tmp_path):
    shipped = carla_fleet.load_variants(TOWN01)
    inpx = Path(os.environ['CARLA_TOWN01_INPX'])
    assert carla_fleet.digest(inpx) == shipped['sources']['inpx']['sha256']
    out = TOWN01.parent / f'_rebuilt_{os.getpid()}.yaml'      # sources の相対パスが同じになる場所
    try:
        assert builder.main(['--inpx', str(inpx), '--geometry', str(MESH), '--rules', str(RULES),
                             '--out', str(out), '--generated-at', shipped['generated_at'],
                             '--name', shipped['name']]) == 0
        rebuilt = carla_fleet.load_variants(out)
    finally:
        out.unlink(missing_ok=True)
    for key in ('assignment', 'skipped', 'expected_bodies_per_100_vehicles', 'rules'):
        assert rebuilt[key] == shipped[key], key
    for key in ('geometry', 'rules'):
        assert rebuilt['sources'][key] == shipped['sources'][key], key
