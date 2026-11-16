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
"""carla_fleet(ブループリント割り当ての共通部品)の規則と起動時の検査。CARLA・Vissim・ROS は使わない。

計画: docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md F1

規則のテストは、移植元 SBIR_sim の tests/core/test_carla_variants.py(3ea1650)の回帰テスト。
入力は移植元の CARLA 0.10 の実メッシュと規則(test/fixtures/carla_fleet/ に凍結した写し)のままにして、
「規則の実装が移植元と同じに動くか」を確かめる。0.9.15 用の期待値は、F4 の生成のテストで足す。
移植元固有のテスト(出荷済みの生成物・登録簿・/data/vissim の入力網・measured_fleet)は持ってきていない。
大型車のテストは、measured_fleet の手書きの割り当てとの一致の代わりに、重ね置きの形(体の数・等間隔・dL)を確かめる。

Run with:
    python3 -m pytest test/test_carla_fleet.py
"""
import math
from pathlib import Path
import shutil

import pytest

from autoware_carla_interface import carla_fleet
from autoware_carla_interface.carla_fleet import VariantError

TEST_DIR = Path(__file__).resolve().parent
FIXTURES = TEST_DIR / "fixtures" / "carla_fleet"
MESH = FIXTURES / "mesh_measurements_carla0.10_20260921.json"
RULES = FIXTURES / "carla_variant_rules_carla0.10.yaml"
PACKAGE_RULES = (TEST_DIR.parent / "src" / "autoware_carla_interface" / "data" / "carla_fleet"
                 / "carla_variant_rules.yaml")


def car(L, W):
    return dict(L=L, W=W, segments=1, categories={"CAR"})


# 移植元 odaiba_v019 の走行用入力網のモデル寸法と、車種内の割合 × 車種の割合
V019 = {
    "1": dict(car(3.5608, 1.8827), share=0.487 * 0.06),
    "2": dict(car(4.2010, 1.9105), share=0.487 * 0.05),
    "3": dict(car(4.3200, 2.0573), share=0.487 * 0.18),
    "4": dict(car(4.4145, 2.0474), share=0.487 * 0.11),
    "5": dict(car(4.7510, 2.0320), share=0.487 * 0.14),
    "6": dict(car(4.7814, 2.0216), share=0.487 * 0.05),
    "7": dict(car(4.9914, 2.1181), share=0.487 * 0.11),
    "8": dict(car(4.3279, 1.9925), share=0.487 * 0.07),
    "9": dict(car(4.5003, 2.1074), share=0.487 * 0.18),
    "10": dict(car(4.5912, 2.0375), share=0.487 * 0.05),
    "23": dict(L=8.45, W=2.57, segments=1, categories={"HGV"}, share=0.265),
    "22": dict(L=14.84, W=2.69, segments=2, categories={"HGV"}, share=0.116),
    "31": dict(L=10.29, W=2.56, segments=1, categories={"BUS"}, share=0.132),
}


@pytest.fixture(scope="module")
def rules():
    return carla_fleet.load_rules(RULES)


@pytest.fixture(scope="module")
def mesh():
    return carla_fleet.load_mesh(MESH)[0]


def blueprints(result):
    return [actor["blueprint"] for actor in result["actors"]]


def assign_all(models, rules, mesh):
    """モデルごとの割り当て(生成ツールの build() の、割り当ての部分だけ)。"""
    return {model: dict(L=entry["L"], W=entry["W"], **carla_fleet.assign(entry, rules, mesh))
            for model, entry in models.items()}


# ---------- 規則(移植元 設計案 §4.1・§4.2) ----------

@pytest.mark.parametrize("model, blueprint", [
    ("1", "vehicle.ue4.audi.tt"), ("2", "vehicle.ue4.audi.tt"), ("3", "vehicle.ue4.audi.tt"),
    ("4", "vehicle.mini.cooper"), ("5", "vehicle.ue4.ford.mustang"), ("6", "vehicle.lincoln.mkz"),
    ("7", "vehicle.dodge.charger"), ("8", "vehicle.ue4.audi.tt"), ("9", "vehicle.mini.cooper"),
    ("10", "vehicle.ue4.bmw.grantourer"),
])
def test_passenger_cars_take_the_nearest_length(rules, mesh, model, blueprint):
    result = carla_fleet.assign(V019[model], rules, mesh)
    assert result["method"] == "nearest"
    assert blueprints(result) == [blueprint]
    assert abs(result["dL"]) <= 0.75


@pytest.mark.parametrize("model", ["23", "22", "31"])
def test_heavy_vehicles_are_stacked_ambulances(rules, mesh, model):
    """1 体で表せない大型車は救急車を等間隔に重ね、外形の長さをモデルの長さに一致させる。"""
    entry = V019[model]
    body = mesh["vehicle.ambulance.ford"]
    result = carla_fleet.assign(entry, rules, mesh)
    count = math.ceil(entry["L"] / body["length"])
    pitch = (entry["L"] - body["length"]) / (count - 1)
    assert result["method"] == "fallback_stack"
    assert blueprints(result) == ["vehicle.ambulance.ford"] * count
    assert [actor["front_offset_m"] for actor in result["actors"]] == pytest.approx(
        [index * pitch for index in range(count)])
    assert result["dL"] == pytest.approx(0.0, abs=1e-9)
    assert result["dW"] == pytest.approx(body["width"] - entry["W"])
    # 連結車(セグメント 2 つ)だけ、2 体目以降が軌跡に従う
    articulated = entry["segments"] >= 2
    assert [bool(actor.get("follows_path")) for actor in result["actors"]] == (
        [False] + [articulated] * (count - 1))


def test_defined_but_unused_models(rules, mesh):
    """規則の汎用性: 移植元の v019 にあって走行では使っていないモデル。"""
    van = carla_fleet.assign(dict(L=6.0415, W=2.3961, segments=1, categories={"CAR"}), rules, mesh)
    assert blueprints(van) == ["vehicle.sprinter.mercedes"]
    tractor = carla_fleet.assign(dict(L=10.26, W=2.30, segments=1, categories={"HGV"}), rules, mesh)
    assert blueprints(tractor) == ["vehicle.ambulance.ford"] * 2
    assert tractor["dW"] == pytest.approx(0.3245, abs=1e-3)
    articulated_bus = carla_fleet.assign(dict(L=18.125, W=2.55, segments=2, categories={"BUS"}), rules, mesh)
    assert blueprints(articulated_bus) == ["vehicle.ambulance.ford"] * 3
    assert [bool(a.get("follows_path")) for a in articulated_bus["actors"]] == [False, True, True]


def test_two_wheelers_have_no_body_in_carla_0_10(rules, mesh):
    """0.10 の実メッシュには二輪が無いので、二輪のモデルには割り当てが作れない。"""
    assert carla_fleet.assign(dict(L=2.0982, W=0.9066, segments=1, categories={"BIKE"}), rules, mesh) is None


def test_carlacola_is_judged_by_width_not_excluded(rules, mesh):
    """carlacola は外さず dL・dW で判定する。幅の近いモデルなら 1 体で表す。"""
    truck = carla_fleet.assign(dict(L=8.0, W=2.9, segments=1, categories={"HGV"}), rules, mesh)
    assert blueprints(truck) == ["vehicle.carlacola.actors"]
    v019_truck = carla_fleet.assign(V019["23"], rules, mesh)
    assert "vehicle.carlacola.actors" not in blueprints(v019_truck)


def test_excluded_blueprints_are_never_chosen(rules, mesh):
    police_sized = carla_fleet.assign(car(5.198, 1.90), rules, mesh)
    assert blueprints(police_sized) != ["vehicle.dodgecop.charger"]
    # taxi.ford と ue4.ford.crown はどちらもタクシー塗装(5.29 x 1.78 m)。寸法がぴったりでも選ばない
    taxi_sized = carla_fleet.assign(car(5.286, 1.778), rules, mesh)
    assert blueprints(taxi_sized)[0] not in ("vehicle.taxi.ford", "vehicle.ue4.ford.crown")
    with_ego = dict(mesh, **{"vehicle.j6.gen2": dict(length=4.5, width=2.1, center_x=0.0, center_y=0.0)})
    assert blueprints(carla_fleet.assign(V019["9"], rules, with_ego)) == ["vehicle.mini.cooper"]


def test_narrow_side_is_strict_for_heavy_vehicles(rules):
    """細い側の許容はカテゴリごと。複数のカテゴリが使うモデルは厳しいほう。"""
    assert carla_fleet.width_under_tolerance(rules, {"CAR"}) == pytest.approx(0.35)
    assert carla_fleet.width_under_tolerance(rules, {"CAR", "HGV"}) == pytest.approx(0.10)
    with pytest.raises(VariantError) as caught:
        carla_fleet.width_under_tolerance(rules, {"UFO"})
    assert caught.value.code == "variant_category_unknown"


def test_ties_are_broken_by_width_then_name(rules):
    mesh = {"b.wide": dict(length=4.5, width=2.3, center_x=0, center_y=0),
            "a.close": dict(length=4.5, width=2.0, center_x=0, center_y=0),
            "c.close": dict(length=4.5, width=2.0, center_x=0, center_y=0)}
    assert blueprints(carla_fleet.assign(car(4.5, 2.0), rules, mesh)) == ["a.close"]


def test_unmeasured_fallback_stops(rules):
    mesh = {"a.small": dict(length=3.0, width=1.8, center_x=0, center_y=0)}
    with pytest.raises(VariantError) as caught:
        carla_fleet.assign(dict(L=12.0, W=2.5, segments=1, categories={"BUS"}), rules, mesh)
    assert caught.value.code == "variant_fallback_unmeasured"


def test_expected_bodies_and_pool_sizes(rules, mesh):
    """移植元 設計案 §6.1 の v019 の試算(POOL_MAX 100)。"""
    total = sum(entry["share"] for entry in V019.values())
    shares = {model: entry["share"] / total for model, entry in V019.items()}
    assignment = assign_all(V019, rules, mesh)
    demand = carla_fleet.blueprint_demand(shares, assignment)
    assert 100.0 * demand["vehicle.ambulance.ford"] == pytest.approx(114.1, abs=0.15)
    assert 100.0 * demand["vehicle.ue4.audi.tt"] == pytest.approx(17.5, abs=0.15)
    assert carla_fleet.pool_sizes(demand, 100) == {
        "vehicle.ambulance.ford": 70, "vehicle.ue4.audi.tt": 11, "vehicle.mini.cooper": 9,
        "vehicle.ue4.ford.mustang": 4, "vehicle.dodge.charger": 3,
        "vehicle.lincoln.mkz": 1, "vehicle.ue4.bmw.grantourer": 1}
    assert carla_fleet.pool_sizes(demand, 100, pool_min=3)["vehicle.lincoln.mkz"] == 3


# ---------- 規則・実メッシュの読み込み ----------

def test_package_rules_load():
    """パッケージの規則(0.9.15 用)が読める形になっている。"""
    rules = carla_fleet.load_rules(PACKAGE_RULES)
    assert rules["fallback"]["blueprint"].startswith("vehicle.")
    assert set(rules["width_under_tolerance_m"]) >= {"CAR", "HGV", "BUS", "BIKE"}


# Town01.inpx(CARLA リポジトリ feature/vissim_windows 4213552)で割合 > 0 のモデル: (L, W, カテゴリ)
TOWN01_MODELS = {
    "1": (3.561, 1.883, "CAR"), "2": (4.201, 1.910, "CAR"), "3": (4.211, 2.004, "CAR"),
    "4": (4.414, 2.047, "CAR"), "5": (4.751, 2.032, "CAR"), "6": (4.781, 2.022, "CAR"),
    "7": (4.991, 2.118, "CAR"), "8": (4.328, 1.992, "CAR"), "9": (4.500, 2.107, "CAR"),
    "10": (4.600, 2.037, "CAR"), "22": (5.960, 2.550, "HGV"), "23": (7.930, 2.550, "HGV"),
    "31": (12.135, 2.550, "BUS"), "70": (2.098, 0.907, "BIKE"),
}


@pytest.mark.parametrize("model, expected", [
    ("1", ["vehicle.nissan.micra"]), ("2", ["vehicle.audi.tt"]), ("3", ["vehicle.audi.tt"]),
    ("4", ["vehicle.mini.cooper_s_2021"]), ("5", ["vehicle.tesla.model3"]),
    ("6", ["vehicle.tesla.model3"]), ("7", ["vehicle.dodge.charger_2020"]),
    ("8", ["vehicle.mini.cooper_s_2021"]), ("9", ["vehicle.mini.cooper_s_2021"]),
    ("10", ["vehicle.nissan.patrol"]),
    ("22", ["vehicle.carlamotors.carlacola"] * 2), ("23", ["vehicle.carlamotors.carlacola"] * 2),
    ("31", ["vehicle.carlamotors.carlacola"] * 3), ("70", ["vehicle.kawasaki.ninja"]),
])
def test_package_rules_on_town01_models(model, expected):
    """0.9.15 の規則と実メッシュ(F3 で決めたもの)での Town01 の割り当て。変わったら計画書の F3 を見直す。"""
    rules = carla_fleet.load_rules(PACKAGE_RULES)
    mesh, meta = carla_fleet.load_mesh(PACKAGE_RULES.parent / "mesh_measurements_carla0.9.15.json")
    assert meta["carla_server_version"] == "0.9.15"
    length, width, category = TOWN01_MODELS[model]
    result = carla_fleet.assign(dict(L=length, W=width, segments=1, categories={category}), rules, mesh)
    assert blueprints(result) == expected
    assert abs(result["dL"]) <= rules["length_tolerance_m"] + 1e-9
    assert not any(carla_fleet.excluded(rules, name) for name in expected)


def test_invalid_rules_stop(tmp_path):
    path = tmp_path / "rules.yaml"
    path.write_text("length_tolerance_m: 0.75\n", encoding="utf-8")
    with pytest.raises(VariantError) as caught:
        carla_fleet.load_rules(path)
    assert caught.value.code == "variant_rules_invalid"


def test_load_mesh_skips_unmeasured_blueprints(tmp_path):
    path = tmp_path / "mesh.json"
    path.write_text('{"carla_server_version": "0.9.15", "measurements": ['
                    '{"blueprint": "a", "mesh_length_m": 4.0, "mesh_width_m": 2.0, "mesh_center_x_m": 0.1},'
                    '{"blueprint": "b", "mesh_length_m": 4.0}]}', encoding="utf-8")
    mesh, meta = carla_fleet.load_mesh(path)
    assert mesh == {"a": dict(length=4.0, width=2.0, center_x=0.1, center_y=0.0)}
    assert meta == {"carla_server_version": "0.9.15"}


# ---------- 起動時の検査(移植元 設計案 §6、計画書 2.5) ----------

def write_sources(directory, relative):
    """生成元(実メッシュ・規則)の写しを directory に置き、carla_variants の sources を返す。"""
    shutil.copy(MESH, directory / "mesh.json")
    shutil.copy(RULES, directory / "rules.yaml")
    sources = {}
    for key, name in (("geometry", "mesh.json"), ("rules", "rules.yaml")):
        path = directory / name
        sources[key] = dict(path=name if relative else str(path), sha256=carla_fleet.digest(path))
    return sources


@pytest.fixture(params=["absolute", "relative"])
def variants(request, tmp_path, rules, mesh):
    """生成元の写し(tmp)を指す carla_variants。sources のパスは絶対・相対(carla_variants から)の両方で試す。"""
    sources = write_sources(tmp_path, relative=request.param == "relative")
    return dict(path=str(tmp_path / "carla_variants.yaml"), assignment=assign_all(V019, rules, mesh),
                skipped={}, sources=sources)


def test_startup_checks_pass_on_the_generated_sources(variants):
    carla_fleet.check_blueprints(variants, carla_fleet.wanted_blueprints(variants))
    carla_fleet.check_sources(variants)
    carla_fleet.check_models(variants, {m: (e["L"], e["W"]) for m, e in V019.items()},
                             {m: e["share"] for m, e in V019.items()})
    mesh, _ = carla_fleet.mesh_for(variants)
    assert "vehicle.ambulance.ford" in mesh


def test_missing_blueprint_stops(variants):
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_blueprints(variants, {"vehicle.ue4.audi.tt"})
    assert caught.value.code == "variant_blueprint_missing"


@pytest.mark.parametrize("source", ["geometry", "rules"])
def test_changed_mesh_or_rules_stop(variants, source):
    path = carla_fleet.resolve_source(variants["sources"][source]["path"], carla_fleet.variants_dir(variants))
    path.write_text(path.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_sources(variants)
    assert caught.value.code == "variant_source_changed"
    assert caught.value.evidence["source"] == source


def test_missing_source_file_stops(variants):
    carla_fleet.resolve_source(variants["sources"]["rules"]["path"], carla_fleet.variants_dir(variants)).unlink()
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_sources(variants)
    assert caught.value.code == "variant_source_changed"
    assert caught.value.evidence["current_sha256"] is None


def test_relative_sources_follow_the_variants_file(tmp_path, rules, mesh):
    """相対パスは carla_variants のあるディレクトリから解決する(インストール先へ一緒に写しても読める)。"""
    original = tmp_path / "src"
    original.mkdir()
    sources = write_sources(original, relative=True)
    installed = tmp_path / "install" / "data"
    shutil.copytree(original, installed)
    variants = dict(path=str(installed / "carla_variants.yaml"), assignment=assign_all(V019, rules, mesh),
                    skipped={}, sources=sources)
    carla_fleet.check_sources(variants)
    (original / "rules.yaml").unlink()          # 元の場所は見ていない
    carla_fleet.check_sources(variants)


def test_relative_source_round_trip(tmp_path):
    base = tmp_path / "vissim_integration" / "data"
    target = tmp_path / "data" / "carla_fleet" / "mesh.json"
    written = carla_fleet.relative_source(target, base)
    assert written == "../../data/carla_fleet/mesh.json"
    assert carla_fleet.resolve_source(written, base).resolve() == target.resolve()
    assert carla_fleet.resolve_source(str(target), base) == target


def test_changed_model_dimensions_stop(variants):
    models = {m: (e["L"], e["W"]) for m, e in V019.items()}
    models["23"] = (7.93, 2.55)
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_models(variants, models, {})
    assert caught.value.code == "variant_source_changed"
    assert caught.value.evidence["model"] == "23"


def test_used_model_without_assignment_stops(variants):
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_models(variants, {}, {"70": 0.005, "61": 0.0})
    assert caught.value.code == "variant_model_empty"
    assert caught.value.evidence["models"] == ["70"]


def test_follows_path_stops_where_unsupported(variants):
    """連結車(22 はセグメント 2 つ)が入っていれば止める。外せば通る。"""
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_no_follows_path(variants)
    assert caught.value.code == "variant_articulated_unsupported"
    assert caught.value.evidence["models"] == ["22"]
    del variants["assignment"]["22"]
    carla_fleet.check_no_follows_path(variants)


def test_carla_version_check():
    assert carla_fleet.check_carla_version({"carla_server_version": "0.9.15"}, "0.9.15") is None
    assert carla_fleet.check_carla_version({}, "0.9.15") is not None      # 記録なし: 注意だけ
    with pytest.raises(VariantError) as caught:
        carla_fleet.check_carla_version({"carla_server_version": "0.10.0"}, "0.9.15")
    assert caught.value.code == "variant_mesh_carla_version_changed"


def test_missing_variants_file_stops(tmp_path):
    with pytest.raises(VariantError) as caught:
        carla_fleet.load_variants(tmp_path / "carla_variants_town99.yaml")
    assert caught.value.code == "variant_config_missing"


def test_load_variants_normalizes_model_keys(tmp_path):
    path = tmp_path / "carla_variants.yaml"
    path.write_text("assignment:\n  3: {L: 4.2, W: 2.0, actors: [{blueprint: a, front_offset_m: 0.0}]}\n"
                    "skipped:\n  61: {L: 1.8, W: 0.6, reason: share_zero}\n", encoding="utf-8")
    variants = carla_fleet.load_variants(path)
    assert set(variants["assignment"]) == {"3"}
    assert set(variants["skipped"]) == {"61"}
    assert variants["path"] == str(path)


# ---------- CARLA の体の配置(計画書 2.4。Vissim・SUMO のブリッジが使う) ----------

MESH_SYNTH = {'a': dict(length=4.0, width=2.0, center_x=0.1, center_y=0.05),
              'b': dict(length=5.0, width=2.5, center_x=-0.3, center_y=0.0)}


def test_body_origin_facing_x():
    (x, y, z), = carla_fleet.body_origins((10.0, 0.0, 1.0), 0.0, 0.0, [dict(blueprint='a', front_offset_m=0.0)],
                                          MESH_SYNTH)
    # 中心は前端の 2.0 m 後ろ。原点は中心からメッシュ中心のずれ(前へ 0.1、右へ 0.05)を戻した所
    assert (x, y, z) == pytest.approx((7.9, -0.05, 1.0))


def test_body_origin_facing_y():
    (x, y, z), = carla_fleet.body_origins((0.0, 10.0, 0.0), 90.0, 0.0, [dict(blueprint='a', front_offset_m=0.0)],
                                          MESH_SYNTH)
    # 前 = +y、右 = -x(CARLA は左手系)
    assert (x, y, z) == pytest.approx((0.05, 7.9, 0.0))


def test_stacked_bodies_follow_their_front_offset():
    actors = [dict(blueprint='b', front_offset_m=0.0), dict(blueprint='b', front_offset_m=3.5)]
    origins = carla_fleet.body_origins((0.0, 0.0, 0.0), 180.0, 0.0, actors, MESH_SYNTH)
    # 前 = -x。体の中心は前端の 2.5 m・6.0 m 後ろ、メッシュ中心が後ろへ 0.3 m ずれているので原点は 0.3 m 前
    assert origins[0] == pytest.approx((2.2, 0.0, 0.0), abs=1e-9)
    assert origins[1] == pytest.approx((5.7, 0.0, 0.0), abs=1e-9)


def test_body_origin_on_a_slope():
    (x, y, z), = carla_fleet.body_origins((10.0, 0.0, 5.0), 0.0, 10.0, [dict(blueprint='a', front_offset_m=0.0)],
                                          MESH_SYNTH)
    # get_carla_transform() と同じく、高さだけ sin(pitch) で変わる
    assert z == pytest.approx(5.0 - math.sin(math.radians(10.0)) * 2.1)
    assert x == pytest.approx(7.9)
