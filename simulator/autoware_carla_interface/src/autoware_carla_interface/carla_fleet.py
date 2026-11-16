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
"""CARLA のブループリント割り当て(carla_variants)。交通シムに依存しない共通部品。

Vissim 構成と SUMO 構成のブリッジ、生成ツール(tools/carla_fleet/build_carla_variants.py)が使う。
ここにあるのは、モデルの寸法(L・W・セグメント数・車種カテゴリ)と規則・実メッシュ寸法だけで
決まる処理: 割り当ての規則の適用、carla_variants の読み込みと起動時の検査、体数の見積もり。
モデルの寸法と割合を交通シムの入力(Vissim の inpx、SUMO の .rou.xml)から読むのは、各ブリッジ・生成ツールの側。

移植元: SBIR_sim `runtime/bridge/carla_fleet.py`(feat/carla-substep-10ms、3ea1650)。
変えたところ(docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md 2.2・2.5):
  - 生成元(sources.*.path)の相対パスは、リポジトリ直下ではなく carla_variants のあるディレクトリから
    解決する。パッケージをインストールした先でも、src/ でも同じに読めるようにするため。
  - 連結車(follows_path)を扱わないブリッジのための検査 check_no_follows_path() を足した。

設計: 移植元 docs/CARLA_blueprint候補拡張_設計案.md(§4 規則、§5 ファイル、§6 検査)
"""
from __future__ import annotations

import fnmatch
import hashlib
import json
import math
import os
from pathlib import Path

# carla_variants の L・W と、交通シム側のモデル寸法を比べる許容 [m](ファイルは小数 3 桁で書く)
DIMENSION_TOLERANCE_M = 0.0015


class VariantError(RuntimeError):
    """割り当てを作れない・使えない。code は止める理由の鍵(計画書 2.5)。

    ブリッジはこの code と evidence をログに出して止める。黙って別のブループリントに替えない。
    """

    def __init__(self, code, **evidence):
        self.code = code
        self.evidence = evidence
        super().__init__(f"{code}: {evidence}")


def digest(path) -> str:
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def resolve_source(path, base_dir) -> Path:
    """carla_variants に書いたパスを絶対パスにする。相対パスは base_dir(carla_variants のあるディレクトリ)から。"""
    path = Path(path)
    return path if path.is_absolute() else Path(base_dir) / path


def relative_source(path, base_dir) -> str:
    """carla_variants に書くパス: base_dir(carla_variants を書くディレクトリ)からの相対パス。"""
    return os.path.relpath(os.path.abspath(path), os.path.abspath(base_dir))


def variants_dir(variants) -> Path:
    """carla_variants のあるディレクトリ(sources.*.path の基準)。"""
    return Path(variants["path"]).resolve().parent


# ---------- 入力 ----------

def load_rules(path) -> dict:
    import yaml
    rules = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    missing = [key for key in ("length_tolerance_m", "width_over_tolerance_m", "width_under_tolerance_m",
                               "select", "fallback", "excluded") if key not in rules]
    if missing:
        raise VariantError("variant_rules_invalid", path=str(path), missing=missing)
    if rules["select"] != "nearest_length" or rules["fallback"].get("kind") != "stack":
        raise VariantError("variant_rules_invalid", path=str(path),
                           select=rules["select"], fallback=rules["fallback"])
    return rules


def load_mesh(path) -> tuple[dict, dict]:
    """measure_blueprint_mesh.py の出力 -> ({blueprint: {length, width, center_x, center_y}}, 付帯情報)。

    幅と長さが測れていないブループリントは入れない(候補にならない)。
    """
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = raw["measurements"] if isinstance(raw, dict) else raw
    meta = {key: value for key, value in raw.items() if key != "measurements"} if isinstance(raw, dict) else {}
    mesh = {row["blueprint"]: dict(length=row["mesh_length_m"], width=row["mesh_width_m"],
                                   center_x=row.get("mesh_center_x_m", 0.0),
                                   center_y=row.get("mesh_center_y_m", 0.0))
            for row in rows if "mesh_width_m" in row and "mesh_length_m" in row}
    return mesh, meta


# ---------- 規則(設計案 §4.1) ----------

def width_under_tolerance(rules, categories) -> float:
    """細い側の許容。複数のカテゴリが使うモデルは最も厳しい値。"""
    table = rules["width_under_tolerance_m"]
    unknown = sorted(set(categories) - set(table))
    if not categories or unknown:
        raise VariantError("variant_category_unknown", categories=sorted(categories), unknown=unknown)
    return min(float(table[category]) for category in categories)


def excluded(rules, blueprint) -> bool:
    return any(fnmatch.fnmatchcase(blueprint, pattern) for pattern in rules["excluded"])


def assign(model, rules, mesh):
    """モデル 1 つの割り当て。作れなければ None。

    model: {L, W, segments, categories}。返す dict: method、dL、dW、actors(front_offset_m の順)。
    """
    length, width = float(model["L"]), float(model["W"])
    under = width_under_tolerance(rules, model["categories"])
    over = float(rules["width_over_tolerance_m"])

    def width_fits(body_width):
        return -under - 1e-9 <= body_width - width <= over + 1e-9

    candidates = []
    for name, body in mesh.items():
        if excluded(rules, name):
            continue
        d_length, d_width = body["length"] - length, body["width"] - width
        if abs(d_length) <= float(rules["length_tolerance_m"]) + 1e-9 and width_fits(body["width"]):
            candidates.append((abs(d_length), abs(d_width), name, d_length, d_width))
    if candidates:
        _, _, name, d_length, d_width = min(candidates)
        return dict(method="nearest", dL=d_length, dW=d_width,
                    actors=[dict(blueprint=name, front_offset_m=0.0)])

    fallback = rules["fallback"]["blueprint"]
    body = mesh.get(fallback)
    if body is None:
        raise VariantError("variant_fallback_unmeasured", blueprint=fallback)
    if length <= body["length"] or not width_fits(body["width"]):
        return None
    count = math.ceil(length / body["length"])
    pitch = (length - body["length"]) / (count - 1)
    articulated = int(model.get("segments", 1)) >= 2
    actors = []
    for index in range(count):
        actor = dict(blueprint=fallback, front_offset_m=index * pitch)
        if articulated and index:
            actor["follows_path"] = True
        actors.append(actor)
    outer = actors[-1]["front_offset_m"] + body["length"]
    return dict(method="fallback_stack", dL=outer - length, dW=body["width"] - width, actors=actors)


# ---------- 体数の見積もり(設計案 §6.1) ----------

def blueprint_demand(shares, assignment) -> dict:
    """{モデル: 割合(交通シムの定義から読んだもの)} -> {blueprint: 重み}。重ね置きのモデルは体数の分だけ数える。"""
    demand = {}
    for model, share in shares.items():
        for actor in assignment[str(model)]["actors"]:
            demand[actor["blueprint"]] = demand.get(actor["blueprint"], 0.0) + share
    return demand


def pool_sizes(demand, pool_max, pool_min=1) -> dict:
    """{blueprint: 重み} -> {blueprint: 作り置きの体数}。max(pool_min, round(pool_max × 重み / 合計))。"""
    total = sum(demand.values()) or 1.0
    return {name: max(int(pool_min), int(round(pool_max * weight / total)))
            for name, weight in sorted(demand.items(), key=lambda item: (-item[1], item[0]))}


# ---------- carla_variants の読み込みと起動時の検査(設計案 §6、計画書 2.5) ----------

def load_variants(path) -> dict:
    import yaml
    path = Path(path)
    if not path.is_file():
        raise VariantError("variant_config_missing", path=str(path))
    variants = yaml.safe_load(path.read_text(encoding="utf-8"))
    variants["assignment"] = {str(model): spec for model, spec in (variants.get("assignment") or {}).items()}
    variants["skipped"] = {str(model): spec for model, spec in (variants.get("skipped") or {}).items()}
    variants["path"] = str(path)
    return variants


def wanted_blueprints(variants) -> set:
    return {actor["blueprint"] for spec in variants["assignment"].values() for actor in spec["actors"]}


def check_blueprints(variants, available) -> None:
    """検査 1: 割り当てたブループリントが CARLA に全部あるか。"""
    missing = sorted(wanted_blueprints(variants) - set(available))
    if missing:
        raise VariantError("variant_blueprint_missing", blueprints=missing, variants=variants["path"])


def check_sources(variants) -> None:
    """検査 2: 実メッシュ・規則が生成時と同じファイルか(carla_variants が古くないか)。"""
    base = variants_dir(variants)
    for key in ("geometry", "rules"):
        source = variants["sources"][key]
        path = resolve_source(source["path"], base)
        current = digest(path) if path.is_file() else None
        if current != source["sha256"]:
            raise VariantError("variant_source_changed", source=key, path=str(path),
                               recorded_sha256=source["sha256"], current_sha256=current,
                               variants=variants["path"])


def check_models(variants, models, shares) -> None:
    """検査 3・4: モデルの L・W が生成時と同じか。交通シムが出しうるモデル(割合 > 0)に割り当てがあるか。

    models: {モデル: (L, W)}(交通シム側で今読んだ寸法)。shares: {モデル: 交通シムの定義から読んだ割合}。
    """
    for model, spec in variants["assignment"].items():
        if model not in models:
            continue
        length, width = models[model]
        if abs(length - spec["L"]) > DIMENSION_TOLERANCE_M or abs(width - spec["W"]) > DIMENSION_TOLERANCE_M:
            raise VariantError("variant_source_changed", source="model_dimensions", model=model,
                               recorded=[spec["L"], spec["W"]], current=[length, width],
                               variants=variants["path"])
    empty = sorted((str(model) for model, share in shares.items()
                    if share > 0 and str(model) not in variants["assignment"]), key=_model_order)
    if empty:
        raise VariantError("variant_model_empty", models=empty, variants=variants["path"])


def check_no_follows_path(variants) -> None:
    """連結車(軌跡に沿わせる体)を扱わないブリッジ向けの検査(計画書 0.2・2.5)。"""
    articulated = sorted((model for model, spec in variants["assignment"].items()
                          if any(actor.get("follows_path") for actor in spec["actors"])), key=_model_order)
    if articulated:
        raise VariantError("variant_articulated_unsupported", models=articulated, variants=variants["path"])


def check_carla_version(meta, server_version):
    """実メッシュを測った CARLA と接続先が同じ版か(設計案 §6.2)。

    違えば止める(ブループリントの形が変わっているかもしれない。測り直す)。測った版が記録されていない
    実メッシュは確かめられないので、注意の文を返す。
    """
    measured = meta.get("carla_server_version")
    if measured is None:
        return "実メッシュに CARLA の版の記録が無い(測り直せば記録される)。版の照合はしない"
    if server_version is not None and measured != server_version:
        raise VariantError("variant_mesh_carla_version_changed", measured=measured, server=server_version)
    return None


def mesh_for(variants) -> tuple[dict, dict]:
    """carla_variants が生成に使った実メッシュ(sources.geometry)を読む。"""
    return load_mesh(resolve_source(variants["sources"]["geometry"]["path"], variants_dir(variants)))


def _model_order(model):
    text = str(model)
    return (0, int(text), "") if text.isdigit() else (1, 0, text)
