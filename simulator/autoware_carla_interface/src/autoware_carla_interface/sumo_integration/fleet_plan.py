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
"""carla_variants(SUMO のモデル -> CARLA の体の並び)を、走行中に引ける形にしたもの。CARLA・traci は使わない。

docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md 2.3・2.4・S4。
Vissim 構成の vissim_integration/fleet_plan.py と同じ作りで、引き方だけが違う(vType の ID で引く)。

- 起動時(SUMO を起動する前)に検査し、通らなければ carla_fleet.VariantError で止める:
  生成物が無い / 割り当てたブループリントが CARLA に無い / 実メッシュ・規則が生成時と違う /
  連結車(follows_path)がある / 実メッシュを測った CARLA と接続先の版が違う /
  渡された .sumocfg の route-files が生成時と違う(sha256)/ 同じ vType が 2 つのモデルにある。
- 走行中は、SUMO の車の vType ID からモデルを引き、その体の並びを返す。表に無ければ npc_model_unknown、
  割合 0 のモデル(skipped)なら variant_model_empty、車の寸法がモデルと違えば variant_source_changed で止める。
  黙って別のブループリントに替えない。
"""

import os

from .. import carla_fleet
from ..carla_fleet import VariantError
from . import sumo_fleet

DATA_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data')

# sumo_carla_variants にこれを渡すと、carla_variants を使わず、今までの選び方(vType = ブループリント名、
# 無ければ vtypes.json から vClass で乱数)にする
LEGACY_VTYPES = 'legacy_vtypes'
# sumo_carla_variants の既定値: .sumocfg の名前から生成物を探し、あれば使う(計画書 5章の3)
AUTO = 'auto'


def auto_variants_path(sumo_cfg_file):
    """`.../Town01.sumocfg` -> `sumo_integration/data/carla_variants_sumo_town01.yaml`(あるかどうかは見ない)。"""
    stem = os.path.splitext(os.path.basename(str(sumo_cfg_file)))[0]
    return os.path.join(DATA_DIR, f'carla_variants_sumo_{stem.lower()}.yaml')


def resolve_variants_path(value, sumo_cfg_file):
    """sumo_carla_variants の値 -> (生成物のパス or None, 理由の文)。None は今までの選び方。

    - `legacy_vtypes`: None。
    - `auto`: auto_variants_path() があればそれ、無ければ None(Town05・Town10HD など、生成物を作っていない構成)。
    - それ以外: 生成物のパス。相対パス(ファイル名だけなど)は sumo_integration/data/ から。無ければ
      FleetPlan が variant_config_missing で止める。
    """
    if value == LEGACY_VTYPES:
        return None, f'sumo_carla_variants={LEGACY_VTYPES}'
    if value == AUTO:
        path = auto_variants_path(sumo_cfg_file)
        if os.path.isfile(path):
            return path, f'sumo_carla_variants={AUTO}'
        return None, f'sumo_carla_variants={AUTO}: no {os.path.basename(path)} for {sumo_cfg_file}'
    return (value if os.path.isabs(value) else os.path.join(DATA_DIR, value)), 'sumo_carla_variants'


def check_route_files(variants, sumo_cfg_file):
    """渡された .sumocfg の route-files が、生成時と同じ名前の並び・同じ中身(sha256)か。"""
    recorded = (variants['sources'].get('route_files') or {})
    current = {}
    for name, path in sumo_fleet.route_file_names(sumo_cfg_file):
        current[name] = carla_fleet.digest(path) if os.path.isfile(path) else None
    if list(current) != list(recorded) or any(current[name] != recorded[name] for name in recorded):
        raise VariantError('variant_source_changed', source='route_files', sumocfg=str(sumo_cfg_file),
                           recorded={name: str(sha)[:12] for name, sha in recorded.items()},
                           current={name: (sha or 'missing')[:12] for name, sha in current.items()},
                           variants=variants['path'])


class FleetPlan(object):
    """carla_variants を読み、起動時の検査をして、走行中に vType ID -> 体の並びを引く。"""

    def __init__(self, path, blueprint_ids, server_version=None, sumo_cfg_file=None):
        """
            :param path: carla_variants_sumo_*.yaml のパス。
            :param blueprint_ids: 接続先の CARLA にあるブループリント名(vehicle.*)。
            :param server_version: 接続先の CARLA の版(client.get_server_version())。
            :param sumo_cfg_file: これから SUMO に渡す .sumocfg。route-files を生成時と照合する(None なら照合しない)。
            :raises VariantError: 検査に通らないとき。
        """
        self.variants = carla_fleet.load_variants(path)
        self.path = self.variants['path']
        self.name = self.variants.get('name', '')
        self.assignment = self.variants['assignment']
        self.skipped = self.variants['skipped']
        carla_fleet.check_blueprints(self.variants, blueprint_ids)
        carla_fleet.check_sources(self.variants)
        carla_fleet.check_no_follows_path(self.variants)
        self.mesh, meta = carla_fleet.mesh_for(self.variants)
        self.version_note = carla_fleet.check_carla_version(meta, server_version)
        unmeasured = sorted(carla_fleet.wanted_blueprints(self.variants) - set(self.mesh))
        if unmeasured:
            raise VariantError('variant_blueprint_unmeasured', blueprints=unmeasured, variants=self.path)
        if sumo_cfg_file is not None:
            check_route_files(self.variants, sumo_cfg_file)
            models, _ = sumo_fleet.read_models(sumo_fleet.route_files(sumo_cfg_file))
            carla_fleet.check_models(self.variants,
                                     {model: (spec['L'], spec['W']) for model, spec in models.items()},
                                     {model: spec['share'] for model, spec in models.items()})

        # vType ID -> モデル。assignment と skipped の両方から作る
        self._models = {}
        for section in (self.assignment, self.skipped):
            for model, spec in section.items():
                if 'vtypes' not in spec:
                    raise VariantError('variant_model_identity_missing', model=model, variants=self.path)
                for vtype in spec['vtypes']:
                    if self._models.setdefault(str(vtype), model) != model:
                        raise VariantError('variant_model_ambiguous', vtype=str(vtype),
                                           models=sorted([self._models[str(vtype)], model]),
                                           variants=self.path)
        self.spawn_counts = {}

    def model_for(self, vtype, extent=None):
        """SUMO の車の vType ID -> モデル。表に無ければ止める。

            :param extent: その車の半分の寸法(SumoActor.extent)。渡せば、モデルの L・W と合うかも見る。
        """
        model = self._models.get(str(vtype))
        if model is None:
            raise VariantError('npc_model_unknown', vtype=str(vtype), variants=self.path)
        if model not in self.assignment:
            # 生成時に割合 0 だったモデルが走行中に出てきた(.rou.xml と carla_variants の食い違い)
            raise VariantError('variant_model_empty', models=[model], during='run', vtype=str(vtype),
                               variants=self.path)
        if extent is not None:
            spec = self.assignment[model]
            length, width = 2.0 * extent.x, 2.0 * extent.y
            if (abs(length - spec['L']) > carla_fleet.DIMENSION_TOLERANCE_M
                    or abs(width - spec['W']) > carla_fleet.DIMENSION_TOLERANCE_M):
                raise VariantError('variant_source_changed', source='vehicle_dimensions', vtype=str(vtype),
                                   model=model, recorded=[spec['L'], spec['W']],
                                   current=[round(length, 4), round(width, 4)], variants=self.path)
        return model

    def actors_for(self, vtype, extent=None):
        """体の並び [{blueprint, front_offset_m}, ...](前から順)。"""
        return self.assignment[self.model_for(vtype, extent)]['actors']

    def spec_for(self, vtype):
        """モデルの割り当て全体(L・W・dL・dW・actors など)。"""
        return self.assignment[self.model_for(vtype)]

    def body(self, blueprint_id):
        """LiDAR に映る体の {length, width, center_x, center_y}(実メッシュ)。"""
        return self.mesh[blueprint_id]

    def record_spawn(self, blueprint_ids):
        for blueprint_id in blueprint_ids:
            self.spawn_counts[blueprint_id] = self.spawn_counts.get(blueprint_id, 0) + 1

    def summary(self):
        """終了時の要約: ブループリント別の spawn 回数と、生成物から見込んだ比率。"""
        expected = self.variants.get('expected_bodies_per_100_vehicles') or {}
        total_expected = sum(expected.values()) or 1.0
        total = sum(self.spawn_counts.values()) or 1
        names = sorted(set(expected) | set(self.spawn_counts),
                       key=lambda name: (-self.spawn_counts.get(name, 0), name))
        return {name: dict(spawned=self.spawn_counts.get(name, 0),
                           share=self.spawn_counts.get(name, 0) / total,
                           expected_share=expected.get(name, 0.0) / total_expected)
                for name in names}
