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
"""carla_variants(Vissim のモデル -> CARLA の体の並び)を、走行中に引ける形にしたもの。CARLA は使わない。

docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md 2.3・2.5・F5。
移植元 SBIR_sim `vissim/vissim_carla_bridge.py` の FleetPlan から、作り置き(プール)を外したもの。

- 起動時(Vissim に接続する前)に検査し、通らなければ carla_fleet.VariantError で止める:
  生成物が無い / 割り当てたブループリントが CARLA に無い / 実メッシュ・規則が生成時と違う /
  連結車(follows_path)がある / 実メッシュを測った CARLA と接続先の版が違う / 同じ (車種, ファイル名) が
  2 つのモデルにある。
- 走行中は、DSI の (VehicleType, ModelFileName) からモデルを引き、その体の並びを返す。表に無ければ
  npc_model_unknown、割合 0 のモデル(skipped)なら variant_model_empty で止める。黙って別の
  ブループリントに替えない。
"""

import os

from .. import carla_fleet
from ..carla_fleet import VariantError
from .inpx_models import model_key

DATA_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data')

# vissim_carla_variants にこれを渡すと、carla_variants を使わず、今までの vtypes.json と乱数で選ぶ(計画書 2.7)
LEGACY_VTYPES = 'legacy_vtypes'


def resolve_variants_path(value):
    """vissim_carla_variants の値 -> 生成物のパス。相対パス(既定はファイル名だけ)は vissim_integration/data/ から。"""
    return value if os.path.isabs(value) else os.path.join(DATA_DIR, value)


class FleetPlan(object):
    """carla_variants を読み、起動時の検査をして、走行中にモデル -> 体の並びを引く。"""

    def __init__(self, path, blueprint_ids, server_version=None):
        """
            :param path: carla_variants_*.yaml のパス。
            :param blueprint_ids: 接続先の CARLA にあるブループリント名(vehicle.*)。
            :param server_version: 接続先の CARLA の版(client.get_server_version())。
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

        # (車種番号 str, モデルファイルの鍵) -> モデル番号。assignment と skipped の両方から作る
        self._models = {}
        for section in (self.assignment, self.skipped):
            for model, spec in section.items():
                if 'file' not in spec or 'types' not in spec:
                    raise VariantError('variant_model_identity_missing', model=model, variants=self.path)
                for vehicle_type in spec['types']:
                    key = (str(vehicle_type), spec['file'])
                    if self._models.setdefault(key, model) != model:
                        raise VariantError('variant_model_ambiguous', vehicle_type=key[0], file=key[1],
                                           models=sorted([self._models[key], model], key=carla_fleet._model_order),
                                           variants=self.path)
        self.spawn_counts = {}

    def model_for(self, vehicle_type, model_filename):
        """DSI の (VehicleType, ModelFileName) -> モデル番号(str)。表に無ければ止める。"""
        key = (str(vehicle_type), model_key(model_filename))
        model = self._models.get(key)
        if model is None:
            raise VariantError('npc_model_unknown', vehicle_type=key[0], model_file=key[1], variants=self.path)
        if model not in self.assignment:
            # 生成時に割合 0 だったモデルが走行中に出てきた(inpx と carla_variants の食い違い)
            raise VariantError('variant_model_empty', models=[model], during='run', vehicle_type=key[0],
                               model_file=key[1], variants=self.path)
        return model

    def actors_for(self, vehicle_type, model_filename):
        """体の並び [{blueprint, front_offset_m}, ...](前から順)。"""
        return self.assignment[self.model_for(vehicle_type, model_filename)]['actors']

    def spec_for(self, vehicle_type, model_filename):
        """モデルの割り当て全体(L・W・dL・dW・actors など)。"""
        return self.assignment[self.model_for(vehicle_type, model_filename)]

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
