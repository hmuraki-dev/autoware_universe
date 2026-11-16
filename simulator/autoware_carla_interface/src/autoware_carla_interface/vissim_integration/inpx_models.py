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
"""Vissim の inpx から、車両の 2D/3D モデル(寸法・ファイル名)と出現割合を読む。CARLA は使わない。

ブループリント割り当ての生成ツール(tools/carla_fleet/build_carla_variants.py)が使う。
ブリッジは実行時に inpx を読まない(inpx は Windows 側にある)。代わりに、生成ツールがここで読んだ
モデルのファイル名(model_key)と車種を生成物に書き、ブリッジはそれで DSI の (VehicleType, ModelFileName)
からモデル番号を引く(docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md 2.3)。

移植元: SBIR_sim `vissim/net.py`(3ea1650)の model_key・model_body・Network._load_vehicle_models・
Network.model_shares。車線・リンクなど、モデル以外の処理は持ってきていない。
"""

import xml.etree.ElementTree as ET

_MODEL_EXTENSIONS = ('.u3dm', '.fbx', '.v3d')


def model_key(filename):
    """モデルファイル名 -> 照合に使う鍵(パスと拡張子を除いた名前)。

    inpx の file3D(例 '#3dmodels#Vehicles\\Road\\Car - Fiat 500 (2018).u3dm')と、DSI の ModelFileName
    (例 b'Car - Fiat 500 (2018)'。2026-11-16 の F0 で確かめた形)のどちらも同じ鍵になる。bytes は UTF-8 で読む。
    """
    if isinstance(filename, (bytes, bytearray)):
        filename = bytes(filename).decode('utf-8', 'replace')
    name = (filename or '').replace('\\', '/').rsplit('/', 1)[-1]
    for extension in _MODEL_EXTENSIONS:
        if name.lower().endswith(extension):
            return name[:-len(extension)]
    return name


def model_body(segments):
    """2D/3D モデルの **車体全体** の (length, width, height) [m]。

    連結車は複数セグメントを持つ。セグメント i+1 は自分の jointFront をセグメント i の jointRear に合わせて
    繋がるので、i+1 の前端は offset[i] + jointRear(i) - jointFront(i+1) に来る。先頭セグメントだけを見ると、
    連結車の全長を牽引車の長さとして扱ってしまうので、必ず全体で取る。
    """
    def number(segment, key):
        try:
            return float(segment.get(key) or 0)
        except (TypeError, ValueError):
            return 0.0
    offset = total = 0.0
    for index, segment in enumerate(segments):
        if index:
            offset += number(segments[index-1], 'jointRear') - number(segment, 'jointFront')
        total = offset + number(segment, 'length')
    return (total, max(number(s, 'width') for s in segments),
            max(number(s, 'height') for s in segments))


class InpxModels(object):
    """inpx の車両モデルの表。

    models:          モデル番号 -> {file, name, length, width, height, segments}
    vehicle_types:   車種番号 -> {category, name, models, model_shares}
    compositions:    車両構成番号 -> {車種番号: 相対流量(合計 1)}
    active_compositions: 需要(timeIntervalVehVolume)が使う車両構成
    driving_sim_vehicle_type: 運転シミュレータ(自車)の車種番号(netPara の drivSimVehType)
    """

    def __init__(self, root):
        parameters = root.find('netPara')
        self.driving_sim_vehicle_type = (parameters.get('drivSimVehType')
                                         if parameters is not None else None)
        self.models = {}
        for model in root.iter('model2D3D'):
            segments = list(model.iter('model2D3DSegment'))
            if not segments:
                continue
            # DSI が返す ModelFileName は先頭セグメントのファイル名なので、鍵は先頭で取る。
            # 寸法は車体全体で取る(model_body)。
            length, width, height = model_body(segments)
            self.models[model.get('no')] = dict(
                file=model_key(segments[0].get('file3D')), name=model.get('name') or '',
                length=length, width=width, height=height, segments=len(segments))
        distributions = {}
        for distribution in root.iter('model2D3DDistribution'):
            elements = list(distribution.iter('model2D3DDistributionElement'))
            total = sum(float(e.get('share') or 0) for e in elements) or 1.0
            distributions[distribution.get('no')] = {
                e.get('model2D3D'): float(e.get('share') or 0)/total for e in elements}
        self.vehicle_types = {}
        for vehicle_type in root.iter('vehicleType'):
            shares = distributions.get(vehicle_type.get('model2D3DDistr'), {})
            self.vehicle_types[vehicle_type.get('no')] = dict(
                category=vehicle_type.get('category') or '', name=vehicle_type.get('name') or '',
                models=list(shares), model_shares=shares)
        active = {volume.get('vehComp') for volume in root.iter('timeIntervalVehVolume')}
        self.compositions = {}
        for composition in root.iter('vehicleComposition'):
            rows = list(composition.iter('vehicleCompositionRelativeFlow'))
            total = sum(float(r.get('relFlow') or 0) for r in rows) or 1.0
            self.compositions[composition.get('no')] = {
                r.get('vehType'): float(r.get('relFlow') or 0)/total for r in rows}
        self.active_compositions = sorted(active & set(self.compositions))

    @classmethod
    def from_file(cls, path):
        return cls(ET.parse(str(path)).getroot())

    def is_traffic_vehicle_type(self, vehicle_type):
        """CARLA に体を出す車種か(歩行者の車種と、自車の車種は出さない)。"""
        item = self.vehicle_types.get(str(vehicle_type))
        return (item is not None and item['category'] != 'PEDESTRIAN'
                and str(vehicle_type) != self.driving_sim_vehicle_type)

    def model_shares(self):
        """モデル -> 網全体での出現割合(合計 1)。

        inpx にはモデル単位の割合が直接は無く、「車両構成の車種の割合」と「車種ごとのモデル分布」の
        2 段で書かれているので、掛け合わせ、需要が使う車両構成で平均する。
        歩行者の車種と自車の車種は入れない。割合 0 のモデルも 0 として入れる。
        """
        compositions = self.active_compositions or sorted(self.compositions)
        shares = {}
        for vehicle_type, item in self.vehicle_types.items():
            if not self.is_traffic_vehicle_type(vehicle_type):
                continue
            flow = sum(self.compositions[number].get(vehicle_type, 0.0)
                       for number in compositions) / max(1, len(compositions))
            for model, share in item['model_shares'].items():
                if model in self.models:
                    shares[model] = shares.get(model, 0.0) + flow*share
        total = sum(shares.values())
        return {model: value/total for model, value in shares.items()} if total > 0 else shares
