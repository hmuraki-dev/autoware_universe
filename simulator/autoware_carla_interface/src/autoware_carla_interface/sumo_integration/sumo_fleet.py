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
"""SUMO の .sumocfg・.rou.xml から、車両のモデル(寸法でまとめた vType)と出現割合を読む。traci・CARLA は使わない。

ブループリント割り当ての生成ツール(tools/carla_fleet/build_carla_variants.py の --sumocfg)と、SUMO ブリッジの
起動時の検査が使う(docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md 2.3・S3)。

- モデル = vType を**寸法(L・W)でまとめたもの**(同じ寸法 = 同じモデル)。名前は `L<長さ>_W<幅>`。
  SUMO は 3D モデルを持たないので、寸法がモデルを決める。
- 寸法: vType の length・width。無ければ vClass の SUMO の既定値(dimension_defaulted に残す)。
- 出現割合: flow・vehicle・trip の台数 × vTypeDistribution の確率。.rou.xml の定義を読んだもので、交通には書き戻さない。
- 車種カテゴリ(細い側の許容に使う): vClass から。vClass が乗用車でも長さが HEAVY_LENGTH_M を超える vType は HGV。
- SUMO の車は 1 本の剛体なので、セグメントは 1。
- 歩行者(vClass pedestrian)の vType はモデルにしない。

移植元: SBIR_sim `runtime/bridge/sumo_fleet.py`(3ea1650)の read_models。変えたところ: .sumocfg の route-files の
複数のファイルにまたがって読む(vType・vTypeDistribution と flow が別のファイルにあってよい)、モデルごとに vType の ID と
vTypeDistribution の ID を返す、歩行者を除く。
"""

import os
import re
import xml.etree.ElementTree as ET

from ..carla_fleet import VariantError

# vType に length・width が無いときの SUMO の既定値(vClass ごと。SUMO の Vehicle Type Parameter Defaults)
VCLASS_DEFAULT_SIZE = {
    'passenger': (5.0, 1.8), 'private': (5.0, 1.8), 'taxi': (5.0, 1.8), 'evehicle': (5.0, 1.8),
    'authority': (5.0, 1.8), 'emergency': (6.5, 2.16), 'army': (5.0, 1.8),
    'truck': (7.1, 2.4), 'trailer': (16.5, 2.55), 'delivery': (6.5, 2.16),
    'bus': (12.0, 2.5), 'coach': (14.0, 2.6),
    'motorcycle': (2.2, 0.9), 'moped': (2.1, 0.8), 'bicycle': (1.6, 0.65), 'scooter': (1.2, 0.5),
}
VCLASS_CATEGORY = {
    'passenger': 'CAR', 'private': 'CAR', 'taxi': 'CAR', 'evehicle': 'CAR', 'authority': 'CAR',
    'army': 'CAR', 'emergency': 'HGV',
    'truck': 'HGV', 'trailer': 'HGV', 'delivery': 'HGV',
    'bus': 'BUS', 'coach': 'BUS',
    'motorcycle': 'BIKE', 'moped': 'BIKE', 'bicycle': 'BIKE', 'scooter': 'BIKE',
}
# vClass が乗用車でもこれより長い vType は大型車(HGV)の許容で見る [m]
HEAVY_LENGTH_M = 6.0
DEFAULT_VTYPE = 'DEFAULT_VEHTYPE'
NOT_VEHICLES = ('pedestrian',)


def model_key(length, width):
    """寸法でまとめたモデルの名前(carla_variants の assignment の鍵)。"""
    return f'L{float(length):.3f}_W{float(width):.3f}'


def route_file_names(sumocfg):
    """.sumocfg の input/route-files -> [(書かれた名前, 絶対パス), ...](.sumocfg のあるディレクトリから解決する)。"""
    root = ET.parse(str(sumocfg)).getroot()
    element = root.find('./input/route-files')
    if element is None or not element.get('value'):
        raise VariantError('variant_rou_unreadable', sumocfg=str(sumocfg), reason='route-files が無い')
    base = os.path.dirname(os.path.abspath(str(sumocfg)))
    return [(name.strip(), os.path.normpath(os.path.join(base, name.strip())))
            for name in element.get('value').split(',') if name.strip()]


def route_files(sumocfg):
    """.sumocfg の input/route-files -> 絶対パスの並び。"""
    return [path for _, path in route_file_names(sumocfg)]


def _flow_count(flow):
    """flow の台数(割合の重み)。number があればそれ、無ければ到着率 × 時間。"""
    if flow.get('number') is not None:
        return float(flow.get('number'))
    begin = float(flow.get('begin', 0.0))
    end = float(flow.get('end', 86400.0))
    duration = max(0.0, end - begin)
    if flow.get('vehsPerHour') is not None:
        return float(flow.get('vehsPerHour')) / 3600.0 * duration
    if flow.get('perHour') is not None:
        return float(flow.get('perHour')) / 3600.0 * duration
    if flow.get('probability') is not None:
        return float(flow.get('probability')) * duration
    period = flow.get('period')
    if period is not None:
        match = re.fullmatch(r'\s*exp\(\s*([0-9.eE+-]+)\s*\)\s*', period)
        rate = float(match.group(1)) if match else 1.0 / float(period)
        return rate * duration
    raise VariantError('variant_rou_unreadable', flow=flow.get('id'), reason='flow の台数を決める属性が無い')


def read_models(files):
    """route ファイルの並び -> ({モデル: {L, W, segments, name, categories, vtypes, distributions, share,
    dimension_defaulted}}, {vType ID: モデル})。share は合計 1(歩行者を除く車の台数で割る)。
    """
    roots = [ET.parse(str(path)).getroot() for path in files]
    vtypes = {}
    for root in roots:
        for element in root.iter('vType'):
            vclass = element.get('vClass', 'passenger')
            default = VCLASS_DEFAULT_SIZE.get(vclass, VCLASS_DEFAULT_SIZE['passenger'])
            vtypes[element.get('id')] = dict(
                L=float(element.get('length', default[0])), W=float(element.get('width', default[1])),
                vclass=vclass, probability=element.get('probability'),
                defaulted=element.get('length') is None or element.get('width') is None)
    vtypes.setdefault(DEFAULT_VTYPE, dict(L=5.0, W=1.8, vclass='passenger', probability=None, defaulted=True))

    distributions = {}
    for root in roots:
        for element in root.iter('vTypeDistribution'):
            members = {}
            names = (element.get('vTypes') or '').split()
            probabilities = (element.get('probabilities') or '').split()
            for index, name in enumerate(names):
                if index < len(probabilities):
                    weight = float(probabilities[index])
                else:
                    weight = float((vtypes.get(name) or {}).get('probability') or 1.0)
                members[name] = members.get(name, 0.0) + weight
            for child in element.findall('vType'):
                members[child.get('id')] = members.get(child.get('id'), 0.0) + float(child.get('probability', 1.0))
            total = sum(members.values()) or 1.0
            distributions[element.get('id')] = {name: weight / total for name, weight in members.items()}

    weights, groups = {}, {}

    def count(type_id, n):
        type_id = type_id or DEFAULT_VTYPE
        members = distributions.get(type_id, {type_id: 1.0})
        for name, p in members.items():
            if name not in vtypes:
                raise VariantError('variant_rou_unreadable', vtype=name, reason='定義の無い vType')
            weights[name] = weights.get(name, 0.0) + n * p
            if type_id in distributions:
                groups.setdefault(name, set()).add(type_id)

    for root in roots:
        for flow in root.iter('flow'):
            count(flow.get('type'), _flow_count(flow))
        for tag in ('vehicle', 'trip'):
            for vehicle in root.iter(tag):
                count(vehicle.get('type'), 1.0)
    total = sum(w for name, w in weights.items() if vtypes[name]['vclass'] not in NOT_VEHICLES)
    if total <= 0:
        raise VariantError('variant_no_active_models')

    models, model_of = {}, {}
    for name, spec in vtypes.items():
        if spec['vclass'] in NOT_VEHICLES or name in distributions:
            continue
        key = model_key(spec['L'], spec['W'])
        model_of[name] = key
        category = VCLASS_CATEGORY.get(spec['vclass'], 'CAR')
        if category == 'CAR' and spec['L'] > HEAVY_LENGTH_M:
            category = 'HGV'
        # L・W は丸めない(割り当ての dL・dW を、同じ寸法の Vissim のモデルと同じにするため)。まとめる鍵だけ小数 3 桁
        entry = models.setdefault(key, dict(L=spec['L'], W=spec['W'], segments=1,
                                            categories=set(), vtypes=[], distributions=set(), share=0.0,
                                            dimension_defaulted=False))
        entry['categories'].add(category)
        entry['vtypes'].append(name)
        entry['distributions'] |= groups.get(name, set())
        entry['share'] += weights.get(name, 0.0) / total
        entry['dimension_defaulted'] |= spec['defaulted']
    for entry in models.values():
        entry['vtypes'].sort()
        entry['name'] = (f"{'/'.join(sorted(entry['categories']))} {entry['L']:.3f}x{entry['W']:.3f} m"
                         f" ({', '.join(entry['vtypes'])})")
    return models, model_of
