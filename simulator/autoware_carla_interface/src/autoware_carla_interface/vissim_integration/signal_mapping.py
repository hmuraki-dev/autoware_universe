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
"""Vissim の信号グループ -> CARLA の信号の対応表(signal_mapping_*.json)を、起動の引数から選ぶ。CARLA は使わない。

対応表は Town ごとに data/ に置く(signal_mapping_town01.json・signal_mapping_town10hd.json)。
起動の引数 vissim_signal_mapping で選ぶ:

- auto(既定): CARLA のマップ名(map_path のフォルダ名、carla_map)から signal_mapping_<town>.json を選ぶ。
  名前は小文字にし、末尾の _Opt を外す(Town10HD_Opt -> signal_mapping_town10hd.json)。
- ファイル名・相対パス: data/ から探す。絶対パス(~ を含む)はそのまま。

選んだファイルが無ければ ValueError で止める(Vissim に接続する前)。以前は data/signal_mapping.json
の 1 つだけを読んでいて、Town を替えるたびに上書きしていた。
"""

import os

DATA_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data')

AUTO = 'auto'


def auto_file_name(carla_map):
    """CARLA のマップ名 -> data/ の対応表のファイル名(Town10HD_Opt -> signal_mapping_town10hd.json)。"""
    name = os.path.basename(str(carla_map).strip().rstrip('/')).lower()
    if name.endswith('_opt'):
        name = name[:-len('_opt')]
    if not name:
        raise ValueError('vissim_signal_mapping:=auto needs the CARLA map name (carla_map), got '
                         f'{carla_map!r}')
    return f'signal_mapping_{name}.json'


def resolve_signal_mapping_path(value, carla_map, data_dir=DATA_DIR):
    """
    vissim_signal_mapping の値 -> 対応表のパス。

        :param value: auto、ファイル名・相対パス(data_dir から)、または絶対パス。
        :param carla_map: CARLA のマップ名(auto のときだけ使う)。
        :raises ValueError: 選んだファイルが無いとき(試したパスと、data_dir にある対応表を書く)。
    """
    value = str(value).strip()
    if not value or value == AUTO:
        path = os.path.join(data_dir, auto_file_name(carla_map))
        how = f'vissim_signal_mapping:=auto for CARLA map {carla_map!r}'
    else:
        path = os.path.expanduser(value)
        if not os.path.isabs(path):
            path = os.path.join(data_dir, path)
        how = f'vissim_signal_mapping:={value}'
    path = os.path.normpath(path)
    if not os.path.isfile(path):
        try:
            known = sorted(n for n in os.listdir(data_dir)
                           if n.startswith('signal_mapping') and n.endswith('.json'))
        except OSError:
            known = []
        raise ValueError(f'{how}: signal mapping file not found: {path} '
                         f'(signal mappings in {data_dir}: {", ".join(known) or "none"})')
    return path
