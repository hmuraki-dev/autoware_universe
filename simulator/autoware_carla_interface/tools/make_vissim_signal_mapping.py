#!/usr/bin/env python3
"""Vissim の .inpx の信号灯から、Vissim ブリッジの信号の対応表(signal_mapping_<map>.json)を作る。

    python3 tools/make_vissim_signal_mapping.py --inpx vissim/Town10HD.inpx \\
        --out src/autoware_carla_interface/vissim_integration/data/signal_mapping_town10hd.json \\
        --out vissim/signal_mapping_town10hd.json

対応表の形は vissim_integration/data/signal_mapping_*.json と同じ:
  {"<信号制御>": {"<信号グループ>": [<CARLA の信号 ID(昇順)>, ...]}}
信号灯の名前「CARLA <CARLA の信号 ID>」と、信号灯の信号グループ(sg="<信号制御> <信号グループ>")から作る。
次のときは止めて、ファイルを書かない:
  - 名前が「CARLA <id>」でない信号灯がある
  - 同じ CARLA の信号の信号灯(車線ごとに 1 つ)が、別の信号グループにある
  - 信号灯の信号制御・信号グループが inpx に無い
  - 「CARLA <id>」の信号灯が 1 つも無い

vissim/Town10HD.inpx の信号灯は、feature/sumo_co-sim の tools/vissim/build_vissim_town.py が CARLA の信号の
停止位置に置いたもの(名前に CARLA の信号 ID を持つ)。そのツールも同じ対応表を書くが、Town10HD.inpx も
作り直す(Vissim で直した設定が消える)。Vissim で直した inpx から対応表だけを作るときは、このツールを使う。
書き方(indent 2・キーの並べ替え・最後の改行)は build_vissim_town.py と同じなので、同じ inpx からは同じバイト列になる。
手順: docs/Vissim(win)-CARLA(0.9.16)-Autoware_co-sim_起動手順.md の 2.4.2。CARLA・Vissim・SUMO は使わない。
"""

import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

CARLA_HEAD_NAME = re.compile(r'CARLA\s+(\S+)')


class MappingError(Exception):
    pass


def read_inpx(path):
    """inpx -> ({信号制御: [信号グループ]}, [(信号灯の番号, 名前, 信号制御, 信号グループ)])。"""
    root = ET.parse(path).getroot()
    controllers = {c.get('no'): [g.get('no') for g in c.iter('signalGroup')]
                   for c in root.iter('signalController')}
    heads = []
    for head in root.iter('signalHead'):
        sg = (head.get('sg') or '').split()
        if len(sg) != 2:
            raise MappingError(f'信号灯 {head.get("no")} の信号グループ sg={head.get("sg")!r} が「<信号制御> <信号グループ>」でない')
        heads.append((head.get('no'), (head.get('name') or '').strip(), sg[0], sg[1]))
    return controllers, heads


def build(controllers, heads):
    """-> {信号制御: {信号グループ: [CARLA の信号 ID(昇順)]}}。"""
    groups_of = {}
    for no, name, controller, group in heads:
        match = CARLA_HEAD_NAME.fullmatch(name)
        if not match:
            raise MappingError(f'信号灯 {no} の名前 {name!r} が「CARLA <信号 ID>」でない')
        carla_id = match.group(1)
        if not carla_id.isdigit():
            raise MappingError(f'信号灯 {no} の名前 {name!r} の CARLA の信号 ID が数でない')
        if controller not in controllers:
            raise MappingError(f'信号灯 {no}(CARLA {carla_id})の信号制御 {controller} が inpx に無い')
        if group not in controllers[controller]:
            raise MappingError(f'信号灯 {no}(CARLA {carla_id})の信号グループ {controller} {group} が信号制御に無い')
        previous = groups_of.setdefault(carla_id, (controller, group))
        if previous != (controller, group):
            raise MappingError(f'CARLA の信号 {carla_id} の信号灯が別の信号グループにある: '
                               f'{" ".join(previous)} と {controller} {group}')
    if not groups_of:
        raise MappingError('名前が「CARLA <信号 ID>」の信号灯が無い')
    mapping = {}
    for carla_id, (controller, group) in groups_of.items():
        mapping.setdefault(controller, {}).setdefault(group, []).append(int(carla_id))
    for groups in mapping.values():
        for ids in groups.values():
            ids.sort()
    return mapping


def render(mapping):
    return json.dumps(mapping, indent=2, sort_keys=True) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--inpx', required=True, help='Vissim の網(例: vissim/Town10HD.inpx)')
    ap.add_argument('--out', required=True, action='append',
                    help='書き出す対応表。2 回以上書けば、同じものをそれぞれに書く')
    args = ap.parse_args(argv)
    try:
        mapping = build(*read_inpx(args.inpx))
    except (MappingError, ET.ParseError, OSError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    text = render(mapping)
    for out in args.out:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        with open(out, 'w', encoding='utf-8') as stream:
            stream.write(text)
    for controller in sorted(mapping, key=int):
        print(f'信号制御 {controller}: ' + '  '.join(
            f'SG {group} -> {",".join(map(str, mapping[controller][group]))}'
            for group in sorted(mapping[controller], key=int)))
    count = sum(len(ids) for groups in mapping.values() for ids in groups.values())
    print(f'信号制御 {len(mapping)}、CARLA の信号 {count} -> {", ".join(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
