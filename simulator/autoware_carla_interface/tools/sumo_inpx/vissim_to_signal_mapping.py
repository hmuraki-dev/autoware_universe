#!/usr/bin/env python3
"""Vissim の .inpx の信号灯から、Vissim ブリッジの信号の対応表(signal_mapping_<map>.json)を作る。

    python3 tools/sumo_inpx/vissim_to_signal_mapping.py \\
        --inpx vissim/Town10HD.inpx --out vissim/signal_mapping_town10hd.json

対応表の形は Vissim ブリッジ(vissim_integration)の data/signal_mapping_*.json と同じ:
  {"<信号制御>": {"<信号グループ>": [<CARLA の信号 ID>, ...]}}
信号灯の名前「CARLA <id>」と、信号灯の信号グループ(sg="<信号制御> <信号グループ>")から作る
(inpx_reader.carla_signal_groups。同じ CARLA の信号の信号灯が別の信号グループにあるときは止める)。
信号灯の信号制御・信号グループが inpx に無いときも止める。

tools/vissim/build_vissim_town.py も同じ形の対応表を書くが、そちらは Town10HD_road.inpx から Town10HD.inpx を
作り直す(Vissim で直した設定が消える)。Vissim で直した Town10HD.inpx から対応表だけを作るときは、このツールを使う。
書き方(indent 2・キーの並べ替え・最後の改行)は build_vissim_town.py と同じ。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inpx_reader import Inpx, InpxError, carla_signal_groups, digest  # noqa: E402


def build(inpx: Inpx) -> dict:
    """inpx -> {信号制御: {信号グループ: [CARLA の信号 ID(昇順)]}}。"""
    mapping = {}
    for carla_id, (controller, group) in carla_signal_groups(inpx).items():
        if controller not in inpx.controllers:
            raise InpxError(f'CARLA の信号 {carla_id} の信号灯の信号制御 {controller} が inpx に無い')
        if group not in inpx.controllers[controller]['groups']:
            raise InpxError(f'CARLA の信号 {carla_id} の信号灯の信号グループ {controller} {group} が信号制御に無い')
        mapping.setdefault(controller, {}).setdefault(group, []).append(int(carla_id))
    if not mapping:
        raise InpxError('名前が「CARLA <信号 ID>」の信号灯が無い')
    for groups in mapping.values():
        for ids in groups.values():
            ids.sort()
    return mapping


def render(mapping: dict) -> str:
    return json.dumps(mapping, indent=2, sort_keys=True) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--inpx', required=True, help='Vissim の網(例: vissim/Town10HD.inpx)')
    ap.add_argument('--out', required=True, help='書き出す対応表(例: vissim/signal_mapping_town10hd.json)')
    args = ap.parse_args(argv)
    try:
        mapping = build(Inpx(args.inpx))
    except InpxError as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as stream:
        stream.write(render(mapping))
    for controller in sorted(mapping, key=int):
        print(f'信号制御 {controller}: ' + '  '.join(
            f'SG {group} -> {",".join(map(str, mapping[controller][group]))}'
            for group in sorted(mapping[controller], key=int)))
    count = sum(len(ids) for groups in mapping.values() for ids in groups.values())
    print(f'{args.out}: 信号制御 {len(mapping)}、CARLA の信号 {count}  sha256 {digest(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
