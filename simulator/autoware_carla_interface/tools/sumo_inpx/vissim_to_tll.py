#!/usr/bin/env python3
"""Vissim の .inpx の信号プログラムを、SUMO の追加ファイル(.tll.xml、新しい信号プログラム)にする。

    python3 tools/sumo_inpx/vissim_to_tll.py \\
        --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \\
        --out sumo/tll/Town10HD_vissim.tll.xml

信号制御と SUMO の信号(tlLogic)の対応は、CARLA の信号 ID でつなぐ:
  Vissim の信号灯の名前「CARLA <id>」-> (信号制御, 信号グループ)
  SUMO の tlLogic の param「linkSignalID:<linkIndex>」= <id>        -> (tlLogic, linkIndex)
1 つの信号制御の CARLA の信号が 2 つ以上の tlLogic にまたがる、tlLogic に信号制御の決まらない linkIndex がある
ときは止める。

信号プログラムは、全信号グループの色が変わる時刻で区切り、区間ごとに 1 つの <phase> にする。
色: 青 G/g、黄 y、赤 r、赤黄 u。青を優先(G)にするか許容(g)にするかは手で決めない。今の網の信号プログラムで、
同じ linkIndex が青のときの文字を写す(netconvert が同じ青の組で決めたもの)。青の組が今のプログラムに無いときは止める。

書き出すのは <tlLogic programID="vissim"> で、今のプログラムの param(linkSignalID)も写す。ブリッジ
(sumo_integration/sumo_simulation.py の SumoTLManager)はプログラムごとの param から CARLA の信号機を引くため。
SUMO は最後に読んだプログラムを使うので、.sumocfg の additional-files に入れれば切り替わる。

出力の先頭に、inpx・網・このツールの sha256 を書く。日時は書かない(同じ入力から同じバイト列)。
docs/SUMO_CARLA_Autoware_inpxからの信号・需要生成_実装計画_v1.0.md の 2.4・S2。
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inpx_reader import AMBER, GREEN, RED, REDAMBER  # noqa: E402
from inpx_reader import Inpx, InpxError, carla_signal_groups, colour_at, digest, fmt, signal_timeline  # noqa: E402

PROGRAM_ID = 'vissim'
SUMO_CHAR = {AMBER: 'y', RED: 'r', REDAMBER: 'u'}


class TllError(Exception):
    pass


def read_net_programs(net_path):
    """網の tlLogic -> {id: dict(phases=[state, ...], params=[(key, value)], links={linkIndex: CARLA の信号 ID})}。"""
    result = {}
    for tl in ET.parse(str(net_path)).getroot().iter('tlLogic'):
        params = [(p.get('key'), p.get('value')) for p in tl.findall('param')]
        links = {int(key.split(':', 1)[1]): value for key, value in params if key.startswith('linkSignalID:')}
        result[tl.get('id')] = dict(phases=[p.get('state') for p in tl.findall('phase')], params=params,
                                    links=links, program=tl.get('programID'))
    return result


def green_chars(phases):
    """今のプログラム -> ({青の linkIndex の組}, {linkIndex: 'G' か 'g'})。同じ linkIndex で G と g が混じるときは止める。"""
    groups, chars = set(), {}
    for state in phases:
        green = frozenset(i for i, c in enumerate(state) if c in 'Gg')
        if green:
            groups.add(green)
        for i in green:
            if chars.setdefault(i, state[i]) != state[i]:
                raise TllError(f'linkIndex {i} が今のプログラムで G と g の両方になる(写す文字が決まらない)')
    return groups, chars


def build(inpx, programs):
    """-> [dict(id, phases=[(duration, state)], params, controller, cycle)](tlLogic の id の順)。"""
    carla = carla_signal_groups(inpx)
    tl_of = {}
    for tl_id, program in programs.items():
        for index, carla_id in program['links'].items():
            tl_of.setdefault(carla_id, set()).add(tl_id)
    out = []
    used = set()
    for controller in sorted(inpx.controllers.values(), key=lambda c: int(c['no'])):
        mine = {cid: group for cid, (sc, group) in carla.items() if sc == controller['no']}
        if not mine:
            raise TllError(f'信号制御 {controller["no"]} に CARLA の信号灯が無い')
        tls = set().union(*(tl_of.get(cid, set()) for cid in mine))
        missing = sorted(cid for cid in mine if cid not in tl_of)
        if missing:
            raise TllError(f'信号制御 {controller["no"]} の CARLA の信号 {missing} が網の linkSignalID に無い')
        if len(tls) != 1:
            raise TllError(f'信号制御 {controller["no"]} の CARLA の信号が 1 つの tlLogic に収まらない: {sorted(tls)}')
        tl_id = tls.pop()
        if tl_id in used:
            raise TllError(f'tlLogic {tl_id} に 2 つ以上の信号制御が対応する')
        used.add(tl_id)
        program = programs[tl_id]
        size = len(program['phases'][0])
        if sorted(program['links']) != list(range(size)):
            raise TllError(f'tlLogic {tl_id} に linkSignalID の無い linkIndex がある')
        unmapped = sorted({cid for cid in program['links'].values()} - set(mine))
        if unmapped:
            raise TllError(f'tlLogic {tl_id} の CARLA の信号 {unmapped} に Vissim の信号灯が無い')

        cycle, timeline = signal_timeline(controller['supply'])
        for group in set(mine.values()):
            if group not in timeline:
                raise TllError(f'信号制御 {controller["no"]} の信号グループ {group} が信号プログラムに無い')
        existing_greens, chars = green_chars(program['phases'])
        cuts = sorted({start for group in set(mine.values()) for start, _ in timeline[group]} | {0.0})
        phases = []
        for begin, end in zip(cuts, cuts[1:] + [cycle]):
            if end <= begin:
                continue
            state = []
            for index in range(size):
                colour = colour_at(timeline[mine[program['links'][index]]], begin)
                state.append(chars.get(index, 'G') if colour == GREEN else SUMO_CHAR[colour])
            green = frozenset(i for i, c in enumerate(state) if c in 'Gg')
            if green and green not in existing_greens:
                raise TllError(f'tlLogic {tl_id} の {begin:g}-{end:g} s の青の組 {sorted(green)} が今のプログラムに無い'
                               '(G/g を写せない)')
            phases.append((end - begin, ''.join(state)))
        out.append(dict(id=tl_id, phases=phases, params=program['params'], controller=controller, cycle=cycle))
    leftover = sorted(set(programs) - used)
    return out, leftover


def render(inpx_path, net_path, logics, leftover):
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<!-- tools/sumo_inpx/vissim_to_tll.py が Vissim の信号プログラムから作ったもの(手で編集しない)。',
        f'     Vissim: {os.path.basename(inpx_path)} sha256 {digest(inpx_path)}',
        f'     網:     {os.path.basename(net_path)} sha256 {digest(net_path)}',
        f'     ツール: vissim_to_tll.py sha256 {digest(os.path.abspath(__file__))}',
        f'     信号制御 {len(logics)} 個を programID="{PROGRAM_ID}" の信号プログラムにする(網の今のプログラムの代わりに使われる)。',
        '     青の G/g は網の今のプログラムから写す。param の linkSignalID は網の今のプログラムと同じ。',
    ]
    for logic in logics:
        lines.append(f'     tlLogic {logic["id"]} <- 信号制御 {logic["controller"]["no"]}'
                     f' ({logic["controller"]["name"]})、周期 {fmt(logic["cycle"])} s')
    if leftover:
        lines.append(f'     信号制御の無い tlLogic(今のプログラムのまま): {", ".join(leftover)}')
    lines[-1] += ' -->'
    lines.append('<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                 'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">')
    for logic in logics:
        lines.append(f'    <tlLogic id={quoteattr(logic["id"])} type="static" programID="{PROGRAM_ID}" offset="0">')
        for duration, state in logic['phases']:
            lines.append(f'        <phase duration="{fmt(duration)}" state="{state}"/>')
        for key, value in logic['params']:
            lines.append(f'        <param key={quoteattr(key)} value={quoteattr(value)}/>')
        lines.append('    </tlLogic>')
    lines.append('</additional>')
    return '\n'.join(lines) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--inpx', required=True, help='Vissim の網(例: vissim/Town10HD.inpx)')
    ap.add_argument('--net', required=True, help='SUMO の網(例: sumo/net/Town10HD.net.xml)')
    ap.add_argument('--out', required=True, help='書き出す .tll.xml(例: sumo/tll/Town10HD_vissim.tll.xml)')
    args = ap.parse_args(argv)
    try:
        logics, leftover = build(Inpx(args.inpx), read_net_programs(args.net))
    except (InpxError, TllError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    text = render(args.inpx, args.net, logics, leftover)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as stream:
        stream.write(text)
    for logic in logics:
        print(f'tlLogic {logic["id"]:>4} <- 信号制御 {logic["controller"]["no"]}: '
              + ' '.join(f'{fmt(d)}:{s}' for d, s in logic['phases']))
    if leftover:
        print(f'信号制御の無い tlLogic(今のプログラムのまま): {", ".join(leftover)}')
    print(f'{args.out} sha256 {digest(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
