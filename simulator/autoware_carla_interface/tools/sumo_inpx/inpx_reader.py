#!/usr/bin/env python3
"""Vissim の .inpx から、SUMO の信号・需要・寸法を作るのに要る部分を読む(tools/sumo_inpx/ の共通部分)。

vissim_to_tll.py(信号)と vissim_to_rou.py(需要・寸法)が使う。CARLA・Vissim・traci は使わない。
SUMO の網との対応づけ(link_to_edge)だけ sumolib を使う($SUMO_HOME/tools を sys.path に入れる)。

座標: Vissim の (x, y) = OpenDRIVE の (x, y)(vissim/README.md)。SUMO の網は OpenDRIVE + netOffset
(netconvert が xodr から作った網。sumo/net/Town10HD.net.xml の location)。地図の変換(transform.json)は要らない。

移植元: SBIR_sim の sumo/vissim_to_rou.py の Vissim クラスと vissim/net.py(inpx の読み取り)。変えたところ:
lanelet2 の地図座標への変換をやめ、SUMO の網の netOffset だけで合わせる。信号制御を複数読み、ステージ方式の
信号プログラム(stageProg)を信号グループごとの色の時刻に展開する(signal_timeline)。
"""

from __future__ import annotations

import base64
import hashlib
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

# Vissim の信号の表示(display の state)
GREEN, AMBER, RED, REDAMBER = 'GREEN', 'AMBER', 'RED', 'REDAMBER'
# link_to_edge の許容
HEADING_TOLERANCE_DEG = 45.0
LATERAL_MARGIN_M = 0.5


class InpxError(Exception):
    pass


def sumo_tools_on_path():
    """$SUMO_HOME/tools を sys.path に入れる(sumolib を import できるように)。"""
    home = os.environ.get('SUMO_HOME', '/usr/share/sumo')
    tools = os.path.join(home, 'tools')
    if not os.path.isdir(tools):
        raise InpxError(f'SUMO_HOME/tools が無い: {tools}(SUMO_HOME を設定する)')
    if tools not in sys.path:
        sys.path.append(tools)


def digest(path) -> str:
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def fmt(value, digits=4) -> str:
    """数値を短く書く(末尾の 0 を落とす)。同じ入力から同じ文字列になる。"""
    text = f'{value:.{digits}f}'.rstrip('0').rstrip('.')
    return text if text not in ('', '-0') else '0'


def _points(element, tag, x='x', y='y'):
    return [(float(p.get(x)), float(p.get(y))) for p in element.iter(tag) if p.get(x) is not None]


def _section(root, tag):
    found = root.find(tag)
    return list(found) if found is not None else []


def _rel_flow(text, decision, route):
    """経路の relFlow「<時間帯の組> <開始>:<割合>」-> 割合。時間帯が 2 つ以上あるときは止める。"""
    if not text:
        return 1.0
    parts = text.split()[1:] or text.split()
    if len(parts) != 1:
        raise InpxError(f'経路決定 {decision} の経路 {route} の割合の時間帯が 1 つでない: {text!r}'
                        '(時間帯ごとの割合は写せない)')
    return float(parts[0].split(':')[-1])


class Inpx:
    """inpx の、リンクの幾何・車種・3D モデル・車両構成・速度分布・加減速の関数・運転挙動・流入・信号。"""

    def __init__(self, path):
        self.path = str(path)
        root = ET.parse(self.path).getroot()

        self.links = {}
        for link in _section(root, 'links'):
            lanes = link.find('lanes')
            self.links[link.get('no')] = dict(
                no=link.get('no'), name=link.get('name') or '',
                pts=_points(link, 'linkPolyPoint'),
                widths=[float(lane.get('width', '3.5')) for lane in (lanes if lanes is not None else [])],
                connector=link.find('fromLinkEndPt') is not None)

        self.models = {}
        for model in root.iter('model2D3D'):
            segments = list(model.iter('model2D3DSegment'))
            if not segments:
                continue
            # 連結車(セグメントが 2 つ以上)は全長の 1 本の箱にする(SUMO の車は 1 本の剛体)
            self.models[model.get('no')] = dict(
                name=model.get('name') or model.get('no'),
                length=sum(float(s.get('length')) for s in segments),
                width=max(float(s.get('width')) for s in segments),
                height=max(float(s.get('height') or 0.0) for s in segments),
                segments=len(segments))

        self.model_distributions = {}
        for distribution in root.iter('model2D3DDistribution'):
            elements = list(distribution.iter('model2D3DDistributionElement'))
            total = sum(float(e.get('share') or 0) for e in elements) or 1.0
            self.model_distributions[distribution.get('no')] = {
                e.get('model2D3D'): float(e.get('share') or 0) / total for e in elements}

        self.vehicle_types = {t.get('no'): dict(t.attrib) for t in _section(root, 'vehicleTypes')}

        self.compositions = {}
        for composition in _section(root, 'vehicleCompositions'):
            self.compositions[composition.get('no')] = dict(
                name=composition.get('name') or composition.get('no'),
                types=[(x.get('vehType'), float(x.get('relFlow')), x.get('desSpeedDistr'))
                       for x in composition.iter('vehicleCompositionRelativeFlow')])

        self.speed = {d.get('no'): [(float(p.get('x')), float(p.get('fx')))
                                    for p in d.iter('speedDistributionDataPoint')]
                      for d in _section(root, 'speedDistributions')}

        self.functions = {}
        for tag in ('maxAccelerationFunctions', 'desAccelerationFunctions',
                    'desDecelerationFunctions', 'maxDecelerationFunctions'):
            self.functions[tag] = {f.get('no'): [(float(p.get('x')), float(p.get('y')))
                                                 for p in list(f.iter())[1:] if p.get('x') is not None]
                                   for f in _section(root, tag)}

        behaviors = {b.get('no'): dict(b.attrib) for b in _section(root, 'drivingBehaviors')}
        link_behaviors = {b.get('no'): b.get('drivBehavDef') for b in _section(root, 'linkBehaviorTypes')}
        used = {link_behaviors.get(link.get('linkBehavType')) for link in _section(root, 'links')}
        if len(used) != 1:
            raise InpxError(f'リンクの運転挙動が 1 つでない: {sorted(map(str, used))}(リンクごとの追従モデルは写せない)')
        self.behavior = behaviors[used.pop()]

        self.inputs = []
        for vi in _section(root, 'vehicleInputs'):
            volumes = list(vi.iter('timeIntervalVehVolume'))
            if len(volumes) != 1:
                raise InpxError(f'流入 {vi.get("no")} の時間帯が 1 つでない(時間帯ごとの流量は写せない)')
            volume = volumes[0]
            if volume.get('volType') != 'STOCHASTIC':
                raise InpxError(f'流入 {vi.get("no")} が STOCHASTIC でない: {volume.get("volType")}')
            self.inputs.append(dict(no=vi.get('no'), link=vi.get('link'), volume=float(volume.get('volume')),
                                    composition=volume.get('vehComp')))

        self.vehicle_classes = {c.get('no'): [e.get('key') for e in c.iter('intObjectRef')]
                                for c in _section(root, 'vehicleClasses')}

        # 静的経路の経路決定: 決定点のリンク -> 行き先リンクまでのリンク列(コネクタを含む)と割合
        self.routing_decisions = []
        for decision in _section(root, 'vehicleRoutingDecisionsStatic'):
            routes = []
            for route in decision.iter('vehicleRouteStatic'):
                routes.append(dict(no=route.get('no'), dest=route.get('destLink'),
                                   dest_pos=float(route.get('destPos')) if route.get('destPos') else None,
                                   share=_rel_flow(route.get('relFlow'), decision.get('no'), route.get('no')),
                                   via=[e.get('key') for e in route.iter('intObjectRef')]))
            self.routing_decisions.append(dict(
                no=decision.get('no'), link=decision.get('link'), pos=float(decision.get('pos') or 0.0),
                all_types=decision.get('allVehTypes', 'true') == 'true', routes=routes))

        # 希望速度決定: 決定点(リンク・車線・位置)-> {車両クラス: 速度分布}
        self.speed_decisions = []
        for decision in _section(root, 'desSpeedDecisions'):
            link, lane = (decision.get('lane') or ' ').split()
            self.speed_decisions.append(dict(
                no=decision.get('no'), link=link, lane=int(lane), pos=float(decision.get('pos') or 0.0),
                per_class={x.get('vehClass'): x.get('desSpeedDistr')
                           for x in decision.iter('vehClassDesSpeedDistribution')}))

        self.pedestrian_inputs = len(_section(root, 'pedestrianInputs'))

        self.controllers = {}
        for controller in root.iter('signalController'):
            supply = controller.get('intSupplyData')
            self.controllers[controller.get('no')] = dict(
                no=controller.get('no'), name=controller.get('name') or '', type=controller.get('type'),
                groups=[g.get('no') for g in controller.iter('signalGroup')],
                supply=base64.b64decode(supply).decode('utf-8') if supply else '')

        self.signal_heads = []
        for head in root.iter('signalHead'):
            controller, group = (head.get('sg') or ' ').split()
            self.signal_heads.append(dict(no=head.get('no'), name=head.get('name') or '',
                                          controller=controller, group=group, lane=head.get('lane')))

    def type_models(self, vehicle_type):
        """車種 -> [(3D モデル, 割合)](割合 0 のモデルを除く。モデル番号の順)。"""
        item = self.vehicle_types[vehicle_type]
        shares = self.model_distributions[item['model2D3DDistr']]
        return [(model, share) for model, share in sorted(shares.items(), key=lambda t: int(t[0])) if share > 0]

    def function_value(self, tag, no, x):
        """区分線形の関数(加速度・減速度)の値。範囲の外は端の値。"""
        points = self.functions[tag][no]
        if x <= points[0][0]:
            return points[0][1]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            if x0 <= x <= x1:
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
        return points[-1][1]


# ---------------------------------------------------------------- 信号プログラム

def signal_timeline(supply_xml):
    """信号制御の intSupplyData(VISSIG の XML)-> (周期 [s], {信号グループ: [(開始 [s], 色), ...]})。

    固定時間の信号プログラムが 1 つであること。次の 2 つの書き方を読む:
    - 信号グループ方式(<prog> の <sg> に色の命令 <cmd>)
    - ステージ方式(<stageProg> の <interstage>。間の色は <interstageProg> の <sg> の命令)
    どちらも、黄(AMBER)は「赤の命令の時刻で終わる」、赤黄(REDAMBER)は「青の命令の時刻で終わる」
    (fixedstate の長さだけ前から)。時刻の並びは 0 から始まり、周期の中で色が変わる時刻だけを持つ。
    """
    root = ET.fromstring(supply_xml)
    states = {d.get('id'): d.get('state') for d in root.iter('display')}
    progs = [p for p in root.iter('prog')]
    stage_progs = [p for p in root.iter('stageProg')]
    if len(progs) + len(stage_progs) != 1:
        raise InpxError(f'信号プログラムが 1 つでない(prog {len(progs)}、stageProg {len(stage_progs)})')

    def commands_of(sg_element, offset_ms):
        """<sg> の命令 -> [(時刻 ms, 色)](黄・赤黄を命令の前に入れる)。"""
        fixed = {states[f.get('display')]: int(f.get('duration'))
                 for f in sg_element.iter('fixedstate')}
        out = []
        for cmd in sg_element.iter('cmd'):
            colour = states[cmd.get('display')]
            begin = offset_ms + int(cmd.get('begin'))
            if colour == RED and fixed.get(AMBER):
                out.append((begin - fixed[AMBER], AMBER))
            if colour == GREEN and fixed.get(REDAMBER):
                out.append((begin - fixed[REDAMBER], REDAMBER))
            out.append((begin, colour))
        return out

    if progs:
        prog = progs[0]
        cycle = int(prog.get('cycletime'))
        commands = {}
        for sg in prog.iter('sg'):
            commands.setdefault(sg.get('sg_id'), []).extend(commands_of(sg, 0))
    else:
        prog = stage_progs[0]
        cycle = int(prog.get('cycletime'))
        stages = {}
        for stage in root.iter('stage'):
            if stage.get('isPseudoStage') == 'true':
                continue
            stages[stage.get('id')] = {a.get('sg_id'): a.get('activation') == 'ON' for a in stage.iter('activation')}
        interstage_progs = {p.get('id'): p for p in root.iter('interstageProg')}
        commands = {}
        for interstage in prog.iter('interstage'):
            ip = interstage_progs[interstage.get('display')]
            begin = int(interstage.get('begin'))
            for sg in ip.iter('sg'):
                commands.setdefault(sg.get('sg_id'), []).extend(commands_of(sg, begin))
            # 移り先のステージの色を、間の終わり(begin + 間の長さ)に置く
            end = begin + int(ip.get('cycletime'))
            for sg_id, on in stages[ip.get('toStage')].items():
                commands.setdefault(sg_id, []).append((end, GREEN if on else RED))
    timeline = {}
    for sg_id, items in commands.items():
        events = {}
        for ms, colour in items:
            events[ms % cycle] = colour   # 同じ時刻は後の命令が勝つ
        ordered = sorted(events.items())
        # 0 の色: 周期の最後の色が続いている
        if ordered[0][0] != 0:
            ordered.insert(0, (0, ordered[-1][1]))
        merged = []
        for ms, colour in ordered:
            if not merged or merged[-1][1] != colour:
                merged.append((ms, colour))
        timeline[sg_id] = [(ms / 1000.0, colour) for ms, colour in merged]
    return cycle / 1000.0, timeline


def colour_at(timeline, t):
    colour = timeline[0][1]
    for start, c in timeline:
        if start <= t:
            colour = c
    return colour


CARLA_HEAD_NAME = re.compile(r'CARLA\s+(\S+)')


def carla_signal_groups(inpx):
    """信号灯の名前「CARLA <信号 ID>」-> {CARLA の信号 ID: (信号制御, 信号グループ)}。

    vissim/Town10HD.inpx の信号灯は tools/vissim/build_vissim_town.py が CARLA の信号の停止位置に置いたもので、
    名前に CARLA の信号 ID を持つ。同じ CARLA の信号の信号灯が別の信号グループにあるときは止める。
    """
    result = {}
    for head in inpx.signal_heads:
        match = CARLA_HEAD_NAME.fullmatch(head['name'].strip())
        if not match:
            raise InpxError(f'信号灯 {head["no"]} の名前 {head["name"]!r} が「CARLA <信号 ID>」でない')
        key = (head['controller'], head['group'])
        previous = result.setdefault(match.group(1), key)
        if previous != key:
            raise InpxError(f'CARLA の信号 {match.group(1)} の信号灯が別の信号グループにある: {previous} と {key}')
    return result


# ---------------------------------------------------------------- SUMO の網との対応

def net_offset(net):
    """sumolib の網 -> netOffset (x, y)。SUMO = OpenDRIVE + netOffset。"""
    return net.getLocationOffset()


def _project(points, x, y):
    """点をポリラインに投影 -> (横のずれ 左が正, 向き deg, 距離)。"""
    best = None
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length == 0.0:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (length * length)))
        px, py = ax + t * dx, ay + t * dy
        dist = math.hypot(x - px, y - py)
        lateral = (dx * (y - ay) - dy * (x - ax)) / length
        if best is None or dist < best[2]:
            best = (lateral, math.degrees(math.atan2(dy, dx)), dist)
    return best


def _midpoint(points):
    """ポリラインの長さの中ほどの点と向き [deg]。"""
    total = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    walked = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if seg > 0 and walked + seg >= total / 2:
            t = (total / 2 - walked) / seg
            return (ax + t * (bx - ax), ay + t * (by - ay)), math.degrees(math.atan2(by - ay, bx - ax))
        walked += seg
    raise InpxError('長さ 0 のポリライン')


def point_at(points, s):
    """ポリラインの始点から s [m] の点(範囲の外は端の点)。"""
    walked = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if seg > 0 and walked + seg >= s:
            t = max(0.0, (s - walked) / seg)
            return (ax + t * (bx - ax), ay + t * (by - ay))
        walked += seg
    return points[-1] if s > 0 else points[0]


def distance_to(points, x, y):
    """点 (x, y) からポリラインまでの距離。"""
    return _project(points, x, y)[2]


def _angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def link_to_edge(inpx, net, link_no):
    """Vissim のリンク -> SUMO の edge(sumolib の Edge)。

    SUMO の edge の、車が走れる車線の中ほどの点が、リンクの幅の中にあり向きが合う edge を選ぶ
    (tools/vissim/build_vissim_town.py の sumo_edge_to_link の逆。流入リンクはその対応で置いたもの)。
    合う edge が無い、または 2 つ以上あるときは止める。
    """
    link = inpx.links[link_no]
    off_x, off_y = net_offset(net)
    half = sum(link['widths']) / 2 + LATERAL_MARGIN_M
    found = []
    for edge in net.getEdges():
        if edge.getFunction() == 'internal':
            continue
        lanes = [lane for lane in edge.getLanes() if lane.allows('passenger')]
        if not lanes:
            continue
        shape = [(x - off_x, y - off_y) for x, y in lanes[0].getShape()]
        (mx, my), heading = _midpoint(shape)
        lateral, link_heading, dist = _project(link['pts'], mx, my)
        if dist <= half and _angle_diff(link_heading, heading) <= HEADING_TOLERANCE_DEG:
            found.append(edge)
    if len(found) != 1:
        raise InpxError(f'リンク {link_no}({link["name"]})に合う SUMO の edge が 1 つでない: '
                        f'{[e.getID() for e in found]}')
    return found[0]
