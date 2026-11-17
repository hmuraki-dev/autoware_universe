#!/usr/bin/env python3
"""Vissim で OpenDRIVE を取り込んだ道路網(.inpx)に、信号・交通需要・車両設定を足して co-sim 用の .inpx を作る。

    python3 tools/vissim/build_vissim_town.py \\
        --road vissim/Town10HD_road.inpx \\
        --template ~/CARLA/Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx \\
        --sumo-net sumo/net/Town10HD.net.xml --sumo-rou sumo/rou/Town10HD.rou.xml \\
        --carla-json ~/autoware_map/Town10HD/carla_tl.json \\
        --out vissim/Town10HD.inpx --signal-mapping-out vissim/signal_mapping_town10hd.json

座標: Vissim (x, y) = OpenDRIVE (x, y) = CARLA (x, -y)。SUMO は OpenDRIVE + netOffset。

足すもの:
- 車両設定: --template(Town01.inpx)の車両タイプ・2D/3D モデル・車両構成・歩行者タイプ・走行挙動などと、
  netPara のドライブシミュレータの設定(drivSimActive・drivSimVehType・drivSimPedType)、simulation。
- 信号制御: SUMO の信号(tlLogic)1 つにつき 1 つ。中身(固定時間の信号プログラム)は --template の信号制御 1 を
  そのまま写す(信号グループ 1・2 が交互に青)。SUMO の各 CARLA 信号(linkSignalID)は、SUMO の最初のフェーズで
  青なら信号グループ 1、そうでなければ 2。取り込みで作られた信号制御・信号灯は消す。
- 信号灯: CARLA の信号の停止位置(--carla-json の stops。road_id・lane_id・位置)ごとに、その位置の車線に置く。
- 信号の対応表(--signal-mapping-out): {"<信号制御>": {"<信号グループ>": [CARLA の信号 ID, ...]}}。
  Vissim ブリッジの data/signal_mapping.json と同じ形。
- 車両入力: SUMO のルートの最初の edge を、Vissim のリンクに置き換えた入口に置く。流量は SUMO の出発台数の
  割合で --total-volume [台/h] を分ける。経路(routing decision)は置かない(Town01.inpx と同じ)。

CARLA・SUMO・Vissim は使わない(ファイルを読むだけ)。
"""

from __future__ import annotations

import argparse
import collections
import copy
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

# --template から写す、車両・歩行者・挙動の定義(道路網の側の同じ要素は置き換える)
LIBRARY_SECTIONS = (
    'anmDefaults', 'colorDistributions', 'desAccelerationFunctions', 'desDecelerationFunctions',
    'displayTypes', 'drivingBehaviors', 'linkBehaviorTypes', 'maxAccelerationFunctions',
    'maxDecelerationFunctions', 'model2D3DDistributions', 'models2D3D', 'occupancyDistributions',
    'pedestrianClasses', 'pedestrianCompositions', 'pedestrianTypes', 'powerDistributions',
    'simulation', 'speedDistributions', 'timeDistributions', 'vehicleClasses', 'vehicleCompositions',
    'vehicleTypes', 'walkingBehaviors', 'weightDistributions',
)
DRIVING_SIMULATOR_ATTRS = ('drivSimActive', 'drivSimPedType', 'drivSimVehType')

# 信号灯を置く車線を決めるときの許容
LANE_MATCH_TOLERANCE_M = 1.0       # 停止位置と車線の中心の横のずれ
HEADING_TOLERANCE_DEG = 45.0       # 停止位置の向きとリンクの向き
SIGNAL_HEAD_END_MARGIN_M = 0.2     # リンクの終わりからこれだけ手前より先には置かない


class BuildError(Exception):
    pass


# ---------------------------------------------------------------- 幾何

def polyline(link):
    return [(float(p.get('x')), float(p.get('y'))) for p in link.iter('linkPolyPoint')]


def project(points, x, y):
    """点をポリラインに投影する -> (距離 s, 左向きを正とする横のずれ d, 向き [deg], 投影の距離)。"""
    best = None
    s0 = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length == 0.0:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (length * length)))
        px, py = ax + t * dx, ay + t * dy
        dist = math.hypot(x - px, y - py)
        lateral = (dx * (y - ay) - dy * (x - ax)) / length
        heading = math.degrees(math.atan2(dy, dx))
        if best is None or dist < best[3]:
            best = (s0 + t * length, lateral, heading, dist)
        s0 += length
    return best


def length_of(points):
    return sum(math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:]))


def lane_offsets(link):
    """車線ごとの中心の横のずれ(左向きが正)。車線 1 が一番右。ポリラインは全車線の中心。"""
    widths = [float(lane.get('width', '3.5')) for lane in link.find('lanes')]
    total = sum(widths)
    offsets, right = [], -total / 2
    for width in widths:
        offsets.append(right + width / 2)
        right += width
    return offsets, widths


def angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


# ---------------------------------------------------------------- 読み込み

def road_links(root):
    """コネクタでないリンク: no -> element。"""
    return {link.get('no'): link for link in root.find('links') if link.find('fromLinkEndPt') is None}


def sumo_signals(net_root):
    """SUMO の信号 -> [(tlLogic id, {CARLA 信号 ID: 信号グループ 1/2})]。"""
    result = []
    for tl in net_root.iter('tlLogic'):
        phases = [p.get('state') for p in tl.findall('phase')]
        links_of = collections.defaultdict(list)
        for param in tl.findall('param'):
            key = param.get('key', '')
            if key.startswith('linkSignalID:'):
                links_of[param.get('value')].append(int(key.split(':', 1)[1]))
        groups = {}
        for landmark, indices in links_of.items():
            first = {phases[0][i] for i in indices}
            if first <= {'G', 'g'}:
                groups[landmark] = 1
            elif first <= {'r'}:
                groups[landmark] = 2
            else:
                raise BuildError(f'tlLogic {tl.get("id")}: CARLA signal {landmark} is {sorted(first)} in the first phase '
                                 '(each CARLA signal must be all green or all red there)')
        if groups:
            result.append((tl.get('id'), groups))
    return result


def sumo_origins(rou_root):
    """SUMO のルートの最初の edge -> 出発台数。"""
    counts = collections.Counter()
    for vehicle in rou_root.iter('vehicle'):
        route = vehicle.find('route')
        if route is not None and route.get('edges'):
            counts[route.get('edges').split()[0]] += 1
    return counts


def is_driving_lane(lane):
    """乗用車が走れる SUMO の車線(allow に passenger がある、または allow/disallow で禁止されていない)。"""
    allow, disallow = lane.get('allow'), lane.get('disallow')
    if allow is not None:
        return 'passenger' in allow.split()
    if disallow is not None:
        return disallow != 'all' and 'passenger' not in disallow.split()
    return True


def sumo_edge_to_link(net_root, links):
    """SUMO の edge -> Vissim のリンク(edge の中ほどの位置と向きが合うもの)。"""
    off_x, off_y = (float(v) for v in net_root.find('location').get('netOffset').split(','))
    result = {}
    for edge in net_root.iter('edge'):
        if edge.get('function') == 'internal' or edge.get('id', '').startswith(':'):
            continue
        lanes = [lane for lane in edge.findall('lane') if is_driving_lane(lane)]
        if not lanes:
            continue
        shape = [tuple(float(v) for v in xy.split(',')) for xy in lanes[0].get('shape').split()]
        shape = [(x - off_x, y - off_y) for x, y in shape]
        half = length_of(shape) / 2
        walked = 0.0
        for (ax, ay), (bx, by) in zip(shape, shape[1:]):
            seg = math.hypot(bx - ax, by - ay)
            if walked + seg >= half and seg > 0:
                t = (half - walked) / seg
                mx, my, heading = ax + t * (bx - ax), ay + t * (by - ay), math.degrees(math.atan2(by - ay, bx - ax))
                break
            walked += seg
        else:
            continue
        best = None
        for no, link in links.items():
            s, lateral, link_heading, dist = project(polyline(link), mx, my)
            offsets, widths = lane_offsets(link)
            if angle_diff(link_heading, heading) > HEADING_TOLERANCE_DEG:
                continue
            if abs(lateral) > sum(widths) / 2 + 0.5:
                continue
            if best is None or dist < best[1]:
                best = (no, dist)
        if best is not None:
            result[edge.get('id')] = best[0]
    return result


# ---------------------------------------------------------------- 信号灯の位置

LINK_NAME = re.compile(r'Road (\d+)-(\d+)-(Right|Left)$')


def link_road(link):
    """取り込みで付いたリンク名 '<no>: Road <road>-<lane section>-<Right|Left>' -> (road, side)。"""
    match = LINK_NAME.search(link.get('name', ''))
    return (int(match.group(1)), match.group(3)) if match else None


def place_stop(links, stop):
    """CARLA の停止位置 -> (リンク no, 車線 no, pos)。

    OpenDRIVE の road・車線の側(lane_id < 0 は Right)が名前で合うリンクを優先する(停止位置が道路の端にあると、
    隣の道路のリンクの端にも同じくらい近いため)。名前で合うリンクに乗らないときだけ、位置だけで選ぶ。
    """
    x, y = stop['x'], -stop['y']                  # CARLA -> Vissim
    heading = -stop['yaw']
    if stop.get('road_id') is not None and stop.get('lane_id'):
        wanted = (int(stop['road_id']), 'Right' if int(stop['lane_id']) < 0 else 'Left')
        named = {no: link for no, link in links.items() if link_road(link) == wanted}
        if named:
            try:
                return _place_stop(named, stop, x, y, heading)
            except BuildError:
                pass
    return _place_stop(links, stop, x, y, heading)


def _place_stop(links, stop, x, y, heading):
    candidates = []
    for no, link in links.items():
        points = polyline(link)
        s, lateral, link_heading, dist = project(points, x, y)
        if angle_diff(link_heading, heading) > HEADING_TOLERANCE_DEG:
            continue
        offsets, widths = lane_offsets(link)
        lane_index = min(range(len(offsets)), key=lambda i: abs(lateral - offsets[i]))
        error = abs(lateral - offsets[lane_index])
        along = abs(dist - abs(lateral)) if dist > abs(lateral) else 0.0   # ポリラインの外(前後)に出た分
        if error <= LANE_MATCH_TOLERANCE_M and along < 0.5:
            pos = min(s, length_of(points) - SIGNAL_HEAD_END_MARGIN_M)
            candidates.append((error, no, lane_index + 1, max(pos, 0.0)))
    if not candidates:
        raise BuildError(f'no Vissim lane at CARLA stop (road {stop.get("road_id")}, lane {stop.get("lane_id")}, '
                         f'x={stop["x"]:.2f}, y={stop["y"]:.2f})')
    candidates.sort()
    _, no, lane, pos = candidates[0]
    return no, lane, pos


# ---------------------------------------------------------------- 組み立て

def replace_section(root, template_root, tag):
    new = template_root.find(tag)
    if new is None:
        return
    old = root.find(tag)
    new = copy.deepcopy(new)
    if old is None:
        insert_sorted(root, new)
    else:
        index = list(root).index(old)
        root.remove(old)
        root.insert(index, new)


def insert_sorted(root, element):
    """network の子は要素名の順(Vissim が書く順)。同じ順になる位置に入れる。"""
    for index, child in enumerate(root):
        if child.tag.lower() > element.tag.lower():
            root.insert(index, element)
            return
    root.append(element)


def section(root, tag):
    element = root.find(tag)
    if element is None:
        element = ET.Element(tag)
        insert_sorted(root, element)
    return element


def build_signal_controllers(root, template_root, signals):
    template_sc = template_root.find('signalControllers')
    if template_sc is None or not len(template_sc):
        raise BuildError('--template has no signal controller to copy')
    template = template_sc[0]
    sg_numbers = [sg.get('no') for sg in template.iter('signalGroup')]
    if sg_numbers[:2] != ['1', '2']:
        raise BuildError(f'--template signal controller 1 must have signal groups 1 and 2 (has {sg_numbers})')
    controllers = section(root, 'signalControllers')
    for child in list(controllers):
        controllers.remove(child)
    for number, (tl_id, groups) in enumerate(signals, start=1):
        sc = copy.deepcopy(template)
        sc.set('no', str(number))
        sc.set('name', f'SUMO tlLogic {tl_id}: ' + ','.join(sorted(groups, key=int)))
        for element in sc.iter('signalOutputConfigurationElement'):
            group = element.get('sg', '').split()
            if len(group) == 2:
                element.set('sg', f'{number} {group[1]}')
        controllers.append(sc)


def build_signal_heads(root, links, signals, carla_lights, template_root):
    stops_of = {str(light['opendrive_id']): light.get('stops') or [] for light in carla_lights}
    heads = section(root, 'signalHeads')
    for child in list(heads):
        heads.remove(child)
    template_head = template_root.find('signalHeads')
    template_head = template_head[0] if template_head is not None and len(template_head) else None
    mapping, placed = {}, []
    number = 0
    for sc_no, (tl_id, groups) in enumerate(signals, start=1):
        for landmark, group in sorted(groups.items(), key=lambda item: int(item[0])):
            if landmark not in stops_of:
                raise BuildError(f'CARLA signal {landmark} (SUMO tlLogic {tl_id}) is not in --carla-json')
            if not stops_of[landmark]:
                raise BuildError(f'CARLA signal {landmark} has no stop positions in --carla-json')
            mapping.setdefault(str(sc_no), {}).setdefault(str(group), []).append(int(landmark))
            for stop in stops_of[landmark]:
                link_no, lane, pos = place_stop(links, stop)
                number += 1
                head = copy.deepcopy(template_head) if template_head is not None else ET.Element('signalHead')
                head.attrib.update({'no': str(number), 'name': f'CARLA {landmark}', 'lane': f'{link_no} {lane}',
                                    'pos': f'{pos:.3f}', 'sg': f'{sc_no} {group}', 'type': 'CIRCULAR',
                                    'allVehTypes': 'true', 'allPedTypes': 'true'})
                heads.append(head)
                placed.append(dict(signal=int(landmark), sc=sc_no, sg=group, link=int(link_no), lane=lane,
                                   pos=round(pos, 3), road_id=stop.get('road_id'), lane_id=stop.get('lane_id')))
    missing = sorted(set(stops_of) - {lm for _, groups in signals for lm in groups}, key=int)
    if missing:
        raise BuildError(f'CARLA signals {missing} are not controlled by any SUMO tlLogic (linkSignalID)')
    return mapping, placed


def build_vehicle_inputs(root, origins, edge_to_link, total_volume, composition):
    inputs = section(root, 'vehicleInputs')
    for child in list(inputs):
        inputs.remove(child)
    per_link = collections.Counter()
    unmatched = {}
    for edge, count in origins.items():
        if edge in edge_to_link:
            per_link[edge_to_link[edge]] += count
        else:
            unmatched[edge] = count
    if unmatched:
        raise BuildError(f'SUMO origin edges without a Vissim link: {unmatched}')
    total = sum(per_link.values())
    result = []
    for number, (link_no, count) in enumerate(sorted(per_link.items(), key=lambda item: int(item[0])), start=1):
        volume = round(total_volume * count / total)
        element = ET.SubElement(inputs, 'vehicleInput', {'anmFlag': 'false', 'link': link_no, 'name': '',
                                                         'no': str(number)})
        vols = ET.SubElement(element, 'timeIntVehVols')
        ET.SubElement(vols, 'timeIntervalVehVolume', {'cont': 'false', 'timeInt': '1 0', 'vehComp': str(composition),
                                                      'volType': 'STOCHASTIC', 'volume': str(volume)})
        result.append(dict(link=int(link_no), sumo_vehicles=count, volume=volume))
    return result


def check_references(root):
    """よく使う参照(番号)の先があるか。写した定義と道路網の組み合わせの確認。"""
    numbers = {tag: {child.get('no') for child in root.find(tag)} for tag in
               ('displayTypes', 'linkBehaviorTypes', 'vehicleTypes', 'model2D3DDistributions', 'speedDistributions',
                'vehicleCompositions', 'pedestrianTypes', 'drivingBehaviors', 'areaBehaviorTypes')
               if root.find(tag) is not None}
    rules = [('links/link', 'linkBehavType', 'linkBehaviorTypes'), ('links/link', 'displayType', 'displayTypes'),
             ('areas/area', 'displayType', 'displayTypes'),
             ('vehicleTypes/vehicleType', 'model2D3DDistr', 'model2D3DDistributions'),
             ('vehicleCompositions/vehicleComposition/vehCompRelFlows/vehicleCompositionRelativeFlow', 'vehType',
              'vehicleTypes'),
             ('vehicleCompositions/vehicleComposition/vehCompRelFlows/vehicleCompositionRelativeFlow', 'desSpeedDistr',
              'speedDistributions'),
             ('vehicleInputs/vehicleInput/timeIntVehVols/timeIntervalVehVolume', 'vehComp', 'vehicleCompositions')]
    problems = []
    for path, attr, tag in rules:
        for element in root.findall(path):
            value = element.get(attr)
            if value is not None and value not in numbers.get(tag, set()):
                problems.append(f'{path}@{attr}={value} not in {tag}')
    net = root.find('netPara')
    for attr, tag in (('drivSimVehType', 'vehicleTypes'), ('drivSimPedType', 'pedestrianTypes')):
        if net.get(attr) not in numbers.get(tag, set()):
            problems.append(f'netPara@{attr}={net.get(attr)} not in {tag}')
    if problems:
        raise BuildError('dangling references:\n  ' + '\n  '.join(sorted(set(problems))[:30]))


def build(road_root, template_root, net_root, rou_root, carla_lights, total_volume):
    root = road_root
    for tag in LIBRARY_SECTIONS:
        replace_section(root, template_root, tag)
    net, template_net = root.find('netPara'), template_root.find('netPara')
    for attr in DRIVING_SIMULATOR_ATTRS:
        net.set(attr, template_net.get(attr))
    links = road_links(root)
    signals = sumo_signals(net_root)
    build_signal_controllers(root, template_root, signals)
    mapping, placed = build_signal_heads(root, links, signals, carla_lights, template_root)
    composition = template_root.find('vehicleCompositions')[0].get('no')
    edge_to_link = sumo_edge_to_link(net_root, links)
    inputs = build_vehicle_inputs(root, sumo_origins(rou_root), edge_to_link, total_volume, composition)
    check_references(root)
    return mapping, placed, inputs


def write_inpx(root, path):
    ET.indent(root, space='\t')
    body = ET.tostring(root, encoding='unicode')
    with open(path, 'w', encoding='utf-8', newline='\r\n') as stream:
        stream.write('<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n')
        stream.write(body)
        stream.write('\n')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--road', required=True, help='Vissim で OpenDRIVE を取り込んだ道路網の .inpx')
    parser.add_argument('--template', required=True,
                        help='車両設定・信号制御を写す .inpx(Town01.inpx)')
    parser.add_argument('--sumo-net', required=True, help='SUMO の net(信号と linkSignalID)')
    parser.add_argument('--sumo-rou', required=True, help='SUMO のルート(出発の edge の割合)')
    parser.add_argument('--carla-json', required=True,
                        help='carla_tl_to_lanelet2.py --dump-json の出力(CARLA の信号と停止位置)')
    parser.add_argument('--out', required=True)
    parser.add_argument('--signal-mapping-out', required=True, help='信号の対応表(signal_mapping.json の形)')
    parser.add_argument('--total-volume', type=int, default=600, help='車両入力の合計 [台/h](既定 600)')
    args = parser.parse_args(argv)
    try:
        road = ET.parse(os.path.expanduser(args.road)).getroot()
        template = ET.parse(os.path.expanduser(args.template)).getroot()
        net = ET.parse(os.path.expanduser(args.sumo_net)).getroot()
        rou = ET.parse(os.path.expanduser(args.sumo_rou)).getroot()
        with open(os.path.expanduser(args.carla_json), encoding='utf-8') as stream:
            carla = json.load(stream)
        lights = carla['traffic_lights'] if isinstance(carla, dict) else carla
        mapping, placed, inputs = build(road, template, net, rou, lights, args.total_volume)
    except BuildError as exc:
        print(f'[build_vissim_town] 止めた: {exc}', file=sys.stderr)
        return 1
    write_inpx(road, args.out)
    with open(args.signal_mapping_out, 'w', encoding='utf-8') as stream:
        json.dump(mapping, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(f'[build_vissim_town] {args.out}: 信号制御 {len(mapping)}、信号灯 {len(placed)}、車両入力 {len(inputs)} '
          f'(合計 {sum(i["volume"] for i in inputs)} 台/h)')
    for head in placed:
        print(f'  signal {head["signal"]}: SC {head["sc"]} SG {head["sg"]} -> link {head["link"]} lane {head["lane"]} '
              f'pos {head["pos"]} (CARLA road {head["road_id"]} lane {head["lane_id"]})')
    for item in inputs:
        print(f'  input link {item["link"]}: {item["volume"]} 台/h (SUMO {item["sumo_vehicles"]} 台)')
    print(f'[build_vissim_town] {args.signal_mapping_out}: {json.dumps(mapping, sort_keys=True)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
