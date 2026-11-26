#!/usr/bin/env python3
"""Vissim の .inpx の車の需要と寸法を、SUMO の .rou.xml にする。

    python3 tools/sumo_inpx/vissim_to_rou.py \\
        --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \\
        --out sumo/rou/Town10HD_vissim.rou.xml
    python3 tools/sumo_inpx/vissim_to_rou.py --inpx ... --net ... --report     # 書き出さずに表示

  Vissim(inpx)                                   .rou.xml での書き方
  流入(vehicleInput)                             流入リンクに対応する edge から出す <vehicle>。出発時刻は流量の
                                                  ポアソン到着(指数分布の間隔)で --horizon 秒まで。出す位置は流入
                                                  リンクの始点を edge に射影した departPos、車線は random
  経路決定の無い流入(車はコネクタを選びながら走る)
                                                  車 1 台ごとに、分岐ごとに次の edge を等しい確率で選んだ経路。
                                                  長さが --route-length に達した edge の終わりで消える
  静的経路(vehicleRoutingDecisionStatic、流入リンクにあるもの)
                                                  経路のリンク列(コネクタを含む)の幾何に沿う edge 列。行き先
                                                  (destLink の destPos)で消える(arrivalPos)。幾何に沿う edge 列が
                                                  分かれるところは分岐ごとに等分する。車 1 台ごとに、経路の割合
                                                  (relFlow)× 分岐の割合で edge 列を選ぶ。網で着けない経路は、同じ
                                                  流入の残りの経路に割合を寄せる
  車種構成(vehicleComposition)                  vTypeDistribution の確率(車種 × 3D モデルの割合 × 希望速度の区間)
  車種                                            vClass: 乗用車 passenger / トラック truck / トレーラー trailer /
                                                  バス bus / バイク motorcycle
  寸法                                            inpx の 3D モデルの長さ・幅・高さ
  希望速度(区分線形の経験分布、km/h)            区間ごとの vType。speedFactor="normc(中央, 100, 下端, 上端)"
                                                  (区間内でほぼ一様)× desiredMaxSpeed="1" が希望速度 [m/s]。
                                                  地図の制限速度に縛られない
  希望速度決定(desSpeedDecision)                車の edge 列が決定点を通るとき、その車に変更後の分布(車両クラス
                                                  ごと)の vTypeDistribution を出発時から使う(SUMO の車は途中で
                                                  vType を変えられない)。決定点が 2 つ以上あるときは最初に通るもの
  追従モデル(運転挙動、Wiedemann 74)            carFollowModel="Wiedemann"、minGap = w74ax
  加速度・減速度(速度の関数)                    accel = 希望加速度の 0〜60 km/h の平均、decel = 希望減速度(50 km/h)、
                                                  emergencyDecel = 最大減速度(50 km/h)

止めるもの: 歩行者の流入。流入リンク以外にある経路決定、1 つの流入リンクに 2 つ以上ある経路決定、一部の
車両クラスだけの経路決定(allVehTypes="false")、時間帯ごとの経路の割合。コネクタの上の希望速度決定、
同じリンクの車線ごとに違う希望速度決定(SUMO の車の車線は出発時に決まらない)。

移植元: SBIR_sim の sumo/vissim_to_rou.py(dedb96c)の vType・静的経路・希望速度決定の規則。変えたところ:
経路決定の無い流入は「分岐ごとに等分」にした(Town10HD.inpx に経路が無く、網に行き止まりが無いので、経路を
数え上げる routeDistribution にできない。車 1 台ずつ書く)。静的経路の edge 列は複数車線の edge で探す
(移植元の網は 1 車線ずつで、車線変更の要る経路は着けなかった)。希望速度決定は、経路決定の無い車にも
その車の edge 列で当てはめる。バイクを割合 0 にしない(CARLA 0.9 には二輪がある)。

乱数は --seed から流入ごとに作る(流入を足しても、ほかの流入の車は変わらない)。出力の先頭に inpx・網・
このツールの sha256 と引数を書く。日時は書かない(同じ入力から同じバイト列)。
docs/SUMO_CARLA_Autoware_inpxからの信号・需要生成_実装計画_v1.0.md の 2.2・2.3・S3。
"""

from __future__ import annotations

import argparse
import os
import random
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inpx_reader import (Inpx, InpxError, digest, distance_to, fmt, link_to_edge, net_offset,  # noqa: E402
                         point_at, sumo_tools_on_path)

PREFIX = 'vissim'
HORIZON_S = 3600.0
ROUTE_LENGTH_M = 2000.0
SEED = 1

# Vissim の車種の category -> SUMO の vClass。HGV は車種の名前で分ける
VCLASS = {'CAR': 'passenger', 'BUS': 'bus', 'BIKE': 'motorcycle'}
HGV_BY_NAME = {'トラック': 'truck', 'トレーラー': 'trailer'}
# vClass の既定の形を上書きする。trailer の既定の形は連結車で、SUMO は向きを前の牽引車だけから決める。
# 一続きの truck の形にする(移植元と同じ。Vissim の 1 枚の 3D モデルと同じ)
GUI_SHAPE = {'trailer': 'truck'}
ACCEL_RANGE_KMH = (0.0, 60.0)
DECEL_AT_KMH = 50.0
# 区間内をほぼ一様にする normc の標準偏差(区間の幅 ≦ 数 m/s に対して十分大きい)
UNIFORM_DEV = 100
# 経路を選ぶときに通れる車線の vClass(Town10HD の運転用の車線はどの車種も通れる)
ROUTE_VCLASS = 'passenger'
# 静的経路の edge 列の当てはめ: Vissim の経路から edge が離れてよい余裕 [m]、edge 列の長さの上限、確かめる間隔 [m]
CORRIDOR_MARGIN_M = 1.0
MAX_ROUTE_EDGES = 60
SAMPLE_M = 1.0


class RouError(Exception):
    pass


def vclass_of(inpx, vehicle_type):
    item = inpx.vehicle_types[vehicle_type]
    category = item.get('category')
    if category == 'HGV':
        if item.get('name') not in HGV_BY_NAME:
            raise RouError(f'HGV の車種 {vehicle_type} {item.get("name")} の vClass が決まっていない')
        return HGV_BY_NAME[item['name']]
    if category not in VCLASS:
        raise RouError(f'車種 {vehicle_type} の category {category} の vClass が決まっていない')
    return VCLASS[category]


def speed_bins(points):
    """Vissim の区分線形の CDF [(km/h, F)] -> [(確率, 下端 m/s, 上端 m/s)]。"""
    return [(f1 - f0, x0 / 3.6, x1 / 3.6) for (x0, f0), (x1, f1) in zip(points, points[1:]) if f1 - f0 > 0]


def dynamics(inpx, vehicle_type):
    item = inpx.vehicle_types[vehicle_type]
    lo, hi = ACCEL_RANGE_KMH
    xs = [lo + (hi - lo) * i / 6 for i in range(7)]
    accel = sum(inpx.function_value('desAccelerationFunctions', item['desAccelFunc'], x) for x in xs) / len(xs)
    decel = -inpx.function_value('desDecelerationFunctions', item['desDecelFunc'], DECEL_AT_KMH)
    emergency = -inpx.function_value('maxDecelerationFunctions', item['maxDecelFunc'], DECEL_AT_KMH)
    return accel, decel, emergency


def add_distribution(inpx, composition, prefix, vtypes, distributions, override=None):
    """車両構成(と希望速度の変更 override {車種: 速度分布})の vTypeDistribution を足し、名前を返す。"""
    behavior = inpx.behavior
    if behavior.get('carFollowModType') != 'WIEDEMANN74':
        raise RouError(f'運転挙動の追従モデルが Wiedemann 74 でない: {behavior.get("carFollowModType")}')
    comp = inpx.compositions[composition]
    types = [(t, share, (override or {}).get(t, speed)) for t, share, speed in comp['types']]
    name = f'{prefix}_c{composition}'
    if override:
        name += '_s' + '_'.join(sorted({speed for _, share, speed in types if share > 0}, key=int))
    if name in distributions:
        return name
    total = sum(share for _, share, _ in types)
    if total <= 0:
        raise RouError(f'車両構成 {composition} の相対交通量の合計が 0')
    members = []
    for vehicle_type, share, speed in types:
        if share <= 0:
            continue
        if speed not in inpx.speed:
            raise RouError(f'車種 {vehicle_type} の速度分布 {speed} が inpx に無い')
        vclass = vclass_of(inpx, vehicle_type)
        accel, decel, emergency = dynamics(inpx, vehicle_type)
        for model, model_share in inpx.type_models(vehicle_type):
            body = inpx.models[model]
            for index, (p, low, high) in enumerate(speed_bins(inpx.speed[speed])):
                key = f'{prefix}_{vclass}{model}_s{speed}_b{index}'
                vtypes[key] = dict(
                    id=key, vClass=vclass, length=fmt(body['length'], 3), width=fmt(body['width'], 3),
                    height=fmt(body['height'], 3), carFollowModel='Wiedemann',
                    minGap=fmt(float(behavior['w74ax']), 3), accel=fmt(accel, 3), decel=fmt(decel, 3),
                    emergencyDecel=fmt(emergency, 3), desiredMaxSpeed='1',
                    speedFactor=f'normc({fmt((low + high) / 2)},{UNIFORM_DEV},{fmt(low)},{fmt(high)})')
                if vclass in GUI_SHAPE:
                    vtypes[key]['guiShape'] = GUI_SHAPE[vclass]
                members.append((key, share / total * model_share * p))
    distributions[name] = members
    return name


def build_types(inpx, compositions, prefix):
    """-> (vtypes {id: 属性}, distributions {id: [(vType, 確率)]})。希望速度の変更の無い分布だけ。"""
    vtypes, distributions = {}, {}
    for composition in sorted(compositions, key=int):
        add_distribution(inpx, composition, prefix, vtypes, distributions)
    return vtypes, distributions


def successors(edge):
    """edge から ROUTE_VCLASS が通れる connection の先の edge(ID の順)。"""
    out = {}
    for lane in edge.getLanes():
        if not lane.allows(ROUTE_VCLASS):
            continue
        for connection in lane.getOutgoing():
            target = connection.getToLane()
            if target.allows(ROUTE_VCLASS):
                out[target.getEdge().getID()] = target.getEdge()
    return [out[k] for k in sorted(out)]


def random_route(origin, depart_pos, route_length, rng):
    """分岐ごとに次の edge を等しい確率で選び、長さ(departPos から)が route_length に達するまで進む。"""
    edges = [origin]
    length = origin.getLength() - depart_pos
    while length < route_length:
        nexts = successors(edges[-1])
        if not nexts:
            break   # 行き止まり(Town10HD には無い)
        edges.append(rng.choice(nexts))
        length += edges[-1].getLength()
    return edges, length


# ---------------------------------------------------------------- 静的経路

def _lane_points(edge, offset):
    """edge の ROUTE_VCLASS が通れる車線の形を SAMPLE_M ごとに刻んだ点(Vissim の座標)。"""
    off_x, off_y = offset
    out = []
    for lane in edge.getLanes():
        if not lane.allows(ROUTE_VCLASS):
            continue
        shape = [(p[0] - off_x, p[1] - off_y) for p in lane.getShape()]
        for (ax, ay), (bx, by) in zip(shape, shape[1:]):
            n = max(1, int(((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5 / SAMPLE_M))
            out += [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n)]
        out.append(shape[-1])
    return out


def _half_width(inpx, link):
    return sum(inpx.links[link]['widths'] or [3.5]) / 2


def match_paths(inpx, net, origin, sequence, end, end_radius):
    """origin の edge から、リンク列 sequence の幾何の近く(半幅 + 余裕)を通り、end の end_radius 以内に
    着く edge 列をすべて返す(edge の ID の順に深さ優先)。"""
    polyline = [p for link in sequence for p in inpx.links[link]['pts']]
    corridor = max(_half_width(inpx, link) for link in sequence) + CORRIDOR_MARGIN_M
    offset = net_offset(net)
    near, reach = {}, {}

    def check(edge):
        if edge.getID() not in near:
            points = _lane_points(edge, offset)
            near[edge.getID()] = any(distance_to(polyline, x, y) <= corridor for x, y in points)
            reach[edge.getID()] = any(((x - end[0]) ** 2 + (y - end[1]) ** 2) ** 0.5 <= end_radius
                                      for x, y in points)
        return near[edge.getID()], reach[edge.getID()]

    def inside_junction(edge, nxt):
        """行き先が edge -> nxt の交差点の中(内部 lane の上)にあるか。"""
        for connection in edge.getConnections(nxt):
            via = connection.getViaLaneID()
            if via and connection.getFromLane().allows(ROUTE_VCLASS):
                shape = [(p[0] - offset[0], p[1] - offset[1]) for p in net.getLane(via).getShape()]
                if distance_to(shape, *end) <= end_radius:
                    return True
        return False

    found = []

    def walk(path):
        is_near, is_reached = check(path[-1])
        if not is_near:
            return
        if is_reached:
            found.append(list(path))
            return
        if len(path) >= MAX_ROUTE_EDGES:
            return
        for nxt in successors(path[-1]):
            if nxt in path:
                continue
            if inside_junction(path[-1], nxt):
                # 交差点の中で消える経路は、交差点を出た edge の頭で消す
                found.append(path + [nxt])
                continue
            walk(path + [nxt])

    walk([origin])
    return found


def branch_shares(paths):
    """edge 列の集まりを、分岐ごとに等分した割合(合計 1)。"""
    shares = [1.0] * len(paths)

    def split(indices, depth):
        groups = {}
        for i in indices:
            if depth < len(paths[i]):
                groups.setdefault(paths[i][depth].getID(), []).append(i)
        if len(groups) > 1:
            for members in groups.values():
                for i in members:
                    shares[i] /= len(groups)
        for members in groups.values():
            split(members, depth + 1)

    split(list(range(len(paths))), 0)
    return shares


def static_routes(inpx, net, item, origin, notes):
    """流入 item の経路決定 -> [(重み, edge 列, arrivalPos, 経路の番号)](経路決定が無ければ [])。"""
    decisions = [d for d in inpx.routing_decisions if d['link'] == item['link']]
    if not decisions:
        return []
    decision = decisions[0]
    off_x, off_y = net_offset(net)
    reached = []
    for route in decision['routes']:
        if route['share'] <= 0:
            continue
        dest = inpx.links[route['dest']]
        sequence = [item['link']] + [no for no in route['via'] if no in inpx.links] + [route['dest']]
        end = point_at(dest['pts'], route['dest_pos']) if route['dest_pos'] is not None else dest['pts'][-1]
        paths = match_paths(inpx, net, origin, sequence, end, _half_width(inpx, route['dest']) + CORRIDOR_MARGIN_M)
        if not paths:
            notes.append(f'流入 {item["no"]}: 経路決定 {decision["no"]} の経路 {route["no"]}(-> リンク {route["dest"]}、'
                         f'割合 {fmt(route["share"], 3)})は SUMO の網で着けない。残りの経路に寄せた')
            continue
        reached.append((route, end, paths))
    if not reached:
        raise RouError(f'流入 {item["no"]}(リンク {item["link"]})の経路がどれも SUMO の網で着けない')
    total = sum(route['share'] for route, _, _ in reached)
    options = []
    for route, end, paths in reached:
        for path, share in zip(paths, branch_shares(paths)):
            lane = next(lane for lane in path[-1].getLanes() if lane.allows(ROUTE_VCLASS))
            arrival = lane.getClosestLanePosAndDist((end[0] + off_x, end[1] + off_y))[0]
            options.append((route['share'] / total * share, path, arrival, route['no']))
    return options


# ---------------------------------------------------------------- 希望速度決定

def speed_points(inpx, net):
    """希望速度決定 -> [dict(edge, pos, link, per_class)](リンクごとに 1 つ)。"""
    by_link = {}
    for decision in inpx.speed_decisions:
        by_link.setdefault(decision['link'], []).append(decision)
    off_x, off_y = net_offset(net)
    points = []
    for link, decisions in sorted(by_link.items(), key=lambda t: int(t[0])):
        if inpx.links[link]['connector']:
            raise RouError(f'希望速度決定 {decisions[0]["no"]} がコネクタ {link} の上にある(リンクの上だけ扱う)')
        if any(d['per_class'] != decisions[0]['per_class'] for d in decisions):
            raise RouError(f'リンク {link} の希望速度決定が車線ごとに違う(SUMO の車の車線は出発時に決まらない)')
        edge = link_to_edge(inpx, net, link)
        lane = next(lane for lane in edge.getLanes() if lane.allows(ROUTE_VCLASS))
        x, y = point_at(inpx.links[link]['pts'], decisions[0]['pos'])
        points.append(dict(edge=edge.getID(), pos=lane.getClosestLanePosAndDist((x + off_x, y + off_y))[0],
                           link=link, per_class=decisions[0]['per_class']))
    return points


def first_speed_decision(points, edges, depart_pos):
    """edge 列が最初に通る希望速度決定(無ければ None)。"""
    for index, edge in enumerate(edges):
        here = sorted((p for p in points if p['edge'] == edge), key=lambda p: p['pos'])
        for point in here:
            if index > 0 or point['pos'] >= depart_pos:
                return point
    return None


def override_of(inpx, point):
    """希望速度決定 -> {車種: 速度分布}(決定点の車両クラスに入る車種)。"""
    out = {}
    for vehicle_class, distribution in sorted(point['per_class'].items(), key=lambda t: int(t[0])):
        for vehicle_type in inpx.vehicle_classes.get(vehicle_class, []):
            out[vehicle_type] = distribution
    return out


# ---------------------------------------------------------------- 車

def build_vehicles(inpx, net, prefix, seed, horizon, route_length, notes=None):
    """-> (vehicles [dict]、origins {流入: dict})。出発時刻の順。

    車の type は車両構成の分布。希望速度決定を通る車は speed(決定点のリンク)を持ち、assign_types で
    変更後の分布にする。"""
    notes = [] if notes is None else notes
    off_x, off_y = net_offset(net)
    points = speed_points(inpx, net)
    vehicles, origins = [], {}
    for item in sorted(inpx.inputs, key=lambda i: int(i['no'])):
        if item['volume'] <= 0:
            continue
        edge = link_to_edge(inpx, net, item['link'])
        lane = next(lane for lane in edge.getLanes() if lane.allows(ROUTE_VCLASS))
        sx, sy = inpx.links[item['link']]['pts'][0]
        depart_pos = max(0.0, min(lane.getClosestLanePosAndDist((sx + off_x, sy + off_y))[0], edge.getLength()))
        options = static_routes(inpx, net, item, edge, notes)
        origins[item['no']] = dict(link=item['link'], edge=edge.getID(), depart_pos=depart_pos,
                                   volume=item['volume'], composition=item['composition'], count=0,
                                   routes=len(options))
        rng = random.Random(f'{seed}:{item["no"]}')
        rate = item['volume'] / 3600.0
        t = rng.expovariate(rate)
        while t < horizon:
            if options:
                _, path, arrival, route_no = rng.choices(options, weights=[o[0] for o in options])[0]
                edges = path
                length = sum(e.getLength() for e in path) - depart_pos - (path[-1].getLength() - arrival)
            else:
                edges, length = random_route(edge, depart_pos, route_length, rng)
                arrival, route_no = None, None
            count = origins[item['no']]['count']
            ids = [e.getID() for e in edges]
            point = first_speed_decision(points, ids, depart_pos)
            vehicles.append(dict(id=f'{prefix}_i{int(item["no"]):02d}_{count:05d}', depart=t,
                                 type=f'{prefix}_c{item["composition"]}', depart_pos=depart_pos,
                                 edges=ids, length=length, input=item['no'],
                                 arrival_pos=arrival, route=route_no,
                                 speed=point['link'] if point else None))
            origins[item['no']]['count'] += 1
            t += rng.expovariate(rate)
    vehicles.sort(key=lambda v: (round(v['depart'], 2), v['id']))
    return vehicles, origins


def assign_types(inpx, net, vehicles, prefix, vtypes, distributions):
    """希望速度決定を通る車の type を、変更後の速度分布の vTypeDistribution にする。"""
    points = {p['link']: p for p in speed_points(inpx, net)}
    for v in vehicles:
        if v['speed'] is not None:
            composition = v['type'][len(f'{prefix}_c'):]
            v['type'] = add_distribution(inpx, composition, prefix, vtypes, distributions,
                                         override_of(inpx, points[v['speed']]))


def check_supported(inpx):
    if inpx.pedestrian_inputs:
        raise RouError(f'inpx に歩行者の流入が {inpx.pedestrian_inputs} 本ある。このツールは歩行者を扱わない')
    input_links = {item['link'] for item in inpx.inputs}
    seen = set()
    for decision in inpx.routing_decisions:
        if decision['link'] not in input_links:
            raise RouError(f'経路決定 {decision["no"]} がリンク {decision["link"]} にある。流入リンクにある経路決定だけを扱う'
                           '(途中で経路を受け取る車は写せない)')
        if decision['link'] in seen:
            raise RouError(f'流入リンク {decision["link"]} に経路決定が 2 つ以上ある')
        seen.add(decision['link'])
        if not decision['all_types']:
            raise RouError(f'経路決定 {decision["no"]} が一部の車両クラスだけのもの(allVehTypes="false")。全車種のものだけを扱う')
        for route in decision['routes']:
            if route['dest'] not in inpx.links:
                raise RouError(f'経路決定 {decision["no"]} の経路 {route["no"]} の行き先リンク {route["dest"]} が無い')


def render(inpx, net_path, args, vtypes, distributions, vehicles, origins, notes=()):
    total = sum(o['volume'] for o in origins.values())
    routed = sorted((no for no, o in origins.items() if o['routes']), key=int)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<!-- tools/sumo_inpx/vissim_to_rou.py が Vissim の需要と寸法から作ったもの(手で編集しない)。',
        f'     Vissim: {os.path.basename(inpx.path)} sha256 {digest(inpx.path)}',
        f'     網:     {os.path.basename(net_path)} sha256 {digest(net_path)}',
        f'     ツール: vissim_to_rou.py sha256 {digest(os.path.abspath(__file__))}',
        # XML のコメントに「--」は書けないので、引数は名前=値で書く
        f'     引数:   prefix={args.prefix} seed={args.seed} horizon={fmt(args.horizon)} route-length={fmt(args.route_length)}',
        f'     流入 {len(origins)} 本(合計 {fmt(total, 3)} 台/h)から、ポアソン到着で 0〜{fmt(args.horizon)} s に {len(vehicles)} 台を出す。',
    ]
    if not routed:
        lines += [
            '     経路: Vissim の網に経路が無い(車はコネクタを選びながら走る)ので、車 1 台ごとに分岐ごとに次の edge を',
            f'     等しい確率で選び、長さ {fmt(args.route_length)} m に達した edge の終わりで消える(Vissim の車は閉じた網を回り続ける)。',
        ]
    else:
        lines += [
            f'     経路: 経路決定のある流入 {len(routed)} 本({"・".join(routed)})は、静的経路のリンク列に沿う edge 列を、経路の割合 x',
            '     分岐ごとの等分の割合で選び、行き先で消える。経路決定の無い流入は、車 1 台ごとに分岐ごとに次の edge を',
            f'     等しい確率で選び、長さ {fmt(args.route_length)} m に達した edge の終わりで消える。',
        ]
    speeds = sorted({v['speed'] for v in vehicles if v['speed'] is not None}, key=int)
    if speeds:
        changed = sum(1 for v in vehicles if v['speed'] is not None)
        lines += [
            f'     希望速度決定: リンク {"・".join(speeds)} の決定点を通る車 {changed} 台に、変更後の速度分布の vTypeDistribution を',
            '     出発時から使う(決定点で切り替わらない)。',
        ]
    lines += [
        '     追従は Wiedemann(minGap = w74ax)。希望速度は vType の speedFactor × desiredMaxSpeed(=1) [m/s]。',
        '     移していないもの: Wiedemann 74 の bx、加速度の速度依存と車ごとのばらつき。',
    ]
    lines += [f'     {note}' for note in notes]
    for no in sorted(origins, key=int):
        o = origins[no]
        lines.append(f'     流入 {no}: リンク {o["link"]} -> edge {o["edge"]} departPos {o["depart_pos"]:.2f}、'
                     f'{fmt(o["volume"], 3)} 台/h、{o["count"]} 台' + (f'、edge 列 {o["routes"]} 本' if o['routes'] else ''))
    lines[-1] += ' -->'
    lines.append('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                 'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">')
    for key in sorted(vtypes):
        attrs = ' '.join(f'{k}={quoteattr(v)}' for k, v in vtypes[key].items())
        lines.append(f'    <vType {attrs}/>')
    for name in sorted(distributions):
        members = distributions[name]
        lines.append(f'    <vTypeDistribution id="{name}" vTypes="{" ".join(k for k, _ in members)}"'
                     f' probabilities="{" ".join(f"{p:.8g}" for _, p in members)}"/>')
    for v in vehicles:
        arrival = f' arrivalPos="{v["arrival_pos"]:.2f}"' if v['arrival_pos'] is not None else ''
        lines.append(f'    <vehicle id="{v["id"]}" type="{v["type"]}" depart="{v["depart"]:.2f}" departLane="random"'
                     f' departPos="{v["depart_pos"]:.2f}" departSpeed="max"{arrival}>')
        lines.append(f'        <route edges="{" ".join(v["edges"])}"/>')
        lines.append('    </vehicle>')
    lines.append('</routes>')
    return '\n'.join(lines) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--inpx', required=True, help='Vissim の網(例: vissim/Town10HD.inpx)')
    ap.add_argument('--net', required=True, help='SUMO の網(例: sumo/net/Town10HD.net.xml)')
    ap.add_argument('--out', help='書き出す .rou.xml(例: sumo/rou/Town10HD_vissim.rou.xml)')
    ap.add_argument('--prefix', default=PREFIX, help=f'vType・車の id の頭(既定 {PREFIX})')
    ap.add_argument('--seed', type=int, default=SEED, help=f'出発時刻と経路の乱数の種(既定 {SEED})')
    ap.add_argument('--horizon', type=float, default=HORIZON_S, help=f'車を出す時間 [s](既定 {fmt(HORIZON_S)})')
    ap.add_argument('--route-length', type=float, default=ROUTE_LENGTH_M,
                    help=f'経路の長さ [m]。達した edge の終わりで消える(既定 {fmt(ROUTE_LENGTH_M)})')
    ap.add_argument('--report', action='store_true', help='流入・車種・台数を表示する(書き出さない)')
    args = ap.parse_args(argv)
    if not args.report and not args.out:
        ap.error('--out か --report を指定する')
    sumo_tools_on_path()
    import sumolib
    try:
        inpx = Inpx(args.inpx)
        check_supported(inpx)
        net = sumolib.net.readNet(args.net, withInternal=True)
        compositions = {item['composition'] for item in inpx.inputs}
        vtypes, distributions = build_types(inpx, compositions, args.prefix)
        notes = []
        vehicles, origins = build_vehicles(inpx, net, args.prefix, args.seed, args.horizon, args.route_length, notes)
        assign_types(inpx, net, vehicles, args.prefix, vtypes, distributions)
    except (InpxError, RouError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    if args.report:
        for no in sorted(origins, key=int):
            o = origins[no]
            print(f'流入 {no:>2}: リンク {o["link"]:>3} -> edge {o["edge"]:>4} departPos {o["depart_pos"]:7.2f}  '
                  f'{o["volume"]:6.1f} 台/h  {o["count"]:4d} 台  edge 列 {o["routes"]} 本')
        for name, members in sorted(distributions.items()):
            for key, p in members:
                spec = vtypes[key]
                print(f'{name}: {key:<36} {p:.4f}  {spec["length"]} x {spec["width"]} m  accel {spec["accel"]} '
                      f'decel {spec["decel"]} {spec["speedFactor"]}')
        for note in notes:
            print(note)
        lengths = sorted(len(v['edges']) for v in vehicles)
        print(f'vType {len(vtypes)}、車 {len(vehicles)} 台、edge 数 {lengths[0]}〜{lengths[-1]}(中央 {lengths[len(lengths) // 2]})')
        return 0
    text = render(inpx, args.net, args, vtypes, distributions, vehicles, origins, notes)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as stream:
        stream.write(text)
    print(f'{args.out}: 車 {len(vehicles)} 台、vType {len(vtypes)}  sha256 {digest(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
