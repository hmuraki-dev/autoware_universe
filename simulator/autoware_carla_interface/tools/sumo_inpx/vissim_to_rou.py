#!/usr/bin/env python3
"""Vissim の .inpx の車の需要と寸法を、SUMO の .rou.xml にする(経路は分岐ごとに等分)。

    python3 tools/sumo_inpx/vissim_to_rou.py \\
        --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \\
        --out sumo/rou/Town10HD_vissim.rou.xml
    python3 tools/sumo_inpx/vissim_to_rou.py --inpx ... --net ... --report     # 書き出さずに表示

  Vissim(inpx)                                   .rou.xml での書き方
  流入(vehicleInput)                             流入リンクに対応する edge から出す <vehicle>。出発時刻は流量の
                                                  ポアソン到着(指数分布の間隔)で --horizon 秒まで。出す位置は流入
                                                  リンクの始点を edge に射影した departPos、車線は random
  経路(無い。車はコネクタを選びながら走る)      車 1 台ごとに、分岐ごとに次の edge を等しい確率で選んだ経路。
                                                  長さが --route-length に達した edge の終わりで消える
  車種構成(vehicleComposition)                  vTypeDistribution の確率(車種 × 3D モデルの割合 × 希望速度の区間)
  車種                                            vClass: 乗用車 passenger / トラック truck / トレーラー trailer /
                                                  バス bus / バイク motorcycle
  寸法                                            inpx の 3D モデルの長さ・幅・高さ
  希望速度(区分線形の経験分布、km/h)            区間ごとの vType。speedFactor="normc(中央, 100, 下端, 上端)"
                                                  (区間内でほぼ一様)× desiredMaxSpeed="1" が希望速度 [m/s]。
                                                  地図の制限速度に縛られない
  追従モデル(運転挙動、Wiedemann 74)            carFollowModel="Wiedemann"、minGap = w74ax
  加速度・減速度(速度の関数)                    accel = 希望加速度の 0〜60 km/h の平均、decel = 希望減速度(50 km/h)、
                                                  emergencyDecel = 最大減速度(50 km/h)

移植元: SBIR_sim の sumo/vissim_to_rou.py(dedb96c)の vType の規則。変えたところ: 経路を静的経路から
「分岐ごとに等分」にした(Town10HD.inpx に経路が無く、網に行き止まりが無いので、経路を数え上げる
routeDistribution にできない。車 1 台ずつ書く)。バイクを割合 0 にしない(CARLA 0.9 には二輪がある)。
歩行者・希望速度の変更(desSpeedDecision)は Town10HD.inpx に無いので扱わない(あれば止める)。

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
from inpx_reader import Inpx, InpxError, digest, fmt, link_to_edge, net_offset, sumo_tools_on_path  # noqa: E402

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


def build_types(inpx, compositions, prefix):
    """-> (vtypes {id: 属性}, distributions {id: [(vType, 確率)]})。"""
    behavior = inpx.behavior
    if behavior.get('carFollowModType') != 'WIEDEMANN74':
        raise RouError(f'運転挙動の追従モデルが Wiedemann 74 でない: {behavior.get("carFollowModType")}')
    vtypes, distributions = {}, {}
    for composition in sorted(compositions, key=int):
        comp = inpx.compositions[composition]
        total = sum(share for _, share, _ in comp['types'])
        if total <= 0:
            raise RouError(f'車両構成 {composition} の相対交通量の合計が 0')
        members = []
        for vehicle_type, share, speed in comp['types']:
            if share <= 0:
                continue
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
        distributions[f'{prefix}_c{composition}'] = members
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


def build_vehicles(inpx, net, prefix, seed, horizon, route_length):
    """-> (vehicles [dict]、origins {流入: dict})。出発時刻の順。"""
    off_x, off_y = net_offset(net)
    vehicles, origins = [], {}
    for item in sorted(inpx.inputs, key=lambda i: int(i['no'])):
        if item['volume'] <= 0:
            continue
        edge = link_to_edge(inpx, net, item['link'])
        lane = next(lane for lane in edge.getLanes() if lane.allows(ROUTE_VCLASS))
        sx, sy = inpx.links[item['link']]['pts'][0]
        depart_pos = max(0.0, min(lane.getClosestLanePosAndDist((sx + off_x, sy + off_y))[0], edge.getLength()))
        origins[item['no']] = dict(link=item['link'], edge=edge.getID(), depart_pos=depart_pos,
                                   volume=item['volume'], composition=item['composition'], count=0)
        rng = random.Random(f'{seed}:{item["no"]}')
        rate = item['volume'] / 3600.0
        t = rng.expovariate(rate)
        while t < horizon:
            edges, length = random_route(edge, depart_pos, route_length, rng)
            count = origins[item['no']]['count']
            vehicles.append(dict(id=f'{prefix}_i{int(item["no"]):02d}_{count:05d}', depart=t,
                                 type=f'{prefix}_c{item["composition"]}', depart_pos=depart_pos,
                                 edges=[e.getID() for e in edges], length=length, input=item['no']))
            origins[item['no']]['count'] += 1
            t += rng.expovariate(rate)
    vehicles.sort(key=lambda v: (round(v['depart'], 2), v['id']))
    return vehicles, origins


def check_supported(inpx):
    if inpx.has_static_routes:
        raise RouError('inpx に静的経路(vehicleRouteStatic)がある。このツールは経路の無い網だけを扱う'
                       '(静的経路は移植元の sumo/vissim_to_rou.py の方法)')
    if inpx.pedestrian_inputs:
        raise RouError(f'inpx に歩行者の流入が {inpx.pedestrian_inputs} 本ある。このツールは歩行者を扱わない')


def render(inpx, net_path, args, vtypes, distributions, vehicles, origins):
    total = sum(o['volume'] for o in origins.values())
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<!-- tools/sumo_inpx/vissim_to_rou.py が Vissim の需要と寸法から作ったもの(手で編集しない)。',
        f'     Vissim: {os.path.basename(inpx.path)} sha256 {digest(inpx.path)}',
        f'     網:     {os.path.basename(net_path)} sha256 {digest(net_path)}',
        f'     ツール: vissim_to_rou.py sha256 {digest(os.path.abspath(__file__))}',
        # XML のコメントに「--」は書けないので、引数は名前=値で書く
        f'     引数:   prefix={args.prefix} seed={args.seed} horizon={fmt(args.horizon)} route-length={fmt(args.route_length)}',
        f'     流入 {len(origins)} 本(合計 {fmt(total, 3)} 台/h)から、ポアソン到着で 0〜{fmt(args.horizon)} s に {len(vehicles)} 台を出す。',
        '     経路: Vissim の網に経路が無い(車はコネクタを選びながら走る)ので、車 1 台ごとに分岐ごとに次の edge を',
        f'     等しい確率で選び、長さ {fmt(args.route_length)} m に達した edge の終わりで消える(Vissim の車は閉じた網を回り続ける)。',
        '     追従は Wiedemann(minGap = w74ax)。希望速度は vType の speedFactor × desiredMaxSpeed(=1) [m/s]。',
        '     移していないもの: Wiedemann 74 の bx、加速度の速度依存と車ごとのばらつき。',
    ]
    for no in sorted(origins, key=int):
        o = origins[no]
        lines.append(f'     流入 {no}: リンク {o["link"]} -> edge {o["edge"]} departPos {o["depart_pos"]:.2f}、'
                     f'{fmt(o["volume"], 3)} 台/h、{o["count"]} 台')
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
        lines.append(f'    <vehicle id="{v["id"]}" type="{v["type"]}" depart="{v["depart"]:.2f}" departLane="random"'
                     f' departPos="{v["depart_pos"]:.2f}" departSpeed="max">')
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
        net = sumolib.net.readNet(args.net)
        compositions = {item['composition'] for item in inpx.inputs}
        vtypes, distributions = build_types(inpx, compositions, args.prefix)
        vehicles, origins = build_vehicles(inpx, net, args.prefix, args.seed, args.horizon, args.route_length)
    except (InpxError, RouError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    if args.report:
        for no in sorted(origins, key=int):
            o = origins[no]
            print(f'流入 {no:>2}: リンク {o["link"]:>3} -> edge {o["edge"]:>4} departPos {o["depart_pos"]:7.2f}  '
                  f'{o["volume"]:6.1f} 台/h  {o["count"]:4d} 台')
        for name, members in sorted(distributions.items()):
            for key, p in members:
                spec = vtypes[key]
                print(f'{name}: {key:<36} {p:.4f}  {spec["length"]} x {spec["width"]} m  accel {spec["accel"]} '
                      f'decel {spec["decel"]} {spec["speedFactor"]}')
        lengths = sorted(len(v['edges']) for v in vehicles)
        print(f'vType {len(vtypes)}、車 {len(vehicles)} 台、edge 数 {lengths[0]}〜{lengths[-1]}(中央 {lengths[len(lengths) // 2]})')
        return 0
    text = render(inpx, args.net, args, vtypes, distributions, vehicles, origins)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as stream:
        stream.write(text)
    print(f'{args.out}: 車 {len(vehicles)} 台、vType {len(vtypes)}  sha256 {digest(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
