#!/usr/bin/env python3
"""Vissim の .inpx の横断歩道(歩行者リンク)を、SUMO の網の横断歩道(crossing)と歩道にする。

    python3 tools/sumo_inpx/vissim_to_crossings.py \\
        --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \\
        --map ~/autoware_map/Town10HD/lanelet2_map_with_tl.osm --out sumo/net/Town10HD_vissim.net.xml
    python3 tools/sumo_inpx/vissim_to_crossings.py --inpx ... --net ... --map ... --report   # 書き出さずに表示

docs/SUMO_inpx変換_統一仕様_v1.0.md の 4 章(横断歩道)の規則:
  - どの横断歩道を作るか: Vissim の歩行者リンク(コネクタでないもの)ごとに、Autoware の地図(lanelet2)の
    横断歩道(subtype=crosswalk)のうち、リンクの中ほどが横断歩道の中心線の近く(半幅 + 1 m)にあり、向きが
    合う(30 度以内)ものを対応付ける。対応する横断歩道が無い歩行者リンクがあれば止める(X2)。
  - 位置と幅: 地図の横断歩道の中心線と幅。両端を 1 m ずつ延ばす(X1・X3)。
  - 渡る車道: 横断歩道の中心線の近く(8 m 以内)で交差点に入る・出る、横断歩道と向きの交わる(45 度以上)
    車の edge。それらの edge が 1 つの交差点に集まらないときは止める(X4)。
  - 優先: priority="true"(X5)。
  - 歩道: 横断歩道の端の近く(15 m 以内)に待ち場所(歩行者入力・歩行者の経路が使うエリア)があれば、
    待ち場所の外側の端から、横断歩道の延長線上に、延ばした横断歩道の端の 0.5 m 手前までの歩道
    ped_area<エリアの番号> を足す。待ち場所が延長線上に無いときは止める。待ち場所の無い端は、網の
    今の歩道(walkingarea)につながる(X6。Town10HD の網には歩道がある)。1 つの待ち場所の歩道は 1 本だけ。
  - 網は作り直さず、今の網に netconvert で部品(.nod・.edg・.con)を足す。車のつながり・linkIndex・
    linkSignalID は変わらない。信号のある交差点では、netconvert が信号プログラムに横断歩道のリンクを足す
    (色は vissim_to_tll.py が歩行者信号から決める)。

歩行者リンクが無い inpx では何も書かない(網はそのまま使う)。部品は --out の横に
<--out の名前>.crossings.{nod,edg,con}.xml として書く(先頭に inpx・網・地図の sha256)。
移植元: SBIR_sim の sumo/add_crosswalk.py(1 本・交差点 1 つ)。座標は Vissim = OpenDRIVE = 地図、
SUMO = OpenDRIVE + netOffset(inpx_reader.py)。
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inpx_reader import Inpx, InpxError, digest, net_offset, sumo_tools_on_path  # noqa: E402

# 地図の横断歩道の両端から延ばす長さ [m](信号待ちの人が地図の横断歩道の外に立つように)
EXTEND_M = 1.0
# 歩行者リンクと横断歩道の対応: 中心線からの余裕 [m]、向きの差 [deg]
LINK_MARGIN_M = 1.0
LINK_ANGLE_DEG = 30.0
# 渡る車道: 交差点側の端が中心線から離れてよい距離 [m]、中心線の端からはみ出してよい長さ [m]、向きの差の下限 [deg]
LEG_DISTANCE_M = 8.0
LEG_OVERHANG_M = 2.0
LEG_ANGLE_DEG = 45.0
# 歩道: 待ち場所を探す距離 [m]、横断歩道の端から離す長さ [m]、待ち場所の外側の端を探す刻みと余裕 [m]、幅 [m]、速さ [m/s]
AREA_SEARCH_M = 15.0
AXIS_GAP_M = 0.5
AXIS_STEP_M = 0.25
AXIS_MARGIN_M = 0.5
SIDEWALK_WIDTH_M = 3.0
WALK_SPEED_MPS = 2.78
PREFIX = 'ped'


class CrossingError(Exception):
    pass


# ---------------------------------------------------------------- 幾何

def _point_in_polygon(p, polygon):
    x, y = p
    inside = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def _unit(a, b):
    length = math.dist(a, b)
    return ((b[0] - a[0]) / length, (b[1] - a[1]) / length)


def _along(start, end, p):
    """p の、start -> end の線上の位置 [m] と、線からの距離 [m]。"""
    u = _unit(start, end)
    dx, dy = p[0] - start[0], p[1] - start[1]
    return dx * u[0] + dy * u[1], abs(dx * u[1] - dy * u[0])


def _angle_between(u, v):
    """2 つの向きの差 [deg](0〜90。向きの正負は見ない)。"""
    dot = abs(u[0] * v[0] + u[1] * v[1])
    return math.degrees(math.acos(min(1.0, dot)))


def _midpoint(points):
    total = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    walked = 0.0
    for a, b in zip(points, points[1:]):
        seg = math.dist(a, b)
        if seg > 0 and walked + seg >= total / 2:
            t = (total / 2 - walked) / seg
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), _unit(a, b)
        walked += seg
    raise CrossingError('長さ 0 の歩行者リンク')


# ---------------------------------------------------------------- 地図の横断歩道

def map_crosswalks(osm):
    """lanelet2 の横断歩道 -> [dict(id, start, end, width, polygon, z)](座標は地図 = Vissim)。"""
    root = ET.parse(osm).getroot()
    nodes = {}
    for node in root.iter('node'):
        tags = {t.get('k'): t.get('v') for t in node.iter('tag')}
        if 'local_x' in tags:
            nodes[node.get('id')] = (float(tags['local_x']), float(tags['local_y']), float(tags.get('ele', 0)))
    ways = {w.get('id'): [nd.get('ref') for nd in w.iter('nd')] for w in root.iter('way')}
    out = []
    for rel in root.iter('relation'):
        tags = {t.get('k'): t.get('v') for t in rel.iter('tag')}
        if tags.get('type') != 'lanelet' or tags.get('subtype') != 'crosswalk':
            continue
        bounds = {m.get('role'): [nodes[i] for i in ways[m.get('ref')]] for m in rel.iter('member')
                  if m.get('role') in ('left', 'right')}
        left, right = bounds['left'], bounds['right']
        start = ((left[0][0] + right[0][0]) / 2, (left[0][1] + right[0][1]) / 2)
        end = ((left[-1][0] + right[-1][0]) / 2, (left[-1][1] + right[-1][1]) / 2)
        width = (math.dist(left[0][:2], right[0][:2]) + math.dist(left[-1][:2], right[-1][:2])) / 2
        out.append(dict(id=rel.get('id'), start=start, end=end, width=width,
                        polygon=[p[:2] for p in left + right[::-1]],
                        z=sum(p[2] for p in left + right) / (len(left) + len(right))))
    return out


def match_links(inpx, crosswalks):
    """歩行者リンク -> 地図の横断歩道 -> {横断歩道の id: [歩行者リンク]}。合わないリンクがあれば止める。"""
    result = {}
    for no, link in sorted(inpx.links.items(), key=lambda t: int(t[0])):
        if not link['is_ped'] or link['connector']:
            continue
        mid, direction = _midpoint(link['pts'])
        found = []
        for cw in crosswalks:
            s, d = _along(cw['start'], cw['end'], mid)
            length = math.dist(cw['start'], cw['end'])
            if (0 <= s <= length and d <= cw['width'] / 2 + LINK_MARGIN_M
                    and _angle_between(direction, _unit(cw['start'], cw['end'])) <= LINK_ANGLE_DEG):
                found.append(cw['id'])
        if len(found) != 1:
            raise CrossingError(f'歩行者リンク {no}({link["name"]})に合う地図の横断歩道が 1 つでない: {found}')
        result.setdefault(found[0], []).append(no)
    return result


# ---------------------------------------------------------------- SUMO の網

def crossing_leg(net, crosswalk):
    """横断歩道が渡る車の edge -> (交差点, [edge](交差点のまわりで時計回りの順))。"""
    off_x, off_y = net_offset(net)
    start, end = crosswalk['start'], crosswalk['end']
    length = math.dist(start, end)
    direction = _unit(start, end)
    by_node = {}
    for edge in net.getEdges(withInternal=False):
        if not any(lane.allows('passenger') for lane in edge.getLanes()):
            continue
        shape = [(x - off_x, y - off_y) for x, y, *_ in edge.getShape()]
        for node, point, other in ((edge.getToNode(), shape[-1], shape[-2]), (edge.getFromNode(), shape[0], shape[1])):
            s, d = _along(start, end, point)
            if not (-LEG_OVERHANG_M <= s <= length + LEG_OVERHANG_M and d <= LEG_DISTANCE_M):
                continue
            if _angle_between(_unit(other, point), direction) < LEG_ANGLE_DEG:
                continue
            by_node.setdefault(node.getID(), set()).add(edge)
    if len(by_node) != 1:
        raise CrossingError(f'地図の横断歩道 {crosswalk["id"]} の渡る車道が 1 つの交差点に集まらない: '
                            f'{ {n: sorted(e.getID() for e in es) for n, es in by_node.items()} }')
    node_id, edges = next(iter(by_node.items()))
    node = net.getNode(node_id)
    cx, cy = node.getCoord()

    def angle(edge):
        p = edge.getShape()[-1] if edge.getToNode() == node else edge.getShape()[0]
        return math.atan2(p[1] - cy, p[0] - cx)

    return node, sorted(edges, key=angle, reverse=True)


def _extend(start, end, d):
    u = _unit(start, end)
    return (start[0] - u[0] * d, start[1] - u[1] * d), (end[0] + u[0] * d, end[1] + u[1] * d)


def ccw_ends(node, net, start, end):
    """横断歩道の両端を、交差点の中心のまわりで反時計回りの順にする -> (始点, 終点)。

    netconvert は crossing の shape をこの向きとみなし、始点の側の walkingarea を手前の歩道に、終点の側を
    向こうの歩道につなぐ。逆向きに渡すと walkingarea が道路をまたいで反対の端まで伸び、歩行者は赤の間に
    walkingarea の上を歩いて車道を横切る(網の上では crossing に入っていないので信号を守ったことになる)。
    """
    off_x, off_y = net_offset(net)
    cx, cy = node.getCoord()
    cx, cy = cx - off_x, cy - off_y
    mx, my = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
    ox, oy = mx - cx, my - cy
    dx, dy = end[0] - start[0], end[1] - start[1]
    return (start, end) if ox * dy - oy * dx > 0 else (end, start)


def check_walkingareas(net_path, items):
    """出来た網で、crossing の両端の walkingarea がその端のまわりに収まっているかを確かめる。

    walkingarea の点が、つながる端から横断歩道の長さの半分より遠くにあれば、道路をまたいでいる。
    """
    root = ET.parse(net_path).getroot()
    shapes, crossings = {}, []
    for edge in root.iter('edge'):
        function = edge.get('function')
        if function in ('crossing', 'walkingarea'):
            lane = edge.find('lane')
            points = [tuple(map(float, p.split(',')[:2])) for p in lane.get('shape').split()]
            shapes[edge.get('id')] = points
            if function == 'crossing':
                crossings.append(edge.get('id'))
    nodes = {item['node'] for item in items}
    bad = []
    for crossing in crossings:
        if crossing[1:].rsplit('_', 1)[0] not in nodes:
            continue
        shape = shapes[crossing]
        half = math.dist(shape[0], shape[-1]) / 2
        for conn in root.iter('connection'):
            if conn.get('to') == crossing and conn.get('from') in shapes:
                walk, end = conn.get('from'), shape[0]
            elif conn.get('from') == crossing and conn.get('to') in shapes:
                walk, end = conn.get('to'), shape[-1]
            else:
                continue
            far = max(math.dist(p, end) for p in shapes[walk])
            if far > half:
                bad.append(f'{walk}({crossing} の端から {far:.1f} m)')
    if bad:
        raise CrossingError('walkingarea が道路をまたいでいる: ' + '、'.join(sorted(set(bad))))


def axis_sidewalk(crossing_end, outward, area):
    """横断歩道の延長線上の歩道の (始点, 終点)。始点は待ち場所の外側の端、終点は横断歩道の端の手前。"""
    def at(t):
        return (crossing_end[0] + outward[0] * t, crossing_end[1] + outward[1] * t)
    inside = [i * AXIS_STEP_M for i in range(1, 400) if _point_in_polygon(at(i * AXIS_STEP_M), area)]
    if not inside or max(inside) - AXIS_MARGIN_M <= AXIS_GAP_M + 1.0:
        return None
    return at(max(inside) - AXIS_MARGIN_M), at(AXIS_GAP_M)


def build(inpx, net, osm):
    """-> [dict(crosswalk, links, node, edges, start, end, width, z, sidewalks)](地図の横断歩道の id の順)。"""
    crosswalks = {cw['id']: cw for cw in map_crosswalks(osm)}
    links = match_links(inpx, list(crosswalks.values()))
    used_areas = set()
    out = []
    for cw_id in sorted(links, key=int):
        cw = crosswalks[cw_id]
        node, edges = crossing_leg(net, cw)
        ext_start, ext_end = _extend(cw['start'], cw['end'], EXTEND_M)
        u = _unit(cw['start'], cw['end'])
        sidewalks = []
        for end, outward in ((ext_start, (-u[0], -u[1])), (ext_end, u)):
            near = [(min(math.dist(end, p) for p in inpx.areas[a]), a) for a in inpx.waiting_areas
                    if a in inpx.areas]
            near = [t for t in near if t[0] <= AREA_SEARCH_M]
            if not near:
                continue
            _, area = min(near, key=lambda t: (t[0], int(t[1])))
            if area in used_areas:
                continue   # 1 つの待ち場所の歩道は 1 本(ほかの横断歩道へは網の歩道を歩く)
            shape = axis_sidewalk(end, outward, inpx.areas[area])
            if shape is None:
                raise CrossingError(f'待ち場所 {area} が地図の横断歩道 {cw_id} の延長線上に無い')
            used_areas.add(area)
            sidewalks.append(dict(id=f'{PREFIX}_area{area}', area=area, shape=list(shape)))
        if len(sidewalks) == 2 and sidewalks[0]['area'] == sidewalks[1]['area']:
            raise CrossingError(f'地図の横断歩道 {cw_id} の両端が同じ待ち場所 {sidewalks[0]["area"]} に近い')
        start, end = ccw_ends(node, net, ext_start, ext_end)
        out.append(dict(crosswalk=cw_id, links=links[cw_id], node=node.getID(), edges=[e.getID() for e in edges],
                        start=start, end=end, width=cw['width'], z=node.getCoord3D()[2],
                        sidewalks=sidewalks))
    return out


def write_parts(items, prefix, net, header):
    """crossing と歩道の部品(.nod・.edg・.con)を書く -> 3 つのパス。座標は SUMO の網の座標。"""
    off_x, off_y = net_offset(net)

    def sumo(p, z):
        return f'{p[0] + off_x:.2f},{p[1] + off_y:.2f},{z:.2f}'

    paths = [f'{prefix}.crossings.{kind}.xml' for kind in ('nod', 'edg', 'con')]
    with open(paths[0], 'w', encoding='utf-8') as f:
        f.write(f'<?xml version="1.0" encoding="UTF-8"?>\n{header}\n<nodes>\n')
        for item in items:
            for sw in item['sidewalks']:
                x, y = sw['shape'][0]
                f.write(f'    <node id="{sw["id"]}" x="{x + off_x:.2f}" y="{y + off_y:.2f}" z="{item["z"]:.2f}"'
                        f' type="dead_end"/>\n')
        f.write('</nodes>\n')
    with open(paths[1], 'w', encoding='utf-8') as f:
        f.write(f'<?xml version="1.0" encoding="UTF-8"?>\n{header}\n<edges>\n')
        for item in items:
            for sw in item['sidewalks']:
                shape = ' '.join(sumo(p, item['z']) for p in sw['shape'])
                f.write(f'    <edge id="{sw["id"]}" from="{sw["id"]}" to="{item["node"]}" numLanes="1"'
                        f' width="{SIDEWALK_WIDTH_M:g}" allow="pedestrian" speed="{WALK_SPEED_MPS:g}"'
                        f' spreadType="center" shape="{shape}"/>\n')
        f.write('</edges>\n')
    with open(paths[2], 'w', encoding='utf-8') as f:
        f.write(f'<?xml version="1.0" encoding="UTF-8"?>\n{header}\n<connections>\n')
        for item in items:
            # 向きは交差点のまわりで反時計回り(ccw_ends)。逆にすると walkingarea が反対の端につながる
            f.write(f'    <!-- 地図の横断歩道 {item["crosswalk"]}、Vissim の歩行者リンク {"・".join(item["links"])} -->\n')
            f.write(f'    <crossing node="{item["node"]}" edges="{" ".join(item["edges"])}" priority="true"'
                    f' width="{item["width"]:.2f}" shape="{sumo(item["start"], item["z"])} {sumo(item["end"], item["z"])}"/>\n')
        f.write('</connections>\n')
    return paths


def run_netconvert(net_path, parts, out):
    command = ['netconvert', '-s', net_path, '-n', parts[0], '-e', parts[1], '-x', parts[2],
               '--offset.disable-normalization', 'true', '-o', out]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        raise CrossingError(f'netconvert が失敗した: {result.stderr.strip()}')
    return result.stderr


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--inpx', required=True, help='Vissim の網(例: vissim/Town10HD.inpx)')
    ap.add_argument('--net', required=True, help='SUMO の今の網(例: sumo/net/Town10HD.net.xml)')
    ap.add_argument('--map', required=True, help='横断歩道の入った Lanelet2 地図(lanelet2_map_with_tl.osm)')
    ap.add_argument('--out', help='書き出す網(例: sumo/net/Town10HD_vissim.net.xml)')
    ap.add_argument('--report', action='store_true', help='対応付けを表示する(書き出さない)')
    args = ap.parse_args(argv)
    if not args.report and not args.out:
        ap.error('--out か --report を指定する')
    sumo_tools_on_path()
    import sumolib
    try:
        inpx = Inpx(args.inpx)
        net = sumolib.net.readNet(args.net, withInternal=True)
        items = build(inpx, net, args.map)
    except (InpxError, CrossingError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    if not items:
        print(f'{args.inpx}: 歩行者リンクが無い。横断歩道は足さない(網は {args.net} をそのまま使う)')
        return 0
    for item in items:
        sidewalks = '、'.join(f'{sw["id"]}' for sw in item['sidewalks']) or 'なし(網の歩道につながる)'
        print(f'地図の横断歩道 {item["crosswalk"]}: 歩行者リンク {"・".join(item["links"])} -> 交差点 {item["node"]} の '
              f'edge {" ".join(item["edges"])}、幅 {item["width"]:.2f} m、歩道 {sidewalks}')
    if args.report:
        return 0
    header = (f'<!-- tools/sumo_inpx/vissim_to_crossings.py が作ったもの(手で編集しない)。'
              f' Vissim: {os.path.basename(args.inpx)} sha256 {digest(args.inpx)}'
              f' 網: {os.path.basename(args.net)} sha256 {digest(args.net)}'
              f' 地図: {os.path.basename(args.map)} sha256 {digest(args.map)} -->')
    prefix = os.path.splitext(os.path.splitext(args.out)[0])[0]
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    parts = write_parts(items, prefix, net, header)
    try:
        run_netconvert(args.net, parts, args.out)
        check_walkingareas(args.out, items)
    except CrossingError as error:
        if os.path.exists(args.out):
            os.remove(args.out)
        print(f'Error: {error}', file=sys.stderr)
        return 1
    print(f'{args.out}: 横断歩道 {len(items)} 本  sha256 {digest(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
