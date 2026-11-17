"""tools/vissim/build_vissim_town.py(取り込んだ道路網の .inpx に信号・交通需要・車両設定を足す)。

前半は小さな入力で幾何と読み込みを確かめる。後半は vissim/Town10HD_road.inpx から作り直して、コミットした
vissim/Town10HD.inpx・signal_mapping_town10hd.json と同じになるかを確かめる(雛形の Town01.inpx と
~/autoware_map/Town10HD/carla_tl.json があるときだけ)。CARLA・SUMO・Vissim は使わない。

Run with:
    python3 -m pytest test/test_build_vissim_town.py
"""
import importlib.util
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('build_vissim_town', PACKAGE / 'tools' / 'vissim' / 'build_vissim_town.py')
tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tool)

VISSIM = PACKAGE / 'vissim'
SUMO = PACKAGE / 'sumo'
TEMPLATE = Path(os.path.expanduser('~/CARLA/Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx'))
CARLA_JSON = Path(os.path.expanduser('~/autoware_map/Town10HD/carla_tl.json'))


def link(no, name, points, widths=(3.5, 3.5)):
    element = ET.Element('link', {'no': str(no), 'name': name})
    pts = ET.SubElement(ET.SubElement(element, 'geometry'), 'linkPolyPts')
    for x, y in points:
        ET.SubElement(pts, 'linkPolyPoint', {'x': str(x), 'y': str(y)})
    lanes = ET.SubElement(element, 'lanes')
    for width in widths:
        ET.SubElement(lanes, 'lane', {'width': str(width)})
    return element


# ---------- 幾何 ----------

def test_lane_offsets_count_from_the_right():
    offsets, widths = tool.lane_offsets(link(1, 'x', [(0, 0), (10, 0)], (3.0, 4.0)))
    # 全幅 7 m、ポリラインは中心。車線 1(右)は -2.0、車線 2 は +1.5
    assert offsets == [-2.0, 1.5] and widths == [3.0, 4.0]


def test_project_gives_distance_along_and_left_offset():
    s, lateral, heading, dist = tool.project([(0, 0), (10, 0), (10, 10)], 4, -1)
    assert (round(s, 6), round(lateral, 6), round(heading, 6), round(dist, 6)) == (4, -1, 0, 1)
    s, lateral, heading, _ = tool.project([(0, 0), (10, 0), (10, 10)], 11, 5)
    assert (round(s, 6), round(lateral, 6), round(heading, 6)) == (15, -1, 90)


def test_place_stop_picks_the_lane_and_prefers_the_named_road():
    # 東向き 2 車線のリンク(road 19 の Right)と、その終わりから続く別の道路のリンク
    links = {'39': link(39, '19: Road 19-0-Right', [(-78, -26.2), (-67.5, -26.2)]),
             '50': link(50, '89: Road 89-0-Right', [(-67.5, -26.2), (-40, -26.2)])}
    # CARLA の停止位置(y は CARLA の向き)。lane -1 は内側 = Vissim の車線 2
    inner = dict(x=-70.0, y=24.45, yaw=0.0, road_id=19, lane_id=-1)
    outer = dict(x=-70.0, y=27.95, yaw=0.0, road_id=19, lane_id=-2)
    assert tool.place_stop(links, inner)[:2] == ('39', 2)
    assert tool.place_stop(links, outer)[:2] == ('39', 1)
    # リンクの終わりにある停止位置も、名前の合う road 19 のリンクに置く(終わりから 0.2 m 手前まで)
    at_end = dict(x=-67.6, y=24.45, yaw=0.0, road_id=19, lane_id=-1)
    no, lane, pos = tool.place_stop(links, at_end)
    assert (no, lane) == ('39', 2) and pos == pytest.approx(10.3)


def test_place_stop_rejects_the_opposite_direction():
    links = {'40': link(40, '19: Road 19-0-Left', [(-67.5, -14.7), (-78, -14.7)])}
    with pytest.raises(tool.BuildError):
        tool.place_stop(links, dict(x=-70.0, y=24.45, yaw=0.0, road_id=19, lane_id=-1))


# ---------- 読み込み ----------

def test_driving_lanes():
    lane = lambda **attrs: ET.Element('lane', attrs)
    assert tool.is_driving_lane(lane())
    assert tool.is_driving_lane(lane(disallow='pedestrian tram rail'))
    assert not tool.is_driving_lane(lane(disallow='all'))
    assert not tool.is_driving_lane(lane(allow='pedestrian'))
    assert tool.is_driving_lane(lane(allow='passenger bus'))


def test_sumo_signals_split_by_the_first_phase():
    net = ET.fromstring("""<net><tlLogic id="7">
        <phase duration="42" state="GGrr"/><phase duration="3" state="yyrr"/>
        <phase duration="42" state="rrGG"/><phase duration="3" state="rryy"/>
        <param key="linkSignalID:0" value="100"/><param key="linkSignalID:1" value="100"/>
        <param key="linkSignalID:2" value="101"/><param key="linkSignalID:3" value="101"/>
    </tlLogic></net>""")
    assert tool.sumo_signals(net) == [('7', {'100': 1, '101': 2})]


def test_sumo_signals_refuse_a_mixed_signal():
    net = ET.fromstring("""<net><tlLogic id="7"><phase duration="42" state="Gr"/>
        <param key="linkSignalID:0" value="100"/><param key="linkSignalID:1" value="100"/></tlLogic></net>""")
    with pytest.raises(tool.BuildError):
        tool.sumo_signals(net)


# ---------- Town10HD ----------

needs_inputs = pytest.mark.skipif(not (TEMPLATE.is_file() and CARLA_JSON.is_file()),
                                  reason='雛形の Town01.inpx か ~/autoware_map/Town10HD/carla_tl.json が無い')


@needs_inputs
def test_town10hd_is_what_the_tool_writes(tmp_path):
    out, mapping = tmp_path / 'Town10HD.inpx', tmp_path / 'signal_mapping.json'
    assert tool.main(['--road', str(VISSIM / 'Town10HD_road.inpx'), '--template', str(TEMPLATE),
                      '--sumo-net', str(SUMO / 'net' / 'Town10HD.net.xml'),
                      '--sumo-rou', str(SUMO / 'rou' / 'Town10HD.rou.xml'),
                      '--carla-json', str(CARLA_JSON), '--out', str(out), '--signal-mapping-out', str(mapping)]) == 0
    assert out.read_bytes() == (VISSIM / 'Town10HD.inpx').read_bytes()
    assert json.loads(mapping.read_text()) == json.loads((VISSIM / 'signal_mapping_town10hd.json').read_text())


def test_committed_town10hd_signals_follow_sumo_and_carla():
    root = ET.parse(VISSIM / 'Town10HD.inpx').getroot()
    mapping = json.loads((VISSIM / 'signal_mapping_town10hd.json').read_text())
    signals = tool.sumo_signals(ET.parse(SUMO / 'net' / 'Town10HD.net.xml').getroot())
    # 信号制御 = SUMO の信号の順、信号グループ = SUMO の最初のフェーズで青なら 1
    expected = {}
    for sc, (_, groups) in enumerate(signals, start=1):
        for landmark, group in groups.items():
            expected.setdefault(str(sc), {}).setdefault(str(group), []).append(int(landmark))
    assert {sc: {sg: sorted(ids) for sg, ids in g.items()} for sc, g in mapping.items()} == \
        {sc: {sg: sorted(ids) for sg, ids in g.items()} for sc, g in expected.items()}
    assert sorted(i for g in mapping.values() for ids in g.values() for i in ids) == \
        [943, 944, 945, 949, 950, 951, 952, 953, 954, 957, 958, 959, 960, 961, 962]
    # 信号灯は 15 信号 × 2 車線。どれも、その信号の信号グループに付く
    heads = list(root.find('signalHeads'))
    assert len(heads) == 30
    group_of = {i: (sc, sg) for sc, g in mapping.items() for sg, ids in g.items() for i in ids}
    for head in heads:
        landmark = int(head.get('name').split()[1])
        assert tuple(head.get('sg').split()) == group_of[landmark]
    # ドライブシミュレータ(co-sim に要る)と、Town01 と同じ自車のタイプ
    net = root.find('netPara')
    assert (net.get('drivSimActive'), net.get('drivSimVehType'), net.get('trafReg')) == ('true', '101', 'RIGHTHANDTRAFFIC')
    assert len(root.find('signalControllers')) == 5
