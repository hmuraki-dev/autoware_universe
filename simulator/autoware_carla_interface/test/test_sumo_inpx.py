"""tools/sumo_inpx/(Vissim の .inpx から SUMO の信号・需要・寸法を作る)。

前半は小さな入力で信号プログラムの展開と読み取りを確かめる。後半は vissim/Town10HD.inpx と
sumo/net/Town10HD.net.xml から作り、コミットした生成物と同じになるか、値が期待どおりかを確かめる
(sumolib が import できるときだけ)。CARLA・Vissim・traci は使わない。

Run with:
    python3 -m pytest test/test_sumo_inpx.py
"""
import base64
import importlib.util
from pathlib import Path
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
TOOLS = PACKAGE / 'tools' / 'sumo_inpx'
VISSIM = PACKAGE / 'vissim'
SUMO = PACKAGE / 'sumo'


def _load(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


reader = _load('inpx_reader')

try:
    reader.sumo_tools_on_path()
    import sumolib  # noqa: F401
    HAVE_SUMOLIB = True
except Exception:  # noqa: BLE001
    HAVE_SUMOLIB = False
needs_sumolib = pytest.mark.skipif(not HAVE_SUMOLIB, reason='sumolib が無い')

DISPLAYS = '''
  <signaldisplays>
    <display id="1" name="Red" state="RED"/>
    <display id="2" name="Red/Amber" state="REDAMBER"/>
    <display id="3" name="Green" state="GREEN"/>
    <display id="4" name="Amber" state="AMBER"/>
  </signaldisplays>'''

# Town10HD.inpx の信号制御と同じ形(ステージ 2 つ、間 5 s、周期 94 s)
STAGE_SUPPLY = f'''<sc id="1">{DISPLAYS}
  <stages>
    <stage id="1" isPseudoStage="false"><activations>
      <activation sg_id="1" activation="ON"/><activation sg_id="2" activation="OFF"/></activations></stage>
    <stage id="2" isPseudoStage="false"><activations>
      <activation sg_id="1" activation="OFF"/><activation sg_id="2" activation="ON"/></activations></stage>
  </stages>
  <interstageProgs>
    <interstageProg id="1" cycletime="5000" fromStage="1" toStage="2"><sgs>
      <sg sg_id="1" signal_sequence="3"><cmds><cmd display="3" begin="0"/><cmd display="1" begin="3000"/></cmds>
        <fixedstates><fixedstate display="4" duration="3000"/></fixedstates></sg>
      <sg sg_id="2" signal_sequence="3"><cmds><cmd display="1" begin="0"/><cmd display="3" begin="5000"/></cmds>
        <fixedstates><fixedstate display="2" duration="0"/></fixedstates></sg></sgs></interstageProg>
    <interstageProg id="2" cycletime="5000" fromStage="2" toStage="1"><sgs>
      <sg sg_id="1" signal_sequence="3"><cmds><cmd display="1" begin="0"/><cmd display="3" begin="5000"/></cmds>
        <fixedstates><fixedstate display="2" duration="0"/></fixedstates></sg>
      <sg sg_id="2" signal_sequence="3"><cmds><cmd display="3" begin="0"/><cmd display="1" begin="3000"/></cmds>
        <fixedstates><fixedstate display="4" duration="3000"/></fixedstates></sg></sgs></interstageProg>
  </interstageProgs>
  <stageProgs>
    <stageProg id="1" cycletime="94000"><interstages>
      <interstage display="1" begin="42000"/><interstage display="2" begin="89000"/></interstages></stageProg>
  </stageProgs>
</sc>'''

# 信号グループ方式(移植元のお台場の形): 青 0-100、黄 100-104、赤。周期 151 s
GROUP_SUPPLY = f'''<sc id="1">{DISPLAYS}
  <progs><prog id="1" cycletime="151000"><sgs>
    <sg sg_id="1" signal_sequence="3"><cmds><cmd display="3" begin="0"/><cmd display="1" begin="104000"/></cmds>
      <fixedstates><fixedstate display="4" duration="4000"/></fixedstates></sg>
  </sgs></prog></progs>
</sc>'''


def test_stage_program_is_expanded_per_signal_group():
    cycle, timeline = reader.signal_timeline(STAGE_SUPPLY)
    assert cycle == 94.0
    assert timeline['1'] == [(0.0, 'GREEN'), (42.0, 'AMBER'), (45.0, 'RED')]
    assert timeline['2'] == [(0.0, 'RED'), (47.0, 'GREEN'), (89.0, 'AMBER'), (92.0, 'RED')]


def test_signal_group_program_amber_ends_at_red_command():
    cycle, timeline = reader.signal_timeline(GROUP_SUPPLY)
    assert cycle == 151.0
    assert timeline['1'] == [(0.0, 'GREEN'), (100.0, 'AMBER'), (104.0, 'RED')]


def test_more_than_one_program_is_refused():
    two = GROUP_SUPPLY.replace('</progs>', '<prog id="2" cycletime="10000"><sgs/></prog></progs>')
    with pytest.raises(reader.InpxError):
        reader.signal_timeline(two)


def test_colour_at():
    timeline = [(0.0, 'RED'), (47.0, 'GREEN'), (89.0, 'AMBER'), (92.0, 'RED')]
    assert reader.colour_at(timeline, 0.0) == 'RED'
    assert reader.colour_at(timeline, 47.0) == 'GREEN'
    assert reader.colour_at(timeline, 91.9) == 'AMBER'


def _inpx(tmp_path, heads='', inputs='', extra=''):
    supply = base64.b64encode(STAGE_SUPPLY.encode()).decode()
    text = f'''<?xml version="1.0" encoding="UTF-8"?>
<network>
  <drivingBehaviors><drivingBehavior no="1" carFollowModType="WIEDEMANN74" w74ax="2"/></drivingBehaviors>
  <linkBehaviorTypes><linkBehaviorType no="1" drivBehavDef="1"/></linkBehaviorTypes>
  <links>
    <link no="1" name="1: Road 1-0-Right" linkBehavType="1"><lanes><lane width="3.5"/><lane width="3.5"/></lanes>
      <linkPolyPts><linkPolyPoint x="0" y="0"/><linkPolyPoint x="100" y="0"/></linkPolyPts></link>
  </links>
  <signalControllers><signalController no="1" name="SC" type="FIXEDTIME" intSupplyData="{supply}">
    <sigGrps><signalGroup no="1"/><signalGroup no="2"/></sigGrps></signalController></signalControllers>
  <signalHeads>{heads}</signalHeads>
  <vehicleInputs>{inputs}</vehicleInputs>
  {extra}
</network>'''
    path = tmp_path / 'small.inpx'
    path.write_text(text, encoding='utf-8')
    return path


def test_reads_controllers_and_signal_heads(tmp_path):
    heads = ('<signalHead no="1" name="CARLA 949" sg="1 1" lane="1 1"/>'
             '<signalHead no="2" name="CARLA 949" sg="1 1" lane="1 2"/>'
             '<signalHead no="3" name="CARLA 950" sg="1 2" lane="1 1"/>')
    inpx = reader.Inpx(_inpx(tmp_path, heads=heads))
    assert list(inpx.controllers) == ['1']
    assert inpx.controllers['1']['groups'] == ['1', '2']
    assert reader.carla_signal_groups(inpx) == {'949': ('1', '1'), '950': ('1', '2')}
    assert inpx.links['1']['widths'] == [3.5, 3.5]
    assert inpx.behavior['carFollowModType'] == 'WIEDEMANN74'


def test_carla_signal_in_two_groups_is_refused(tmp_path):
    heads = ('<signalHead no="1" name="CARLA 949" sg="1 1" lane="1 1"/>'
             '<signalHead no="2" name="CARLA 949" sg="1 2" lane="1 2"/>')
    with pytest.raises(reader.InpxError):
        reader.carla_signal_groups(reader.Inpx(_inpx(tmp_path, heads=heads)))


def test_input_with_two_time_intervals_is_refused(tmp_path):
    inputs = ('<vehicleInput no="1" link="1"><timeIntVehVols>'
              '<timeIntervalVehVolume timeInt="1 0" vehComp="1" volType="STOCHASTIC" volume="10"/>'
              '<timeIntervalVehVolume timeInt="1 900" vehComp="1" volType="STOCHASTIC" volume="20"/>'
              '</timeIntVehVols></vehicleInput>')
    with pytest.raises(reader.InpxError):
        reader.Inpx(_inpx(tmp_path, inputs=inputs))


def test_function_value_is_piecewise_linear(tmp_path):
    extra = ('<desAccelerationFunctions><desAccelerationFunction no="1"><accelFuncDataPts>'
             '<accelerationFunctionDataPoint x="0" y="3"/><accelerationFunctionDataPoint x="10" y="2"/>'
             '</accelFuncDataPts></desAccelerationFunction></desAccelerationFunctions>')
    inpx = reader.Inpx(_inpx(tmp_path, extra=extra))
    assert inpx.function_value('desAccelerationFunctions', '1', 5) == pytest.approx(2.5)
    assert inpx.function_value('desAccelerationFunctions', '1', 50) == pytest.approx(2.0)


# ---------------------------------------------------------------- Town10HD

@pytest.fixture(scope='module')
def town10hd():
    import sumolib
    return (reader.Inpx(VISSIM / 'Town10HD.inpx'),
            sumolib.net.readNet(str(SUMO / 'net' / 'Town10HD.net.xml'), withInternal=True))


@needs_sumolib
def test_town10hd_inputs_map_to_one_edge_each(town10hd):
    inpx, net = town10hd
    edges = {item['link']: reader.link_to_edge(inpx, net, item['link']).getID() for item in inpx.inputs}
    assert len(edges) == 19
    assert len(set(edges.values())) == 19
    # 1 車線の Road 11 と、名前と edge の番号が違う Road 10(netconvert が道路をつないだ edge)
    assert edges['23'] == '-11'
    assert edges['21'] == '-3'


@needs_sumolib
def test_town10hd_controllers(town10hd):
    inpx, _ = town10hd
    assert len(inpx.controllers) == 5
    for controller in inpx.controllers.values():
        cycle, timeline = reader.signal_timeline(controller['supply'])
        assert cycle == 94.0
        assert timeline['1'] == [(0.0, 'GREEN'), (42.0, 'AMBER'), (45.0, 'RED')]
    assert len(reader.carla_signal_groups(inpx)) == 15
    assert inpx.routing_decisions == []
    assert inpx.speed_decisions == []
    assert inpx.pedestrian_inputs == 0


# ---------------------------------------------------------------- S2: 信号プログラム(vissim_to_tll.py)

tll = _load('vissim_to_tll')
TLL = SUMO / 'tll' / 'Town10HD_vissim.tll.xml'


def _program(phases, links):
    return dict(phases=phases, links=links, program='0',
                params=[(f'linkSignalID:{i}', v) for i, v in sorted(links.items())])


def test_green_chars_are_copied_from_the_current_program():
    groups, chars = tll.green_chars(['GgGrr', 'yyyrr', 'rrrGG', 'rrryy'])
    assert groups == {frozenset({0, 1, 2}), frozenset({3, 4})}
    assert chars == {0: 'G', 1: 'g', 2: 'G', 3: 'G', 4: 'G'}


def test_mixed_g_and_G_for_one_link_is_refused():
    with pytest.raises(tll.TllError):
        tll.green_chars(['Gr', 'gr'])


def test_small_controller_becomes_six_phases(tmp_path):
    heads = ('<signalHead no="1" name="CARLA 949" sg="1 1" lane="1 1"/>'
             '<signalHead no="2" name="CARLA 950" sg="1 2" lane="1 1"/>')
    inpx = reader.Inpx(_inpx(tmp_path, heads=heads))
    programs = {'189': _program(['GgGrr', 'yyyrr', 'rrrGG', 'rrryy'],
                                {0: '949', 1: '949', 2: '949', 3: '950', 4: '950'})}
    logics, leftover = tll.build(inpx, programs)
    assert leftover == []
    assert logics[0]['phases'] == [(42.0, 'GgGrr'), (3.0, 'yyyrr'), (2.0, 'rrrrr'),
                                   (42.0, 'rrrGG'), (3.0, 'rrryy'), (2.0, 'rrrrr')]


def test_green_group_not_in_the_current_program_is_refused(tmp_path):
    # Vissim では 949 と 950 が別の信号グループだが、網では同じフェーズで青 -> 青の組が今のプログラムに無い
    heads = ('<signalHead no="1" name="CARLA 949" sg="1 1" lane="1 1"/>'
             '<signalHead no="2" name="CARLA 950" sg="1 2" lane="1 1"/>')
    inpx = reader.Inpx(_inpx(tmp_path, heads=heads))
    programs = {'189': _program(['GG', 'yy'], {0: '949', 1: '950'})}
    with pytest.raises(tll.TllError):
        tll.build(inpx, programs)


def test_carla_signal_missing_in_the_net_is_refused(tmp_path):
    heads = '<signalHead no="1" name="CARLA 999" sg="1 1" lane="1 1"/>'
    inpx = reader.Inpx(_inpx(tmp_path, heads=heads))
    with pytest.raises(tll.TllError):
        tll.build(inpx, {'189': _program(['G'], {0: '949'})})


def test_town10hd_tll_is_reproduced(tmp_path):
    out = tmp_path / 'Town10HD_vissim.tll.xml'
    assert tll.main(['--inpx', str(VISSIM / 'Town10HD.inpx'), '--net', str(SUMO / 'net' / 'Town10HD.net.xml'),
                     '--out', str(out)]) == 0
    assert out.read_bytes() == TLL.read_bytes()


def test_town10hd_tll_values():
    import xml.etree.ElementTree as ET
    net = tll.read_net_programs(SUMO / 'net' / 'Town10HD.net.xml')
    logics = {t.get('id'): t for t in ET.parse(str(TLL)).getroot().iter('tlLogic')}
    assert sorted(logics) == sorted(net) == ['189', '23', '468', '532', '719']
    for tl_id, logic in logics.items():
        assert logic.get('programID') == 'vissim'
        assert logic.get('offset') == '0'
        phases = [(float(p.get('duration')), p.get('state')) for p in logic.findall('phase')]
        assert [d for d, _ in phases] == [42, 3, 2, 42, 3, 2]
        current = net[tl_id]['phases']
        # 青の 2 フェーズと黄の 2 フェーズは、今のプログラム(青 42・黄 3 ×2)と同じ。全赤 2 s を足しただけ
        assert [phases[0][1], phases[1][1], phases[3][1], phases[4][1]] == current
        assert set(phases[2][1]) == set(phases[5][1]) == {'r'}
        # ブリッジが CARLA の信号機を引く param は、今のプログラムと同じ
        assert [(p.get('key'), p.get('value')) for p in logic.findall('param')] == net[tl_id]['params']


# ---------------------------------------------------------------- S3: 需要と寸法(vissim_to_rou.py)

rou = _load('vissim_to_rou')
ROU = SUMO / 'rou' / 'Town10HD_vissim.rou.xml'


def test_speed_bins():
    assert rou.speed_bins([(30.0, 0.0), (35.0, 1.0)]) == [(1.0, 30 / 3.6, 35 / 3.6)]
    assert rou.speed_bins([(30.0, 0.0), (35.0, 0.5), (35.0, 0.5), (40.0, 1.0)]) == \
        [(0.5, 30 / 3.6, 35 / 3.6), (0.5, 35 / 3.6, 40 / 3.6)]


INPUT_ON_LINK_1 = ('<vehicleInput no="1" link="1"><timeIntVehVols>'
                   '<timeIntervalVehVolume timeInt="1 0" vehComp="1" volType="STOCHASTIC" volume="10"/>'
                   '</timeIntVehVols></vehicleInput>')


def _decision(link='1', all_types='true', rel_flow='1 0:3'):
    return (f'<vehicleRoutingDecisionsStatic><vehicleRoutingDecisionStatic no="1" link="{link}" pos="5" '
            f'allVehTypes="{all_types}"><vehRoutSta>'
            f'<vehicleRouteStatic no="1" destLink="1" destPos="20" relFlow="{rel_flow}">'
            '<linkSeq><intObjectRef key="1"/></linkSeq></vehicleRouteStatic>'
            '</vehRoutSta></vehicleRoutingDecisionStatic></vehicleRoutingDecisionsStatic>')


def test_static_routes_are_read(tmp_path):
    inpx = reader.Inpx(_inpx(tmp_path, inputs=INPUT_ON_LINK_1, extra=_decision()))
    assert inpx.routing_decisions == [dict(no='1', link='1', pos=5.0, all_types=True, routes=[
        dict(no='1', dest='1', dest_pos=20.0, share=3.0, via=['1'])])]
    rou.check_supported(inpx)


def test_route_share_with_two_time_intervals_is_refused(tmp_path):
    with pytest.raises(reader.InpxError):
        reader.Inpx(_inpx(tmp_path, inputs=INPUT_ON_LINK_1, extra=_decision(rel_flow='1 0:3 900:1')))


def test_routing_decision_off_the_input_link_is_refused(tmp_path):
    with pytest.raises(rou.RouError):
        rou.check_supported(reader.Inpx(_inpx(tmp_path, extra=_decision())))


def test_routing_decision_for_some_classes_is_refused(tmp_path):
    with pytest.raises(rou.RouError):
        rou.check_supported(reader.Inpx(_inpx(tmp_path, inputs=INPUT_ON_LINK_1, extra=_decision(all_types='false'))))


def test_speed_decisions_are_read(tmp_path):
    extra = ('<desSpeedDecisions><desSpeedDecision no="1" lane="1 2" pos="10"><desSpeedDistrs>'
             '<vehClassDesSpeedDistribution vehClass="10" desSpeedDistr="50"/></desSpeedDistrs></desSpeedDecision>'
             '</desSpeedDecisions><vehicleClasses><vehicleClass no="10"><vehTypes><intObjectRef key="100"/>'
             '</vehTypes></vehicleClass></vehicleClasses>')
    inpx = reader.Inpx(_inpx(tmp_path, extra=extra))
    assert inpx.speed_decisions == [dict(no='1', link='1', lane=2, pos=10.0, per_class={'10': '50'})]
    assert inpx.vehicle_classes == {'10': ['100']}
    assert rou.override_of(inpx, dict(per_class={'10': '50'})) == {'100': '50'}


def test_branch_shares_split_at_each_branch():
    class E:
        def __init__(self, name):
            self.name = name

        def getID(self):
            return self.name

    a, b, c, d, e = E('a'), E('b'), E('c'), E('d'), E('e')
    assert rou.branch_shares([[a, b], [a, c, d], [a, c, e]]) == [0.5, 0.25, 0.25]
    assert rou.branch_shares([[a, b]]) == [1.0]


def test_first_speed_decision_on_the_path():
    points = [dict(edge='-2', pos=10.0, link='5'), dict(edge='4', pos=30.0, link='3')]
    assert rou.first_speed_decision(points, ['4', '-2'], 0.0)['link'] == '3'
    # 出発位置より後ろの決定点は通らない
    assert rou.first_speed_decision(points, ['4', '-2'], 40.0)['link'] == '5'
    assert rou.first_speed_decision(points, ['1', '2'], 0.0) is None


# Town10HD.inpx に足す経路決定(流入 1 のリンク 3 -> 90 -> 5 -> 82 -> 7 と、-> 85 -> 44)と
# 希望速度決定(リンク 5 の 2 車線。乗用車 50 km/h、バス 40 km/h)
TOWN10HD_ROUTES = '''
  <vehicleRoutingDecisionsStatic>
    <vehicleRoutingDecisionStatic allVehTypes="true" link="3" no="1" pos="5"><vehRoutSta>
      <vehicleRouteStatic destLink="7" destPos="20" no="1" relFlow="1 0:3"><linkSeq>
        <intObjectRef key="10002"/><intObjectRef key="90"/><intObjectRef key="10126"/><intObjectRef key="5"/>
        <intObjectRef key="10004"/><intObjectRef key="82"/><intObjectRef key="10118"/></linkSeq></vehicleRouteStatic>
      <vehicleRouteStatic destLink="44" destPos="20" no="2" relFlow="1 0:1"><linkSeq>
        <intObjectRef key="10002"/><intObjectRef key="90"/><intObjectRef key="10126"/><intObjectRef key="5"/>
        <intObjectRef key="10006"/><intObjectRef key="85"/><intObjectRef key="10121"/></linkSeq></vehicleRouteStatic>
      <vehicleRouteStatic destLink="82" destPos="23" no="3" relFlow="1 0:0"><linkSeq>
        <intObjectRef key="10002"/><intObjectRef key="90"/><intObjectRef key="10126"/><intObjectRef key="5"/>
        <intObjectRef key="10004"/></linkSeq></vehicleRouteStatic>
    </vehRoutSta></vehicleRoutingDecisionStatic>
  </vehicleRoutingDecisionsStatic>
  <desSpeedDecisions>
    <desSpeedDecision lane="5 1" no="1" pos="10"><desSpeedDistrs>
      <vehClassDesSpeedDistribution desSpeedDistr="50" vehClass="10"/>
      <vehClassDesSpeedDistribution desSpeedDistr="40" vehClass="30"/></desSpeedDistrs></desSpeedDecision>
    <desSpeedDecision lane="5 2" no="2" pos="10"><desSpeedDistrs>
      <vehClassDesSpeedDistribution desSpeedDistr="50" vehClass="10"/>
      <vehClassDesSpeedDistribution desSpeedDistr="40" vehClass="30"/></desSpeedDistrs></desSpeedDecision>
  </desSpeedDecisions>
'''


def _town10hd_with_routes(tmp_path, extra=TOWN10HD_ROUTES):
    text = (VISSIM / 'Town10HD.inpx').read_text(encoding='utf-8')
    path = tmp_path / 'Town10HD_routes.inpx'
    path.write_text(text.replace('\t<vehicleTypes>', extra + '\t<vehicleTypes>', 1), encoding='utf-8')
    return path


@needs_sumolib
def test_town10hd_static_routes_follow_the_route(tmp_path, town10hd):
    _, net = town10hd
    inpx = reader.Inpx(_town10hd_with_routes(tmp_path))
    rou.check_supported(inpx)
    notes = []
    options = rou.static_routes(inpx, net, inpx.inputs[0], net.getEdge('4'), notes)
    assert notes == []
    paths = {tuple(e.getID() for e in path): (round(weight, 3), round(arrival, 2), route)
             for weight, path, arrival, route in options}
    # 経路 3(割合 0)は使わない。リンク 7 は 4 m なので、destPos 20 は edge -3 の 4 m(リンクの終わり)
    assert paths == {('4', '-2', '-3'): (0.75, 4.0, '1'), ('4', '-2', '21'): (0.25, 20.63, '2')}
    vehicles, origins = rou.build_vehicles(inpx, net, 'vissim', 1, 3600.0, 2000.0)
    assert origins['1']['routes'] == 2
    routed = [v for v in vehicles if v['input'] == '1']
    assert {tuple(v['edges']) for v in routed} == set(paths)
    assert all(v['arrival_pos'] is not None for v in routed)
    assert all(v['arrival_pos'] is None for v in vehicles if v['input'] != '1')


@needs_sumolib
def test_town10hd_route_ending_inside_a_junction(tmp_path, town10hd):
    _, net = town10hd
    extra = TOWN10HD_ROUTES.replace('relFlow="1 0:3"', 'relFlow="1 0:0"').replace(
        'relFlow="1 0:1"', 'relFlow="1 0:0"').replace('destPos="23" no="3" relFlow="1 0:0"',
                                                      'destPos="23" no="3" relFlow="1 0:1"')
    inpx = reader.Inpx(_town10hd_with_routes(tmp_path, extra))
    options = rou.static_routes(inpx, net, inpx.inputs[0], net.getEdge('4'), [])
    # リンク 82(46 m)は交差点の中の道。その中ほどで消える経路は、交差点を出た edge -3 の頭で消す
    assert [[e.getID() for e in path] for _, path, _, _ in options] == [['4', '-2', '-3']]
    assert options[0][2] < 1.0


@needs_sumolib
def test_town10hd_speed_decision(tmp_path, town10hd):
    _, net = town10hd
    inpx = reader.Inpx(_town10hd_with_routes(tmp_path))
    assert [(p['edge'], p['link']) for p in rou.speed_points(inpx, net)] == [('-2', '5')]
    vehicles, _ = rou.build_vehicles(inpx, net, 'vissim', 1, 600.0, 2000.0)
    for v in vehicles:
        assert (v['speed'] == '5') == ('-2' in v['edges'])
    vtypes, distributions = rou.build_types(inpx, {'1'}, 'vissim')
    rou.assign_types(inpx, net, vehicles, 'vissim', vtypes, distributions)
    changed = {v['type'] for v in vehicles if v['speed']}
    assert changed == {'vissim_c1_s30_40_50'}
    members = dict(distributions['vissim_c1_s30_40_50'])
    assert sum(members.values()) == pytest.approx(1.0)
    speeds = {vtypes[key]['vClass']: key.split('_s')[1].split('_')[0] for key in members}
    assert speeds == {'passenger': '50', 'bus': '40', 'truck': '30', 'trailer': '30', 'motorcycle': '30'}
    assert {v['type'] for v in vehicles if not v['speed']} == {'vissim_c1'}


@needs_sumolib
def test_speed_decisions_that_differ_by_lane_are_refused(tmp_path, town10hd):
    _, net = town10hd
    extra = TOWN10HD_ROUTES.replace('desSpeedDistr="50" vehClass="10"/>\n      <vehClassDesSpeedDistribution '
                                    'desSpeedDistr="40" vehClass="30"/></desSpeedDistrs></desSpeedDecision>\n  </des',
                                    'desSpeedDistr="60" vehClass="10"/></desSpeedDistrs></desSpeedDecision>\n  </des')
    inpx = reader.Inpx(_town10hd_with_routes(tmp_path, extra))
    with pytest.raises(rou.RouError):
        rou.speed_points(inpx, net)


@needs_sumolib
def test_town10hd_types(town10hd):
    inpx, _ = town10hd
    vtypes, distributions = rou.build_types(inpx, {'1'}, 'vissim')
    members = dict(distributions['vissim_c1'])
    assert sum(members.values()) == pytest.approx(1.0)
    share = {}
    for key, p in members.items():
        share[vtypes[key]['vClass']] = share.get(vtypes[key]['vClass'], 0.0) + p
    assert share == pytest.approx({'passenger': 0.5, 'truck': 0.1, 'trailer': 0.1, 'bus': 0.2, 'motorcycle': 0.1})
    sizes = {key: (vtypes[key]['length'], vtypes[key]['width']) for key in vtypes}
    assert sizes['vissim_passenger1001_s30_b0'] == ('4.95', '1.85')       # アルファード
    assert sizes['vissim_truck1021_s30_b0'] == ('11.81', '2.49')          # GIGA
    assert sizes['vissim_motorcycle70_s30_b0'] == ('2.098', '0.907')      # MT-07
    assert vtypes['vissim_trailer22_s30_b0']['guiShape'] == 'truck'
    for spec in vtypes.values():
        assert spec['carFollowModel'] == 'Wiedemann'
        assert spec['minGap'] == '2'
        assert spec['speedFactor'] == 'normc(9.0278,100,8.3333,9.7222)'   # 30〜35 km/h


@needs_sumolib
def test_town10hd_routes_follow_the_net(town10hd):
    inpx, net = town10hd
    vehicles, origins = rou.build_vehicles(inpx, net, 'vissim', 1, 600.0, 2000.0)
    assert len(origins) == 19
    assert [v['depart'] for v in vehicles] == sorted(v['depart'] for v in vehicles)
    for v in vehicles:
        assert v['length'] >= 2000.0
        for a, b in zip(v['edges'], v['edges'][1:]):
            assert b in [e.getID() for e in rou.successors(net.getEdge(a))]
    again, _ = rou.build_vehicles(inpx, net, 'vissim', 1, 600.0, 2000.0)
    assert again == vehicles
    other, _ = rou.build_vehicles(inpx, net, 'vissim', 2, 600.0, 2000.0)
    assert other != vehicles


@needs_sumolib
def test_town10hd_rou_is_reproduced(tmp_path):
    out = tmp_path / 'Town10HD_vissim.rou.xml'
    assert rou.main(['--inpx', str(VISSIM / 'Town10HD.inpx'), '--net', str(SUMO / 'net' / 'Town10HD.net.xml'),
                     '--out', str(out)]) == 0
    assert out.read_bytes() == ROU.read_bytes()
    text = out.read_text(encoding='utf-8')
    assert text.count('<vehicle ') == 582
    assert '--' not in text.split('-->')[0][len('<?xml version="1.0" encoding="UTF-8"?>\n<!--'):]


# ---------------------------------------------------------------- 横断歩道(vissim_to_crossings.py)

crossings = _load('vissim_to_crossings')
HAVE_NETCONVERT = __import__('shutil').which('netconvert') is not None
needs_netconvert = pytest.mark.skipif(not HAVE_NETCONVERT, reason='netconvert が無い')

# Town10HD の地図(lanelet2_map_with_tl.osm)の横断歩道 9725(交差点 189 の北の枝を渡る)だけの地図
CROSSWALK_9725 = dict(left=[(-36.702, -4.993), (-57.289, -4.104)], right=[(-35.726, -2.535), (-58.324, -1.554)])


def _crosswalk_map(tmp_path):
    nodes, ids = [], {}
    for side in ('left', 'right'):
        for k, (x, y) in enumerate(CROSSWALK_9725[side]):
            ids[(side, k)] = 100 + len(ids)
            nodes.append(f'<node id="{ids[(side, k)]}" lat="0" lon="0"><tag k="ele" v="0"/>'
                         f'<tag k="local_x" v="{x}"/><tag k="local_y" v="{y}"/></node>')
    text = ('<?xml version="1.0"?>\n<osm version="0.6">\n' + '\n'.join(nodes)
            + f'\n<way id="200"><nd ref="{ids[("left", 0)]}"/><nd ref="{ids[("left", 1)]}"/></way>'
            + f'\n<way id="201"><nd ref="{ids[("right", 0)]}"/><nd ref="{ids[("right", 1)]}"/></way>'
            + '\n<relation id="9725"><member type="way" ref="200" role="left"/><member type="way" ref="201" role="right"/>'
            + '<tag k="type" v="lanelet"/><tag k="subtype" v="crosswalk"/></relation>\n</osm>\n')
    path = tmp_path / 'lanelet2_map_with_tl.osm'
    path.write_text(text, encoding='utf-8')
    return path


def _town10hd_with_crosswalk(tmp_path, head='1 1', offset=0.0, ped_volume=60):
    """Town10HD.inpx に、横断歩道 9725 に沿う歩行者リンク 9001・両端の待ち場所 9001/9002・歩行者入力・
    歩行者信号灯(信号グループ head。'' なら無し)を足す。offset [m] は歩行者リンクを横にずらす量。"""
    left, right = CROSSWALK_9725['left'], CROSSWALK_9725['right']
    s = ((left[0][0] + right[0][0]) / 2, (left[0][1] + right[0][1]) / 2)
    e = ((left[1][0] + right[1][0]) / 2, (left[1][1] + right[1][1]) / 2)
    length = ((e[0] - s[0]) ** 2 + (e[1] - s[1]) ** 2) ** 0.5
    u = ((e[0] - s[0]) / length, (e[1] - s[1]) / length)
    n = (-u[1], u[0])
    s, e = (s[0] + n[0] * offset, s[1] + n[1] * offset), (e[0] + n[0] * offset, e[1] + n[1] * offset)

    def rect(p, out):
        a = (p[0] + out[0] * 1.5, p[1] + out[1] * 1.5)
        b = (p[0] + out[0] * 5.5, p[1] + out[1] * 5.5)
        return [(a[0] + n[0] * 1.5, a[1] + n[1] * 1.5), (b[0] + n[0] * 1.5, b[1] + n[1] * 1.5),
                (b[0] - n[0] * 1.5, b[1] - n[1] * 1.5), (a[0] - n[0] * 1.5, a[1] - n[1] * 1.5)]

    def area(no, pts):
        return (f'<area no="{no}" name="wait"><geometry><points>'
                + ''.join(f'<point x="{x}" y="{y}"/>' for x, y in pts) + '</points></geometry></area>\n')

    text = (VISSIM / 'Town10HD.inpx').read_text(encoding='utf-8')
    link = (f'<link no="9001" name="crosswalk" isPedArea="true" linkBehavType="1"><geometry><linkPolyPts>'
            f'<linkPolyPoint x="{s[0]}" y="{s[1]}"/><linkPolyPoint x="{e[0]}" y="{e[1]}"/></linkPolyPts></geometry>'
            '<lanes><lane width="2.7"/></lanes></link>\n')
    text = text.replace('\t</links>', link + '\t</links>', 1)
    text = text.replace('\t</areas>', area(9001, rect(s, (-u[0], -u[1]))) + area(9002, rect(e, u)) + '\t</areas>', 1)
    ped = ('<pedestrianInputs><pedestrianInput no="1"><location area="9001"/><timeIntPedVols>'
           f'<timeIntervalPedVolume timeInt="1 0" pedComp="1" volType="STOCHASTIC" volume="{ped_volume}"/>'
           '</timeIntPedVols></pedestrianInput></pedestrianInputs>\n'
           '<pedestrianRoutingDecisionsStatic><pedestrianRoutingDecisionStatic no="1"><location area="9001"/>'
           '<pedRoutSta><pedestrianRouteStatic no="1" relFlow="1 0:1"><pedestrianRouteLocations><pedestrianRouteLocation area="9002"/>'
           '</pedestrianRouteLocations></pedestrianRouteStatic></pedRoutSta></pedestrianRoutingDecisionStatic>'
           '</pedestrianRoutingDecisionsStatic>\n')
    text = text.replace('\t<pedestrianTypes>', ped + '\t<pedestrianTypes>', 1)
    # 歩行者リンクと車のリンク 1 の競合エリア(前の余裕 0.5 s)
    text = text.replace('\t</conflictAreas>', '<conflictArea no="9001" linkA="9001" linkB="1" frontGapDef="0.5"'
                        ' rearGapDef="0.5"/>\n\t</conflictAreas>', 1)
    if head:
        text = text.replace('\t</signalHeads>',
                            f'<signalHead no="9001" name="ped" lane="9001 1" pos="1" sg="{head}"/>\n\t</signalHeads>', 1)
    path = tmp_path / 'Town10HD_ped.inpx'
    path.write_text(text, encoding='utf-8')
    return path


def test_pedestrian_links_areas_and_heads_are_read(tmp_path):
    inpx = reader.Inpx(_town10hd_with_crosswalk(tmp_path))
    assert inpx.links['9001']['is_ped'] and not inpx.links['1']['is_ped']
    assert inpx.waiting_areas == {'9001', '9002'}
    assert len(inpx.areas['9001']) == 4
    assert reader.pedestrian_signal_heads(inpx) == {'9001': {('1', '1')}}
    # 歩行者信号灯は CARLA の信号機の対応に入れない(名前が「CARLA <信号ID>」でなくても止めない)
    assert len(reader.carla_signal_groups(inpx)) == 15


def test_flashing_green_is_put_before_red():
    supply = GROUP_SUPPLY.replace('<display id="4" name="Amber" state="AMBER"/>',
                                  '<display id="4" name="Amber" state="AMBER"/>'
                                  '<display id="5" name="Flash" state="FLASHING_GREEN"/>')
    supply = supply.replace('<fixedstate display="4" duration="4000"/>', '<fixedstate display="5" duration="5000"/>')
    _, timeline = reader.signal_timeline(supply)
    assert timeline['1'] == [(0.0, 'GREEN'), (99.0, 'FLASHING_GREEN'), (104.0, 'RED')]


@needs_sumolib
def test_town10hd_has_no_crossing(tmp_path, town10hd, capsys):
    _, net = town10hd
    assert crossings.build(reader.Inpx(VISSIM / 'Town10HD.inpx'), net, _crosswalk_map(tmp_path)) == []
    out = tmp_path / 'net' / 'Town10HD_vissim.net.xml'
    assert crossings.main(['--inpx', str(VISSIM / 'Town10HD.inpx'), '--net', str(SUMO / 'net' / 'Town10HD.net.xml'),
                           '--map', str(_crosswalk_map(tmp_path)), '--out', str(out)]) == 0
    assert '歩行者リンクが無い' in capsys.readouterr().out
    assert not out.exists()


@needs_sumolib
def test_crossing_is_matched_to_the_leg(tmp_path, town10hd):
    _, net = town10hd
    items = crossings.build(reader.Inpx(_town10hd_with_crosswalk(tmp_path)), net, _crosswalk_map(tmp_path))
    assert len(items) == 1
    item = items[0]
    assert (item['crosswalk'], item['links'], item['node'], item['edges']) == ('9725', ['9001'], '189', ['-22', '-15'])
    assert [sw['id'] for sw in item['sidewalks']] == ['ped_area9001', 'ped_area9002']


@needs_sumolib
def test_pedestrian_link_off_the_crosswalks_is_refused(tmp_path, town10hd):
    _, net = town10hd
    inpx = reader.Inpx(_town10hd_with_crosswalk(tmp_path, offset=6.0))
    with pytest.raises(crossings.CrossingError):
        crossings.build(inpx, net, _crosswalk_map(tmp_path))


@needs_sumolib
@needs_netconvert
def test_crossing_net_and_pedestrian_signal(tmp_path):
    inpx = _town10hd_with_crosswalk(tmp_path)
    out = tmp_path / 'net' / 'Town10HD_vissim.net.xml'
    assert crossings.main(['--inpx', str(inpx), '--net', str(SUMO / 'net' / 'Town10HD.net.xml'),
                           '--map', str(_crosswalk_map(tmp_path)), '--out', str(out)]) == 0
    base = tll.read_net_programs(SUMO / 'net' / 'Town10HD.net.xml')
    programs = tll.read_net_programs(out)
    # 車の linkIndex・linkSignalID はそのまま。横断歩道は後ろ(18)に付く
    assert programs['189']['params'] == base['189']['params']
    assert list(programs['189']['crossings']) == [18]
    assert all(programs[t]['phases'] == base[t]['phases'] for t in base if t != '189')
    logics, _ = tll.build(reader.Inpx(inpx), programs)
    phases = {logic['id']: logic['phases'] for logic in logics}
    # 歩行者信号灯は信号グループ 1(車の 0〜3・8〜12 と同じ時間に青)。その間、交わる車のリンク 0 は g
    assert phases['189'] == [(42.0, 'gGGgrrrrGGGggrrrrrG'), (3.0, 'yyyyrrrryyyyyrrrrrr'), (2.0, 'r' * 19),
                             (42.0, 'rrrrGGGgrrrrrGGGggr'), (3.0, 'rrrryyyyrrrrryyyyyr'), (2.0, 'r' * 19)]
    # 横断歩道の無い信号は、今の網から作るものと同じ
    before, _ = tll.build(reader.Inpx(inpx), base)
    assert {k: v for k, v in phases.items() if k != '189'} == \
        {logic['id']: logic['phases'] for logic in before if logic['id'] != '189'}


@needs_sumolib
@needs_netconvert
def test_crossing_without_pedestrian_signal_is_refused(tmp_path):
    inpx = _town10hd_with_crosswalk(tmp_path, head='')
    out = tmp_path / 'net' / 'Town10HD_vissim.net.xml'
    assert crossings.main(['--inpx', str(inpx), '--net', str(SUMO / 'net' / 'Town10HD.net.xml'),
                           '--map', str(_crosswalk_map(tmp_path)), '--out', str(out)]) == 0
    with pytest.raises(tll.TllError):
        tll.build(reader.Inpx(inpx), tll.read_net_programs(out))


# ---------------------------------------------------------------- 歩行者(vissim_to_rou.py)

def test_pedestrian_inputs_and_routes_are_read(tmp_path):
    inpx = reader.Inpx(_town10hd_with_crosswalk(tmp_path))
    assert inpx.ped_inputs == [dict(no='1', area='9001', volume=60.0, composition='1')]
    assert inpx.ped_routes == {'9001': [dict(no='1', dest='9002', share=1.0)]}
    assert inpx.ped_compositions['1'] == [('100', 1.0, '1022'), ('200', 1.0, '1023')]
    assert inpx.ped_conflict_gaps == [0.5]
    rou.check_supported(inpx)


def test_pedestrian_input_without_route_is_refused(tmp_path):
    text = _town10hd_with_crosswalk(tmp_path).read_text(encoding='utf-8')
    path = tmp_path / 'no_route.inpx'
    path.write_text(text.replace('<location area="9001"/>\n<pedRoutSta>', '<location area="8888"/>\n<pedRoutSta>')
                    .replace('<pedestrianRoutingDecisionStatic no="1"><location area="9001"/>',
                             '<pedestrianRoutingDecisionStatic no="1"><location area="8888"/>'), encoding='utf-8')
    with pytest.raises(rou.RouError):
        rou.check_supported(reader.Inpx(path))


@needs_sumolib
def test_pedestrians_need_the_sidewalks(tmp_path, town10hd):
    _, net = town10hd
    inpx = reader.Inpx(_town10hd_with_crosswalk(tmp_path))
    notes = []
    assert rou.build_persons(inpx, net, 'vissim', 1, 3600.0, {}, {}, notes) == []
    assert 'ped_area9001' in notes[0]


@needs_sumolib
@needs_netconvert
def test_pedestrians_walk_between_the_waiting_areas(tmp_path):
    import sumolib
    path = _town10hd_with_crosswalk(tmp_path)
    out = tmp_path / 'net' / 'Town10HD_vissim.net.xml'
    assert crossings.main(['--inpx', str(path), '--net', str(SUMO / 'net' / 'Town10HD.net.xml'),
                           '--map', str(_crosswalk_map(tmp_path)), '--out', str(out)]) == 0
    net = sumolib.net.readNet(str(out), withInternal=True)
    inpx = reader.Inpx(path)
    vtypes, distributions = rou.build_types(inpx, {'1'}, 'vissim')
    notes = []
    persons = rou.build_persons(inpx, net, 'vissim', 1, 3600.0, vtypes, distributions, notes)
    assert 40 <= len(persons) <= 80                      # 60 人/h
    assert {(p['origin'], p['dest'], p['type']) for p in persons} == {('ped_area9001', 'ped_area9002', 'vissim_pc1')}
    members = dict(distributions['vissim_pc1'])
    assert sum(members.values()) == pytest.approx(1.0)
    assert {vtypes[k]['vClass'] for k in members} == {'pedestrian'}
    assert rou.build_persons(inpx, net, 'vissim', 1, 3600.0, vtypes, distributions, []) == persons
    # 車の jmCrossingGap = 0.5 s x 歩行者の最大の希望速度 + 1 m
    gap = rou.crossing_gap(inpx, vtypes, notes)
    fastest = max(float(vtypes[k]['speedFactor'].split(',')[-1].rstrip(')')) for k in members)
    assert gap == pytest.approx(0.5 * fastest + 1.0)
    assert all(spec.get('jmCrossingGap') == reader.fmt(gap, 3) for spec in vtypes.values()
               if spec['vClass'] != 'pedestrian')
    rou_out = tmp_path / 'Town10HD_vissim.rou.xml'
    assert rou.main(['--inpx', str(path), '--net', str(out), '--out', str(rou_out), '--horizon', '600']) == 0
    text = rou_out.read_text(encoding='utf-8')
    assert '<person ' in text and '<walk from="ped_area9001" to="ped_area9002"' in text
