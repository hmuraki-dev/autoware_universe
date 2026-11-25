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
    assert not inpx.has_static_routes
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


def test_static_routes_are_refused(tmp_path):
    extra = '<vehicleRoutingDecisionsStatic><vehicleRoutingDecisionStatic no="1" link="1">' \
            '<vehRoutSta><vehicleRouteStatic no="1" destLink="1"/></vehRoutSta>' \
            '</vehicleRoutingDecisionStatic></vehicleRoutingDecisionsStatic>'
    with pytest.raises(rou.RouError):
        rou.check_supported(reader.Inpx(_inpx(tmp_path, extra=extra)))


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


# ---------------------------------------------------------------- Vissim ブリッジの信号の対応表(vissim_to_signal_mapping.py)

mapping_tool = _load('vissim_to_signal_mapping')
MAPPING = VISSIM / 'signal_mapping_town10hd.json'


def test_signal_mapping_from_heads(tmp_path):
    heads = ('<signalHead no="1" name="CARLA 952" sg="1 1" lane="1 1"/>'
             '<signalHead no="2" name="CARLA 949" sg="1 1" lane="1 2"/>'
             '<signalHead no="3" name="CARLA 949" sg="1 1" lane="1 1"/>'
             '<signalHead no="4" name="CARLA 950" sg="1 2" lane="1 1"/>')
    mapping = mapping_tool.build(reader.Inpx(_inpx(tmp_path, heads=heads)))
    assert mapping == {'1': {'1': [949, 952], '2': [950]}}


def test_signal_mapping_group_missing_in_the_controller_is_refused(tmp_path):
    heads = '<signalHead no="1" name="CARLA 949" sg="1 3" lane="1 1"/>'
    with pytest.raises(reader.InpxError):
        mapping_tool.build(reader.Inpx(_inpx(tmp_path, heads=heads)))


def test_signal_mapping_without_carla_heads_is_refused(tmp_path):
    with pytest.raises(reader.InpxError):
        mapping_tool.build(reader.Inpx(_inpx(tmp_path)))


def test_town10hd_signal_mapping_is_reproduced(tmp_path):
    """vissim/signal_mapping_town10hd.json(build_vissim_town.py が作ったもの)と同じバイト列になる。"""
    out = tmp_path / 'signal_mapping_town10hd.json'
    assert mapping_tool.main(['--inpx', str(VISSIM / 'Town10HD.inpx'), '--out', str(out)]) == 0
    assert out.read_bytes() == MAPPING.read_bytes()
