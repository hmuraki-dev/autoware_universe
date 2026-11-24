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
