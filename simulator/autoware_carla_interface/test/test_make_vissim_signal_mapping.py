"""Vissim の inpx の信号灯から信号の対応表を作るツール(tools/make_vissim_signal_mapping.py)。

- 信号灯の名前「CARLA <id>」と信号グループから {信号制御: {信号グループ: [CARLA の信号 ID]}} を作る。
- 写せない信号灯(名前・信号グループの食い違い)では止める。
- 今の vissim/Town10HD.inpx からは、コミットした対応表(data/ と vissim/)と同じバイト列になる。

CARLA・Vissim・SUMO は使わない。

Run with:
    python3 -m pytest test/test_make_vissim_signal_mapping.py
"""
import importlib.util
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
DATA = PACKAGE / 'src' / 'autoware_carla_interface' / 'vissim_integration' / 'data'
VISSIM = PACKAGE / 'vissim'

_spec = importlib.util.spec_from_file_location('make_vissim_signal_mapping',
                                               PACKAGE / 'tools' / 'make_vissim_signal_mapping.py')
tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tool)


def _inpx(tmp_path, heads):
    path = tmp_path / 'small.inpx'
    path.write_text(f'''<?xml version="1.0" encoding="UTF-8"?>
<network>
  <signalControllers>
    <signalController no="1" name="SC1"><sigGrps><signalGroup no="1"/><signalGroup no="2"/></sigGrps></signalController>
    <signalController no="2" name="SC2"><sigGrps><signalGroup no="1"/><signalGroup no="2"/></sigGrps></signalController>
  </signalControllers>
  <signalHeads>{heads}</signalHeads>
</network>''', encoding='utf-8')
    return path


def _build(tmp_path, heads):
    return tool.build(*tool.read_inpx(_inpx(tmp_path, heads)))


def test_mapping_from_heads(tmp_path):
    heads = ('<signalHead no="1" name="CARLA 952" sg="1 1" lane="1 1"/>'
             '<signalHead no="2" name="CARLA 949" sg="1 1" lane="1 2"/>'
             '<signalHead no="3" name="CARLA 949" sg="1 1" lane="1 1"/>'
             '<signalHead no="4" name="CARLA 950" sg="1 2" lane="1 1"/>'
             '<signalHead no="5" name=" CARLA 943 " sg="2 1" lane="2 1"/>')
    assert _build(tmp_path, heads) == {'1': {'1': [949, 952], '2': [950]}, '2': {'1': [943]}}


@pytest.mark.parametrize('heads', [
    '<signalHead no="1" name="Signal head 1" sg="1 1" lane="1 1"/>',               # 名前が CARLA <id> でない
    '<signalHead no="1" name="CARLA x9" sg="1 1" lane="1 1"/>',                    # ID が数でない
    '<signalHead no="1" name="CARLA 949" sg="1 1" lane="1 1"/>'
    '<signalHead no="2" name="CARLA 949" sg="1 2" lane="1 2"/>',                   # 同じ信号が別の信号グループ
    '<signalHead no="1" name="CARLA 949" sg="3 1" lane="1 1"/>',                   # 信号制御が無い
    '<signalHead no="1" name="CARLA 949" sg="1 3" lane="1 1"/>',                   # 信号グループが無い
    '<signalHead no="1" name="CARLA 949" lane="1 1"/>',                            # 信号グループが付いていない
    '',                                                                            # 信号灯が無い
])
def test_heads_that_cannot_be_mapped_are_refused(tmp_path, heads):
    with pytest.raises(tool.MappingError):
        _build(tmp_path, heads)


def test_nothing_is_written_when_refused(tmp_path):
    out = tmp_path / 'mapping.json'
    inpx = _inpx(tmp_path, '<signalHead no="1" name="CARLA 949" sg="1 3" lane="1 1"/>')
    assert tool.main(['--inpx', str(inpx), '--out', str(out)]) == 1
    assert not out.exists()


def test_town10hd_mapping_is_reproduced(tmp_path):
    """今の Town10HD.inpx から作ると、コミットした対応表(data/・vissim/)と同じバイト列になる。"""
    outs = [tmp_path / 'data.json', tmp_path / 'vissim.json']
    assert tool.main(['--inpx', str(VISSIM / 'Town10HD.inpx'),
                      '--out', str(outs[0]), '--out', str(outs[1])]) == 0
    for out in outs:
        assert out.read_bytes() == (DATA / 'signal_mapping_town10hd.json').read_bytes()
        assert out.read_bytes() == (VISSIM / 'signal_mapping_town10hd.json').read_bytes()
