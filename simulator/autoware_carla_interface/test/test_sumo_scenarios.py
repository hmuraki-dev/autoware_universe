"""sumo_cfg_file の解決(sumo_integration/sumo_scenarios.py)。

相対パスはインストール先の share/autoware_carla_interface/sumo/examples から、絶対パスはそのまま使う。
ここではインストール先の代わりに、ソースの sumo/examples(または一時ディレクトリ)を base_dir に渡す。
SUMO・CARLA・ROS は使わない。

Run with:
    python3 -m pytest test/test_sumo_scenarios.py
"""
import os
from pathlib import Path

import pytest

from autoware_carla_interface.sumo_integration import sumo_fleet
from autoware_carla_interface.sumo_integration.sumo_scenarios import resolve_sumo_cfg_file

PACKAGE = Path(__file__).resolve().parents[1]
EXAMPLES = PACKAGE / 'sumo' / 'examples'


def test_relative_path_is_taken_from_the_examples_dir():
    assert resolve_sumo_cfg_file('Town01.sumocfg', EXAMPLES) == str(EXAMPLES / 'Town01.sumocfg')
    assert resolve_sumo_cfg_file(' Town10HD.sumocfg ', EXAMPLES) == str(EXAMPLES / 'Town10HD.sumocfg')


def test_absolute_path_is_used_as_it_is(tmp_path):
    cfg = tmp_path / 'x.sumocfg'
    cfg.write_text('<configuration/>', encoding='utf-8')
    # base_dir は使わない
    assert resolve_sumo_cfg_file(str(cfg), EXAMPLES) == str(cfg)


def test_home_is_expanded(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    (tmp_path / 'x.sumocfg').write_text('<configuration/>', encoding='utf-8')
    assert resolve_sumo_cfg_file('~/x.sumocfg', EXAMPLES) == str(tmp_path / 'x.sumocfg')


def test_empty_stays_empty():
    assert resolve_sumo_cfg_file('', EXAMPLES) == ''


def test_missing_file_names_the_path_tried(tmp_path):
    with pytest.raises(ValueError) as caught:
        resolve_sumo_cfg_file('Town99.sumocfg', tmp_path)
    message = str(caught.value)
    assert str(tmp_path / 'Town99.sumocfg') in message and 'relative' in message
    with pytest.raises(ValueError):
        resolve_sumo_cfg_file(str(tmp_path / 'nothing.sumocfg'), EXAMPLES)


@pytest.mark.parametrize('town', ['Town01', 'Town05', 'Town10HD'])
def test_every_scenario_resolves_with_its_route_files(town):
    path = resolve_sumo_cfg_file(f'{town}.sumocfg', EXAMPLES)
    # .sumocfg の中の相対パス(route-files)も、同じディレクトリから読める
    assert all(os.path.isfile(p) for _, p in sumo_fleet.route_file_names(path))


def test_cmake_installs_the_scenarios():
    text = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'install(DIRECTORY sumo DESTINATION share/${PROJECT_NAME}/)' in text
