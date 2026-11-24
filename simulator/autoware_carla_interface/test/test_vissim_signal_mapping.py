"""Vissim の信号の対応表を起動の引数 vissim_signal_mapping で選ぶ(vissim_integration/signal_mapping.py)。

- auto: CARLA のマップ名から data/signal_mapping_<town>.json を選ぶ(Town10HD_Opt -> town10hd)。
- ファイル名は data/ から、絶対パスはそのまま。無いときは ValueError。
- data/ の対応表が JSON として読め、Town10HD 用が vissim/signal_mapping_town10hd.json(生成物)と同じ。
- InitializeInterface が use_vissim・sync_traffic_lights のときだけ選び、無ければ起動を止める。

ROS 2・CARLA・Vissim は使わない(carla_autoware の ROS・CARLA に依存するモジュールはモックにする)。

Run with:
    python3 -m pytest test/test_vissim_signal_mapping.py
"""
import importlib.util
import json
from pathlib import Path
import sys
from unittest import mock

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE / 'src'
DATA = SRC / 'autoware_carla_interface' / 'vissim_integration' / 'data'

_spec = importlib.util.spec_from_file_location(
    'vissim_signal_mapping_under_test',
    SRC / 'autoware_carla_interface' / 'vissim_integration' / 'signal_mapping.py')
signal_mapping = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(signal_mapping)


@pytest.mark.parametrize('carla_map, name', [
    ('Town01', 'signal_mapping_town01.json'),
    ('Town10HD_Opt', 'signal_mapping_town10hd.json'),
    ('Town10HD', 'signal_mapping_town10hd.json'),
    ('/home/user/autoware_map/Town10HD_Opt/', 'signal_mapping_town10hd.json'),
])
def test_auto_file_name(carla_map, name):
    assert signal_mapping.auto_file_name(carla_map) == name


def test_auto_without_map_name_is_refused():
    with pytest.raises(ValueError):
        signal_mapping.auto_file_name('')


def test_auto_picks_the_town_file(tmp_path):
    (tmp_path / 'signal_mapping_town10hd.json').write_text('{}')
    assert signal_mapping.resolve_signal_mapping_path('auto', 'Town10HD_Opt', tmp_path) == \
        str(tmp_path / 'signal_mapping_town10hd.json')
    # 空の値も auto と同じ
    assert signal_mapping.resolve_signal_mapping_path('', 'Town10HD_Opt', tmp_path) == \
        str(tmp_path / 'signal_mapping_town10hd.json')


def test_file_name_is_looked_up_in_data_dir(tmp_path):
    (tmp_path / 'mine.json').write_text('{}')
    assert signal_mapping.resolve_signal_mapping_path('mine.json', 'Town01', tmp_path) == \
        str(tmp_path / 'mine.json')


def test_absolute_path_is_used_as_is(tmp_path):
    other = tmp_path / 'elsewhere'
    other.mkdir()
    path = other / 'mine.json'
    path.write_text('{}')
    assert signal_mapping.resolve_signal_mapping_path(str(path), 'Town01', tmp_path / 'data') == \
        str(path)


def test_missing_file_names_the_path_and_the_known_mappings(tmp_path):
    (tmp_path / 'signal_mapping_town01.json').write_text('{}')
    with pytest.raises(ValueError) as error:
        signal_mapping.resolve_signal_mapping_path('auto', 'Town05', tmp_path)
    assert 'signal_mapping_town05.json' in str(error.value)
    assert 'signal_mapping_town01.json' in str(error.value)


def test_packaged_mappings():
    names = sorted(p.name for p in DATA.glob('signal_mapping*.json'))
    # 以前の data/signal_mapping.json(Town を替えるたびに上書きしていたもの)は無い
    assert names == ['signal_mapping_town01.json', 'signal_mapping_town10hd.json']
    for name in names:
        mapping = json.loads((DATA / name).read_text())
        assert mapping and all(isinstance(ids, list) for groups in mapping.values() for ids in groups.values())
    town01 = json.loads((DATA / 'signal_mapping_town01.json').read_text())
    assert len(town01) == 12
    assert town01['1'] == {'1': [360, 362], '2': [361]}


def test_town10hd_mapping_is_the_generated_one():
    # vissim/signal_mapping_town10hd.json は tools/vissim/build_vissim_town.py(feature/sumo_co-sim)の生成物
    assert (DATA / 'signal_mapping_town10hd.json').read_bytes() == \
        (PACKAGE / 'vissim' / 'signal_mapping_town10hd.json').read_bytes()


# ---------------------------------------------------------------- InitializeInterface

_STUBBED_MODULES = [
    'carla', 'zmq', 'msgpack',
    'autoware_carla_interface.carla_ros',
    'autoware_carla_interface.modules',
    'autoware_carla_interface.modules.carla_data_provider',
    'autoware_carla_interface.modules.carla_wrapper',
    'autoware_carla_interface.modules.traffic_light_publisher',
]

_BASE_PARAMS = {
    'host': 'localhost', 'port': 2000, 'timeout': 20, 'sync_mode': True, 'fixed_delta_seconds': 0.05,
    'carla_map': 'Town10HD_Opt', 'ego_vehicle_role_name': 'ego_vehicle',
    'vehicle_type': 'vehicle.toyota.prius', 'spawn_point': 'None', 'use_traffic_manager': False,
    'max_real_delta_seconds': 0.05, 'weather': 'ClearNoon',
    'traffic_light.publish': False, 'traffic_light.force_green': False, 'traffic_light.map_path': '',
    'traffic_light.match_distance': 5.0, 'traffic_light.match_ratio': 0.6, 'traffic_light.id_map': '',
    'use_vissim': True, 'vissim_adapter_host': '127.0.0.1', 'vissim_adapter_port': 5555,
    'vissim_connect_timeout_ms': 60000, 'vissim_rpc_timeout_ms': 2000, 'vissim_simulator_vehicles': 1,
    'sync_traffic_lights': True, 'vissim_sim_period': 600, 'vissim_max_consecutive_failures': 3,
    'vissim_carla_variants': 'legacy_vtypes', 'vissim_signal_mapping': 'auto',
    'vissim_warmup_time': 0, 'ego_spawn_front_margin': 20.0, 'ego_spawn_rear_margin': 20.0,
    'ego_spawn_wait_timeout': 60,
}


@pytest.fixture
def ca():
    stubs = {name: mock.MagicMock() for name in _STUBBED_MODULES}
    with mock.patch.dict(sys.modules, stubs):
        sys.path.insert(0, str(SRC))
        try:
            from autoware_carla_interface import carla_autoware
            yield carla_autoware
        finally:
            sys.path.remove(str(SRC))


def _make(ca, **overrides):
    ca.carla_ros2_interface.return_value.get_param.return_value = dict(_BASE_PARAMS, **overrides)
    return ca.InitializeInterface()


def test_interface_resolves_auto_from_the_carla_map(ca):
    assert _make(ca).vissim_signal_mapping_path == str(DATA / 'signal_mapping_town10hd.json')
    assert _make(ca, carla_map='Town01').vissim_signal_mapping_path == \
        str(DATA / 'signal_mapping_town01.json')


def test_interface_uses_the_given_file(ca):
    interface = _make(ca, vissim_signal_mapping='signal_mapping_town01.json')
    assert interface.vissim_signal_mapping_path == str(DATA / 'signal_mapping_town01.json')


def test_interface_refuses_a_missing_mapping(ca):
    with pytest.raises(ValueError):
        _make(ca, carla_map='Town05')
    with pytest.raises(ValueError):
        _make(ca, vissim_signal_mapping='no_such_mapping.json')


def test_interface_ignores_the_mapping_without_signal_sync(ca):
    # 信号を同期しないとき・Vissim を使わないときは、対応表が無くても起動できる(今までと同じ)
    assert _make(ca, carla_map='Town05', sync_traffic_lights=False).vissim_signal_mapping_path is None
    assert _make(ca, carla_map='Town05', use_vissim=False).vissim_signal_mapping_path is None
