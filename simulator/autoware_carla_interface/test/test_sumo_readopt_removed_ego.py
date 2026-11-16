# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""SUMO が CARLA 由来の車(EGO)を自分で消したとき、ブリッジが落ちずに SUMO へ入れ直すこと。

SUMO は、time-to-teleport(既定 300 秒)止まったままの車を teleport する(行き先の辺を越えると消す)。
Autoware が走り出す前の EGO がこれで消され、sync_carla_to_sumo() が消えた ID を読んで KeyError で落ちていた
(docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md S5 の結果の 7)。

CARLA サーバ・SUMO・ROS は使わない。

Run with:
    python3 -m pytest test/test_sumo_readopt_removed_ego.py
"""
import os
import sys
import types
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stand_ins import install_stand_ins  # noqa: E402  pylint: disable=wrong-import-position

install_stand_ins()

import carla  # noqa: E402  pylint: disable=wrong-import-position,import-error

from autoware_carla_interface.sumo_integration import bridge_helper as bh  # noqa: E402
from autoware_carla_interface.sumo_integration import simulation_synchronization as ss  # noqa: E402

EGO = 900


def make_sync(sumo_destroyed=(), carla_destroyed=()):
    sync = ss.SimulationSynchronization.__new__(ss.SimulationSynchronization)
    sync.sync_vehicle_color = False
    sync.sync_vehicle_lights = False
    sync.tls_manager = 'none'
    sync.ego_role_name = 'ego_vehicle'
    sync.sumo2carla_ids = {}
    sync.sumo2carla_extra_ids = {}
    sync.carla2sumo_ids = {EGO: 'carla0'}
    sync.sumo = mock.MagicMock()
    sync.sumo.destroyed_actors = set(sumo_destroyed)
    alive_in_sumo = {'carla0'} - set(sumo_destroyed)

    def get_actor(sumo_id):
        if sumo_id not in alive_in_sumo:
            raise KeyError(79)             # traci の購読結果に無い(SUMO が消した車)
        return types.SimpleNamespace(signals=0)
    sync.sumo.get_actor.side_effect = get_actor

    def spawn_actor(type_id, color):
        alive_in_sumo.add('carla1')
        return 'carla1'
    sync.sumo.spawn_actor.side_effect = spawn_actor
    sync.carla = mock.MagicMock()
    sync.carla.spawned_actors = set()
    sync.carla.destroyed_actors = set(carla_destroyed)
    sync.carla.get_actor.return_value = types.SimpleNamespace(
        attributes={'role_name': 'ego_vehicle', 'color': '1,2,3'}, get_transform=lambda: carla.Transform(),
        bounding_box=types.SimpleNamespace(extent=carla.Vector3D(2.0, 1.0, 0.7)))
    return sync


def run(sync):
    with mock.patch.object(bh.BridgeHelper, 'get_sumo_vtype', return_value='vehicle.toyota.prius'), \
            mock.patch.object(bh.BridgeHelper, 'get_sumo_transform', return_value=carla.Transform()), \
            mock.patch.object(ss.logging, 'warning') as warning:
        sync.sync_carla_to_sumo()
    return warning


def test_ego_removed_by_sumo_is_adopted_again():
    sync = make_sync(sumo_destroyed={'carla0'})
    warning = run(sync)
    assert sync.carla2sumo_ids == {EGO: 'carla1'}
    sync.sumo.spawn_actor.assert_called_once_with('vehicle.toyota.prius', ss.EGO_SUMO_COLOR)
    sync.sumo.subscribe.assert_called_once_with('carla1')
    # 同じ step のうちに、新しい ID で位置を送る(古い ID は読まない)
    assert [c.args[0] for c in sync.sumo.synchronize_vehicle.call_args_list] == ['carla1']
    sync.sumo.destroy_actor.assert_not_called()
    warning.assert_called_once()
    assert 'adopting it into sumo again' in warning.call_args.args[0] % warning.call_args.args[1:]


def test_ego_removed_in_both_is_dropped():
    """CARLA 側でも同じ step で消えていれば、入れ直さない。"""
    sync = make_sync(sumo_destroyed={'carla0'}, carla_destroyed={EGO})
    run(sync)
    assert sync.carla2sumo_ids == {}
    sync.sumo.spawn_actor.assert_not_called()
    sync.sumo.synchronize_vehicle.assert_not_called()


def test_nothing_changes_while_sumo_keeps_the_ego():
    sync = make_sync()
    warning = run(sync)
    assert sync.carla2sumo_ids == {EGO: 'carla0'}
    sync.sumo.spawn_actor.assert_not_called()
    assert [c.args[0] for c in sync.sumo.synchronize_vehicle.call_args_list] == ['carla0']
    warning.assert_not_called()


def test_sumo_npc_arrivals_do_not_touch_the_ego():
    """SUMO の車(sumo2carla)が到着しても、EGO の対応は変わらない。"""
    sync = make_sync(sumo_destroyed={'in1_A.3'})
    run(sync)
    assert sync.carla2sumo_ids == {EGO: 'carla0'}
    sync.sumo.spawn_actor.assert_not_called()
