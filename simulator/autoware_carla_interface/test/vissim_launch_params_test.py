#!/usr/bin/env python3

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
"""
Static check of launch/autoware_carla_interface.launch.xml's two autoware_carla_interface node
declarations (see docs/Vissim_CARLA_Autoware_シミュレーション期間管理_実装計画_v1.0.md Step V6).

The node is declared twice, selected by use_vissim, because only the use_vissim variant must
carry on_exit="shutdown" (on_exit does not accept substitutions). This test guards against the two
<param> lists drifting apart, and against launch-side parameters that the node does not declare
(or vice versa).

Needs neither ROS 2 nor CARLA - it only parses XML/Python source. Run with:
    python3 test/vissim_launch_params_test.py
Exits with a non-zero status and an AssertionError if any check fails.
"""

import ast
import os
import re
import xml.etree.ElementTree as ET

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LAUNCH_FILE = os.path.join(_PKG_DIR, 'launch', 'autoware_carla_interface.launch.xml')
_CARLA_ROS_FILE = os.path.join(_PKG_DIR, 'src', 'autoware_carla_interface', 'carla_ros.py')


def _interface_nodes(root):
    return [
        node for node in root.iter('node')
        if node.get('pkg') == 'autoware_carla_interface' and
        node.get('exec') == 'autoware_carla_interface'
    ]


def _params(node):
    return [(param.get('name'), param.get('value')) for param in node.findall('param')]


def _declared_ros_parameters():
    """
    Returns the parameter names declared in carla_ros.py's `self.parameters = {...}` table,
    read from the source (importing carla_ros.py would require rclpy).
    """
    tree = ast.parse(open(_CARLA_ROS_FILE, encoding='utf-8').read())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1 and
                isinstance(node.targets[0], ast.Attribute) and
                node.targets[0].attr == 'parameters' and isinstance(node.value, ast.Dict)):
            return {key.value for key in node.value.keys}
    raise AssertionError('parameter table not found in %s' % _CARLA_ROS_FILE)


def check_two_variants_selected_by_use_vissim():
    nodes = _interface_nodes(ET.parse(_LAUNCH_FILE).getroot())
    assert len(nodes) == 2, 'expected 2 autoware_carla_interface nodes, found %d' % len(nodes)

    without_vissim = [n for n in nodes if n.get('unless') == '$(var use_vissim)']
    with_vissim = [n for n in nodes if n.get('if') == '$(var use_vissim)']
    assert len(without_vissim) == 1 and len(with_vissim) == 1, [n.attrib for n in nodes]

    # Only the use_vissim variant shuts down the whole launch when the node exits; CARLA-only
    # operation keeps its previous behavior.
    assert with_vissim[0].get('on_exit') == 'shutdown', with_vissim[0].attrib
    assert without_vissim[0].get('on_exit') is None, without_vissim[0].attrib

    # Apart from if/unless/on_exit, both declarations must be the same node.
    strip = lambda attrib: {k: v for k, v in attrib.items() if k not in ('if', 'unless', 'on_exit')}
    assert strip(with_vissim[0].attrib) == strip(without_vissim[0].attrib)


def check_param_lists_identical():
    nodes = _interface_nodes(ET.parse(_LAUNCH_FILE).getroot())
    first, second = (_params(n) for n in nodes)
    assert first == second, 'the two <param> lists differ:\n  %s\n  %s' % (
        sorted(set(first) - set(second)), sorted(set(second) - set(first)))
    names = [name for name, _ in first]
    assert len(names) == len(set(names)), 'duplicate <param> names: %s' % names


def check_params_match_node_declarations():
    root = ET.parse(_LAUNCH_FILE).getroot()
    launch_params = {name for name, _ in _params(_interface_nodes(root)[0])}
    declared = _declared_ros_parameters()
    assert launch_params == declared, (
        'launch <param>s not declared by carla_ros.py: %s / declared by carla_ros.py but not '
        'passed by the launch file: %s' % (sorted(launch_params - declared),
                                            sorted(declared - launch_params)))


def check_param_values_reference_declared_args():
    root = ET.parse(_LAUNCH_FILE).getroot()
    defined = {arg.get('name') for arg in root.iter('arg')} | \
        {let.get('name') for let in root.iter('let')}
    for name, value in _params(_interface_nodes(root)[0]):
        for var in re.findall(r'\$\(var ([^)\s]+)\)', value):
            assert var in defined, '<param name="%s"> references undefined $(var %s)' % (name, var)


def run():
    check_two_variants_selected_by_use_vissim()
    check_param_lists_identical()
    check_params_match_node_declarations()
    check_param_values_reference_declared_args()
    print('All launch parameter checks passed.')


if __name__ == '__main__':
    run()
