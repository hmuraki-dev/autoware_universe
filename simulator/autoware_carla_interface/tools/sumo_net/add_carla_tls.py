#!/usr/bin/env python3
"""
Adds SUMO traffic lights linked to the CARLA traffic lights to an existing (hand-edited) SUMO net.

The net in examples/net was edited with netedit (crosswalks, sidewalks), so it cannot be regenerated
with netconvert_carla.py without losing those edits. This script instead:

1. generates a reference net with netconvert_carla.py from the OpenDRIVE file. It tells which SUMO
   connection is controlled by which CARLA traffic light (landmark id).
2. takes every 'priority' junction of the input net whose connections are signalized in the reference
   net, and builds a static program for it. The signalized approaches are grouped by direction: two
   approaches coming from opposite sides (heading differs by more than 135 degrees) form one group.
   Each group gets its own green (42s) -> yellow (3s), one group after the other:
       T-junction:  main road green/yellow -> side road green/yellow
       crossroads:  one road (2 opposite approaches) green/yellow -> the crossing road green/yellow
   Groups with a straight ('s') connection go first, so on a T-junction the main road comes first.
   The left turn and U-turn of a group with 2 approaches get 'g' (they yield to the opposite
   straight). All links of one CARLA landmark always share a colour.
   Each link gets <param key="linkSignalID:<linkIndex>" value="<landmark id>"/>, which the
   co-simulation bridge uses to link SUMO and CARLA traffic lights.
3. turns those junctions into traffic lights (netconvert --tls.set) and loads the programs
   (netconvert --tllogic-files). Junctions that already are traffic lights are kept as they are.

Usage:
    SUMO_HOME=... python3 tools/sumo_net/add_carla_tls.py <map>.xodr <in>.net.xml -o <out>.net.xml \
        --netconvert-carla <CARLA>/Co-Simulation/Sumo/util/netconvert_carla.py

Copied from the CARLA repository (~/CARLA, branch main, commit c0cefa8, Co-Simulation/Sumo/util/add_carla_tls.py)
on 2026-11-25. The only change is --netconvert-carla: the original runs netconvert_carla.py from its own
directory; here it is given, so that the netconvert_carla.py shipped with the CARLA in use is run.
Steps: docs/SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md, 2.2.2.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

GREEN_TIME = 42
YELLOW_TIME = 3
NETCONVERT_CARLA = '~/CARLA_0.9.16_jp/LinuxNoEditor/Co-Simulation/Sumo/util/netconvert_carla.py'


def reference_signals(ref_net, edge_suffix):
    """Returns {(from, to, fromLane, toLane): landmark_id} of the reference net."""
    root = ET.parse(ref_net).getroot()
    signals = {}
    for tl in root.findall('tlLogic'):
        params = {int(p.get('key').split(':')[1]): p.get('value') for p in tl.findall('param')}
        for c in root.findall('connection'):
            if c.get('tl') == tl.get('id'):
                key = (c.get('from') + edge_suffix, c.get('to') + edge_suffix, c.get('fromLane'),
                       c.get('toLane'))
                signals[key] = params[int(c.get('linkIndex'))]
    return signals


def lane_headings(root):
    """Returns {lane id: driving direction in degrees at the end of the lane}."""
    headings = {}
    for lane in root.iter('lane'):
        points = [tuple(map(float, p.split(',')[:2])) for p in lane.get('shape').split()]
        (x0, y0), (x1, y1) = points[-2], points[-1]
        headings[lane.get('id')] = math.degrees(math.atan2(y1 - y0, x1 - x0))
    return headings


def approach_groups(jid, links, headings):
    """Groups the approaches of a junction into roads: 2 approaches coming from opposite sides form one
    group, any other approach is a group of its own. Groups with a straight connection come first,
    then in linkIndex order. Returns a list of sets of edge ids."""
    approaches = []
    for key, _, _ in links:
        if key[0] not in approaches:
            approaches.append(key[0])
    heading = {e: headings['{}_{}'.format(e, next(k[2] for k, _, _ in links if k[0] == e))]
               for e in approaches}

    def angle(a, b):
        return abs((heading[a] - heading[b] + 180) % 360 - 180)

    groups = []
    left = list(approaches)
    while left:
        a = left.pop(0)
        opposite = [b for b in left if angle(a, b) > 135]
        if len(opposite) > 1:
            sys.exit('junction {}: approach {} has several opposite approaches {}'.format(
                jid, a, opposite))
        group = {a}
        if opposite:
            left.remove(opposite[0])
            group.add(opposite[0])
        groups.append(group)
    straight = {key[0] for key, d, _ in links if d == 's'}
    groups.sort(key=lambda g: not (g & straight))  # stable: keeps linkIndex order otherwise
    return groups


def build_programs(net, signals):
    """Returns (tllogics xml root, [junction ids]) for the priority junctions to be signalized."""
    root = ET.parse(net).getroot()
    connections = root.findall('connection')
    headings = lane_headings(root)
    out = ET.Element('tlLogics')
    junctions = []
    for junction in root.findall('junction'):
        if junction.get('type') != 'priority':
            continue
        incoming = []
        for lane in junction.get('incLanes').split():
            edge = lane.rsplit('_', 1)[0]
            if edge not in incoming and not edge.startswith(':'):
                incoming.append(edge)
        links = []  # (connection key, dir, landmark id) in linkIndex order
        for edge in incoming:
            for c in connections:
                key = (c.get('from'), c.get('to'), c.get('fromLane'), c.get('toLane'))
                if c.get('from') == edge and key in signals:
                    links.append((key, c.get('dir'), signals[key]))
        if not links:
            continue

        jid = junction.get('id')
        for landmark in {s for _, _, s in links}:
            if len({key[0] for key, _, s in links if s == landmark}) != 1:
                sys.exit('junction {}: landmark {} controls several approaches'.format(jid, landmark))

        groups = approach_groups(jid, links, headings)

        def state(group):
            opposed = len(group) == 2
            return ''.join(('g' if opposed and d in 'lLt' else 'G') if key[0] in group else 'r'
                           for key, d, _ in links)

        phases = []
        for group in groups:
            green = state(group)
            phases.append((green, GREEN_TIME))
            phases.append((green.replace('G', 'y').replace('g', 'y'), YELLOW_TIME))
        tl = ET.SubElement(out, 'tlLogic', id=jid, type='static', programID='0', offset='0')
        for st, duration in phases:
            ET.SubElement(tl, 'phase', duration=str(duration), state=st)
        for i, (_, _, landmark) in enumerate(links):
            ET.SubElement(tl, 'param', key='linkSignalID:{}'.format(i), value=landmark)
        for i, (key, _, _) in enumerate(links):
            ET.SubElement(out, 'connection', {'from': key[0], 'to': key[1], 'fromLane': key[2],
                                              'toLane': key[3], 'tl': jid, 'linkIndex': str(i)})
        junctions.append(jid)
        print('junction {}: landmarks {} {}'.format(
            jid, sorted({s for _, _, s in links}), [p for p, _ in phases]))
    return out, junctions


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('xodr_file', help='opendrive file (*.xodr)')
    parser.add_argument('net_file', help='sumo net to add the traffic lights to (*.net.xml)')
    parser.add_argument('--output', '-o', required=True, help='output sumo net (*.net.xml)')
    parser.add_argument('--edge-suffix', default='.0.00',
                        help='suffix of the edge ids of net_file compared to the reference net '
                        '(default: .0.00)')
    parser.add_argument('--netconvert-carla', default=os.path.expanduser(NETCONVERT_CARLA),
                        help='netconvert_carla.py of the CARLA in use, which makes the reference net '
                        '(default: {})'.format(NETCONVERT_CARLA))
    args = parser.parse_args()
    if not os.path.isfile(args.netconvert_carla):
        parser.error('netconvert_carla.py not found: {}'.format(args.netconvert_carla))

    with tempfile.TemporaryDirectory() as tmpdir:
        ref_net = os.path.join(tmpdir, 'reference.net.xml')
        subprocess.check_call([sys.executable, args.netconvert_carla,
                               args.xodr_file, '--output', ref_net])

        programs, junctions = build_programs(args.net_file,
                                             reference_signals(ref_net, args.edge_suffix))
        if not junctions:
            print('No junctions to signalize.')
            return
        tll = os.path.join(tmpdir, 'carla_tls.tll.xml')
        ET.ElementTree(programs).write(tll, encoding='UTF-8', xml_declaration=True)

        tls_set_net = os.path.join(tmpdir, 'tls_set.net.xml')
        subprocess.check_call(['netconvert', '--sumo-net-file', args.net_file,
                               '--tls.set', ','.join(junctions),
                               '--offset.disable-normalization', 'true',
                               '--output-file', tls_set_net])
        subprocess.check_call(['netconvert', '--sumo-net-file', tls_set_net,
                               '--tllogic-files', tll,
                               '--offset.disable-normalization', 'true',
                               '--output-file', args.output])
    print('Added traffic lights at junctions {} -> {}'.format(','.join(junctions), args.output))


if __name__ == '__main__':
    main()
