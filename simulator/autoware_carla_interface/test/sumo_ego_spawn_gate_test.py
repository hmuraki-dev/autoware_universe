#!/usr/bin/env python3
"""
Step S5 unit tests for the EGO spawn gap check `ego_spawn_gate.evaluate_spawn_gap()` (see
docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md, section 2.6 / Step S5).

`ego_spawn_gate` imports neither carla, traci nor ROS, so this runs anywhere. Not wired into
colcon/ament (no pytest infra exists for this package yet); run manually with:
    python3 test/sumo_ego_spawn_gate_test.py

Exits with a non-zero status and an AssertionError if any check fails.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from autoware_carla_interface.sumo_integration.ego_spawn_gate import (  # noqa: E402  pylint: disable=wrong-import-position
    EGO_SPAWN_SEARCH_RANGE_M,
    GapActor,
    describe_gap_result,
    evaluate_spawn_gap,
)

# EGO: 4 m x 2 m at the origin heading +x (yaw 0), on a 3.5 m wide lane; both margins 20 m.
EGO_LENGTH = 4.0
EGO_WIDTH = 2.0
LANE_WIDTH = 3.5
MARGIN = 20.0


def _car(actor_id, s, d=0.0, yaw=0.0, length=4.0, width=2.0, source='carla', ego_yaw=0.0):
    """An actor at longitudinal s / lateral d in the frame of an EGO at the origin with ego_yaw."""
    a = math.radians(ego_yaw)
    x = s * math.cos(a) - d * math.sin(a)
    y = s * math.sin(a) + d * math.cos(a)
    return GapActor(actor_id, x, y, ego_yaw + yaw, length, width, source)


def _check(actors, ego_yaw=0.0, front_margin=MARGIN, rear_margin=MARGIN):
    return evaluate_spawn_gap(0.0, 0.0, ego_yaw, EGO_LENGTH, EGO_WIDTH, LANE_WIDTH, actors,
                              front_margin, rear_margin)


def test_no_vehicles():
    result = _check([])
    assert result.safe and result.front is None and result.rear is None and not result.overlaps


def test_front_clearance():
    # clearance = s - 4/2 - 4/2 = s - 4
    below = _check([_car('f', 23.9)])
    assert not below.safe and abs(below.front.clearance - 19.9) < 1e-9, below
    exact = _check([_car('f', 24.0)])
    assert exact.safe and exact.front.clearance == 20.0, 'clearance == margin is safe: %r' % (exact,)
    above = _check([_car('f', 30.0)])
    assert above.safe and above.front.clearance == 26.0 and above.rear is None


def test_rear_clearance():
    below = _check([_car('r', -23.9)])
    assert not below.safe and abs(below.rear.clearance - 19.9) < 1e-9 and below.front is None
    exact = _check([_car('r', -24.0)])
    assert exact.safe and exact.rear.clearance == 20.0
    above = _check([_car('r', -30.0)])
    assert above.safe and above.rear.clearance == 26.0


def test_lengths_and_separate_margins():
    # A 10 m bus ahead: 30 - 2 - 5 = 23 m. A 2 m bike behind: 10 - 2 - 1 = 7 m.
    result = _check([_car('bus', 30.0, length=10.0), _car('bike', -10.0, length=2.0, width=0.8)],
                    front_margin=20.0, rear_margin=5.0)
    assert result.safe and result.front.clearance == 23.0 and result.rear.clearance == 7.0
    assert not _check([_car('bike', -10.0, length=2.0, width=0.8)], rear_margin=7.5).safe


def test_nearest_vehicle_is_used():
    result = _check([_car('far', 60.0), _car('near', 30.0), _car('behind_far', -80.0),
                     _car('behind_near', -40.0)])
    assert result.front.actor.actor_id == 'near' and result.rear.actor.actor_id == 'behind_near'


def test_adjacent_lane_is_ignored():
    result = _check([_car('left', 5.0, d=-3.5), _car('right', -5.0, d=3.5)])
    assert result.safe and result.front is None and result.rear is None and not result.overlaps
    # just inside half a lane width counts as the same lane
    assert not _check([_car('drifting', 10.0, d=1.7)]).safe


def test_opposite_lane_is_ignored():
    result = _check([_car('oncoming', 10.0, d=-3.5, yaw=180.0)])
    assert result.safe and result.front is None
    # opposite heading in the EGO lane itself (e.g. a U-turning vehicle) is not a leader either,
    # but it is still caught by the overlap check when it touches the EGO footprint
    assert _check([_car('wrong_way', 10.0, yaw=180.0)]).front is None
    assert not _check([_car('wrong_way_on_top', 1.0, yaw=180.0)]).safe


def test_heading_tolerance_and_wraparound():
    assert _check([_car('turning', 10.0, yaw=40.0)]).front is not None
    assert _check([_car('crossing', 10.0, yaw=90.0)]).front is None
    # EGO at yaw 180 (like spawn point #37) and a vehicle reported at -180 / 179: same direction
    assert _check([_car('a', 10.0, yaw=-360.0, ego_yaw=180.0)], ego_yaw=180.0).front is not None
    assert _check([_car('b', 10.0, yaw=-1.0, ego_yaw=180.0)], ego_yaw=180.0).front is not None


def test_rotated_ego():
    # Same geometry as test_front_clearance / test_rear_clearance, EGO turned to yaw 180 and 90.
    for ego_yaw in (180.0, 90.0, -37.0):
        result = _check([_car('f', 23.9, ego_yaw=ego_yaw), _car('r', -30.0, ego_yaw=ego_yaw)],
                        ego_yaw=ego_yaw)
        assert not result.safe and result.front.actor.actor_id == 'f', (ego_yaw, result)
        assert abs(result.front.clearance - 19.9) < 1e-9 and abs(result.rear.clearance - 26.0) < 1e-9
        result = _check([_car('left', 5.0, d=-3.5, ego_yaw=ego_yaw)], ego_yaw=ego_yaw)
        assert result.safe and result.front is None, (ego_yaw, result)


def test_overlap_from_adjacent_lane():
    # A wide truck whose center is in the next lane (d = 1.95 >= 3.5 / 2) but whose body sticks
    # 0.3 m into the EGO footprint: inner edge at d = 1.95 - 1.25 = 0.7 < EGO half width 1.0.
    truck = _car('truck', 0.5, d=1.95, length=8.0, width=2.5)
    result = _check([truck])
    assert not result.safe and result.front is None and [a.actor_id for a in result.overlaps] == [
        'truck'], result
    # touching only (inner edge exactly at the EGO side) is not an overlap
    assert _check([_car('touching', 0.0, d=2.25, width=2.5)]).safe


def test_overlap_in_junction_crossing():
    crossing = _car('crossing', 1.0, d=0.5, yaw=90.0)
    result = _check([crossing])
    assert not result.safe and result.front is None and result.overlaps == [crossing]


def test_pedestrian():
    on_spot = _car('ped', 0.5, d=0.5, length=0.5, width=0.5, source='pedestrian')
    result = _check([on_spot])
    assert not result.safe and result.overlaps == [on_spot] and result.front is None
    ahead = _car('ped_ahead', 5.0, length=0.5, width=0.5, source='pedestrian')
    result = _check([ahead])
    assert result.safe and result.front is None, 'a pedestrian is never a leader/follower'


def test_sumo_only_vehicle():
    # Not in CARLA: judged with its SUMO center/length like any other vehicle (section 2.6.3).
    result = _check([_car('sumo:tram.0', 30.0, length=16.5, source='sumo_only')])
    assert not result.safe and abs(result.front.clearance - 19.75) < 1e-9


def test_search_range():
    edge = EGO_SPAWN_SEARCH_RANGE_M
    assert _check([_car('at_edge', edge)]).front is not None
    assert _check([_car('beyond', edge + 0.1)]).front is None
    assert _check([_car('beyond_rear', -(edge + 0.1))]).rear is None


def test_describe():
    result = _check([_car('sumo:in1_A.3/carla:123', 23.0), _car('sumo:in1_B.1/carla:124', -40.0)])
    line = describe_gap_result(result)
    assert line == ('front=sumo:in1_A.3/carla:123 clearance=19.0 m '
                    'rear=sumo:in1_B.1/carla:124 clearance=36.0 m overlap=none result=WAIT'), line
    assert describe_gap_result(_check([])) == 'front=- rear=- overlap=none result=SAFE'


def main():
    tests = [
        test_no_vehicles,
        test_front_clearance,
        test_rear_clearance,
        test_lengths_and_separate_margins,
        test_nearest_vehicle_is_used,
        test_adjacent_lane_is_ignored,
        test_opposite_lane_is_ignored,
        test_heading_tolerance_and_wraparound,
        test_rotated_ego,
        test_overlap_from_adjacent_lane,
        test_overlap_in_junction_crossing,
        test_pedestrian,
        test_sumo_only_vehicle,
        test_search_range,
        test_describe,
    ]
    for test in tests:
        test()
        print('PASS %s' % test.__name__)
    print('All %d checks passed.' % len(tests))


if __name__ == '__main__':
    main()
