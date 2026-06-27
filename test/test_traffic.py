"""Unit tests for the traffic-manager core (wb.conflict_holds) and the
nearest-idle assignment used by the task allocator."""

import wb_common as wb

PRIORITY = {'robot1': 0, 'robot2': 1, 'robot3': 2}


def test_no_conflict_when_far():
    poses = {'robot1': (0.0, 0.0), 'robot2': (5.0, 0.0), 'robot3': (10.0, 0.0)}
    assert wb.conflict_holds(poses, PRIORITY, warn=0.6, clear=0.9) == set()


def test_lower_priority_robot_is_held():
    poses = {'robot1': (0.0, 0.0), 'robot2': (0.4, 0.0)}
    # robot2 has higher number -> lower priority -> it yields
    assert wb.conflict_holds(poses, PRIORITY, 0.6, 0.9) == {'robot2'}


def test_hysteresis_holds_through_warn_clear_band():
    # 0.75 m apart: inside CLEAR(0.9) but outside WARN(0.6).
    poses = {'robot1': (0.0, 0.0), 'robot2': (0.75, 0.0)}
    # not previously held -> WARN applies -> clear
    assert wb.conflict_holds(poses, PRIORITY, 0.6, 0.9, prev_held=set()) == set()
    # already held -> CLEAR applies -> stays held (no flap)
    assert wb.conflict_holds(poses, PRIORITY, 0.6, 0.9,
                             prev_held={'robot2'}) == {'robot2'}


def test_release_once_beyond_clear():
    poses = {'robot1': (0.0, 0.0), 'robot2': (1.2, 0.0)}
    assert wb.conflict_holds(poses, PRIORITY, 0.6, 0.9,
                             prev_held={'robot2'}) == set()


def test_nearest_idle_selection():
    # allocator picks the closest idle robot to a pickup station
    idle = {'robot2': (1.0, 0.0), 'robot3': (4.0, 0.0)}
    assert wb.nearest(idle, (0.0, 0.0)) == 'robot2'
    assert wb.nearest(idle, (5.0, 0.0)) == 'robot3'
