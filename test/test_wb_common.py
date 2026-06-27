"""Unit tests for the ROS-free helpers in wb_common."""

import math
import os

import pytest

import wb_common as wb

CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'config', 'fleet.yaml'))


# ------------------------------------------------------------------- geometry
@pytest.mark.parametrize('yaw', [0.0, 0.5, -1.2, math.pi / 2, 3.0, -3.0])
def test_yaw_quat_roundtrip(yaw):
    z, w = wb.quat_from_yaw(yaw)
    assert wb.yaw_from_quat(0.0, 0.0, z, w) == pytest.approx(yaw, abs=1e-9)


def test_norm_angle_wraps():
    # +-pi are the same point; both are valid wraps of +-3pi.
    assert abs(wb.norm_angle(3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert abs(wb.norm_angle(-3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert wb.norm_angle(0.3) == pytest.approx(0.3, abs=1e-9)
    assert -math.pi - 1e-9 <= wb.norm_angle(123.4) <= math.pi + 1e-9


def test_smooth_keeps_endpoints_and_shortens_zigzag():
    pts = [(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]
    out = wb.smooth_polyline(pts, passes=3, alpha=0.5)
    assert out[0] == pytest.approx(pts[0])
    assert out[-1] == pytest.approx(pts[-1])

    def length(p):
        return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))

    assert length(out) < length(pts)


# --------------------------------------------------------------- arc length
def test_point_at_arclength():
    trail = [(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (1.0, 1.0, 2.0)]
    assert wb.point_at_arclength(trail, -5) == (0.0, 0.0)        # clamp low
    assert wb.point_at_arclength(trail, 0.5) == pytest.approx((0.5, 0.0))
    assert wb.point_at_arclength(trail, 1.5) == pytest.approx((1.0, 0.5))
    assert wb.point_at_arclength(trail, 99) == (1.0, 1.0)        # clamp high
    assert wb.point_at_arclength([], 1.0) is None


def test_nearest():
    cands = {'a': (0.0, 0.0), 'b': (5.0, 5.0), 'c': (1.0, 0.0)}
    assert wb.nearest(cands, (0.9, 0.0)) == 'c'
    assert wb.nearest(cands, (6.0, 6.0)) == 'b'
    assert wb.nearest({}, (0.0, 0.0)) is None


def test_reached():
    assert wb.reached((0.0, 0.0), (0.2, 0.0), tol=0.35)
    assert not wb.reached((0.0, 0.0), (1.0, 0.0), tol=0.35)


def test_mission_advance():
    # no loop: walk to the end then stop
    assert wb.mission_advance(0, 3, loop=False) == 1
    assert wb.mission_advance(1, 3, loop=False) == 2
    assert wb.mission_advance(2, 3, loop=False) is None
    # loop: wrap back to the start
    assert wb.mission_advance(2, 3, loop=True) == 0


# -------------------------------------------------------------------- config
def test_config_loads_and_is_consistent():
    cfg = wb.load_fleet_config(CONFIG)
    names = wb.robot_names(cfg)
    assert len(names) == len(set(names)) >= 1          # unique, non-empty

    spawns = wb.robot_spawns(cfg)
    assert set(spawns) == set(names)

    leader = cfg['convoy']['leader']
    followers = wb.convoy_followers(cfg)
    assert leader in names
    assert set(followers).issubset(set(names))
    assert leader not in followers                      # leader isn't a follower

    ahead = wb.convoy_ahead(cfg)
    assert set(ahead) == set(followers)
    # the first follower trails the leader; gaps are positive and increasing-ish
    assert ahead[list(followers)[0]] == leader
    assert all(g > 0 for g in followers.values())


def test_stations_present():
    cfg = wb.load_fleet_config(CONFIG)
    for key, st in cfg['stations'].items():
        assert {'x', 'y', 'yaw'} <= set(st), f'station {key} missing fields'
