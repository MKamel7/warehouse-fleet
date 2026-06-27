#!/usr/bin/env python3
"""Shared helpers and the fleet-config loader for the warehouse_bot nodes.

Pure functions only (geometry, polyline maths, arc-length interpolation,
config loading) so the roster and tunables live in one place
(config/fleet.yaml) and the logic can be tested without a running ROS graph.
Does not import rclpy.
"""

import math
import os

import yaml

CONFIG_BASENAME = 'fleet.yaml'


# --------------------------------------------------------------------- maths
def yaw_from_quat(x, y, z, w):
    """Yaw (rad) from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    """Return (z, w) of a yaw-only (z-axis) quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def norm_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def smooth_polyline(pts, passes=3, alpha=0.5):
    """Moving-average smoothing of a polyline; endpoints stay fixed."""
    p = [tuple(q) for q in pts]
    for _ in range(passes):
        q = list(p)
        for i in range(1, len(p) - 1):
            q[i] = (p[i][0] + alpha * (p[i - 1][0] + p[i + 1][0] - 2 * p[i][0]),
                    p[i][1] + alpha * (p[i - 1][1] + p[i + 1][1] - 2 * p[i][1]))
        p = q
    return p


def point_at_arclength(trail, target_s):
    """Linear-interpolate (x, y) at cumulative arc-length ``target_s`` along a
    ``trail`` of (x, y, s) points. Clamps to the trail ends."""
    if not trail:
        return None
    if target_s <= trail[0][2]:
        return trail[0][0], trail[0][1]
    for k in range(1, len(trail)):
        if trail[k][2] >= target_s:
            x0, y0, s0 = trail[k - 1]
            x1, y1, s1 = trail[k]
            r = (target_s - s0) / max(s1 - s0, 1e-6)
            return x0 + r * (x1 - x0), y0 + r * (y1 - y0)
    return trail[-1][0], trail[-1][1]


def conflict_holds(poses, priority, warn, clear, prev_held=()):
    """Traffic-manager core (pure): decide which robots to pause.

    Given map-frame ``poses`` {name: (x, y, ...)}, a ``priority`` map
    {name: int} where a LOWER number wins, ``warn``/``clear`` distances and the
    previously-held set, return the new set of robots to hold. For each pair the
    CLEAR distance applies while either robot is already held (hysteresis, so the
    hold does not flap), otherwise WARN. On a conflict the lower-priority robot
    (higher number, e.g. later in the roster) is held."""
    prev = set(prev_held)
    names = list(poses)
    held = set()
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            d = math.hypot(poses[a][0] - poses[b][0],
                           poses[a][1] - poses[b][1])
            active = a in prev or b in prev
            if d >= (clear if active else warn):
                continue
            held.add(a if priority[a] > priority[b] else b)
    return held


def nearest(candidates, target):
    """Return the key of the nearest candidate to ``target`` (x, y), or None.

    ``candidates`` maps name -> (x, y, ...). Used by the task allocator to pick
    the closest idle robot to a station."""
    best, bestd = None, float('inf')
    tx, ty = target[0], target[1]
    for name, p in candidates.items():
        d = (p[0] - tx) ** 2 + (p[1] - ty) ** 2
        if d < bestd:
            bestd, best = d, name
    return best


# -------------------------------------------------------------------- config
def reached(pos, target, tol):
    """True if (x, y) ``pos`` is within ``tol`` of (x, y) ``target``."""
    return math.hypot(pos[0] - target[0], pos[1] - target[1]) <= tol


def mission_advance(idx, count, loop):
    """Next waypoint index after ``idx`` for a mission of ``count`` waypoints.
    Wraps to 0 when ``loop`` is set, else returns None at the end."""
    nxt = idx + 1
    if nxt < count:
        return nxt
    return 0 if loop else None


def default_config_path():
    """Locate config/fleet.yaml: explicit env override, else the installed
    package share dir, else a source-tree fallback (for unit tests)."""
    env = os.environ.get('WAREHOUSE_FLEET_CONFIG')
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('warehouse_bot_package')
        return os.path.join(share, 'config', CONFIG_BASENAME)
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, '..', 'config',
                                             CONFIG_BASENAME))


def load_fleet_config(path=None):
    """Load and return the fleet config (the mapping under top-level 'fleet:')."""
    path = path or default_config_path()
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get('fleet', data)


def robot_names(cfg):
    """Ordered list of robot names from the config."""
    return [r['name'] for r in cfg['robots']]


def robot_spawns(cfg):
    """Map name -> (x, y, yaw) of each robot's spawn pose."""
    return {r['name']: (r['x'], r['y'], r['yaw']) for r in cfg['robots']}


def convoy_followers(cfg):
    """Map follower name -> gap (m) behind the leader, in convoy order."""
    return {f['name']: f['gap'] for f in cfg['convoy']['followers']}


def convoy_ahead(cfg):
    """Map each follower -> the robot immediately in front of it (leader for the
    first follower, the previous follower otherwise)."""
    leader = cfg['convoy']['leader']
    order = [f['name'] for f in cfg['convoy']['followers']]
    ahead = {}
    prev = leader
    for name in order:
        ahead[name] = prev
        prev = name
    return ahead
