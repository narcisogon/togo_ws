"""Offline tests of tools/rover_drive_test.py metrics (no ROS / Gazebo needed).

Run:  /home/nargon/.venvs/lunar_world/bin/python -m pytest -q src/lunar_world_gen/tests/test_sim_drive_test.py
"""
import argparse
import importlib.util
import math
import os

import numpy as np
import pytest

_TOOL = os.path.join(os.path.dirname(__file__), '..', 'tools', 'rover_drive_test.py')
_spec = importlib.util.spec_from_file_location('rover_drive_test', _TOOL)
rdt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rdt)


def _args(**kw):
    a = argparse.Namespace(settle_window=1.0, settle_lin=0.01, settle_ang=0.02, gravity=None)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def _synthetic(v=0.5, wz=0.0, slip=0.0, yaw_eff=1.0, dt=0.02, t_fwd=(10.0, 16.0), t_turn=(19.0, 25.0), z=0.136):
    """GT / odom / joints for: rest, straight at v (true speed v*(1-slip)), rest, turn at wz*yaw_eff."""
    t = np.arange(5.0, 30.0 + 1e-9, dt)
    x = np.zeros_like(t); y = np.zeros_like(t); yaw = np.zeros_like(t)
    vt = np.where((t >= t_fwd[0]) & (t < t_fwd[1]), v * (1 - slip), 0.0)
    wt = np.where((t >= t_turn[0]) & (t < t_turn[1]), wz * yaw_eff, 0.0)
    for i in range(1, len(t)):
        yaw[i] = yaw[i - 1] + wt[i - 1] * dt
        x[i] = x[i - 1] + vt[i - 1] * dt * math.cos(yaw[i - 1])
        y[i] = y[i - 1] + vt[i - 1] * dt * math.sin(yaw[i - 1])
    gt = []
    for i in range(len(t)):
        qz, qw = math.sin(yaw[i] / 2), math.cos(yaw[i] / 2)
        gt.append((t[i], x[i], y[i], z, 0.0, 0.0, qz, qw, 0.0, 0.0, math.atan2(math.sin(yaw[i]), math.cos(yaw[i])), 0.0))
    # wheel rates: commanded speed / true radius  (so slip = 1 - v_true / v_wheel)
    wfwd = np.where((t >= t_fwd[0]) & (t < t_fwd[1]), v / rdt.WHEEL_RADIUS_TRUE, 0.0)
    wturn = np.where((t >= t_turn[0]) & (t < t_turn[1]), wz * rdt.TRACK_TRUE / 2 / rdt.WHEEL_RADIUS_TRUE, 0.0)
    joints = [(t[i], 0, 0, 0, 0, wfwd[i] - wturn[i], wfwd[i] - wturn[i], wfwd[i] + wturn[i], wfwd[i] + wturn[i], 0.0)
              for i in range(len(t))]
    odom = [(t[i], x[i], y[i], 0.0, math.atan2(math.sin(yaw[i]), math.cos(yaw[i])), vt[i], 0.0, wt[i], 0.0) for i in range(len(t))]
    stats = [(float(k), t[i], t[i], 1.0, i, 0) for k, i in enumerate(range(0, len(t), 5))]
    data = {'gt': gt, 'odom': odom, 'cmd': [], 'cmd_out': [], 'joints': joints, 'stats': stats}
    segments = [
        {'name': 'fwd', 'vx': v, 'wz': 0.0, 'duration_cmd': t_fwd[1] - t_fwd[0], 't0': t_fwd[0], 't1': t_fwd[1]},
        {'name': 'stop1', 'vx': 0.0, 'wz': 0.0, 'duration_cmd': 3.0, 't0': t_fwd[1], 't1': t_turn[0]},
        {'name': 'turn', 'vx': 0.0, 'wz': wz, 'duration_cmd': t_turn[1] - t_turn[0], 't0': t_turn[0], 't1': t_turn[1]},
    ]
    return data, segments


def _seg(S, name):
    return next(s for s in S['segments'] if s['name'] == name)


def test_perfect_tracking():
    data, segs = _synthetic(v=0.5, wz=0.6)
    S = rdt.summarize(data, segs, [], _args(), {'status': 'ok'}, 0.0)
    f = _seg(S, 'fwd')
    assert f['gt_dist_long_m'] == pytest.approx(3.0, abs=0.02)
    assert f['speed_tracking_ratio'] == pytest.approx(1.0, abs=0.01)
    assert abs(f['long_slip_ratio']) < 0.01
    assert abs(f['gt_dist_lat_m']) < 1e-6
    t = _seg(S, 'turn')
    assert t['gt_dyaw_deg'] == pytest.approx(math.degrees(0.6 * 6.0), abs=1.5)   # yaw unwrapped past 180 deg
    assert t['yawrate_tracking_ratio'] == pytest.approx(1.0, abs=0.01)
    assert t['effective_track_m'] == pytest.approx(rdt.TRACK_TRUE, rel=0.02)
    assert S['spawn']['settle_time_after_first_pose_s'] == pytest.approx(0.0, abs=0.05)


def test_slip_and_poor_turning():
    data, segs = _synthetic(v=0.5, wz=0.6, slip=0.2, yaw_eff=0.25)
    S = rdt.summarize(data, segs, [], _args(), {'status': 'ok'}, 0.0)
    f = _seg(S, 'fwd')
    assert f['long_slip_ratio'] == pytest.approx(0.2, abs=0.01)
    assert f['long_slip_ratio_segment'] == pytest.approx(0.2, abs=0.02)
    t = _seg(S, 'turn')
    assert t['yawrate_tracking_ratio'] == pytest.approx(0.25, abs=0.01)
    assert t['yaw_slip_ratio'] == pytest.approx(0.75, abs=0.02)


def test_ballistic_metric_detects_free_fall():
    data, segs = _synthetic(v=0.5, wz=0.0)
    g = 1.62
    gt = [list(r) for r in data['gt']]
    # 0.3 s free-fall hop in the middle of the forward segment
    for r in gt:
        if 12.0 <= r[0] <= 12.3:
            tau = r[0] - 12.0
            r[3] += 0.243 * tau - 0.5 * g * tau * tau      # v0 = g*0.15 -> lands at 0.3 s
    data['gt'] = [tuple(r) for r in gt]
    S = rdt.summarize(data, segs, [], _args(gravity=g), {'status': 'ok'}, 0.0)
    f = _seg(S, 'fwd')
    assert f['max_ballistic_s'] >= 0.2
    assert 0.02 < f['ballistic_frac'] < 0.1


def test_parse_sequence_step_and_unique_names():
    a = argparse.Namespace(sequence_file=None, sequence=None, preset='standard',
                           step=['fwd:0.5:0:6', 'stop:0:0:3', 'stop:0:0:3'])
    seq = rdt.parse_sequence(a)
    assert [s['duration'] for s in seq] == [6.0, 3.0, 3.0]
    assert len({s['name'] for s in seq}) == 3
