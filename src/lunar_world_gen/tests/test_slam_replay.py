"""scripts/replay_slam_bag.sh (arguments, --dry-run, container path translation) and the scoring maths of
tools/slam_eval.py on synthetic trajectories. No container, ROS or bag is started by these tests."""
from __future__ import annotations

import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]
SCRIPT = PKG / "scripts" / "replay_slam_bag.sh"
LUNAR_CFG = PKG / "config" / "slam" / "seyond_dlio_graph_lunar.yaml"
sys.path.insert(0, str(PKG / "tools"))
import gt_check  # noqa: E402
import slam_eval  # noqa: E402

needs_bash = pytest.mark.skipif(shutil.which("bash") is None or os.name == "nt", reason="needs bash (Linux)")


@pytest.fixture
def fake_bag(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 60000000000\n")
    (bag / "ground_truth.tum").write_text("0 0 0 0 0 0 0 1\n")
    return bag


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["bash", str(SCRIPT), *args], capture_output=True, text=True, timeout=60)


@needs_bash
def test_syntax_and_help():
    assert subprocess.run(["bash", "-n", str(SCRIPT)]).returncode == 0
    res = run("--help")
    assert res.returncode == 0
    for opt in ("--bag", "--config", "--name", "--label", "--rate", "--duration", "--no-rviz", "--no-record",
                "--no-eval", "--start-delay", "--tail", "--ready-timeout", "--no-truth-path", "--output-in-place",
                "--dry-run"):
        assert opt in res.stdout


@needs_bash
@pytest.mark.parametrize("args, msg", [
    (["--rate"], "--rate needs a value"),
    (["--rate", "fast"], "--rate must be a number"),
    (["--duration", "--no-rviz"], "--duration needs a value"),
    (["--bogus"], "unknown option '--bogus'"),
])
def test_option_errors(args, msg):
    res = run(*args)
    assert res.returncode == 2 and msg in res.stderr


@needs_bash
def test_dry_run_default_config_maps_into_the_container(fake_bag):
    res = run("--bag", str(fake_bag), "--dry-run", "--duration", "75", "--no-rviz")
    assert res.returncode == 0, res.stderr
    cfg = "/home/er4-user/ws/src/lunar_world_gen/config/slam/seyond_dlio_graph_lunar.yaml"
    assert f"Config:  {cfg}" in res.stdout and "SLAM_RVIZ=false" in res.stdout
    # by default SLAM gets a per-run copy whose map/DEM outputs go to slam_output/replay_runs/NAME
    out_c = "/home/er4-user/slam_ws/src/lidarslam_ros2/output/replay_runs/"
    assert f"COMBINED_PARAM_FILE={out_c}" in res.stdout and "/config.yaml" in res.stdout
    assert f"map_save_dir: {out_c}" in res.stdout and "/lunar_dem" in res.stdout
    assert "--playback-duration 75" in res.stdout and "--disable-keyboard-controls" in res.stdout
    assert "slam_eval.py" in res.stdout and "ros2 bag record -s sqlite3" in res.stdout
    assert "/hazard/diagnostics" in res.stdout and "/dlio/frontend_diagnostics" in res.stdout
    assert "proc_sampler.py --root-cmd run_live_seyond_dlio_slam.sh" in res.stdout and "--resources" in res.stdout
    # the bag starts only after the readiness wait
    out = res.stdout
    assert out.index("slam_replay_check.py wait") < out.index("ros2 bag play")


@needs_bash
def test_dry_run_config_paths(fake_bag, tmp_path):
    slam_yaml = REPO / "lidarslam_ros2" / "lidarslam" / "param" / "seyond_dlio_graph.yaml"
    if slam_yaml.is_file():
        res = run("--bag", str(fake_bag), "--config", str(slam_yaml), "--dry-run", "--output-in-place")
        assert res.returncode == 0, res.stderr
        assert ("COMBINED_PARAM_FILE=/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml"
                in res.stdout)
        assert "map_save_dir:" not in res.stdout
    # a container path cannot be copied: SLAM uses it as it is
    res = run("--bag", str(fake_bag), "--config", "/home/er4-user/ws/src/x.yaml", "--dry-run")
    assert res.returncode == 0 and "COMBINED_PARAM_FILE=/home/er4-user/ws/src/x.yaml" in res.stdout
    assert "container path" in res.stderr
    outside = tmp_path / "outside.yaml"
    outside.write_text("x: 1\n")
    res = run("--bag", str(fake_bag), "--config", str(outside), "--dry-run")
    assert res.returncode == 2 and "the folders the container mounts" in res.stderr


@needs_bash
def test_truth_path_publisher_runs_before_the_bag(fake_bag):
    gt_tool = REPO / "lidarslam_ros2" / "scripts" / "togo" / "gazebo_gt_path_publisher.py"
    res = run("--bag", str(fake_bag), "--dry-run")                   # this bag has no /model/togo/pose
    assert res.returncode == 0 and "gazebo_gt_path_publisher.py" not in res.stdout
    assert "no /model/togo/pose" in res.stderr or not gt_tool.is_file()
    meta = fake_bag / "metadata.yaml"
    meta.write_text(meta.read_text() + "  topics_with_message_count:\n    - topic_metadata:\n"
                    "        name: /model/togo/pose\n        type: geometry_msgs/msg/PoseArray\n")
    res = run("--bag", str(fake_bag), "--dry-run")
    assert res.returncode == 0, res.stderr
    if gt_tool.is_file():
        out = res.stdout
        assert "gazebo_gt_path_publisher.py   # /ground_truth/path" in out   # and no use_sim_time: it reads no clock
        assert "-e SLAM_REFERENCE_PATH=false" in out                           # replays skip the reference-path node
        assert out.index("gazebo_gt_path_publisher.py") < out.index("ros2 bag play")
    res = run("--bag", str(fake_bag), "--dry-run", "--no-truth-path")
    assert res.returncode == 0 and "gazebo_gt_path_publisher.py" not in res.stdout


@needs_bash
def test_no_ground_truth_means_no_scoring(fake_bag):
    (fake_bag / "ground_truth.tum").unlink()
    res = run("--bag", str(fake_bag), "--dry-run")
    assert res.returncode == 0 and "slam_eval.py" not in res.stdout and "not scored" in res.stderr


@needs_bash
def test_not_a_bag(tmp_path):
    res = run("--bag", str(tmp_path), "--dry-run")
    assert res.returncode == 2 and "no metadata.yaml" in res.stderr


# ------------------------------------------------------------------ scoring maths
def loop_trajectory(n=2000, rate=50.0):
    """Ground truth: a 30 x 20 m rounded loop with gentle z, heading along the path, 50 Hz."""
    t = np.arange(n) / rate
    a = 2 * math.pi * np.arange(n) / n
    x, y, z = 15 * np.cos(a), 10 * np.sin(a), 0.5 * np.sin(2 * a)
    yaw = np.arctan2(np.gradient(y), np.gradient(x))
    q = np.c_[np.zeros(n), np.zeros(n), np.sin(yaw / 2), np.cos(yaw / 2)]
    return np.c_[t, x, y, z, q]


def rigid(traj, yaw, t):
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    out = traj.copy()
    out[:, 1:4] = (r @ traj[:, 1:4].T).T + t
    rots = r @ slam_eval.quat_to_rot(traj[:, 4:8])
    out[:, 4:8] = slam_eval.rot_to_quat(rots)
    return out


def test_quaternion_round_trip():
    q = loop_trajectory(200)[:, 4:8]
    back = slam_eval.rot_to_quat(slam_eval.quat_to_rot(q))
    assert np.allclose(np.abs(np.sum(back * q, axis=1)), 1.0, atol=1e-9)


def test_umeyama_recovers_a_rigid_transform():
    gt = loop_trajectory()
    est = rigid(gt, 0.7, np.array([3.0, -2.0, 0.4]))
    r, t = slam_eval.umeyama(est[:, 1:4], gt[:, 1:4])
    assert np.allclose((r @ est[:, 1:4].T).T + t, gt[:, 1:4], atol=1e-9)


def test_score_perfect_and_noisy_and_drifting():
    gt = loop_trajectory()
    est = rigid(gt, -1.2, np.array([5.0, 1.0, 0.0]))       # another frame: alignment must remove it
    sc = slam_eval.score(est, gt)
    assert sc["ate_rmse_m"] < 1e-6 and sc["rpe_10m_mean_m"] < 1e-6
    rng = np.random.default_rng(1)
    noisy = est.copy()
    noisy[:, 1:4] += rng.normal(0, 0.02, (len(est), 3))
    sc = slam_eval.score(noisy, gt)
    assert 0.025 < sc["ate_rmse_m"] < 0.045                 # sqrt(3) * 2 cm, minus what the alignment absorbs
    drift = gt.copy()
    drift[:, 1] += np.linspace(0, 1.0, len(gt))              # 1 m of x drift over the loop
    sc = slam_eval.score(drift, gt)
    assert sc["end_gap_m"] > 0.9 and sc["rpe_10m_mean_m"] > 0.05


def test_dlio_placeholder_poses_are_not_scored():
    gt = loop_trajectory()
    est = rigid(gt, 0.4, np.array([1.0, 2.0, 0.0]))
    est[:300, 1:8] = [0, 0, 0, 0, 0, 0, 1]                    # identity until "calibration" ends, as DLIO does
    est[:100, 0] = 0.0                                         # the first ones even stamped 0
    sc = slam_eval.score(est, gt)
    assert sc["matched"] == len(gt) - 300 and sc["drift_end_m"] < 1e-6   # anchored at the first real pose
    assert len(slam_eval.drop_placeholders(gt)) == len(gt)    # nothing dropped from a real trajectory


def test_associate_respects_the_time_window():
    gt_t = np.arange(0, 10, 0.02)
    est_t = np.array([0.001, 0.5, 5.009, 20.0])
    ie, ig = slam_eval.associate(est_t, gt_t)
    assert list(ie) == [0, 1, 2] and np.allclose(gt_t[ig], [0.0, 0.5, 5.0])


def test_compose_map_uses_the_latest_correction():
    odom = loop_trajectory(100)
    corr = np.array([[odom[0, 0], 0, 0, 0, 0, 0, 0, 1],                       # identity at the start
                     [odom[50, 0], 1.0, 2.0, 0.0, 0, 0, 0, 1]])                # then shifted by (1, 2)
    out = slam_eval.compose_map(odom, corr)
    assert np.allclose(out[:50, 1:4], odom[:50, 1:4])
    assert np.allclose(out[50:, 1:4], odom[50:, 1:4] + [1.0, 2.0, 0.0])
    assert slam_eval.compose_map(odom, None) is None


def test_drift_is_zero_for_a_rigidly_moved_trajectory_and_grows_with_heading_drift():
    gt = loop_trajectory()
    sc = slam_eval.score(rigid(gt, 0.9, np.array([4.0, -3.0, 1.0])), gt)
    assert sc["drift_end_m"] < 1e-6 and abs(sc["drift_end_yaw_deg"]) < 1e-6
    drifted = gt.copy()                                       # heading error growing to 2 deg at the end
    yaw_err = np.radians(np.linspace(0, 2.0, len(gt)))
    rots = slam_eval.quat_to_rot(gt[:, 4:8])
    rz = np.stack([np.stack([np.cos(yaw_err), -np.sin(yaw_err), 0 * yaw_err], -1),
                   np.stack([np.sin(yaw_err), np.cos(yaw_err), 0 * yaw_err], -1),
                   np.stack([0 * yaw_err, 0 * yaw_err, 1 + 0 * yaw_err], -1)], -2)
    drifted[:, 4:8] = slam_eval.rot_to_quat(rz @ rots)
    sc = slam_eval.score(drifted, gt)
    assert abs(sc["drift_end_yaw_deg"] - 2.0) < 0.01 and sc["rpe_10m_rot_mean_deg"] > 0.05


def test_zigzag_measures_lateral_jitter_but_not_a_steady_offset():
    gt = loop_trajectory(4000)                                # 80 s
    yaw = np.arctan2(2 * (gt[:, 7] * gt[:, 6]), 1 - 2 * gt[:, 6] ** 2)
    lateral = np.c_[-np.sin(yaw), np.cos(yaw), np.zeros(len(gt))]
    wobble = gt.copy()
    wobble[:, 1:4] += lateral * (0.05 * np.sin(2 * np.pi * 1.0 * gt[:, 0]))[:, None]
    sc = slam_eval.score(wobble, gt)
    assert abs(sc["zigzag_rms_m"] - 0.05 / np.sqrt(2)) < 0.004
    offset = gt.copy()
    offset[:, 1:4] += lateral * 0.2                            # a constant 20 cm offset is not zig-zag
    assert slam_eval.score(offset, gt)["zigzag_rms_m"] < 0.01


def test_segments_get_their_own_ate():
    gt = loop_trajectory()
    est = gt.copy()
    est[1000:, 1] += 0.1 * np.sin(2 * np.pi * gt[1000:, 0])    # zero-mean error only in the second half
    sc = slam_eval.score(est, gt, segments={"first": [0, 19.0], "second": [21.0, 40.0]})
    assert sc["segments"]["first"]["ate_rmse_m"] < sc["segments"]["second"]["ate_rmse_m"]


def test_align_4dof_recovers_heading_and_offset():
    gt = loop_trajectory()
    est = rigid(gt, -2.1, np.array([-7.0, 4.0, 2.5]))
    r, t = slam_eval.align_4dof(est[:, 1:4], gt[:, 1:4])
    assert np.allclose((r @ est[:, 1:4].T).T + t, gt[:, 1:4], atol=1e-9)
    assert np.allclose(r[2], [0, 0, 1])                        # never tilts: roll / pitch errors stay visible


def test_interpolate_gt_is_linear_and_skips_gaps():
    t = np.r_[np.arange(0, 1.0, 0.02), np.arange(1.5, 2.0, 0.02)]           # a 0.52 s gap after 0.98 s
    gt = np.c_[t, 2 * t, -t, 0 * t, np.zeros((len(t), 3)), np.ones(len(t))]
    q = np.array([-1.0, 0.011, 0.5, 1.2, 1.51, 3.0])
    idx, g = slam_eval.interpolate_gt(gt, q)
    assert list(idx) == [1, 2, 4]                              # before the start, in the gap and after the end: out
    assert np.allclose(g[:, 1], 2 * q[idx]) and np.allclose(g[:, 2], -q[idx])


def s_curve(seconds=60.0, rate=50.0):
    """Ground truth weaving left and right (yaw rate up to ~0.6 rad/s, both signs), flat, 50 Hz."""
    t = np.arange(0, seconds, 1 / rate)
    x, y = 0.5 * t, 2.0 * np.sin(0.4 * t)
    yaw = np.arctan2(0.8 * np.cos(0.4 * t), 0.5 * np.ones(len(t)))
    return np.c_[t, x, y, 0 * t, np.zeros(len(t)), np.zeros(len(t)), np.sin(yaw / 2), np.cos(yaw / 2)]


def test_heading_lag_is_measured_in_turns():
    gt = s_curve()
    assert abs(slam_eval.score(gt.copy(), gt)["heading_lag_ms"]) < 2
    idx, late = slam_eval.interpolate_gt(gt, gt[:, 0] - 0.05)  # each pose published 50 ms after its time
    est = np.c_[gt[idx, 0], late[:, 1:]]
    assert abs(slam_eval.score(est, gt)["heading_lag_ms"] - 50) < 3


def test_vertical_scale_error_is_measured_against_the_climb():
    gt = loop_trajectory()                                     # z spans 1 m
    est = gt.copy()
    est[:, 3] = gt[0, 3] + 1.03 * (gt[:, 3] - gt[0, 3])        # climbs and descents 3 % too big
    sc = slam_eval.score(est, gt)
    assert abs(sc["vertical_vs_climb_pct"] - 3.0) < 0.01 and sc["vertical_vs_climb_r2"] > 0.999
    flat = gt.copy()
    flat[:, 3] = 0.0
    assert "vertical_vs_climb_pct" not in slam_eval.score(flat, flat)


def test_worst_spots_are_distinct_places():
    s = np.arange(0, 100, 0.1)
    mag = np.exp(-((s - 10) / 2) ** 2) + 0.5 * np.exp(-((s - 60) / 2) ** 2)   # a wide bad stretch and a smaller one
    k = slam_eval.worst_spots(mag, s, n=2, sep_m=5.0)
    assert abs(s[k[0]] - 10) < 0.11 and abs(s[k[1]] - 60) < 0.11


def test_terrain_takes_each_tiles_own_cell(tmp_path):
    """Two 1 m cells with 0.25 m margins that overlap, as the build writes them; tile A's margin is wrong on
    purpose where it overlaps B's cell: there B must be used; margins only count at the outer border."""
    import json
    f = lambda x, y: 0.3 * x - 0.2 * y + 0.1                  # noqa: E731 - linear, so bilinear is exact
    for name, cx in (("tile_00_00", 0.0), ("tile_01_00", 1.0)):
        x0, y0, s, n = cx - 0.25, -0.25, 0.05, 31
        X, Y = np.meshgrid(x0 + s * np.arange(n), y0 + s * np.arange(n))
        g = f(X, Y)
        if name == "tile_00_00":
            g[(X > 1.0 + 1e-9) & (Y >= 0.0) & (Y <= 1.0)] += 5.0
        np.save(tmp_path / f"{name}.npy", g.astype(np.float32))
        (tmp_path / f"{name}.json").write_text(json.dumps({"x0": x0, "y0": y0, "spacing": s, "nx": n, "ny": n,
                                                           "meta": {"margin_m": 0.25}}))
    X, Y = np.meshgrid(np.linspace(-0.2, 2.2, 97), np.linspace(-0.2, 1.2, 29))
    z = gt_check.Terrain(tmp_path).height(X.ravel(), Y.ravel())
    assert np.allclose(z, f(X.ravel(), Y.ravel()), atol=1e-5)
    assert np.isnan(gt_check.Terrain(tmp_path).height(np.array([5.0]), np.array([0.5])))[0]


def test_ground_tilt_recovers_a_plane():
    rng = np.random.default_rng(0)
    u, v = rng.uniform(-10, 10, 30000), rng.uniform(-10, 10, 30000)
    r = np.tan(np.radians(0.1)) * u - np.tan(np.radians(0.05)) * v + rng.normal(0, 0.005, len(u))
    tilt = gt_check.ground_tilt(r, u, v, np.repeat(np.arange(10), 3000))
    assert abs(tilt["forward_slope_deg"] - 0.1) < 0.002 and abs(tilt["left_slope_deg"] + 0.05) < 0.002
    assert tilt["scans"] == 10 and abs(tilt["per_scan_forward_median_abs_deg"] - 0.1) < 0.01


def test_scan_matching_statistics():
    rows = np.zeros((10, 58))
    rows[:, slam_eval.DIAG["status"]] = [3, 3, 5, 5, 5, 5, 5, 6, 5, 5]   # 2 scans before initialisation
    rows[:, slam_eval.DIAG["gicp_ms"]] = np.arange(10)
    rows[:, slam_eval.DIAG["correction_t_m"]] = 0.002
    rows[7, slam_eval.DIAG["correction_rejected"]] = 1
    rows[:, slam_eval.DIAG["lidar_callbacks_started"]] = np.arange(1, 11)
    sm = slam_eval.scan_matching(rows)
    assert sm["scans_processed"] == 8 and sm["status_counts"] == {"3": 2, "5": 7, "6": 1}
    assert sm["gicp_ms"]["mean"] == 5.5 and sm["corrections_rejected"] == 1
    assert sm["correction_mm"]["mean"] == 2.0 and sm["lidar_callbacks_started"] == 10


def test_resources_from_the_sampler_csv(tmp_path):
    csv_path = tmp_path / "resources.csv"
    lines = ["t,pid,name,rss_mb,cpu_s,threads,mem_available_mb"]
    for k in range(121):                                       # 60 s at 0.5 s
        t = 1000.0 + k * 0.5
        lines.append(f"{t},10,dlio_odom_node,{500 + k * 0.5},{k * 0.5 * 0.4},12,9000")   # 40 % CPU, +60 MB/min
        lines.append(f"{t},11,graph_based_slam_node,800,{k * 0.5 * 1.0},20,9000")        # 100 % CPU, flat
    csv_path.write_text("\n".join(lines) + "\n")
    res = slam_eval.resources(csv_path, 1000.0, 1060.0, scans=1200)
    d = res["processes"]["dlio_odom_node"]
    assert abs(d["cpu_pct_mean"] - 40) < 0.5 and abs(d["rss_growth_mb_per_min"] - 60) < 1
    assert abs(res["processes"]["graph_based_slam_node"]["cpu_pct_mean"] - 100) < 0.5
    assert abs(res["dlio_cpu_ms_per_scan"] - 20.0) < 0.2        # 24 CPU s / 1200 scans
    assert abs(res["total_cpu_pct_mean"] - 140) < 1


def test_compare_verdicts(tmp_path):
    import json
    for label, ates, gicp, lags in (("base", [0.20, 0.21, 0.19], [10.0, 10.2, 9.8], [50.0, 49.0, 51.0]),
                                    ("cand", [0.10, 0.11, 0.09], [10.1, 9.9, 10.0], [-10.0, -11.0, -9.0])):
        for i, (ate, g, lag) in enumerate(zip(ates, gicp, lags)):
            d = tmp_path / f"{label}_{i}"
            d.mkdir()
            (d / "run_info.txt").write_text(f"label={label}\n")
            (d / "metrics.json").write_text(json.dumps({"accuracy": {"ate_rmse_m": ate, "heading_lag_ms": lag},
                                                        "scan_matching": {"gicp_ms": {"mean": g}}}))
    res = subprocess.run([sys.executable, str(PKG / "tools" / "slam_compare.py"), "--root", str(tmp_path),
                          "--baseline", "base", "--candidate", "cand"], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr
    row = lambda name: next(l for l in res.stdout.splitlines() if l.startswith(f"| {name}")).rstrip(" |")  # noqa: E731
    assert row("ATE RMSE").endswith("better") and row("GICP solve time, mean").endswith("~")
    assert row("heading lag").endswith("better")              # signed: judged by its size, -10 beats +50
    assert "(n=3)" in res.stdout


@needs_bash
def test_rescore_script_arguments(tmp_path):
    rescore = PKG / "scripts" / "rescore_slam_runs.sh"
    assert subprocess.run(["bash", "-n", str(rescore)]).returncode == 0
    res = subprocess.run(["bash", str(rescore), "--help"], capture_output=True, text=True)
    assert res.returncode == 0 and "--label" in res.stdout
    res = subprocess.run(["bash", str(rescore), "--label", "nope", "--root", str(tmp_path)], capture_output=True, text=True)
    assert res.returncode == 2 and "no runs with label 'nope'" in res.stderr
    res = subprocess.run(["bash", str(rescore), str(tmp_path)], capture_output=True, text=True, timeout=60)
    assert res.returncode != 0 and "not a replay run folder" in res.stderr


@needs_bash
def test_bag_recorder_arguments_and_dry_run(tmp_path):
    rec = PKG / "tools" / "bag_record" / "record_bag.sh"
    assert subprocess.run(["bash", "-n", str(rec)]).returncode == 0
    res = subprocess.run(["bash", str(rec), "--help"], capture_output=True, text=True)
    assert res.returncode == 0 and "--laps" in res.stdout
    for args, msg in ((["--laps", "0"], "positive integer"), (["--domain", "29"], "other than 29"),
                      (["--route", str(tmp_path / "none.json")], "route file not found")):
        res = subprocess.run(["bash", str(rec), *args], capture_output=True, text=True)
        assert res.returncode == 2 and msg in res.stderr, (args, res.stderr)
    env = dict(os.environ, HOME=str(tmp_path))
    res = subprocess.run(["bash", str(rec), "--dry-run", "--name", "t1", "--laps", "1"], capture_output=True,
                         text=True, env=env, timeout=60)
    assert res.returncode == 0, res.stderr
    assert str(tmp_path / "togo_assets/lunar/bags/t1") in res.stdout and "0.0 -19.5 -0.659 -1.5464" in res.stdout
    assert not (tmp_path / "togo_assets").exists()            # a dry run creates nothing


def test_trim_bag_finds_the_lap_ends():
    import trim_bag
    lap = loop_trajectory(1000)                                # a 30 x 20 m loop from (15, 0) back to it

    def laps(n, extra=0):
        g = np.concatenate([lap] + [lap[1:]] * (n - 1) + [lap[1:1 + extra]])
        g[:, 0] = np.arange(len(g)) / 50.0
        return g
    assert np.allclose(trim_bag.lap_ends(laps(2)), [19.98, 39.96], atol=0.05)
    assert len(trim_bag.lap_ends(laps(1, extra=400))) == 1   # a lap and 40 % of the next: one complete lap


def test_bag_record_tools_compile(tmp_path):
    import py_compile
    for f in ("bag_drive.py", "check_bag.py", "plan_route.py"):
        py_compile.compile(str(PKG / "tools" / "bag_record" / f), cfile=str(tmp_path / f"{f}c"), doraise=True)


@needs_bash
def test_benchmark_script_arguments():
    bench = PKG / "scripts" / "benchmark_slam.sh"
    assert subprocess.run(["bash", "-n", str(bench)]).returncode == 0
    res = subprocess.run(["bash", str(bench), "--help"], capture_output=True, text=True)
    assert res.returncode == 0 and "--compare-to" in res.stdout and "--repeats" in res.stdout
    res = subprocess.run(["bash", str(bench)], capture_output=True, text=True)
    assert res.returncode == 2 and "--label is required" in res.stderr
    res = subprocess.run(["bash", str(bench), "--label", "x", "--repeats", "0"], capture_output=True, text=True)
    assert res.returncode == 2 and "positive integer" in res.stderr
