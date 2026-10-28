#!/usr/bin/env python3
"""Scripted drive test for the TOGO rover (A300) in Gazebo Harmonic + ROS 2 Jazzy.

What it does
  1. waits for the simulation clock (gz /stats, or ROS /clock with --clock-source ros), for the
     Gazebo ground-truth pose stream and for the diff-drive controller
     ``platform_velocity_controller`` to be *active* (asks /controller_manager/list_controllers,
     so it does not depend on the spawner's exit code);
  2. (optionally) waits until the robot has settled after the spawn drop;
  3. executes a scripted command sequence, publishing geometry_msgs/TwistStamped
     (sim-time stamps) at 50 Hz (sim time) on /platform_velocity_controller/cmd_vel;
  4. records, at their native rates, to CSV:
       gt.csv       ground truth: Gazebo PosePublisher (gz topic /model/<model>/pose,
                    gz.msgs.Pose_V, entry "<model>" = model/base_link pose in the world, 50 Hz)
       odom.csv     /platform_velocity_controller/odom (nav_msgs/Odometry, 50 Hz)
       cmd.csv      the commands this tool sent
       cmd_out.csv  /platform_velocity_controller/cmd_vel_out (limited command, if published)
       joints.csv   /joint_states wheel positions/velocities
       stats.csv    gz /stats (gz.msgs.WorldStatistics: sim time, real-time factor)
  5. writes summary.json: per-segment achieved vs commanded distance / heading change,
     steady-state speed and yaw-rate tracking, longitudinal slip ratio, lateral drift,
     max roll/pitch, z oscillation (bounce; with --gravity also the fraction of time the
     chassis is ballistic, i.e. all wheels off the ground), stopping distance, odometry
     error, the time to settle after spawn and the real-time factor.
  --resummarize recomputes summary.json offline from the CSVs of an earlier run.

Requirements: the system python3 with ROS 2 Jazzy sourced (rclpy, nav_msgs, sensor_msgs,
controller_manager_msgs) and the Gazebo python bindings (python3-gz-transport13 /
python3-gz-msgs10). Run it with the SAME GZ_PARTITION and ROS_DOMAIN_ID as the simulation.

Examples
  rover_drive_test.py --out /tmp/run1                               # "standard" sequence
  rover_drive_test.py --out /tmp/run2 --preset extended
  rover_drive_test.py --out /tmp/run3 --step fwd:0.5:0:6 --step stop:0:0:3 --step turn:0:0.6:6 --step stop:0:0:3
  rover_drive_test.py --out /tmp/run4 --sequence '[{"name":"fwd","vx":0.5,"wz":0.0,"duration":6}]'
  rover_drive_test.py --out /tmp/run5 --sequence-file seq.json      # same JSON list format
  rover_drive_test.py --out /tmp/run1 --resummarize --gravity 1.62  # offline re-analysis

Exit codes: 0 ok, 2 timeout while waiting for the simulation/controller, 3 error, 130 interrupted.
Outputs are written in every case (status field in summary.json); the robot is left with a
zero command on exit.
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time

import numpy as np

# --------------------------------------------------------------------------------------
# Robot constants (from the generated URDF/SDF, see docs/research/rover_baseline.md)
WHEEL_RADIUS_TRUE = 0.1651      # collision cylinder radius [m]
WHEEL_RADIUS_CTRL = 0.1625      # diff_drive_controller wheel_radius [m]
TRACK_TRUE = 0.5468             # lateral distance between wheel centres (+0.2829 / -0.2639) [m]
WHEEL_SEP_CTRL = 0.562          # diff_drive_controller wheel_separation (multiplier 1.75 applied on top)
WHEEL_JOINTS = {
    'fl': 'front_left_wheel_joint', 'rl': 'rear_left_wheel_joint',
    'fr': 'front_right_wheel_joint', 'rr': 'rear_right_wheel_joint',
}

PRESETS = {
    # the baseline sequence requested for the lunar world work
    'standard': [
        {'name': 'fwd', 'vx': 0.5, 'wz': 0.0, 'duration': 6.0},
        {'name': 'stop1', 'vx': 0.0, 'wz': 0.0, 'duration': 3.0},
        {'name': 'turn', 'vx': 0.0, 'wz': 0.6, 'duration': 6.0},
        {'name': 'stop2', 'vx': 0.0, 'wz': 0.0, 'duration': 3.0},
    ],
    # extra manoeuvres: opposite and fast turns, an arc, reverse, max speed
    'extended': [
        {'name': 'turn_neg', 'vx': 0.0, 'wz': -0.6, 'duration': 6.0},
        {'name': 'stop1', 'vx': 0.0, 'wz': 0.0, 'duration': 2.0},
        {'name': 'turn_fast', 'vx': 0.0, 'wz': 1.5, 'duration': 5.0},
        {'name': 'stop2', 'vx': 0.0, 'wz': 0.0, 'duration': 2.0},
        {'name': 'arc', 'vx': 0.4, 'wz': 0.3, 'duration': 6.0},
        {'name': 'stop3', 'vx': 0.0, 'wz': 0.0, 'duration': 2.0},
        {'name': 'fwd_max', 'vx': 0.8, 'wz': 0.0, 'duration': 5.0},
        {'name': 'stop4', 'vx': 0.0, 'wz': 0.0, 'duration': 3.0},
        {'name': 'reverse', 'vx': -0.3, 'wz': 0.0, 'duration': 4.0},
        {'name': 'stop5', 'vx': 0.0, 'wz': 0.0, 'duration': 2.0},
    ],
}


def quat_to_rpy(qx, qy, qz, qw):
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def parse_sequence(args):
    if args.sequence_file:
        with open(args.sequence_file) as f:
            seq = json.load(f)
    elif args.sequence:
        seq = json.loads(args.sequence)
    elif args.step:
        seq = []
        for s in args.step:
            p = s.split(':')
            if len(p) != 4:
                raise ValueError(f'--step must be name:vx:wz:duration, got {s!r}')
            seq.append({'name': p[0], 'vx': float(p[1]), 'wz': float(p[2]), 'duration': float(p[3])})
    else:
        seq = PRESETS[args.preset]
    out = []
    for i, s in enumerate(seq):
        d = {'name': str(s.get('name', f'seg{i}')), 'vx': float(s.get('vx', 0.0)),
             'wz': float(s.get('wz', 0.0)), 'duration': float(s['duration'])}
        if d['duration'] <= 0:
            raise ValueError(f'segment {d} has non-positive duration')
        out.append(d)
    names = [s['name'] for s in out]
    if len(set(names)) != len(names):  # make names unique
        for i, s in enumerate(out):
            s['name'] = f"{i:02d}_{s['name']}"
    return out


class Recorder:
    """Thread-safe in-memory buffers for all recorded streams."""

    def __init__(self):
        self.lock = threading.Lock()
        self.gt = []        # t, x, y, z, qx, qy, qz, qw, roll, pitch, yaw, wall
        self.odom = []      # t, x, y, z, yaw, vx, vy, wz, wall
        self.cmd = []       # t, seg, vx, wz, wall
        self.cmd_out = []   # t, vx, wz, wall
        self.joints = []    # t, pos fl rl fr rr, vel fl rl fr rr, wall
        self.stats = []     # wall, sim, real, rtf, iterations, paused
        self.clock_first_wall = None
        self.last_clock = None        # from ROS /clock   (--clock-source ros)
        self.last_stats_sim = None    # from gz stats     (--clock-source gz)
        self.last_gt_sim = None       # from pose stamps  (--clock-source gz)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='output directory (created)')
    ap.add_argument('--model', default='togo', help='Gazebo model name (default togo)')
    ap.add_argument('--pose-topic', default=None, help='gz Pose_V topic (default /model/<model>/pose)')
    ap.add_argument('--stats-topic', default='/stats', help='gz WorldStatistics topic (default /stats)')
    ap.add_argument('--cmd-topic', default='/platform_velocity_controller/cmd_vel')
    ap.add_argument('--odom-topic', default='/platform_velocity_controller/odom')
    ap.add_argument('--cmd-out-topic', default='/platform_velocity_controller/cmd_vel_out')
    ap.add_argument('--joint-topic', default='/joint_states')
    ap.add_argument('--controller', default='platform_velocity_controller')
    ap.add_argument('--controller-manager', default='/controller_manager')
    ap.add_argument('--preset', default='standard', choices=sorted(PRESETS))
    ap.add_argument('--sequence', help='JSON list of {name,vx,wz,duration}')
    ap.add_argument('--sequence-file', help='file with a JSON list of {name,vx,wz,duration}')
    ap.add_argument('--step', action='append', help='name:vx:wz:duration (repeatable)')
    ap.add_argument('--rate', type=float, default=50.0, help='command rate in sim-time Hz (default 50)')
    ap.add_argument('--clock-source', default='gz', choices=['gz', 'ros'],
                    help='sim time from gz (/stats throttled to 50 Hz + pose stamps; cheap, default) or '
                         'ros (/clock at ~1 kHz with use_sim_time; costs ~0.5 CPU core in Python)')
    ap.add_argument('--clock-timeout', type=float, default=180.0, help='wall s to wait for the sim clock')
    ap.add_argument('--gt-timeout', type=float, default=240.0, help='wall s to wait for ground-truth poses')
    ap.add_argument('--controller-timeout', type=float, default=300.0, help='wall s to wait for an active controller')
    ap.add_argument('--no-settle', action='store_true', help='do not wait for the robot to settle')
    ap.add_argument('--settle-timeout', type=float, default=30.0, help='sim s to wait for settling (then continue)')
    ap.add_argument('--settle-window', type=float, default=1.0, help='sim s window for the settle criterion')
    ap.add_argument('--settle-lin', type=float, default=0.01, help='settled if horizontal and vertical speed < this [m/s]')
    ap.add_argument('--settle-ang', type=float, default=0.02, help='settled if roll/pitch/yaw rate < this [rad/s]')
    ap.add_argument('--pre-roll', type=float, default=1.0, help='sim s of zero command before the sequence')
    ap.add_argument('--post-roll', type=float, default=1.0, help='sim s of zero command after the sequence')
    ap.add_argument('--max-wall', type=float, default=1200.0, help='overall wall-clock limit [s]')
    ap.add_argument('--gravity', type=float, default=None,
                    help='world gravity magnitude [m/s^2]; enables the ballistic (airborne) metrics')
    ap.add_argument('--resummarize', action='store_true',
                    help='offline: recompute <out>/summary.json from the CSVs and segments of an earlier run '
                         '(no ROS/Gazebo needed; the old summary is kept as summary_prev.json)')
    args = ap.parse_args()

    if args.resummarize:
        return resummarize(args)
    seq = parse_sequence(args)
    pose_topic = args.pose_topic or f'/model/{args.model}/pose'
    os.makedirs(args.out, exist_ok=True)

    # imports that need the ROS / gz environment (kept here so --help works anywhere)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from controller_manager_msgs.srv import ListControllers
    from builtin_interfaces.msg import Time as TimeMsg
    from gz.transport13 import Node as GzNode, SubscribeOptions
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.world_stats_pb2 import WorldStatistics

    rec = Recorder()
    status = {'status': 'running', 'events': {}}
    wall0 = time.time()
    stop_flag = threading.Event()

    def sim_now():
        """latest known simulation time [s] (None before the first clock/stats/pose message)"""
        if args.clock_source == 'ros':
            return rec.last_clock
        c = [t for t in (rec.last_stats_sim, rec.last_gt_sim) if t is not None]
        return max(c) if c else None

    def ev(name, extra=None):
        status['events'][name] = {'wall_s': round(time.time() - wall0, 3)}
        if sim_now() is not None:
            status['events'][name]['sim_s'] = round(sim_now(), 4)
        if extra:
            status['events'][name].update(extra)
        print(f'[rover_drive_test] {time.time() - wall0:8.2f}s wall  {name} {extra or ""}', flush=True)

    def on_signal(signum, _frame):
        print(f'[rover_drive_test] signal {signum}: stopping', flush=True)
        status['interrupted'] = signum
        stop_flag.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # ------------------------------------------------------------------ gz-transport
    gz = GzNode()

    def on_pose(msg):
        wall = time.time()
        for p in msg.pose:
            if p.name == args.model:
                t = p.header.stamp.sec + p.header.stamp.nsec * 1e-9
                o = p.orientation
                r, pi, y = quat_to_rpy(o.x, o.y, o.z, o.w)
                with rec.lock:
                    rec.gt.append((t, p.position.x, p.position.y, p.position.z, o.x, o.y, o.z, o.w, r, pi, y, wall))
                    rec.last_gt_sim = t
                break

    def on_stats(msg):
        # gz publishes stats up to every iteration (~1 kHz); the subscription is throttled to 50/s
        ts = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        with rec.lock:
            rec.stats.append((time.time(), ts,
                              msg.real_time.sec + msg.real_time.nsec * 1e-9, msg.real_time_factor,
                              msg.iterations, int(msg.paused)))
            rec.last_stats_sim = ts

    if not gz.subscribe(Pose_V, pose_topic, on_pose):
        print(f'failed to subscribe to gz topic {pose_topic}', file=sys.stderr)
    sopt = SubscribeOptions()
    sopt.msgs_per_sec = 50
    gz.subscribe(WorldStatistics, args.stats_topic, on_stats, sopt)

    # ------------------------------------------------------------------ ROS 2
    rclpy.init(args=None)
    node = Node('rover_drive_test', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, args.clock_source == 'ros')])
    qos_be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    qos_rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

    def on_clock(msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if rec.clock_first_wall is None:
            rec.clock_first_wall = time.time()
        rec.last_clock = t

    def on_odom(msg):
        q = msg.pose.pose.orientation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with rec.lock:
            rec.odom.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw,
                             msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z, time.time()))

    def on_cmd_out(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with rec.lock:
            rec.cmd_out.append((t, msg.twist.linear.x, msg.twist.angular.z, time.time()))

    def on_joints(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            pos = [msg.position[idx[WHEEL_JOINTS[k]]] for k in ('fl', 'rl', 'fr', 'rr')]
            vel = [msg.velocity[idx[WHEEL_JOINTS[k]]] for k in ('fl', 'rl', 'fr', 'rr')]
        except (KeyError, IndexError):
            return
        with rec.lock:
            rec.joints.append((t, *pos, *vel, time.time()))

    if args.clock_source == 'ros':
        node.create_subscription(Clock, '/clock', on_clock, qos_be)
    node.create_subscription(Odometry, args.odom_topic, on_odom, qos_rel)
    node.create_subscription(TwistStamped, args.cmd_out_topic, on_cmd_out, qos_rel)
    node.create_subscription(JointState, args.joint_topic, on_joints, qos_rel)
    cmd_pub = node.create_publisher(TwistStamped, args.cmd_topic, 10)
    lc_cli = node.create_client(ListControllers, f'{args.controller_manager}/list_controllers')

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    def wall_left():
        return args.max_wall - (time.time() - wall0)

    def wait_until(pred, timeout_wall, what):
        t_end = time.time() + min(timeout_wall, max(0.0, wall_left()))
        while time.time() < t_end:
            if stop_flag.is_set():
                raise KeyboardInterrupt
            if pred():
                return True
            time.sleep(0.02)
        raise TimeoutError(f'timeout ({timeout_wall:.0f} s wall) waiting for {what}')

    def publish(vx, wz):
        m = TwistStamped()
        if args.clock_source == 'ros':
            m.header.stamp = node.get_clock().now().to_msg()
        else:  # stamp with the latest known sim time (the controller rejects stamps older than cmd_vel_timeout)
            t = sim_now() or 0.0
            m.header.stamp = TimeMsg(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1000000000)
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(vx)
        m.twist.angular.z = float(wz)
        cmd_pub.publish(m)
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def controller_active():
        if not lc_cli.service_is_ready():
            return False
        fut = lc_cli.call_async(ListControllers.Request())
        t_end = time.time() + 5.0
        while not fut.done() and time.time() < t_end:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return False
        for c in fut.result().controller:
            if c.name == args.controller:
                status['controller_state'] = c.state
                return c.state == 'active'
        return False

    def run_segment(name, vx, wz, duration, seg_log):
        period = 1.0 / args.rate
        t_start = sim_now()
        last_pub = -1e9
        while True:
            if stop_flag.is_set():
                raise KeyboardInterrupt
            if wall_left() <= 0:
                raise TimeoutError('max wall time exceeded during the sequence')
            now = sim_now()
            if now - t_start >= duration:
                break
            if now - last_pub >= period - 1e-6:
                ts = publish(vx, wz)
                with rec.lock:
                    rec.cmd.append((ts, name, vx, wz, time.time()))
                last_pub = now
            time.sleep(0.002)
        t_end = sim_now()
        seg_log.append({'name': name, 'vx': vx, 'wz': wz, 'duration_cmd': duration,
                        't0': t_start, 't1': t_end})

    exit_code = 0
    segments = []
    try:
        wait_until(lambda: sim_now() is not None, args.clock_timeout,
                   'sim clock (' + ('/clock' if args.clock_source == 'ros' else f'gz {args.stats_topic}') + ')')
        ev('clock_received')
        wait_until(lambda: len(rec.gt) > 0, args.gt_timeout, f'ground truth on gz {pose_topic}')
        ev('ground_truth_received', {'first_gt_sim_s': rec.gt[0][0], 'spawn_z': rec.gt[0][3]})
        last_poll = 0.0

        def ctrl_pred():
            nonlocal last_poll
            if time.time() - last_poll < 1.0:
                return False
            last_poll = time.time()
            return controller_active()
        wait_until(ctrl_pred, args.controller_timeout, f'controller {args.controller} active')
        ev('controller_active')
        wait_until(lambda: len(rec.odom) > 0, 30.0, f'odometry on {args.odom_topic}')
        ev('odometry_received')

        if not args.no_settle:
            t_lim = sim_now() + args.settle_timeout
            settled = False
            while sim_now() < t_lim:
                if stop_flag.is_set():
                    raise KeyboardInterrupt
                if wall_left() <= 0:
                    raise TimeoutError('max wall time exceeded while settling')
                with rec.lock:
                    g = np.array([r[:11] for r in rec.gt[-int(args.settle_window * 60) - 5:]]) if rec.gt else None
                if g is not None and len(g) > 10 and g[-1, 0] - g[0, 0] >= args.settle_window * 0.9:
                    w = g[g[:, 0] >= g[-1, 0] - args.settle_window]
                    if _is_settled(w, args.settle_lin, args.settle_ang):
                        settled = True
                        break
                time.sleep(0.05)
            ev('settled' if settled else 'settle_timeout')

        # pre-roll with zero command
        if args.pre_roll > 0:
            run_segment('pre', 0.0, 0.0, args.pre_roll, segments)
        ev('sequence_start')
        for s in seq:
            run_segment(s['name'], s['vx'], s['wz'], s['duration'], segments)
        if args.post_roll > 0:
            run_segment('post', 0.0, 0.0, args.post_roll, segments)
        ev('sequence_done')
        status['status'] = 'ok'
    except TimeoutError as e:
        status['status'] = 'timeout'
        status['error'] = str(e)
        print(f'[rover_drive_test] TIMEOUT: {e}', file=sys.stderr, flush=True)
        exit_code = 2
    except KeyboardInterrupt:
        status['status'] = 'interrupted'
        exit_code = 130
    except Exception as e:  # noqa: BLE001 - report anything, still write outputs
        status['status'] = 'error'
        status['error'] = repr(e)
        print(f'[rover_drive_test] ERROR: {e!r}', file=sys.stderr, flush=True)
        exit_code = 3
    finally:
        # always leave the robot with a zero command
        try:
            for _ in range(5):
                publish(0.0, 0.0)
                time.sleep(0.02)
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.3)  # let the last messages arrive
        try:
            gz.unsubscribe(pose_topic)
            gz.unsubscribe(args.stats_topic)
        except Exception:  # noqa: BLE001
            pass
        with rec.lock:
            data = {k: list(getattr(rec, k)) for k in ('gt', 'odom', 'cmd', 'cmd_out', 'joints', 'stats')}
        write_csvs(args.out, data)
        try:
            summary = summarize(data, segments, seq, args, status, wall0)
        except Exception as e:  # noqa: BLE001
            summary = {'status': status, 'summary_error': repr(e)}
        with open(os.path.join(args.out, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2, default=_json_default)
        print(f"[rover_drive_test] wrote {args.out}/summary.json (status {status['status']})", flush=True)
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return exit_code


def run():
    """Entry point: exits with os._exit so that the gz-transport C++ threads cannot abort
    the interpreter teardown ('terminate called without an active exception', exit 134)."""
    try:
        code = main()
    except SystemExit as e:  # argparse errors / --help
        code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    except BaseException:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        code = 3
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def read_csvs(out):
    data = {}
    for k in ('gt', 'odom', 'cmd', 'cmd_out', 'joints', 'stats'):
        rows = []
        p = os.path.join(out, f'{k}.csv')
        if os.path.exists(p):
            with open(p) as f:
                rd = csv.reader(f)
                next(rd, None)
                for row in rd:
                    vals = []
                    for v in row:
                        try:
                            vals.append(float(v))
                        except ValueError:
                            vals.append(v)
                    rows.append(tuple(vals))
        data[k] = rows
    return data


def resummarize(args):
    old_p = os.path.join(args.out, 'summary.json')
    with open(old_p) as f:
        old = json.load(f)
    data = read_csvs(args.out)
    segments = [{k: s[k] for k in ('name', 'vx', 'wz', 'duration_cmd', 't0', 't1')} for s in old.get('segments', [])]
    status = old.get('status', {'status': 'unknown'})
    for k, v in old.get('args', {}).items():   # keep the original run settings unless overridden here
        if k not in ('out', 'resummarize', 'gravity') and hasattr(args, k):
            setattr(args, k, v)
    S = summarize(data, segments, old.get('sequence', []), args, status, time.time())
    S['resummarized'] = True
    os.replace(old_p, os.path.join(args.out, 'summary_prev.json'))
    with open(old_p, 'w') as f:
        json.dump(S, f, indent=2, default=_json_default)
    print(f'[rover_drive_test] re-summarized {old_p}')
    return 0


def _json_default(o):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def _is_settled(w, lin, ang):
    """w: rows (t, x, y, z, qx, qy, qz, qw, roll, pitch, yaw) over a window."""
    t = w[:, 0]
    if t[-1] - t[0] <= 0:
        return False
    dt = t[-1] - t[0]
    v_h = math.hypot(w[-1, 1] - w[0, 1], w[-1, 2] - w[0, 2]) / dt
    v_z = (w[:, 3].max() - w[:, 3].min()) / dt
    rates = [np.ptp(np.unwrap(w[:, k])) / dt for k in (8, 9, 10)]
    return v_h < lin and v_z < lin and max(rates) < ang


def write_csvs(out, data):
    heads = {
        'gt': ['t_sim', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw', 'wall'],
        'odom': ['t_sim', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'wz', 'wall'],
        'cmd': ['t_sim', 'segment', 'vx', 'wz', 'wall'],
        'cmd_out': ['t_sim', 'vx', 'wz', 'wall'],
        'joints': ['t_sim', 'pos_fl', 'pos_rl', 'pos_fr', 'pos_rr', 'vel_fl', 'vel_rl', 'vel_fr', 'vel_rr', 'wall'],
        'stats': ['wall', 'sim_time', 'real_time', 'real_time_factor', 'iterations', 'paused'],
    }
    for k, h in heads.items():
        with open(os.path.join(out, f'{k}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(h)
            for r in data[k]:
                w.writerow([f'{v:.6f}' if isinstance(v, float) else v for v in r])


def _interp(t, tt, yy):
    return float(np.interp(t, tt, yy))


_trapz = getattr(np, 'trapezoid', None) or np.trapz  # numpy 2.x / 1.x


def summarize(data, segments, seq, args, status, wall0):
    gt = np.array([r[:11] for r in data['gt']], dtype=float) if data['gt'] else np.zeros((0, 11))
    od = np.array([r[:8] for r in data['odom']], dtype=float) if data['odom'] else np.zeros((0, 8))
    jt = np.array([r[:9] for r in data['joints']], dtype=float) if data['joints'] else np.zeros((0, 9))
    co = np.array([r[:3] for r in data['cmd_out']], dtype=float) if data['cmd_out'] else np.zeros((0, 3))
    st = np.array([r[:5] for r in data['stats']], dtype=float) if data['stats'] else np.zeros((0, 5))
    S = {'status': status, 'args': vars(args), 'sequence': seq,
         'constants': {'wheel_radius_true': WHEEL_RADIUS_TRUE, 'wheel_radius_ctrl': WHEEL_RADIUS_CTRL,
                       'track_true': TRACK_TRUE, 'wheel_separation_ctrl': WHEEL_SEP_CTRL},
         'samples': {k: len(v) for k, v in data.items()}}
    if len(gt) > 1:
        # sort & de-duplicate by sim time
        _, ui = np.unique(gt[:, 0], return_index=True)
        gt = gt[ui]
        gt[:, 8] = np.unwrap(gt[:, 8]); gt[:, 9] = np.unwrap(gt[:, 9]); gt[:, 10] = np.unwrap(gt[:, 10])
        dts = np.diff(gt[:, 0])
        S['rates_hz'] = {'gt': float(1.0 / np.median(dts)) if len(dts) else None}
    if len(od) > 1:
        _, ui = np.unique(od[:, 0], return_index=True)
        od = od[ui]
        od[:, 4] = np.unwrap(od[:, 4])
        S.setdefault('rates_hz', {})['odom'] = float(1.0 / np.median(np.diff(od[:, 0])))
    if len(jt) > 1:
        _, ui = np.unique(jt[:, 0], return_index=True)
        jt = jt[ui]
    # ---------------- spawn / settle
    if len(gt) > 5:
        sp = {'first_gt_sim_s': gt[0, 0], 'spawn_z': gt[0, 3], 'spawn_xy': [gt[0, 1], gt[0, 2]]}
        win = args.settle_window
        t_settle = None
        i0 = 0
        for i in range(len(gt)):
            while gt[i, 0] - gt[i0, 0] > win:
                i0 += 1
            if gt[i, 0] - gt[0, 0] >= win and gt[i, 0] - gt[i0, 0] >= 0.9 * win:
                if _is_settled(gt[i0:i + 1], args.settle_lin, args.settle_ang):
                    t_settle = gt[i0, 0]
                    break
        if t_settle is not None:
            k = np.searchsorted(gt[:, 0], t_settle)
            sp.update({'settle_time_after_first_pose_s': t_settle - gt[0, 0], 'settled_sim_s': t_settle,
                       'settled_z': gt[k, 3], 'drop_height_m': gt[0, 3] - gt[k, 3],
                       'settled_roll_deg': math.degrees(gt[k, 8]), 'settled_pitch_deg': math.degrees(gt[k, 9]),
                       'drift_xy_before_settle_m': math.hypot(gt[k, 1] - gt[0, 1], gt[k, 2] - gt[0, 2])})
            pre = gt[gt[:, 0] <= t_settle]
            sp['max_abs_roll_deg_before_settle'] = math.degrees(np.abs(pre[:, 8]).max())
            sp['max_abs_pitch_deg_before_settle'] = math.degrees(np.abs(pre[:, 9]).max())
        else:
            sp['settle_time_after_first_pose_s'] = None
        S['spawn'] = sp
    # ---------------- segments
    segs = []
    for s in segments:
        t0, t1 = s['t0'], s['t1']
        r = dict(s)
        r['duration'] = t1 - t0
        g = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)] if len(gt) else gt
        if len(g) < 3:
            r['note'] = 'not enough ground-truth samples'
            segs.append(r)
            continue
        x0, y0, yaw0 = _interp(t0, gt[:, 0], gt[:, 1]), _interp(t0, gt[:, 0], gt[:, 2]), _interp(t0, gt[:, 0], gt[:, 10])
        x1, y1, yaw1 = _interp(t1, gt[:, 0], gt[:, 1]), _interp(t1, gt[:, 0], gt[:, 2]), _interp(t1, gt[:, 0], gt[:, 10])
        c, sn = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x1 - x0, y1 - y0
        r['gt_dist_long_m'] = dx * c + dy * sn
        r['gt_dist_lat_m'] = -dx * sn + dy * c
        r['gt_disp_m'] = math.hypot(dx, dy)
        r['gt_path_m'] = float(np.sum(np.hypot(np.diff(g[:, 1]), np.diff(g[:, 2]))))
        r['gt_dyaw_deg'] = math.degrees(yaw1 - yaw0)
        r['cmd_dist_nominal_m'] = s['vx'] * r['duration']
        r['cmd_dyaw_nominal_deg'] = math.degrees(s['wz'] * r['duration'])
        if len(co) > 2:
            cm = co[(co[:, 0] >= t0) & (co[:, 0] <= t1)]
            if len(cm) > 2:
                r['cmd_dist_limited_m'] = float(_trapz(cm[:, 1], cm[:, 0]))
                r['cmd_dyaw_limited_deg'] = math.degrees(float(_trapz(cm[:, 2], cm[:, 0])))
        if r['cmd_dist_nominal_m'] != 0:
            r['dist_ratio_gt_over_cmd'] = r['gt_dist_long_m'] / r['cmd_dist_nominal_m']
        if s['wz'] != 0:
            r['dyaw_ratio_gt_over_cmd'] = r['gt_dyaw_deg'] / r['cmd_dyaw_nominal_deg']
        # attitude / bounce
        r['max_abs_roll_deg'] = math.degrees(np.abs(g[:, 8]).max())
        r['max_abs_pitch_deg'] = math.degrees(np.abs(g[:, 9]).max())
        r['max_roll_dev_deg'] = math.degrees(np.abs(g[:, 8] - g[0, 8]).max())
        r['max_pitch_dev_deg'] = math.degrees(np.abs(g[:, 9] - g[0, 9]).max())
        tz = g[:, 0] - g[0, 0]
        if len(g) > 5 and tz[-1] > 0:
            pz = np.polyfit(tz, g[:, 3], 1)
            zres = g[:, 3] - np.polyval(pz, tz)
            r['z_detrended_std_mm'] = 1000 * float(np.std(zres))
            r['z_detrended_p2p_mm'] = 1000 * float(np.ptp(zres))
            vz = np.diff(g[:, 3]) / np.diff(g[:, 0])
            r['vz_rms_mps'] = float(np.sqrt(np.mean(vz ** 2)))
            if args.gravity and len(g) > 6:
                # ballistic (all wheels off the ground) proxy: vertical acceleration within 15 % of -g
                az = np.diff(vz) / (0.5 * (g[2:, 0] - g[:-2, 0]))
                ball = np.abs(az + args.gravity) < 0.15 * args.gravity
                r['ballistic_frac'] = float(np.mean(ball))
                runs, cur = [], 0
                for b in ball:
                    cur = cur + 1 if b else 0
                    runs.append(cur)
                r['max_ballistic_s'] = float(max(runs) * np.median(np.diff(g[:, 0]))) if runs else 0.0
                r['max_abs_az_mps2'] = float(np.abs(az).max())
        # steady-state window (skip the acceleration phase)
        ts0 = t0 + (2.0 if r['duration'] >= 4.0 else r['duration'] / 2.0)
        gs = gt[(gt[:, 0] >= ts0) & (gt[:, 0] <= t1)]
        if len(gs) > 5 and gs[-1, 0] - gs[0, 0] > 0.5:
            T = gs[-1, 0] - gs[0, 0]
            yawm = 0.5 * (gs[0, 10] + gs[-1, 10])
            ddx, ddy = gs[-1, 1] - gs[0, 1], gs[-1, 2] - gs[0, 2]
            r['ss_window_s'] = [ts0, t1]
            r['gt_v_ss_mps'] = (ddx * math.cos(yawm) + ddy * math.sin(yawm)) / T
            r['gt_vlat_ss_mps'] = (-ddx * math.sin(yawm) + ddy * math.cos(yawm)) / T
            r['gt_wz_ss_radps'] = (gs[-1, 10] - gs[0, 10]) / T
            if s['vx'] != 0:
                r['speed_tracking_ratio'] = r['gt_v_ss_mps'] / s['vx']
            if s['wz'] != 0:
                r['yawrate_tracking_ratio'] = r['gt_wz_ss_radps'] / s['wz']
            if len(jt) > 5:
                js = jt[(jt[:, 0] >= ts0) & (jt[:, 0] <= t1)]
                if len(js) > 3:
                    wl = float(np.mean((js[:, 5] + js[:, 6]) / 2.0))   # left wheels rad/s
                    wr = float(np.mean((js[:, 7] + js[:, 8]) / 2.0))   # right wheels rad/s
                    r['wheel_rate_left_radps'] = wl
                    r['wheel_rate_right_radps'] = wr
                    v_w = WHEEL_RADIUS_TRUE * (wl + wr) / 2.0
                    r['wheel_v_true_radius_mps'] = v_w
                    if abs(v_w) > 1e-3:
                        r['long_slip_ratio'] = 1.0 - r['gt_v_ss_mps'] / v_w
                    dv = WHEEL_RADIUS_TRUE * (wr - wl)
                    r['wheel_rim_speed_diff_mps'] = dv
                    if abs(r['gt_wz_ss_radps']) > 1e-4 and abs(dv) > 1e-4:
                        # wheel separation that would make wheel kinematics match the true yaw rate
                        r['effective_track_m'] = dv / r['gt_wz_ss_radps']
                        r['implied_separation_multiplier'] = (WHEEL_RADIUS_CTRL * (wr - wl) / r['gt_wz_ss_radps']) / WHEEL_SEP_CTRL
                        r['yaw_slip_ratio'] = 1.0 - r['gt_wz_ss_radps'] * TRACK_TRUE / dv
        # whole-segment wheel-based slip for straight segments
        if len(jt) > 5 and s['vx'] != 0 and s['wz'] == 0:
            js = jt[(jt[:, 0] >= t0) & (jt[:, 0] <= t1)]
            if len(js) > 3:
                vwheel = WHEEL_RADIUS_TRUE * (js[:, 5] + js[:, 6] + js[:, 7] + js[:, 8]) / 4.0
                d_wheel = float(_trapz(vwheel, js[:, 0]))
                r['wheel_dist_true_radius_m'] = d_wheel
                if abs(d_wheel) > 1e-3:
                    r['long_slip_ratio_segment'] = 1.0 - r['gt_dist_long_m'] / d_wheel
        # odometry over the segment
        if len(od) > 5:
            ox0, oy0, oyaw0 = _interp(t0, od[:, 0], od[:, 1]), _interp(t0, od[:, 0], od[:, 2]), _interp(t0, od[:, 0], od[:, 4])
            ox1, oy1, oyaw1 = _interp(t1, od[:, 0], od[:, 1]), _interp(t1, od[:, 0], od[:, 2]), _interp(t1, od[:, 0], od[:, 4])
            odx, ody = ox1 - ox0, oy1 - oy0
            r['odom_dist_long_m'] = odx * math.cos(oyaw0) + ody * math.sin(oyaw0)
            r['odom_dyaw_deg'] = math.degrees(oyaw1 - oyaw0)
            r['odom_minus_gt_dist_m'] = r['odom_dist_long_m'] - r['gt_dist_long_m']
            r['odom_minus_gt_dyaw_deg'] = r['odom_dyaw_deg'] - r['gt_dyaw_deg']
        # stopping behaviour (zero-command segment following motion)
        if s['vx'] == 0 and s['wz'] == 0 and len(g) > 10:
            vh = np.hypot(np.diff(g[:, 1]), np.diff(g[:, 2])) / np.diff(g[:, 0])
            wzz = np.abs(np.diff(g[:, 10]) / np.diff(g[:, 0]))
            moving = np.where((vh > 0.02) | (wzz > 0.02))[0]
            r['stop_time_s'] = float(g[moving[-1] + 1, 0] - t0) if len(moving) else 0.0
            k = moving[-1] + 1 if len(moving) else 0
            r['stop_dist_m'] = float(math.hypot(g[k, 1] - g[0, 1], g[k, 2] - g[0, 2]))
            r['stop_dyaw_deg'] = float(math.degrees(g[k, 10] - g[0, 10]))
            last = g[g[:, 0] >= t1 - 1.0]
            if len(last) > 3 and last[-1, 0] - last[0, 0] > 0.3:
                r['residual_creep_mps'] = math.hypot(last[-1, 1] - last[0, 1], last[-1, 2] - last[0, 2]) / (last[-1, 0] - last[0, 0])
                r['residual_yaw_rate_degps'] = math.degrees((last[-1, 10] - last[0, 10]) / (last[-1, 0] - last[0, 0]))
        segs.append(r)
    S['segments'] = segs
    # ---------------- run-level
    if len(gt) and segments:
        ta, tb = segments[0]['t0'], segments[-1]['t1']
        gr = gt[(gt[:, 0] >= ta) & (gt[:, 0] <= tb)]
        if len(gr):
            S['run'] = {'sim_s': tb - ta, 'max_abs_roll_deg': math.degrees(np.abs(gr[:, 8]).max()),
                        'max_abs_pitch_deg': math.degrees(np.abs(gr[:, 9]).max()),
                        'start_xyz': gr[0, 1:4].tolist(), 'end_xyz': gr[-1, 1:4].tolist(),
                        'start_yaw_deg': math.degrees(gr[0, 10]), 'end_yaw_deg': math.degrees(gr[-1, 10])}
            if len(st) > 2:
                ss = st[(st[:, 1] >= ta) & (st[:, 1] <= tb)]
                if len(ss) > 2:
                    S['run']['rtf_mean'] = float(np.mean(ss[:, 3]))
                    S['run']['rtf_min'] = float(np.min(ss[:, 3]))
                    S['run']['rtf_from_clocks'] = float((ss[-1, 1] - ss[0, 1]) / (ss[-1, 2] - ss[0, 2])) if ss[-1, 2] > ss[0, 2] else None
                    S['run']['wall_s'] = float(ss[-1, 0] - ss[0, 0])
    S['tool_wall_s'] = time.time() - wall0
    return S


if __name__ == '__main__':
    run()
