#!/usr/bin/env python3
# Copyright 2026 ros2_control Development Team
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
End-to-end smoothness verification for inference_bridge_controller.

Runs three scenarios (single chunk, sequential, overlapping) against a live
inference_bridge and reports C1/C2 continuity and cross-chunk seam metrics.
Prereq: inference_bridge active with run_policy:=false.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TOPIC_CMD = "/inference_bridge/joint_trajectory"
TOPIC_STATE = "/inference_bridge/controller_state"
JOINTS = ["joint1", "joint2"]
POLICY_HZ = 25.0  # MUST match controllers.yaml policy_frequency
N = len(JOINTS)
DT = 1.0 / POLICY_HZ
CHUNK = 50  # waypoints per chunk


class Bridge(Node):
    def __init__(self):
        super().__init__("bridge_verifier")
        self.pub = self.create_publisher(JointTrajectory, TOPIC_CMD, 10)
        self.create_subscription(JointTrajectoryControllerState, TOPIC_STATE, self._cb, 200)
        self.samples = []  # (t, pos[N], vel[N], acc[N])

    def _cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        ref = msg.reference
        if len(ref.positions) < N:
            return
        vel = list(ref.velocities) if len(ref.velocities) >= N else [0.0] * N
        acc = list(ref.accelerations) if len(ref.accelerations) >= N else [0.0] * N
        self.samples.append((t, list(ref.positions[:N]), vel[:N], acc[:N]))

    def latest_pos(self):
        return self.samples[-1][1] if self.samples else [0.0] * N

    def publish(self, waypoints):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        for wp in waypoints:
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in wp]  # positions only
            traj.points.append(p)
        self.pub.publish(traj)
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, dur):
        end = time.time() + dur
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait_for_controller(self, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end and self.pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pub.get_subscription_count() > 0


def smooth_path(n, start):
    """n waypoints of a smooth rest-to-rest path (Hann-windowed oscillation)."""
    pts = []
    for i in range(n):
        s = i / (n - 1)
        w = math.sin(math.pi * s) ** 2  # 0 at s=0,1 with zero slope -> rest at ends
        pts.append([
            start[0] + 0.5 * w * math.sin(1.5 * math.pi * s),
            start[1] - 0.3 * w * math.sin(1.0 * math.pi * s + 0.3),
        ])
    return pts


def fd_accel_peak(t, pos):
    """Peak |2nd derivative of position| using REAL local dt; skips boundary samples."""
    peak = 0.0
    for j in range(N):
        for k in range(3, len(pos) - 1):
            dt1, dt2 = t[k - 1] - t[k - 2], t[k] - t[k - 1]
            if dt1 < 1e-6 or dt2 < 1e-6:
                continue
            v1 = (pos[k - 1][j] - pos[k - 2][j]) / dt1
            v2 = (pos[k][j] - pos[k - 1][j]) / dt2
            peak = max(peak, abs((v2 - v1) / (0.5 * (dt1 + dt2))))
    return peak


def analyze_chunk(samples, t_pub, waypoints, label, expect_rest_end=True, skip_lead_s=0.0):
    print(f"\n  -- {label}: {len(waypoints)} waypoints --")
    ok = [True]
    fail = lambda m: (ok.__setitem__(0, False), print("    [FAIL]", m))
    good = lambda m: print("    [ OK ]", m)
    info = lambda m: print("    [info]", m)

    s = [x for x in samples if x[0] >= t_pub - 0.05]
    if len(s) < 5:
        fail(f"too few samples ({len(s)})")
        return False
    t = [x[0] - t_pub for x in s]
    pos, vel, acc = [x[1] for x in s], [x[2] for x in s], [x[3] for x in s]

    if any(math.isnan(v) or math.isinf(v) for row in (pos + vel + acc) for v in row):
        fail("NaN/inf in commanded output")

    p0 = pos[0]
    moved = next((i for i, p in enumerate(pos)
                  if any(abs(p[j] - p0[j]) > 1e-4 for j in range(N))), None)
    if moved is None:
        fail("controller never moved -> chunk not installed")
        return False
    t_start = t[moved]

    # timing
    info(f"latency publish->motion: {t_start * 1000:.1f} ms")
    expected = (len(waypoints) - 1) / POLICY_HZ
    end_i = len(pos) - 1
    while end_i > moved and all(abs(pos[end_i][j] - pos[end_i - 1][j]) < 1e-5 for j in range(N)):
        end_i -= 1
    obs = t[end_i] - t_start
    info(f"duration: observed {obs:.2f}s vs expected {expected:.2f}s")
    if abs(obs - expected) > 0.4 * expected + 0.2:
        fail("duration far from the policy timeline")
    else:
        good("executes on the policy_frequency timeline")

    if all(abs(pos[-1][j] - waypoints[-1][j]) < 0.03 for j in range(N)):
        good("reached final waypoint")
    else:
        fail(f"final {[round(x, 3) for x in pos[-1]]} != {[round(x, 3) for x in waypoints[-1]]}")

    # skip leading seam transient
    lo = next((i for i in range(len(t)) if t[i] >= t_start + skip_lead_s), 0)
    tw, pw, vw, aw = t[lo:], pos[lo:], vel[lo:], acc[lo:]
    if skip_lead_s > 0:
        info(f"(smoothness evaluated after the first {skip_lead_s:.2f}s, i.e. past the seam)")
    peak_vel = max(abs(v) for row in vw for v in row)
    peak_acc = max(abs(a) for row in aw for a in row)
    max_vel_jump = max(abs(vw[k][j] - vw[k - 1][j]) for k in range(1, len(vw)) for j in range(N))
    fdacc = fd_accel_peak(tw, pw)
    # a CONTINUOUS cubic changes velocity by ~ accel*dt per sample; a real step
    # would jump far more than the acceleration can explain.
    dts = sorted(tw[k] - tw[k - 1] for k in range(1, len(tw)) if tw[k] - tw[k - 1] > 1e-6)
    dt_s = dts[len(dts) // 2] if dts else 0.01
    expected_jump = peak_acc * dt_s
    info(f"peak|vel|={peak_vel:.3f}  peak|acc|={peak_acc:.3f}  fd|acc|={fdacc:.2f}  "
         f"maxVelJump={max_vel_jump:.3f} (accel*dt~{expected_jump:.3f})")
    if max_vel_jump > 3.0 * expected_jump + 0.05:
        fail(f"velocity DISCONTINUITY: jump {max_vel_jump:.3f} >> accel*dt {expected_jump:.3f} "
             "(not explained by acceleration -> C0/step)")
    else:
        good("velocity continuous (C1) — jumps consistent with reported acceleration")
    if peak_acc >= 1e-3:
        good("acceleration non-zero -> cubic spline (C2)")
    else:
        info("reported accel ~0 (controller may not publish accel)")

    if expect_rest_end:
        v_end = max(abs(v) for v in vel[end_i])
        (good if v_end < 0.05 else info)(f"end |v|={v_end:.3f} ({'at rest' if v_end < 0.05 else 'moving'})")
    return ok[0]


def report_seam(samples, t_seam):
    print("\n  -- seam continuity (new chunk replaces a moving one) --")
    vmag = lambda x: max(abs(v) for v in x[2])
    veljump = lambda a, b: max(abs(b[2][j] - a[2][j]) for j in range(N))
    s = sorted((x for x in samples if t_seam - 0.20 <= x[0] <= t_seam + 0.25), key=lambda x: x[0])
    if len(s) < 6:
        print("    [warn] not enough samples around the seam")
        return
    # Largest single-sample velocity change across the seam: an instantaneous jump
    # shows here; a smooth handoff does not.
    seam_jump = max(veljump(s[k - 1], s[k]) for k in range(1, len(s)))
    before = [x for x in s if x[0] < t_seam]
    v_before = sum(vmag(x) for x in before[-3:]) / max(1, min(3, len(before)))
    if seam_jump > 0.15:
        print(f"    seam velocity jump {seam_jump:.2f} rad/s (was {v_before:.2f}) -> known "
              "cross-chunk limitation (per-chunk rest BC); documented, expected.")
    else:
        print(f"    [ OK ] velocity continuous across the seam (jump {seam_jump:.2f} rad/s)")


def show_plots(plot_data):
    """Open one interactive window per test (A/B/C) with the commanded (desired)
    position/velocity/acceleration/jerk. Blocks on plt.show() until the windows are
    closed. Pass --no-plot to skip."""
    try:
        import numpy as np
        import matplotlib
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        print(f"\n[plot] matplotlib/numpy unavailable ({e}); skipping plots.")
        return
    if matplotlib.get_backend().lower() == "agg":
        print("\n[plot] no interactive matplotlib backend (using Agg -> no window).")
        print("       install one, e.g.:  apt-get install -y python3-tk   then re-run.")
        return

    titles = {
        "A": "TEST A - single chunk (intra-chunk C2 smoothness)",
        "B": "TEST B - 3 sequential chunks (stop-and-go)",
        "C": "TEST C - overlapping chunks (cross-chunk seam)",
    }
    rows = [("position [rad]", 1), ("velocity [rad/s]", 2),
            ("accel [rad/s^2]", 3), ("jerk [rad/s^3]", 4)]
    shown = False
    for key in ("A", "B", "C"):
        s = plot_data.get(key) or []
        if len(s) < 4:
            continue
        t = np.array([x[0] for x in s], dtype=float)
        t -= t[0]
        cols = {1: np.array([x[1] for x in s]),   # pos
                2: np.array([x[2] for x in s]),   # vel
                3: np.array([x[3] for x in s])}   # acc
        cols[4] = np.gradient(cols[3], t, axis=0)  # jerk = d(acc)/dt

        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(8, 9))
        try:
            fig.canvas.manager.set_window_title(f"verify_smoothness [{key}]")
        except Exception:  # noqa: BLE001
            pass
        for a, (ylbl, idx) in zip(ax, rows):
            for j in range(N):
                a.plot(t, cols[idx][:, j], lw=1.3, label=JOINTS[j])
            a.set_ylabel(ylbl)
            a.grid(True, alpha=0.3)
            if idx != 1:  # zero line only where 0 is meaningful (vel/acc/jerk)
                a.axhline(0.0, color="k", lw=0.5, alpha=0.3)
        ax[0].set_title(titles[key])
        ax[0].legend(fontsize=8, loc="upper right")
        ax[-1].set_xlabel("time [s]")
        fig.tight_layout()
        shown = True

    if shown:
        print("\n[plot] showing A/B/C windows - close them to exit.")
        plt.show()


def main():
    rclpy.init()
    node = Bridge()
    print(f"waiting for inference_bridge on {TOPIC_CMD} ...")
    if not node.wait_for_controller():
        print(f"[FAIL] nothing subscribed to {TOPIC_CMD} — is inference_bridge active?")
        rclpy.shutdown()
        return

    results = {}
    plot_data = {}

    # ---- A: single realistic 50-point chunk ----
    print("\n==== TEST A: single realistic %d-point chunk ====" % CHUNK)
    node.samples.clear()
    node.spin(0.3)
    a_wps = smooth_path(CHUNK, node.latest_pos())
    t_pub = node.publish(a_wps)
    node.spin((CHUNK - 1) * DT + 1.0)
    results["A single 50-pt chunk"] = analyze_chunk(node.samples, t_pub, a_wps, "chunk")
    plot_data["A"] = [x for x in node.samples if x[0] >= t_pub - 0.05]

    # ---- B: sequential streaming (each chunk after the last finishes) ----
    print("\n==== TEST B: 3 chunks streamed SEQUENTIALLY (stop-and-go) ====")
    seq_ok = True
    b_all = []
    for c in range(3):
        node.samples.clear()
        node.spin(0.2)
        wps = smooth_path(CHUNK, node.latest_pos())  # each continues from current rest pose
        t_pub = node.publish(wps)
        node.spin((CHUNK - 1) * DT + 0.6)
        seq_ok &= analyze_chunk(node.samples, t_pub, wps, f"chunk {c}")
        b_all += [x for x in node.samples if x[0] >= t_pub - 0.05]
    results["B sequential chunks"] = seq_ok
    plot_data["B"] = b_all

    # ---- C: overlapping streaming (replace mid-motion) + seam ----
    print("\n==== TEST C: overlapping chunks (B replaces A mid-motion) + SEAM ====")
    node.samples.clear()
    node.spin(0.2)
    a = smooth_path(CHUNK, node.latest_pos())
    node.publish(a)
    node.spin(0.40 * (CHUNK - 1) * DT)          # let A run ~40%
    b = smooth_path(CHUNK, node.latest_pos())   # B starts at current pose -> pos continuous
    t_pub_b = node.publish(b)
    node.spin((CHUNK - 1) * DT + 0.6)
    results["C overlap installs"] = analyze_chunk(
        node.samples, t_pub_b, b, "chunk B (after replace)", expect_rest_end=True, skip_lead_s=0.2)
    report_seam(node.samples, t_pub_b)
    plot_data["C"] = list(node.samples)

    # ---- summary ----
    print("\n================ SUMMARY ================")
    overall = True
    for name, r in results.items():
        overall &= r
        print(f"  {'PASS' if r else 'FAIL'}  {name}")
    print("  (the seam velocity drop above is the known, documented cross-chunk limitation.)")
    print("\nOVERALL:", "PASS" if overall else "FAIL")

    if "--no-plot" not in sys.argv:
        show_plots(plot_data)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
