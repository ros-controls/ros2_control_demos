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
End-to-end Cartesian path-fidelity verification for the cartesian_trajectory_controller.

Unlike a joint controller, the thing to verify here is that the TOOL CENTER POINT traces the
commanded Cartesian path. We observe the executed TCP exactly as a user would: via tf2
(base_link -> tool0), while this node publishes the commanded poses. Scenarios:

  A  LIN straight line     -> TCP stays on the start->goal segment (perpendicular deviation)
  B  orientation SLERP     -> TCP reorients in place, monotonically, no position drift
  C  multi-pose chunk      -> TCP passes through each commanded waypoint (policy/streaming case)
  D  Cartesian smoothness  -> TCP velocity continuous (no steps), from finite differences
  E  joint-limit checks    -> count /joint_states samples past URDF limits (deferred-limits metric)
"""

import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Transform
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

CONTROLLER = "cartesian_motion"
BASE = "base_link"
TIP = "tool0"
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
JOINT_LIMIT = math.pi  # r6bot: every joint is [-pi, pi]
POS_TOL = 0.01  # m
DEV_TOL = 0.005  # m, max perpendicular deviation from a commanded straight line
ORI_TOL = 0.05  # rad


# small quaternion helpers ([x, y, z, w])
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def quat_axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)])


def quat_angle(a, b):
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(a, b)))))


class Verifier(Node):
    def __init__(self):
        super().__init__("cartesian_verifier")
        self.pub = self.create_publisher(
            MultiDOFJointTrajectory, f"/{CONTROLLER}/cartesian_reference", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.limit_violations = 0
        self.create_subscription(JointState, "/joint_states", self._js_cb, 50)

    def _js_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS and abs(pos) > JOINT_LIMIT + 1e-6:
                self.limit_violations += 1

    def tcp(self):
        try:
            msg = self.tf_buffer.lookup_transform(BASE, TIP, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        tf = msg.transform
        p = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        q = np.array([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
        return p, q, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def wait_for_tcp(self, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tcp() is not None:
                return True
        return False

    def publish_chunk(self, poses, dt):
        """poses: list of (pos(3), quat(4)); spaced dt apart."""
        traj = MultiDOFJointTrajectory()
        traj.header.frame_id = BASE
        for k, (pos, quat) in enumerate(poses):
            pt = MultiDOFJointTrajectoryPoint()
            tf = Transform()
            tf.translation.x, tf.translation.y, tf.translation.z = (float(v) for v in pos)
            tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w = (float(v) for v in quat)
            pt.transforms.append(tf)
            pt.time_from_start = rclpy.duration.Duration(seconds=(k + 1) * dt).to_msg()
            traj.points.append(pt)
        self.pub.publish(traj)

    def record(self, dur, rate=100.0):
        """Sample the TCP for dur seconds -> list of (t, pos, quat)."""
        out = []
        end = time.time() + dur
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=1.0 / rate)
            s = self.tcp()
            if s is not None and (not out or s[2] > out[-1][0]):
                out.append((s[2], s[0], s[1]))
        return out


def fmt(v):
    return "[" + ", ".join(f"{x:.3f}" for x in v) + "]"


def perp_deviation(samples, a, b):
    """Max perpendicular distance of recorded positions from the segment a->b."""
    d = b - a
    n = np.linalg.norm(d)
    if n < 1e-9:
        return 0.0
    d = d / n
    peak = 0.0
    for _, p, _ in samples:
        v = p - a
        perp = v - np.dot(v, d) * d
        peak = max(peak, float(np.linalg.norm(perp)))
    return peak


def cartesian_smoothness(samples):
    """Peak |vel|, and the largest single-step velocity jump vs accel*dt (C1 sanity)."""
    if len(samples) < 4:
        return 0.0, 0.0, 0.0
    t = np.array([s[0] for s in samples])
    p = np.array([s[1] for s in samples])
    vel, acc = [], []
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        if dt > 1e-6:
            vel.append((np.linalg.norm(p[k] - p[k - 1]) / dt, t[k]))
    peak_v = max((v for v, _ in vel), default=0.0)
    for k in range(1, len(vel)):
        dt = vel[k][1] - vel[k - 1][1]
        if dt > 1e-6:
            acc.append(abs(vel[k][0] - vel[k - 1][0]) / dt)
    peak_a = max(acc, default=0.0)
    max_vjump = max((abs(vel[k][0] - vel[k - 1][0]) for k in range(1, len(vel))), default=0.0)
    return peak_v, peak_a, max_vjump


def tcp_now(node, timeout=2.0):
    """Current (pos, quat), spinning until base_link->tool0 is available so a transient TF miss does
    not crash the caller's tuple-unpack; raises after ``timeout`` if the transform is truly gone.
    """
    end = time.time() + timeout
    while time.time() < end:
        s = node.tcp()
        if s is not None:
            return s[0], s[1]
        rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError("base_link->tool0 transform unavailable")


def main():
    rclpy.init()
    node = Verifier()
    print(f"waiting for {TIP} transform (is cartesian_motion active?) ...")
    if not node.wait_for_tcp():
        print("[FAIL] no tool0 transform - is the controller active and run_policy:=false?")
        rclpy.shutdown()
        return

    results = {}

    def ok(name, cond, detail=""):
        results[name] = cond
        print(f"    [{'  OK  ' if cond else ' FAIL '}] {name}  {detail}")

    # A: LIN straight line
    print("\n==== A: LIN straight line ====")
    p0, q0 = tcp_now(node)
    goal = p0 + np.array([0.15, 0.0, 0.0])
    node.publish_chunk([(goal, q0)], dt=3.0)
    s = node.record(4.5)
    dev = perp_deviation(s, p0, goal)
    pf, _ = tcp_now(node)
    ok("A path-fidelity (TCP on the line)", dev < DEV_TOL, f"max dev {dev * 1000:.1f} mm")
    ok("A reached goal", float(np.linalg.norm(pf - goal)) < POS_TOL, f"err {fmt(pf - goal)}")

    # B: orientation SLERP in place
    print("\n==== B: orientation SLERP (reorient in place) ====")
    p0, q0 = tcp_now(node)
    goal_q = quat_mul(q0, quat_axis_angle([0, 1, 0], 0.6))
    node.publish_chunk([(p0, goal_q)], dt=3.0)
    s = node.record(4.5)
    drift = max((float(np.linalg.norm(p - p0)) for _, p, _ in s), default=0.0)
    angs = [quat_angle(q, goal_q) for _, _, q in s]
    monotonic = all(angs[i] <= angs[i - 1] + 0.02 for i in range(1, len(angs)))
    pf, qf = tcp_now(node)
    ok("B position held during reorient", drift < 0.02, f"max drift {drift * 1000:.1f} mm")
    ok("B orientation converges monotonically (SLERP)", monotonic)
    ok(
        "B reached goal orientation",
        quat_angle(qf, goal_q) < ORI_TOL,
        f"err {quat_angle(qf, goal_q):.3f} rad",
    )

    # C: multi-pose action chunk (arc)
    print("\n==== C: multi-pose chunk (Cartesian arc) ====")
    p0, q0 = tcp_now(node)
    wps = []
    for i in range(1, 6):
        a = 0.5 * math.pi * i / 5.0
        wps.append((p0 + np.array([0.12 * math.sin(a), 0.0, 0.12 * (1 - math.cos(a))]), q0))
    node.publish_chunk(wps, dt=0.6)
    s = node.record(0.6 * len(wps) + 1.5)
    hit = all(any(float(np.linalg.norm(p - wp)) < 0.02 for _, p, _ in s) for wp, _ in wps)
    pf, _ = tcp_now(node)
    ok("C passes through all waypoints", hit)
    ok("C reached final waypoint", float(np.linalg.norm(pf - wps[-1][0])) < POS_TOL)

    # D: Cartesian smoothness (reuse the arc recording)
    print("\n==== D: Cartesian smoothness ====")
    peak_v, peak_a, max_vjump = cartesian_smoothness(s)

    vjump_limit = 0.35 * peak_v + 0.02
    ok(
        "D TCP velocity continuous (no step)",
        max_vjump < vjump_limit,
        f"peak|v|={peak_v:.3f} peak|a|={peak_a:.2f} maxVjump={max_vjump:.3f} (limit {vjump_limit:.3f})",
    )

    # E: joint-limit violations (Denis's metric)
    print("\n==== E: joint-limit violations ====")
    ok("E no joint-limit violations", node.limit_violations == 0, f"count {node.limit_violations}")

    # summary
    print("\n================ SUMMARY ================")
    overall = all(results.values())
    for name, r in results.items():
        print(f"  {'PASS' if r else 'FAIL'}  {name}")
    print(
        "\n  (A plain JointTrajectoryController given these endpoints in joint space would bow the"
    )
    print("   TCP off the straight line - see userdoc. This controller keeps it on the line.)")
    print("\nOVERALL:", "PASS" if overall else "FAIL")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()
