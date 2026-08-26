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
Mock Cartesian action-chunking policy: streams end-effector pose chunks to the
cartesian_trajectory_controller the way a real VLA / action-chunking policy does.

It "runs inference" at ``policy_hz`` and each call emits ``horizon`` seconds of FUTURE poses at
``action_hz``. Because the horizon is much longer than the replan period (``horizon >> 1/policy_hz``),
consecutive chunks heavily overlap -- a receding horizon. So, unlike a single pre-baked trajectory,
the controller continuously receives new chunks that supersede the previous one and must stitch them
together at the seams (this is the realistic streaming case).

The full commanded path (circle / square / line) is also published once as an RViz LINE_STRIP marker.
"""

import math

import rclpy
from geometry_msgs.msg import Point, Transform
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker


class MockCartesianPolicy(Node):
    def __init__(self):
        super().__init__("mock_cartesian_policy")
        self.declare_parameter("controller", "cartesian_motion")
        self.declare_parameter("base", "base_link")
        self.declare_parameter("tip", "tool0")
        self.declare_parameter("pattern", "circle")  # circle | square | line
        # Action-chunking policy characteristics (receding horizon -> overlapping chunks):
        self.declare_parameter("policy_hz", 5.0)  # inference / replan rate
        self.declare_parameter("action_hz", 30.0)  # in-chunk action rate
        self.declare_parameter("horizon", 1.0)  # s of future poses per chunk (>> 1/policy_hz)
        self.declare_parameter("period", 24.0)  # s to trace one full pattern (bounds TCP speed)
        self.declare_parameter("radius", 0.2)  # m, circle radius / pattern size

        controller = self.get_parameter("controller").value
        self.base = self.get_parameter("base").value
        self.tip = self.get_parameter("tip").value
        self.pattern = self.get_parameter("pattern").value
        self.policy_hz = float(self.get_parameter("policy_hz").value)
        self.action_hz = float(self.get_parameter("action_hz").value)
        self.horizon = float(self.get_parameter("horizon").value)
        self.cycle = float(self.get_parameter("period").value)
        self.r = float(self.get_parameter("radius").value)
        if self.pattern not in ("circle", "square", "line"):
            self.get_logger().warn(
                f"unknown pattern '{self.pattern}'; expected circle | square | line -> using 'line'."
            )
            self.pattern = "line"

        self.pub = self.create_publisher(
            MultiDOFJointTrajectory, f"/{controller}/cartesian_reference", 10
        )
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, f"/{controller}/commanded_path", latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.anchor = None  # (px,py,pz) captured once; pattern is relative to it
        self.quat = None  # fixed tool orientation from the start pose
        self.t0 = None  # phase clock start (set when the anchor is captured)
        # "Run inference" at policy_hz; each call streams the next horizon of poses.
        self.timer = self.create_timer(1.0 / self.policy_hz, self._infer)
        self.get_logger().info(
            f"mock action-chunking policy -> /{controller}/cartesian_reference : "
            f"pattern={self.pattern}, replan {self.policy_hz:.0f} Hz, "
            f"{int(self.horizon * self.action_hz)} poses/chunk @ {self.action_hz:.0f} Hz, "
            f"horizon {self.horizon:.1f}s ({self.horizon * self.policy_hz:.0f}x overlap)"
        )

    def _lookup_tcp(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.base, self.tip, rclpy.time.Time()
            ).transform
        except Exception:  # noqa: BLE001 - transform not ready yet
            return None

    def _pose_at(self, s):
        """Position on the pattern at pattern-time s (seconds), relative to the anchor."""
        ax, ay, az = self.anchor
        if self.pattern == "circle":
            # Anchor is the top of a vertical circle in the base X-Z plane, so s=0 starts exactly at
            # the anchor (no startup dive); the circle then hangs below it.
            a = 2.0 * math.pi * (s / self.cycle)
            return (ax + self.r * math.sin(a), ay, (az - self.r) + self.r * math.cos(a))
        if self.pattern == "square":
            edge = self.cycle / 4.0
            corners = [(0.0, 0.0), (self.r, 0.0), (self.r, self.r), (0.0, self.r)]  # (dx, dz)
            k = int((s % self.cycle) / edge)
            f = ((s % self.cycle) - k * edge) / edge
            (x0, z0), (x1, z1) = corners[k], corners[(k + 1) % 4]
            return (ax + x0 + f * (x1 - x0), ay, az + z0 + f * (z1 - z0))
        # line: back-and-forth along base X (triangle wave)
        tri = abs(((s / self.cycle) % 1.0) * 2.0 - 1.0)  # 0..1..0
        return (ax + self.r * (1.0 - tri), ay, az)

    def _make_transform(self, pos):
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = pos
        tf.rotation = self.quat
        return tf

    def _infer(self):
        if self.anchor is None:
            tcp = self._lookup_tcp()
            if tcp is None:
                self.get_logger().info("waiting for tool0 transform...", throttle_duration_sec=2.0)
                return
            self.anchor = (tcp.translation.x, tcp.translation.y, tcp.translation.z)
            self.quat = tcp.rotation
            self.t0 = self.get_clock().now()
            self._publish_marker()

        # Receding horizon: emit the next `horizon` s of poses from the current phase. time_from_start
        # is relative to now, so successive (overlapping) chunks reference the same wall-clock phase.
        now = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        n = max(2, int(self.horizon * self.action_hz))
        dt = 1.0 / self.action_hz
        traj = MultiDOFJointTrajectory()
        traj.header.frame_id = self.base  # required: poses are in the kinematics base frame
        for k in range(1, n + 1):
            point = MultiDOFJointTrajectoryPoint()
            point.transforms.append(self._make_transform(self._pose_at(now + k * dt)))
            point.time_from_start = rclpy.duration.Duration(seconds=k * dt).to_msg()
            traj.points.append(point)
        self.pub.publish(traj)

    def _publish_marker(self):
        m = Marker()
        m.header.frame_id = self.base
        m.ns = "commanded_path"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.005
        m.color.a = 1.0
        m.color.g = 1.0  # green intended path
        for i in range(201):
            px, py, pz = self._pose_at(self.cycle * i / 200.0)
            p = Point()
            p.x, p.y, p.z = px, py, pz
            m.points.append(p)
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = MockCartesianPolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
