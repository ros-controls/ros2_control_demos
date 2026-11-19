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
Trace the word "ROS" with the end-effector of the cartesian_trajectory_controller.

Each letter is one continuous stroke in the base X-Z plane, placed low and centered so the word
stays inside the arm's reachable workspace. A green marker per letter shows the intended path in
RViz. Run against a demo started with run_policy:=false, so this node owns ~/cartesian_reference:

    ros2 launch ros2_control_demo_example_20 cartesian_demo.launch.py run_policy:=false
    ros2 run ros2_control_demo_example_20 ros_writer.py
"""

import math

import rclpy
from geometry_msgs.msg import Point, Transform
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

# Each letter is one continuous polyline in a unit box (u: 0..1 left->right, v: 0..1 bottom->top).
STROKE_R = [
    (0.0, 0.0),
    (0.0, 1.0),
    (0.65, 1.0),
    (0.82, 0.85),
    (0.82, 0.62),
    (0.62, 0.5),
    (0.0, 0.5),
    (0.85, 0.0),
]
STROKE_O = [
    (0.5 + 0.5 * math.cos(t), 0.5 + 0.5 * math.sin(t))
    for t in [2.0 * math.pi * i / 22 for i in range(23)]
]
STROKE_S = [
    (0.85, 0.82),
    (0.5, 1.0),
    (0.15, 0.85),
    (0.18, 0.62),
    (0.5, 0.52),
    (0.82, 0.42),
    (0.82, 0.16),
    (0.5, 0.0),
    (0.13, 0.15),
]
WORD = [STROKE_R, STROKE_O, STROKE_S]


class RosWriter(Node):
    def __init__(self):
        super().__init__("ros_writer")
        self.declare_parameter("controller", "cartesian_motion")
        self.declare_parameter("base", "base_link")
        self.declare_parameter("tip", "tool0")
        self.declare_parameter("letter_h", 0.24)  # m, letter height
        self.declare_parameter("letter_w", 0.10)  # m, letter width
        self.declare_parameter("gap", 0.04)  # m, space between letters
        self.declare_parameter("center_down", 0.16)  # m, lower the word to stay within reach
        self.declare_parameter("center_x", 0.0)  # m, shift the word into the reachable -x zone
        self.declare_parameter("speed", 0.06)  # m/s, tool speed along the path

        controller = self.get_parameter("controller").value
        self.base = self.get_parameter("base").value
        self.tip = self.get_parameter("tip").value
        self.lh = float(self.get_parameter("letter_h").value)
        self.lw = float(self.get_parameter("letter_w").value)
        self.gap = float(self.get_parameter("gap").value)
        self.down = float(self.get_parameter("center_down").value)
        self.cx = float(self.get_parameter("center_x").value)
        self.speed = float(self.get_parameter("speed").value)

        self.pub = self.create_publisher(
            MultiDOFJointTrajectory, f"/{controller}/cartesian_reference", 10
        )
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, f"/{controller}/ros_letters", latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.anchor = None
        self.quat = None
        self.total_time = 0.0
        self.last_pub = None
        self.timer = self.create_timer(0.5, self._tick)

    def _tcp(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.base, self.tip, rclpy.time.Time()
            ).transform
        except Exception:  # noqa: BLE001
            return None

    def _place(self, letter_idx, u, v):
        """Map a unit-box (u, v) of letter_idx to a base-frame (x, z)."""
        ax, _, az = self.anchor
        word_w = len(WORD) * self.lw + (len(WORD) - 1) * self.gap
        x_center = ax - self.cx
        x = x_center + word_w / 2.0 - (letter_idx * (self.lw + self.gap) + u * self.lw)
        z = (az - self.down) - self.lh / 2.0 + v * self.lh
        return x, z

    def _tf(self, x, y, z):
        t = Transform()
        t.translation.x, t.translation.y, t.translation.z = x, y, z
        t.rotation = self.quat
        return t

    def _build(self):
        """Return (all waypoints, per-letter strokes for the markers).

        Everything stays at one Y: a right-angle pen lift makes the cubic overshoot and diverge the
        IK, so the letters are connected in-plane and the TCP draws a faint link between them.
        """
        y_pen = self.anchor[1]
        waypoints = []
        letter_strokes = []
        for idx, letter in enumerate(WORD):
            stroke = []
            for u, v in letter:
                x, z = self._place(idx, u, v)
                stroke.append((x, y_pen, z))
            waypoints.extend(stroke)
            letter_strokes.append(stroke)
        return waypoints, letter_strokes

    @staticmethod
    def _densify(waypoints, max_seg=0.008):
        """Split segments longer than max_seg, so the cubic does not overshoot at letter corners."""
        dense = [waypoints[0]]
        for a, b in zip(waypoints[:-1], waypoints[1:]):
            d = math.dist(a, b)
            steps = max(1, int(math.ceil(d / max_seg)))
            for k in range(1, steps + 1):
                f = k / steps
                dense.append(tuple(a[j] + f * (b[j] - a[j]) for j in range(3)))
        return dense

    def _publish_word(self, start, waypoints):
        """Trace the word from `start`, the current tool position, so the run-in to the first
        letter is paced at `speed` like every other segment instead of being a jump."""
        waypoints = self._densify([start] + waypoints)
        traj = MultiDOFJointTrajectory()
        traj.header.frame_id = self.base
        t = 0.0
        prev = None
        for wp in waypoints:
            if prev is not None:
                dist = math.dist(wp, prev)
                t += max(dist / self.speed, 1e-3)
            prev = wp
            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms.append(self._tf(*wp))
            pt.time_from_start = rclpy.duration.Duration(seconds=t + 1e-2).to_msg()
            traj.points.append(pt)
        self.total_time = t
        self.pub.publish(traj)

    def _publish_markers(self, letter_strokes):
        arr = MarkerArray()
        for idx, stroke in enumerate(letter_strokes):
            m = Marker()
            m.header.frame_id = self.base
            m.ns = "ROS"
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.006
            m.color.a = 1.0
            m.color.g = 1.0  # green
            for x, y, z in stroke:
                p = Point()
                p.x, p.y, p.z = x, y, z
                m.points.append(p)
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _tick(self):
        tcp = self._tcp()
        if tcp is None:
            self.get_logger().info("waiting for tool0 transform...", throttle_duration_sec=2.0)
            return
        here = (tcp.translation.x, tcp.translation.y, tcp.translation.z)
        if self.anchor is None:
            self.anchor = here
            self.quat = tcp.rotation
            waypoints, strokes = self._build()
            self._publish_markers(strokes)
            self._publish_word(here, waypoints)
            self.last_pub = self.get_clock().now()
            self.get_logger().info(f"writing ROS ({self.total_time:.1f}s per pass)")
            return
        elapsed = (self.get_clock().now() - self.last_pub).nanoseconds * 1e-9
        if elapsed >= self.total_time + 1.0:  # re-trace once the pass completes
            waypoints, _ = self._build()
            self._publish_word(here, waypoints)
            self.last_pub = self.get_clock().now()


def main():
    rclpy.init()
    node = RosWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
