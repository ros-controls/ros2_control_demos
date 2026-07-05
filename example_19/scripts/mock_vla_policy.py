#!/usr/bin/env python3
"""
Mock VLA-style action-policy node: streams positions-only action chunks
at a configurable replan period. Matches the InferenceBridgeController's
policy_frequency for timing synthesis.
"""
import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MockVlaPolicy(Node):
    def __init__(self):
        super().__init__("mock_vla_policy")
        self.declare_parameter("controller", "inference_bridge")
        self.declare_parameter("joints", ["joint1", "joint2"])
        self.declare_parameter("policy_hz", 25.0)      # MUST match controller policy_frequency
        self.declare_parameter("chunk_size", 30)        # waypoints per action chunk
        self.declare_parameter("replan_period", 0.6)    # s between published chunks (overlap if < chunk span)
        self.declare_parameter("amplitude", 0.3)        # rad, peak deviation from home (keep small!)
        self.declare_parameter("home", [])              # home pose; defaults to zeros if empty

        controller = self.get_parameter("controller").value
        self.joints = list(self.get_parameter("joints").value)
        self.policy_hz = float(self.get_parameter("policy_hz").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.amp = float(self.get_parameter("amplitude").value)
        home = list(self.get_parameter("home").value)
        self.home = home if len(home) == len(self.joints) else [0.0] * len(self.joints)

        self.pub = self.create_publisher(JointTrajectory, f"/{controller}/joint_trajectory", 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(float(self.get_parameter("replan_period").value), self._replan)
        self.get_logger().info(
            f"mock VLA policy -> /{controller}/joint_trajectory : {len(self.joints)} joints, "
            f"chunk={self.chunk_size} @ {self.policy_hz} Hz, replan every "
            f"{self.get_parameter('replan_period').value}s")

    def _ref(self, joint_idx, t):
        """Smooth bounded per-joint reference: each joint a sine with its own phase."""
        phase = joint_idx * (2.0 * math.pi / max(1, len(self.joints)))
        return self.home[joint_idx] + self.amp * math.sin(0.5 * math.pi * t + phase)

    def _replan(self):
        """Emit one positions-only action chunk of future waypoints from 'now'."""
        now = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        dt = 1.0 / self.policy_hz
        traj = JointTrajectory()
        traj.joint_names = self.joints
        for k in range(self.chunk_size):
            t = now + k * dt  # absolute time on the reference -> chunk continues the motion
            point = JointTrajectoryPoint()
            point.positions = [self._ref(j, t) for j in range(len(self.joints))]

            traj.points.append(point)
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = MockVlaPolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
