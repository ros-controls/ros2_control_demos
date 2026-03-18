#!/usr/bin/env python3
#
# Copyright (C) 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Julia Jia

"""
Test motion sequences for Open Duck Mini robot.

Notes:
- Assumes controller runs at 50 Hz with stabilization (50 steps) + blend-in (200 steps) warm-up.
- Publishes 7D commands: [lin_x, lin_y, ang_z, neck_pitch, head_pitch, head_yaw, head_roll] (head defaults to 0).
- Optionally ramps velocity to reduce oscillation on step changes.
"""

import rclpy
from rclpy.node import Node
from ros2_control_demo_example_18.msg import VelocityCommandWithHead
import time


class DriveForward(Node):
    def __init__(self):
        super().__init__("drive_forward")
        self.publisher_ = self.create_publisher(
            VelocityCommandWithHead, "/motion_controller/cmd_velocity_with_head", 10
        )
        self.get_logger().info("Drive forward node started")

        # Controller timing parameters (matching motion_controller)
        self.controller_update_rate = 50.0  # Hz
        self.stabilization_delay_steps = 50  # Steps before ONNX starts
        self.blend_in_steps = 200  # Steps to blend in ONNX actions
        self.stabilization_delay_time = (
            self.stabilization_delay_steps / self.controller_update_rate
        )  # ~1.0s
        self.blend_in_time = self.blend_in_steps / self.controller_update_rate  # ~4.0s
        self.controller_warmup_time = self.stabilization_delay_time + self.blend_in_time  # ~5.0s

    def _sleep_time(self) -> float:
        return 1.0 / float(self.controller_update_rate)

    def _wait_for_subscriber(self, timeout_s: float = 10.0) -> None:
        self.get_logger().info("Waiting for subscribers...")
        start = time.time()
        while self.publisher_.get_subscription_count() == 0 and rclpy.ok():
            if time.time() - start > timeout_s:
                raise RuntimeError("Timed out waiting for controller subscriber")
            time.sleep(0.1)
        self.get_logger().info(
            f"Connected to {self.publisher_.get_subscription_count()} subscriber(s)"
        )
        time.sleep(0.5)  # let connection settle

    @staticmethod
    def _make_cmd(
        lin_x: float = 0.0,
        lin_y: float = 0.0,
        ang_z: float = 0.0,
        head_cmds=None,
    ) -> VelocityCommandWithHead:
        msg = VelocityCommandWithHead()
        msg.base_velocity.linear.x = float(lin_x)
        msg.base_velocity.linear.y = float(lin_y)
        msg.base_velocity.angular.z = float(ang_z)
        msg.head_commands = head_cmds if head_cmds is not None else [0.0, 0.0, 0.0, 0.0]
        return msg

    def _publish_for(
        self, duration_s: float, msg: VelocityCommandWithHead, log_every_n: int = 200
    ) -> None:
        sleep_time = self._sleep_time()
        start = time.time()
        count = 0
        while rclpy.ok() and (time.time() - start) < float(duration_s):
            self.publisher_.publish(msg)
            count += 1
            if log_every_n > 0 and count % log_every_n == 0:
                self.get_logger().info(
                    f"Published {count} messages (elapsed: {time.time() - start:.1f}s)"
                )
            time.sleep(sleep_time)

    def _ramp_velocity(
        self,
        duration_s: float,
        lin_vel_x: float,
        lin_vel_y: float,
        ang_vel_z: float,
        start_increment: float,
    ) -> None:
        """Ramp the dominant axis (x or y) from ±start_increment to its target."""
        publish_rate = float(self.controller_update_rate)
        sleep_time = self._sleep_time()

        dominant_is_x = abs(lin_vel_x) >= abs(lin_vel_y)
        target_vel = float(lin_vel_x) if dominant_is_x else float(lin_vel_y)
        axis = "x" if dominant_is_x else "y"
        if duration_s <= 0 or abs(target_vel) <= 1e-3:
            return

        steps = max(1, int(float(duration_s) * publish_rate))
        start_vel = float(start_increment if target_vel > 0 else -start_increment)
        delta = (target_vel - start_vel) / float(steps)

        self.get_logger().info(f"Ramping vel_{axis} to {target_vel:.3f} over {duration_s:.1f}s")
        for i in range(steps):
            current = start_vel + float(i) * delta
            if dominant_is_x:
                msg = self._make_cmd(lin_x=current, lin_y=lin_vel_y, ang_z=ang_vel_z)
            else:
                msg = self._make_cmd(lin_x=lin_vel_x, lin_y=current, ang_z=ang_vel_z)
            self.publisher_.publish(msg)
            if i % 25 == 0:
                self.get_logger().info(f"Ramp: vel_{axis}={current:.3f}")
            time.sleep(sleep_time)

    def drive_forward(
        self,
        duration: float = 2.0,
        lin_vel_x: float = 0.15,
        lin_vel_y: float = 0.0,
        ang_vel_z: float = 0.0,
        stabilize_time: float | None = None,
        wait_for_warmup: bool = True,
        ramp_up_duration: float = 3.0,
        ramp_up_increment: float = 0.01,
    ) -> None:
        """Publish velocity commands for a test segment."""
        self._wait_for_subscriber()

        if stabilize_time is None:
            stabilize_time = self.controller_warmup_time if wait_for_warmup else 0.0

        if stabilize_time > 0:
            self.get_logger().info(f"Stabilizing {stabilize_time:.1f}s (zero cmd)")
            self._publish_for(stabilize_time, self._make_cmd(), log_every_n=0)

        if duration > 0:
            if ramp_up_duration > 0:
                self._ramp_velocity(
                    duration_s=ramp_up_duration,
                    lin_vel_x=lin_vel_x,
                    lin_vel_y=lin_vel_y,
                    ang_vel_z=ang_vel_z,
                    start_increment=ramp_up_increment,
                )
            self.get_logger().info(
                f"Commanding for {duration:.1f}s: lin_x={lin_vel_x:.3f}, lin_y={lin_vel_y:.3f}, ang_z={ang_vel_z:.3f}"
            )
            self._publish_for(
                duration, self._make_cmd(lin_x=lin_vel_x, lin_y=lin_vel_y, ang_z=ang_vel_z)
            )

        self.get_logger().info("Stopping (2.0s zero cmd)")
        # Send zero velocity for 2s to bring the robot to a stop.
        self._publish_for(2.0, self._make_cmd(), log_every_n=0)


def main(args=None):
    rclpy.init(args=args)
    node = DriveForward()

    node.get_logger().info("=== Starting Multi-Motion Test ===")
    node.get_logger().info(
        f"Controller timing: {node.stabilization_delay_time:.1f}s stabilization + "
        f"{node.blend_in_time:.1f}s blend-in = {node.controller_warmup_time:.1f}s total warm-up"
    )

    # Warm-up: allow stabilization + blend-in before motion
    node.get_logger().info(f"Warm-up ({node.controller_warmup_time:.1f}s)")
    node.drive_forward(
        duration=0.0, stabilize_time=node.controller_warmup_time, wait_for_warmup=True
    )

    # Forward walk (only test)
    node.get_logger().info("Forward walk (20s at lin_x=0.15)")
    node.drive_forward(
        duration=20.0,
        lin_vel_x=0.15,
        lin_vel_y=0.0,
        stabilize_time=0.0,
        wait_for_warmup=False,
    )

    node.get_logger().info("=== Multi-Motion Test Complete ===")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
