#!/usr/bin/env python3

"""Publish a small smooth task-space trajectory directly to lowlevel."""

import argparse
import math

import rclpy
from rclpy.node import Node

from yumi_interfaces.msg import TaskSpacePose
from yumi_interfaces.msg import TaskSpaceReference


class ManualTaskSpaceTrajectoryPublisher(Node):
    def __init__(self, args):
        super().__init__("manual_task_space_trajectory_publisher")
        self.publisher = self.create_publisher(TaskSpaceReference, "/task_space_reference", 10)

        self.rate_hz = args.rate
        self.duration_sec = args.duration
        self.hold_final = args.hold_final
        self.total_steps = max(1, int(self.rate_hz * self.duration_sec))
        self.step_index = 0

        self.left_start_y = args.left_start_y
        self.left_goal_y = args.left_goal_y
        self.right_start_y = args.right_start_y
        self.right_goal_y = args.right_goal_y
        self.x = args.x
        self.z = args.z

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_next_reference)
        self.get_logger().info(
            f"Publishing smooth task-space trajectory: "
            f"left_y {self.left_start_y:.3f} -> {self.left_goal_y:.3f}, "
            f"right_y {self.right_start_y:.3f} -> {self.right_goal_y:.3f}, "
            f"duration {self.duration_sec:.2f} s, rate {self.rate_hz:.1f} Hz."
        )

    def build_pose(self, x, y, z):
        pose = TaskSpacePose()
        pose.x = x
        pose.y = y
        pose.z = z
        # pitch = pi/2 的近似四元数，用于保持末端朝向箱体侧面。
        pose.qx = 0.0
        pose.qy = 0.7071
        pose.qz = 0.0
        pose.qw = 0.7071
        return pose

    def smooth_ratio(self):
        raw_ratio = min(1.0, self.step_index / float(self.total_steps))
        # 半余弦插值保证起止速度接近 0，比线性插值更不容易“刹不住”。
        return 0.5 - 0.5 * math.cos(math.pi * raw_ratio)

    def publish_next_reference(self):
        if self.step_index > self.total_steps and not self.hold_final:
            self.get_logger().info("Task-space trajectory finished. Stop publishing.")
            rclpy.shutdown()
            return

        ratio = self.smooth_ratio()
        left_y = self.left_start_y + ratio * (self.left_goal_y - self.left_start_y)
        right_y = self.right_start_y + ratio * (self.right_goal_y - self.right_start_y)

        reference = TaskSpaceReference()
        reference.stamp = self.get_clock().now().to_msg()
        reference.reference_mode = TaskSpaceReference.MODE_VELOCITY
        reference.use_object_target = False
        reference.enable_force_control = False
        reference.left_target_pose = self.build_pose(self.x, left_y, self.z)
        reference.right_target_pose = self.build_pose(self.x, right_y, self.z)
        reference.object_target_pose = self.build_pose(0.0, 0.0, 0.0)
        reference.target_internal_force = 0.0

        self.publisher.publish(reference)
        self.step_index += 1


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish a smooth dual-arm TaskSpaceReference trajectory."
    )
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate in Hz.")
    parser.add_argument("--duration", type=float, default=8.0, help="Trajectory duration in seconds.")
    parser.add_argument("--x", type=float, default=0.350, help="Target x position for both arms.")
    parser.add_argument("--z", type=float, default=0.158, help="Target z position for both arms.")
    parser.add_argument("--left-start-y", type=float, default=0.150)
    parser.add_argument("--left-goal-y", type=float, default=0.135)
    parser.add_argument("--right-start-y", type=float, default=-0.150)
    parser.add_argument("--right-goal-y", type=float, default=-0.135)
    parser.add_argument(
        "--hold-final",
        action="store_true",
        help="Keep publishing the final point after the trajectory finishes.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ManualTaskSpaceTrajectoryPublisher(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
