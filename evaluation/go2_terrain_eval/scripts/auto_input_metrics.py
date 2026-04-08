#!/usr/bin/env python3

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from control_input_msgs.msg import Inputs
import yaml


def quaternion_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class AutoInputMetricsNode(Node):
    def __init__(self, scenario_path: Path, result_json: Path):
        super().__init__("go2_auto_input_metrics")
        with open(scenario_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        self.result_json = result_json
        self.result_json.parent.mkdir(parents=True, exist_ok=True)

        self.publisher = self.create_publisher(Inputs, "control_input", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 50)

        self.publish_rate_hz = float(self.config.get("publish_rate_hz", 50.0))
        self.period = 1.0 / self.publish_rate_hz
        self.steps = self.config["steps"]
        self.timeout_sec = float(self.config["timeout_sec"])
        self.monitoring_start_sec = float(self.config.get("monitoring_start_sec", 0.0))
        self.quality = self.config.get("quality", {})
        fall = self.config["fall_detection"]
        self.min_base_z = float(fall["min_base_z"])
        self.max_base_z_drop = float(fall.get("max_base_z_drop", 1e9))
        self.max_abs_roll_deg = float(fall["max_abs_roll_deg"])
        self.max_abs_pitch_deg = float(fall["max_abs_pitch_deg"])

        self.start_wall = time.monotonic()
        self.latest_odom = None
        self.start_pose = None
        self.last_pose = None
        self.path_length = 0.0
        self.roll_samples = []
        self.pitch_samples = []
        self.yaw_samples = []
        self.z_samples = []
        self.min_z_seen = float("inf")
        self.fall_reason = None
        self.fall_time = None
        self.completed = False

        self.timer = self.create_timer(self.period, self.tick)

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        elapsed = time.monotonic() - self.start_wall
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(quat.x, quat.y, quat.z, quat.w)
        self.roll_samples.append(roll)
        self.pitch_samples.append(pitch)
        self.yaw_samples.append(yaw)
        self.z_samples.append(pos.z)
        self.min_z_seen = min(self.min_z_seen, pos.z)

        current_pose = (pos.x, pos.y, pos.z)
        if self.start_pose is None:
            self.start_pose = current_pose
        if self.last_pose is not None:
            dx = current_pose[0] - self.last_pose[0]
            dy = current_pose[1] - self.last_pose[1]
            self.path_length += math.hypot(dx, dy)
        self.last_pose = current_pose

        if elapsed < self.monitoring_start_sec:
            return

        start_z = self.start_pose[2] if self.start_pose is not None else pos.z

        if pos.z < self.min_base_z:
            self.mark_failed("base_z")
        if (start_z - pos.z) > self.max_base_z_drop:
            self.mark_failed("base_z_drop")
        if abs(math.degrees(roll)) > self.max_abs_roll_deg:
            self.mark_failed("roll_limit")
        if abs(math.degrees(pitch)) > self.max_abs_pitch_deg:
            self.mark_failed("pitch_limit")

    def mark_failed(self, reason: str):
        if self.completed:
            return
        self.fall_reason = reason
        self.fall_time = time.monotonic() - self.start_wall
        self.completed = True

    def current_input(self):
        elapsed = time.monotonic() - self.start_wall
        remaining = elapsed
        for step in self.steps:
            duration = float(step["duration"])
            if remaining <= duration:
                msg = Inputs()
                msg.command = int(step.get("command", 0))
                msg.lx = float(step.get("lx", 0.0))
                msg.ly = float(step.get("ly", 0.0))
                msg.rx = float(step.get("rx", 0.0))
                msg.ry = float(step.get("ry", 0.0))
                return msg
            remaining -= duration
        return Inputs()

    def tick(self):
        elapsed = time.monotonic() - self.start_wall
        if elapsed > self.timeout_sec:
            self.completed = True

        msg = self.current_input()
        self.publisher.publish(msg)
        if int(elapsed * self.publish_rate_hz) % 50 == 0:
            self.get_logger().info(
                f"[AutoInputDebug] t={elapsed:.2f} command={msg.command} lx={msg.lx:.3f} ly={msg.ly:.3f} rx={msg.rx:.3f} ry={msg.ry:.3f}"
            )

        if self.completed:
            self.finalize()
            rclpy.shutdown()

    def finalize(self):
        end_pose = self.last_pose or (0.0, 0.0, 0.0)
        start_pose = self.start_pose or end_pose
        distance_xy = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        roll_rms = math.sqrt(sum(r * r for r in self.roll_samples) / len(self.roll_samples)) if self.roll_samples else 0.0
        pitch_rms = math.sqrt(sum(p * p for p in self.pitch_samples) / len(self.pitch_samples)) if self.pitch_samples else 0.0
        yaw_rms = math.sqrt(sum(y * y for y in self.yaw_samples) / len(self.yaw_samples)) if self.yaw_samples else 0.0
        duration_executed = time.monotonic() - self.start_wall
        mean_forward_velocity = distance_xy / duration_executed if duration_executed > 1e-9 else 0.0
        yaw_change = 0.0
        if len(self.yaw_samples) >= 2:
            yaw_change = math.atan2(
                math.sin(self.yaw_samples[-1] - self.yaw_samples[0]),
                math.cos(self.yaw_samples[-1] - self.yaw_samples[0]),
            )
        z_mean = sum(self.z_samples) / len(self.z_samples) if self.z_samples else 0.0
        base_z_std = (
            math.sqrt(sum((z - z_mean) * (z - z_mean) for z in self.z_samples) / len(self.z_samples))
            if self.z_samples
            else 0.0
        )
        yaw_change_deg = math.degrees(yaw_change)

        target_yaw_change_deg = self.quality.get("target_yaw_change_deg")
        min_abs_yaw_change_deg = self.quality.get("min_abs_yaw_change_deg")
        max_abs_yaw_error_deg = self.quality.get("max_abs_yaw_error_deg")

        yaw_change_error_deg = None
        if target_yaw_change_deg is not None:
            yaw_change_error_deg = yaw_change_deg - float(target_yaw_change_deg)

        meets_yaw_goal = True
        if min_abs_yaw_change_deg is not None:
            meets_yaw_goal = meets_yaw_goal and abs(yaw_change_deg) >= float(min_abs_yaw_change_deg)
        if max_abs_yaw_error_deg is not None and yaw_change_error_deg is not None:
            meets_yaw_goal = meets_yaw_goal and abs(yaw_change_error_deg) <= float(max_abs_yaw_error_deg)

        task_completion_success = (self.fall_reason is None) and meets_yaw_goal

        result = {
            "scenario": self.config.get("name", "unknown"),
            "success": self.fall_reason is None,
            "task_completion_success": task_completion_success,
            "fall_reason": self.fall_reason,
            "time_to_failure": self.fall_time,
            "duration_executed": duration_executed,
            "distance_xy": distance_xy,
            "mean_forward_velocity": mean_forward_velocity,
            "path_length": self.path_length,
            "start_pose": start_pose,
            "end_pose": end_pose,
            "roll_rms_deg": math.degrees(roll_rms),
            "pitch_rms_deg": math.degrees(pitch_rms),
            "yaw_rms_deg": math.degrees(yaw_rms),
            "yaw_change_deg": yaw_change_deg,
            "yaw_change_error_deg": yaw_change_error_deg,
            "meets_yaw_goal": meets_yaw_goal,
            "min_base_z": None if self.min_z_seen == float("inf") else self.min_z_seen,
            "base_z_std": base_z_std,
        }

        with open(self.result_json, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)


def append_csv(result_json: Path, csv_path: Path, extra: dict):
    with open(result_json, "r", encoding="utf-8") as f:
        row = json.load(f)
    row.update(extra)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(row.keys())
    exists = csv_path.exists()
    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--result-json", required=True)
    parser.add_argument("--append-csv")
    parser.add_argument("--terrain")
    parser.add_argument("--mode")
    args = parser.parse_args()

    rclpy.init()
    node = AutoInputMetricsNode(Path(args.scenario), Path(args.result_json))
    rclpy.spin(node)

    if args.append_csv:
        append_csv(
            Path(args.result_json),
            Path(args.append_csv),
            {"terrain": args.terrain, "mode": args.mode},
        )


if __name__ == "__main__":
    main()
