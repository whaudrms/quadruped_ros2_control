#!/usr/bin/env python3

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


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
        super().__init__("go2_auto_input_metrics_dev_v2")
        with open(scenario_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        self.result_json = result_json
        self.result_json.parent.mkdir(parents=True, exist_ok=True)

        self.control_pub = self.create_publisher(Inputs, "/control_input", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 50)

        self.publish_rate_hz = float(self.config.get("publish_rate_hz", 50.0))
        self.period = 1.0 / self.publish_rate_hz
        self.steps = self.config["steps"]
        self.timeout_sec = float(self.config["timeout_sec"])
        self.monitoring_start_sec = float(self.config.get("monitoring_start_sec", 0.0))
        self.initial_window_sec = float(self.config.get("initial_window_sec", 2.0))
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
        self.body_forward_path_length = 0.0
        self.body_lateral_path_length = 0.0
        self.initial_window_body_forward_path_length = 0.0
        self.initial_window_body_lateral_path_length = 0.0
        self.command_active_body_forward_path_length = 0.0
        self.command_active_body_lateral_path_length = 0.0
        self.roll_samples = []
        self.pitch_samples = []
        self.yaw_samples = []
        self.z_samples = []
        self.min_z_seen = float("inf")
        self.fall_reason = None
        self.fall_time = None
        self.completed = False
        self.last_step_name = None

        self.command_active_window_start_sec = None
        self.command_active_window_end_sec = None
        self.command_active_window_start_pose = None
        self.command_active_window_end_pose = None
        self._compute_command_active_window()

        self.timer = self.create_timer(self.period, self.tick)

    def _compute_command_active_window(self):
        elapsed = 0.0
        active_start = None
        active_end = None
        for step in self.steps:
            duration = float(step["duration"])
            twist = step.get("twist", {})
            lin = twist.get("linear", {})
            ang = twist.get("angular", {})
            is_active = any(
                abs(float(v)) > 1e-9
                for v in (
                    lin.get("x", 0.0),
                    lin.get("y", 0.0),
                    lin.get("z", 0.0),
                    ang.get("x", 0.0),
                    ang.get("y", 0.0),
                    ang.get("z", 0.0),
                )
            )
            if is_active:
                if active_start is None:
                    active_start = elapsed
                active_end = elapsed + duration
            elapsed += duration
        self.command_active_window_start_sec = active_start
        self.command_active_window_end_sec = active_end

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
            yaw_ref = self.yaw_samples[-2] if len(self.yaw_samples) >= 2 else yaw
            cos_yaw = math.cos(yaw_ref)
            sin_yaw = math.sin(yaw_ref)
            body_forward = cos_yaw * dx + sin_yaw * dy
            body_lateral = -sin_yaw * dx + cos_yaw * dy
            self.body_forward_path_length += body_forward
            self.body_lateral_path_length += body_lateral
            if elapsed <= self.initial_window_sec:
                self.initial_window_body_forward_path_length += body_forward
                self.initial_window_body_lateral_path_length += body_lateral
            if (
                self.command_active_window_start_sec is not None
                and self.command_active_window_end_sec is not None
                and self.command_active_window_start_sec <= elapsed <= self.command_active_window_end_sec
            ):
                self.command_active_body_forward_path_length += body_forward
                self.command_active_body_lateral_path_length += body_lateral
        self.last_pose = current_pose

        if (
            self.command_active_window_start_sec is not None
            and self.command_active_window_end_sec is not None
            and self.command_active_window_start_sec <= elapsed <= self.command_active_window_end_sec
        ):
            if self.command_active_window_start_pose is None:
                self.command_active_window_start_pose = current_pose
            self.command_active_window_end_pose = current_pose

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

    def current_step(self):
        elapsed = time.monotonic() - self.start_wall
        remaining = elapsed
        for step in self.steps:
            duration = float(step["duration"])
            if remaining <= duration:
                return step
            remaining -= duration
        return None

    @staticmethod
    def build_inputs(step: dict) -> Inputs:
        msg = Inputs()
        ci = step.get("control_input", {})
        msg.command = int(ci.get("command", 0))
        msg.lx = float(ci.get("lx", 0.0))
        msg.ly = float(ci.get("ly", 0.0))
        msg.rx = float(ci.get("rx", 0.0))
        msg.ry = float(ci.get("ry", 0.0))
        return msg

    @staticmethod
    def build_twist(step: dict) -> Twist:
        msg = Twist()
        twist = step.get("twist", {})
        linear = twist.get("linear", {})
        angular = twist.get("angular", {})
        msg.linear.x = float(linear.get("x", 0.0))
        msg.linear.y = float(linear.get("y", 0.0))
        msg.linear.z = float(linear.get("z", 0.0))
        msg.angular.x = float(angular.get("x", 0.0))
        msg.angular.y = float(angular.get("y", 0.0))
        msg.angular.z = float(angular.get("z", 0.0))
        return msg

    def tick(self):
        elapsed = time.monotonic() - self.start_wall
        if elapsed > self.timeout_sec:
            self.completed = True

        step = self.current_step() or {}
        step_name = step.get("name", "complete")
        if step_name != self.last_step_name:
            twist_x = step.get("twist", {}).get("linear", {}).get("x", 0.0)
            command = step.get("control_input", {}).get("command", 0)
            self.get_logger().info(
                f"scenario step='{step_name}' t={elapsed:.2f}s "
                f"command={command} cmd_vel.x={twist_x}"
            )
            self.last_step_name = step_name
        control_msg = self.build_inputs(step)
        twist_msg = self.build_twist(step)
        self.control_pub.publish(control_msg)
        self.cmd_vel_pub.publish(twist_msg)

        if self.completed:
            self.finalize()
            rclpy.shutdown()

    def finalize(self):
        end_pose = self.last_pose or (0.0, 0.0, 0.0)
        start_pose = self.start_pose or end_pose
        distance_xy = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        duration_executed = time.monotonic() - self.start_wall
        mean_forward_velocity = distance_xy / duration_executed if duration_executed > 1e-9 else 0.0
        roll_rms = math.sqrt(sum(r * r for r in self.roll_samples) / len(self.roll_samples)) if self.roll_samples else 0.0
        pitch_rms = math.sqrt(sum(p * p for p in self.pitch_samples) / len(self.pitch_samples)) if self.pitch_samples else 0.0
        yaw_rms = math.sqrt(sum(y * y for y in self.yaw_samples) / len(self.yaw_samples)) if self.yaw_samples else 0.0
        z_mean = sum(self.z_samples) / len(self.z_samples) if self.z_samples else 0.0
        base_z_std = (
            math.sqrt(sum((z - z_mean) * (z - z_mean) for z in self.z_samples) / len(self.z_samples))
            if self.z_samples
            else 0.0
        )
        start_yaw = self.yaw_samples[0] if self.yaw_samples else 0.0
        dx_total = end_pose[0] - start_pose[0]
        dy_total = end_pose[1] - start_pose[1]
        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)
        body_frame_forward_progress = cos_yaw * dx_total + sin_yaw * dy_total
        body_frame_lateral_progress = -sin_yaw * dx_total + cos_yaw * dy_total

        command_active_body_frame_forward_progress = 0.0
        command_active_body_frame_lateral_progress = 0.0
        if self.command_active_window_start_pose and self.command_active_window_end_pose and self.yaw_samples:
            active_dx = self.command_active_window_end_pose[0] - self.command_active_window_start_pose[0]
            active_dy = self.command_active_window_end_pose[1] - self.command_active_window_start_pose[1]
            command_yaw = start_yaw
            command_active_body_frame_forward_progress = math.cos(command_yaw) * active_dx + math.sin(command_yaw) * active_dy
            command_active_body_frame_lateral_progress = -math.sin(command_yaw) * active_dx + math.cos(command_yaw) * active_dy

        result = {
            "scenario": self.config.get("name", ""),
            "success": self.fall_reason is None,
            "task_completion_success": self.fall_reason is None,
            "fall_reason": self.fall_reason,
            "time_to_failure": self.fall_time,
            "duration_executed": duration_executed,
            "distance_xy": distance_xy,
            "mean_forward_velocity": mean_forward_velocity,
            "path_length": self.path_length,
            "body_forward_path_length": self.body_forward_path_length,
            "body_lateral_path_length": self.body_lateral_path_length,
            "body_frame_forward_progress": body_frame_forward_progress,
            "body_frame_lateral_progress": body_frame_lateral_progress,
            "initial_window_sec": self.initial_window_sec,
            "initial_window_body_forward_path_length": self.initial_window_body_forward_path_length,
            "initial_window_body_lateral_path_length": self.initial_window_body_lateral_path_length,
            "command_active_window_start_sec": self.command_active_window_start_sec,
            "command_active_window_end_sec": self.command_active_window_end_sec,
            "command_active_body_forward_path_length": self.command_active_body_forward_path_length,
            "command_active_body_lateral_path_length": self.command_active_body_lateral_path_length,
            "command_active_body_frame_forward_progress": command_active_body_frame_forward_progress,
            "command_active_body_frame_lateral_progress": command_active_body_frame_lateral_progress,
            "start_pose": list(start_pose),
            "end_pose": list(end_pose),
            "roll_rms_deg": math.degrees(roll_rms),
            "pitch_rms_deg": math.degrees(pitch_rms),
            "yaw_rms_deg": math.degrees(yaw_rms),
            "min_base_z": self.min_z_seen if self.z_samples else None,
            "base_z_std": base_z_std,
        }

        with open(self.result_json, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f"result written to {self.result_json}")


def append_summary_row(summary_csv: Path, result_json: Path, terrain: str, mode: str):
    with open(result_json, "r", encoding="utf-8") as f:
        result = json.load(f)

    summary_csv.parent.mkdir(parents=True, exist_ok=True)
    file_exists = summary_csv.exists()
    fieldnames = [
        "terrain",
        "mode",
        "success",
        "task_completion_success",
        "fall_reason",
        "duration_executed",
        "body_frame_forward_progress",
        "command_active_body_frame_forward_progress",
        "body_frame_lateral_progress",
        "roll_rms_deg",
        "pitch_rms_deg",
        "yaw_rms_deg",
        "min_base_z",
        "base_z_std",
        "result_json",
    ]
    row = {
        "terrain": terrain,
        "mode": mode,
        "success": result.get("success"),
        "task_completion_success": result.get("task_completion_success"),
        "fall_reason": result.get("fall_reason"),
        "duration_executed": result.get("duration_executed"),
        "body_frame_forward_progress": result.get("body_frame_forward_progress"),
        "command_active_body_frame_forward_progress": result.get("command_active_body_frame_forward_progress"),
        "body_frame_lateral_progress": result.get("body_frame_lateral_progress"),
        "roll_rms_deg": result.get("roll_rms_deg"),
        "pitch_rms_deg": result.get("pitch_rms_deg"),
        "yaw_rms_deg": result.get("yaw_rms_deg"),
        "min_base_z": result.get("min_base_z"),
        "base_z_std": result.get("base_z_std"),
        "result_json": str(result_json),
    }
    with open(summary_csv, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--result-json", required=True)
    parser.add_argument("--append-csv", default="")
    parser.add_argument("--terrain", default="")
    parser.add_argument("--mode", default="")
    args = parser.parse_args()

    rclpy.init()
    node = AutoInputMetricsNode(Path(args.scenario), Path(args.result_json))
    rclpy.spin(node)

    if args.append_csv:
        append_summary_row(Path(args.append_csv), Path(args.result_json), args.terrain, args.mode)


if __name__ == "__main__":
    main()
