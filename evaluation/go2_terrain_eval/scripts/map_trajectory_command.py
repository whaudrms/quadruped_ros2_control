#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from ocs2_msgs.msg import MpcInput, MpcObservation, MpcState, MpcTargetTrajectories


def load_waypoints(path: Path) -> List[Tuple[float, float, float, float, float, float, float]]:
    waypoints = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 5:
                raise ValueError(f"Invalid waypoint line: {line.rstrip()}")
            time_s = float(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            yaw = float(parts[4])
            pitch = float(parts[5]) if len(parts) > 5 else 0.0
            roll = float(parts[6]) if len(parts) > 6 else 0.0
            waypoints.append((time_s, x, y, z, yaw, pitch, roll))
    if len(waypoints) < 2:
        raise ValueError("Need at least 2 trajectory waypoints.")
    return waypoints


class MapTrajectoryCommandNode(Node):
    def __init__(
        self,
        topic_prefix: str,
        trajectory_file: Path,
        start_delay: float,
        republish_period: float,
        min_build_time: float,
        min_base_z: float,
    ):
        super().__init__("map_trajectory_command")
        self.topic_prefix = topic_prefix
        self.trajectory_file = trajectory_file
        self.start_delay = start_delay
        self.republish_period = republish_period
        self.min_build_time = min_build_time
        self.min_base_z = min_base_z
        self.observation: Optional[MpcObservation] = None
        self.target_msg: Optional[MpcTargetTrajectories] = None
        self.waypoints = load_waypoints(trajectory_file)

        self.publisher = self.create_publisher(MpcTargetTrajectories, f"{topic_prefix}_mpc_target", 1)
        self.subscription = self.create_subscription(
            MpcObservation, f"{topic_prefix}_mpc_observation", self.observation_callback, 1
        )
        self.timer = self.create_timer(self.republish_period, self.timer_callback)

        self.get_logger().info(
            f"Waiting for observation on {topic_prefix}_mpc_observation, trajectory={trajectory_file}"
        )

    def observation_callback(self, msg: MpcObservation) -> None:
        self.observation = msg
        if self.target_msg is None:
            if msg.time < self.min_build_time:
                return
            if len(msg.state.value) < 12 or msg.state.value[8] < self.min_base_z:
                return
            self.target_msg = self.build_target_message(msg)
            self.get_logger().info(
                "Built target trajectories: "
                f"waypoints={len(self.target_msg.time_trajectory)} "
                f"start={self.target_msg.time_trajectory[0]:.3f} "
                f"end={self.target_msg.time_trajectory[-1]:.3f}"
            )

    def build_target_message(self, observation: MpcObservation) -> MpcTargetTrajectories:
        state_dim = len(observation.state.value)
        input_dim = len(observation.input.value)
        if state_dim < 18:
            raise RuntimeError(f"Unexpected observation state dimension: {state_dim}")

        current_state = list(observation.state.value)
        current_pose = current_state[6:12]
        x_offset = current_pose[0] - self.waypoints[0][1]
        y_offset = current_pose[1] - self.waypoints[0][2]
        yaw_offset = current_pose[3] - self.waypoints[0][4]
        z_ref = current_pose[2]
        pitch_ref = current_pose[4]
        roll_ref = current_pose[5]

        msg = MpcTargetTrajectories()
        start_time = observation.time + self.start_delay

        for i, wp in enumerate(self.waypoints):
            next_wp = self.waypoints[min(i + 1, len(self.waypoints) - 1)]
            dt = max(next_wp[0] - wp[0], 1e-3)
            vx = (next_wp[1] - wp[1]) / dt
            vy = (next_wp[2] - wp[2]) / dt

            state = list(current_state)
            state[0] = float(vx)
            state[1] = float(vy)
            state[2] = 0.0
            state[6] = float(wp[1] + x_offset)
            state[7] = float(wp[2] + y_offset)
            state[8] = float(z_ref)
            state[9] = float(wp[4] + yaw_offset)
            state[10] = float(pitch_ref)
            state[11] = float(roll_ref)

            mpc_state = MpcState()
            mpc_state.value = [float(v) for v in state]
            mpc_input = MpcInput()
            mpc_input.value = [0.0] * input_dim

            msg.time_trajectory.append(float(start_time + wp[0]))
            msg.state_trajectory.append(mpc_state)
            msg.input_trajectory.append(mpc_input)

        return msg

    def timer_callback(self) -> None:
        if self.target_msg is None:
            return
        self.publisher.publish(self.target_msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish absolute OCS2 target trajectories from a waypoint file.")
    parser.add_argument("--trajectory-file", required=True)
    parser.add_argument("--topic-prefix", default="go2")
    parser.add_argument("--start-delay", type=float, default=1.0)
    parser.add_argument("--republish-period", type=float, default=0.5)
    parser.add_argument("--min-build-time", type=float, default=9.0)
    parser.add_argument("--min-base-z", type=float, default=0.28)
    args = parser.parse_args()

    rclpy.init()
    node = MapTrajectoryCommandNode(
        topic_prefix=args.topic_prefix,
        trajectory_file=Path(args.trajectory_file),
        start_delay=args.start_delay,
        republish_period=args.republish_period,
        min_build_time=args.min_build_time,
        min_base_z=args.min_base_z,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
