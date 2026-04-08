#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import List, Tuple


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def unwrap_yaws(yaws: List[float]) -> List[float]:
    if not yaws:
        return yaws
    unwrapped = [yaws[0]]
    for yaw in yaws[1:]:
        prev = unwrapped[-1]
        delta = wrap_to_pi(yaw - prev)
        unwrapped.append(prev + delta)
    return unwrapped


def load_control_points(path: Path) -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 2:
                raise ValueError(f"Invalid control point line: {line.rstrip()}")
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) > 2 else float("nan")
            points.append((x, y, yaw))
    if len(points) < 2:
        raise ValueError("Need at least 2 control points.")
    return points


def compute_segment_yaw(points: List[Tuple[float, float, float]], idx: int) -> float:
    x0, y0, yaw0 = points[idx]
    if not math.isnan(yaw0):
        return yaw0
    if idx < len(points) - 1:
        x1, y1, _ = points[idx + 1]
    else:
        x1, y1, _ = points[idx - 1]
        x0, y0 = x1, y1
        x1, y1, _ = points[idx]
    return math.atan2(y1 - y0, x1 - x0)


def build_waypoints(
    points: List[Tuple[float, float, float]],
    speed: float,
    sample_dt: float,
    z: float,
    pitch: float,
    roll: float,
) -> List[Tuple[float, float, float, float, float, float, float]]:
    if speed <= 0.0:
        raise ValueError("speed must be positive")
    if sample_dt <= 0.0:
        raise ValueError("sample_dt must be positive")

    segment_yaws = [compute_segment_yaw(points, i) for i in range(len(points))]
    segment_yaws = unwrap_yaws(segment_yaws)

    out: List[Tuple[float, float, float, float, float, float, float]] = []
    current_time = 0.0

    for idx in range(len(points) - 1):
        x0, y0, _ = points[idx]
        x1, y1, _ = points[idx + 1]
        yaw0 = segment_yaws[idx]
        yaw1 = segment_yaws[idx + 1]

        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        duration = max(dist / speed, sample_dt)
        samples = max(1, int(math.ceil(duration / sample_dt)))

        for step in range(samples):
            s = step / samples
            x = x0 + s * dx
            y = y0 + s * dy
            yaw = yaw0 + s * (yaw1 - yaw0)
            if not out or math.hypot(x - out[-1][1], y - out[-1][2]) > 1e-9:
                out.append((current_time + s * duration, x, y, z, yaw, pitch, roll))

        current_time += duration

    x_last, y_last, _ = points[-1]
    yaw_last = segment_yaws[-1]
    out.append((current_time, x_last, y_last, z, yaw_last, pitch, roll))
    return out


def write_waypoints(path: Path, waypoints: List[Tuple[float, float, float, float, float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# time  x  y  z  yaw  pitch  roll\n")
        for t, x, y, z, yaw, pitch, roll in waypoints:
            f.write(f"{t:.3f} {x:.6f} {y:.6f} {z:.6f} {yaw:.6f} {pitch:.6f} {roll:.6f}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate an OCS2 map trajectory file from world-frame control points.")
    parser.add_argument("--control-points", required=True, help="Text file with columns: x y [yaw]")
    parser.add_argument("--output", required=True, help="Output waypoint trajectory file")
    parser.add_argument("--speed", type=float, default=0.12, help="Nominal path speed in m/s")
    parser.add_argument("--sample-dt", type=float, default=0.5, help="Sampling interval in s")
    parser.add_argument("--z", type=float, default=0.36, help="Reference base z")
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    args = parser.parse_args()

    control_points = load_control_points(Path(args.control_points))
    waypoints = build_waypoints(
        control_points,
        speed=args.speed,
        sample_dt=args.sample_dt,
        z=args.z,
        pitch=args.pitch,
        roll=args.roll,
    )
    write_waypoints(Path(args.output), waypoints)
    print(f"[done] wrote {len(waypoints)} waypoints to {args.output}")


if __name__ == "__main__":
    main()
