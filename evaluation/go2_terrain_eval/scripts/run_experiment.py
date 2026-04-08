#!/usr/bin/env python3

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path("/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval")


def main():
    parser = argparse.ArgumentParser(description="Run multiple automatic GO2 terrain trials.")
    parser.add_argument("--terrains", required=True, help="Comma-separated terrain names")
    parser.add_argument("--modes", default="baseline", help="Comma-separated modes: baseline,perceptive")
    parser.add_argument("--scenario", default=str(ROOT / "configs" / "scenarios" / "standing_trot_forward.yaml"))
    parser.add_argument("--repeats", type=int, default=1)
    args = parser.parse_args()

    terrains = [s.strip() for s in args.terrains.split(",") if s.strip()]
    modes = [s.strip() for s in args.modes.split(",") if s.strip()]

    for terrain in terrains:
        for mode in modes:
            for repeat in range(args.repeats):
                cmd = [
                    sys.executable,
                    str(ROOT / "scripts" / "run_trial.py"),
                    "--terrain",
                    terrain,
                    "--mode",
                    mode,
                    "--scenario",
                    args.scenario,
                    "--tag",
                    f"r{repeat + 1}",
                ]
                subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
