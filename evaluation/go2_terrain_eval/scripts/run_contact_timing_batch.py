#!/usr/bin/env python3
import argparse
import subprocess
import sys


DEFAULT_TERRAINS = [
    "ocs2_stepdown",
    "ocs2_stepdown_ctp005",
    "ocs2_stepdown_ctp007",
    "ocs2_stepdown_ctp008",
    "ocs2_stepdown_ctp009",
    "ocs2_stepdown_ctp010",
    "ocs2_stepdown_ctp012",
    "ocs2_stepdown_ctp015",
]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="height_only_nominal_v1")
    parser.add_argument("--scenario", default="ocs2_stepdown_d_sweep")
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--terrains", nargs="*", default=DEFAULT_TERRAINS)
    parser.add_argument("--tag-prefix", default="contactflagbias_batch")
    return parser.parse_args()


def main():
    args = parse_args()
    for terrain in args.terrains:
        for repeat_idx in range(1, args.repeats + 1):
            tag = f"{args.tag_prefix}_{terrain}_r{repeat_idx:02d}"
            cmd = [
                sys.executable,
                "/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/scripts/run_trial.py",
                "--terrain",
                terrain,
                "--mode",
                args.mode,
                "--scenario",
                args.scenario,
                "--tag",
                tag,
            ]
            print(f"[run] {' '.join(cmd)}", flush=True)
            result = subprocess.run(cmd)
            if result.returncode != 0:
                print(f"[error] terrain={terrain} repeat={repeat_idx} returncode={result.returncode}", flush=True)
                return result.returncode
    print("[done] batch complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
