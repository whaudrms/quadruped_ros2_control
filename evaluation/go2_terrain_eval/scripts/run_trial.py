#!/usr/bin/env python3

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml


ROOT = Path("/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval")
RESULTS_DIR = ROOT / "results"
TERRAIN_CATALOG = ROOT / "configs" / "terrains.yaml"
UNCERTAINTY_CONTACT_METRICS_FILE = Path("/tmp/uncertainty_contact_metrics.log")
HEIGHT_ONLY_REFERENCE_DEBUG_FILE = Path("/tmp/height_only_reference_debug.log")
UNCERTAINTY_CONTACT_RE = re.compile(
    r"\[UncertaintyContactMetrics\]\s+t=(?P<t>[-+0-9.eE]+)\s+"
    r"total_contacts=(?P<total>\d+)\s+"
    r"total_leg_checks=(?P<legs>\d+)\s+"
    r"active_margin_contacts=(?P<active>\d+)\s+"
    r"high_uncertainty_contacts=(?P<high>\d+)\s+"
    r"timing_window_contacts=(?P<timing>\d+)\s+"
    r"uncertainty_mean=(?P<mean>[-+0-9.eE]+)\s+"
    r"uncertainty_max=(?P<max>[-+0-9.eE]+)"
)


def load_terrains():
    with open(TERRAIN_CATALOG, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["terrains"]


def require_scene(terrain_name: str, terrain: dict):
    scene_file = Path(terrain["scene_file"])
    if scene_file.exists():
        return scene_file

    cmd = [sys.executable, str(ROOT / "scripts" / "generate_terrains.py"), "--terrain", terrain_name]
    subprocess.run(cmd, check=True)
    if not scene_file.exists():
        raise FileNotFoundError(f"Failed to generate scene: {scene_file}")
    return scene_file


def launch_process(command: str, log_path: Path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    return process


def create_controller_yaml(base_yaml: Path, output_yaml: Path, *, task_file_override: str = "", reference_file_override: str = "", gait_file_override: str = "", dataset_log_csv_path: str = "", nominal_terrain_height_override: float = 0.0, contact_timing_bias: float = 0.0):
    with open(base_yaml, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    controller_key = "ocs2_quadruped_controller_height_only_v1"
    params = config[controller_key]["ros__parameters"]
    params["task_file_override"] = task_file_override
    params["reference_file_override"] = reference_file_override
    params["gait_file_override"] = gait_file_override
    params["dataset_log_csv_path"] = dataset_log_csv_path
    params["nominal_terrain_height_override"] = float(nominal_terrain_height_override)
    params["contact_timing_bias"] = float(contact_timing_bias)

    with open(output_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False)


def stop_process(process: subprocess.Popen):
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def merge_uncertainty_metrics(result_json: Path):
    if not result_json.exists():
        return

    total_contacts = 0
    total_leg_checks = 0
    active_margin_contacts = 0
    high_uncertainty_contacts = 0
    timing_window_contacts = 0
    weighted_uncertainty_sum = 0.0
    uncertainty_max = 0.0

    metrics_path = UNCERTAINTY_CONTACT_METRICS_FILE
    if not metrics_path.exists():
        return

    with open(metrics_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = UNCERTAINTY_CONTACT_RE.search(line)
            if not match:
                continue
            contacts = int(match.group("total"))
            total_contacts += contacts
            total_leg_checks += int(match.group("legs"))
            active_margin_contacts += int(match.group("active"))
            high_uncertainty_contacts += int(match.group("high"))
            timing_window_contacts += int(match.group("timing"))
            weighted_uncertainty_sum += contacts * float(match.group("mean"))
            uncertainty_max = max(uncertainty_max, float(match.group("max")))

    if total_contacts == 0 and total_leg_checks == 0:
        return

    with open(result_json, "r", encoding="utf-8") as f:
        result = json.load(f)

    result["contact_margin_shrink_activation_ratio"] = (
        active_margin_contacts / total_contacts if total_contacts > 0 else 0.0
    )
    result["high_uncertainty_region_contact_ratio"] = (
        high_uncertainty_contacts / total_contacts if total_contacts > 0 else 0.0
    )
    result["timing_uncertainty_window_activation_ratio"] = (
        timing_window_contacts / total_leg_checks if total_leg_checks > 0 else 0.0
    )
    result["uncertainty_contact_samples"] = total_contacts
    result["timing_uncertainty_leg_samples"] = total_leg_checks
    result["uncertainty_contact_mean"] = weighted_uncertainty_sum / total_contacts if total_contacts > 0 else 0.0
    result["uncertainty_contact_max"] = uncertainty_max

    with open(result_json, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description="Run one automatic GO2 terrain trial.")
    parser.add_argument("--terrain", required=True)
    parser.add_argument(
        "--mode",
        choices=["baseline", "perceptive", "perceptive_maptraj", "uncertainty_v1", "height_only_v1", "height_only_nominal_v1", "rgdemo"],
        default="baseline",
    )
    parser.add_argument("--scenario", default=str(ROOT / "configs" / "scenarios" / "standing_trot_forward.yaml"))
    parser.add_argument(
        "--trajectory-file",
        default="/home/ho/ros2_ws/src/quadruped_ros2_control/descriptions/unitree/go2_description_test1/config/ocs2/map_trajectory_flat_long.txt",
    )
    parser.add_argument("--tag", default="")
    args = parser.parse_args()

    scenario_path = Path(args.scenario)
    if not scenario_path.exists():
        candidate = ROOT / "configs" / "scenarios" / args.scenario
        if candidate.exists():
            scenario_path = candidate
        else:
            candidate_yaml = ROOT / "configs" / "scenarios" / f"{args.scenario}.yaml"
            if candidate_yaml.exists():
                scenario_path = candidate_yaml
            else:
                raise FileNotFoundError(f"Scenario file not found: {args.scenario}")

    terrains = load_terrains()
    if args.terrain not in terrains:
        raise KeyError(f"Unknown terrain: {args.terrain}")
    terrain = terrains[args.terrain]
    if args.mode in ("perceptive", "perceptive_maptraj", "uncertainty_v1", "height_only_v1", "rgdemo") and not terrain.get("perceptive_supported", False):
        raise ValueError(f"Terrain '{args.terrain}' does not currently define a perceptive input image.")

    scene_file = require_scene(args.terrain, terrain)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_name = f"{timestamp}_{args.terrain}_{args.mode}"
    if args.tag:
        run_name += f"_{args.tag}"
    run_dir = RESULTS_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    if args.mode == "uncertainty_v1" and UNCERTAINTY_CONTACT_METRICS_FILE.exists():
        UNCERTAINTY_CONTACT_METRICS_FILE.unlink()
    if args.mode == "height_only_v1" and HEIGHT_ONLY_REFERENCE_DEBUG_FILE.exists():
        HEIGHT_ONLY_REFERENCE_DEBUG_FILE.unlink()

    mujoco_cmd = f"cd /home/ho/unitree_mujoco/simulate/build && ./unitree_mujoco -r go2 -s {scene_file.name}"
    if args.mode == "baseline":
        controller_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/ho/ros2_ws/install/setup.bash && "
            "ros2 launch ocs2_quadruped_controller_test1 mujoco_test1.launch.py "
            "pkg_description:=go2_description_test1"
        )
    elif args.mode == "perceptive":
        controller_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/ho/ros2_ws/install/setup.bash && "
            "ros2 launch ocs2_quadruped_controller_test1 mujoco_perceptive_test1.launch.py "
            "pkg_description:=go2_description_test1 "
            "launch_fake_elevation_map:=true "
            "launch_plane_decomposition:=true "
            f"terrain_image:={terrain['perceptive_image']} "
            f"terrain_height_scale:={terrain['terrain_height_scale']}"
        )
    elif args.mode == "perceptive_maptraj":
        controller_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/ho/ros2_ws/install/setup.bash && "
            "ros2 launch ocs2_quadruped_controller_test1 mujoco_perceptive_test1.launch.py "
            "pkg_description:=go2_description_test1 "
            "controller_config:=robot_control_perceptive_test1_maptraj.yaml "
            "launch_fake_elevation_map:=true "
            "launch_plane_decomposition:=true "
            f"terrain_image:={terrain['perceptive_image']} "
            f"terrain_height_scale:={terrain['terrain_height_scale']}"
        )
    elif args.mode == "rgdemo":
        controller_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/ho/ros2_ws/install/setup.bash && "
            "ros2 launch ocs2_quadruped_controller_rgdemo mujoco_perceptive_rgdemo.launch.py "
            "pkg_description:=go2_description_rgdemo "
            "launch_fake_elevation_map:=true "
            "launch_plane_decomposition:=true "
            f"terrain_image:={terrain['perceptive_image']} "
            f"terrain_height_scale:={terrain['terrain_height_scale']}"
        )
    elif args.mode == "height_only_v1":
        fake_map_mode = terrain.get("fake_map_mode", "image")
        params = terrain.get("parameters", {})
        size = params.get("size", [0.36, 1.10, 0.08])
        position = params.get("position", [0.55, 0.0, 0.04])
        task_file_override = terrain.get("task_file_override", "")
        controller_yaml = run_dir / "robot_control_perceptive_height_only_v1.runtime.yaml"
        create_controller_yaml(
            Path("/home/ho/ros2_ws/src/quadruped_ros2_control/descriptions/unitree/go2_description_height_only_v1/config/robot_control_perceptive_height_only_v1.yaml"),
            controller_yaml,
            task_file_override=task_file_override,
        )
        controller_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/ho/ros2_ws/install/setup.bash && "
            "ros2 launch ocs2_quadruped_controller_height_only_v1 mujoco_perceptive_height_only_v1.launch.py "
            "pkg_description:=go2_description_height_only_v1 "
            f"controller_config_path:={controller_yaml} "
            "launch_fake_elevation_map:=true "
            "launch_plane_decomposition:=false "
            f"fake_map_mode:={fake_map_mode} "
            f"terrain_image:={terrain['perceptive_image']} "
            f"terrain_height_scale:={terrain['terrain_height_scale']} "
            f"terrain_resolution:={terrain.get('terrain_resolution', 0.03)} "
            f"terrain_map_length_x:={terrain.get('map_length_x', 2.5)} "
            f"terrain_map_length_y:={terrain.get('map_length_y', 1.6)} "
            f"terrain_box_center_x:={position[0]} "
            f"terrain_box_center_y:={position[1]} "
            f"terrain_box_size_x:={size[0]} "
            f"terrain_box_size_y:={size[1]} "
            f"terrain_box_height:={size[2]}"
        )
    elif args.mode == "height_only_nominal_v1":
        task_file_override = terrain.get("task_file_override", "")
        nominal_terrain_height_override = terrain.get("nominal_terrain_height_override", 0.0)
        contact_timing_bias = terrain.get("contact_timing_bias", 0.0)
        controller_yaml = run_dir / "robot_control_height_only_v1.runtime.yaml"
        create_controller_yaml(
            Path("/home/ho/ros2_ws/src/quadruped_ros2_control/descriptions/unitree/go2_description_height_only_v1/config/robot_control_height_only_v1.yaml"),
            controller_yaml,
            task_file_override=task_file_override,
            dataset_log_csv_path=str(run_dir / "controller_state_input.csv"),
            nominal_terrain_height_override=nominal_terrain_height_override,
            contact_timing_bias=contact_timing_bias,
        )
        controller_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/ho/ros2_ws/install/setup.bash && "
            "ros2 launch ocs2_quadruped_controller_height_only_v1 mujoco_perceptive_height_only_v1.launch.py "
            "pkg_description:=go2_description_height_only_v1 "
            f"controller_config_path:={controller_yaml} "
            "launch_fake_elevation_map:=false "
            "launch_plane_decomposition:=false "
        )
    else:
        controller_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/ho/ros2_ws/install/setup.bash && "
            "ros2 launch ocs2_quadruped_controller_uncertainty_v1 mujoco_perceptive_uncertainty_v1.launch.py "
            "pkg_description:=go2_description_uncertainty_v1 "
            "launch_fake_elevation_map:=true "
            "launch_plane_decomposition:=true "
            f"terrain_image:={terrain['perceptive_image']} "
            f"terrain_height_scale:={terrain['terrain_height_scale']}"
        )

    metrics_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/ho/ros2_ws/install/setup.bash && "
        f"python3 {ROOT / 'scripts' / 'auto_input_metrics.py'} "
        f"--scenario {scenario_path} "
        f"--result-json {run_dir / 'result.json'} "
        f"--append-csv {RESULTS_DIR / 'summary.csv'} "
        f"--terrain {args.terrain} --mode {args.mode}"
    )

    mujoco = None
    controller = None
    trajectory_cmd_proc = None
    try:
        mujoco = launch_process(mujoco_cmd, run_dir / "mujoco.log")
        time.sleep(2.5)
        controller = launch_process(controller_cmd, run_dir / "controller.log")
        time.sleep(8.0)
        if args.mode == "perceptive_maptraj":
            trajectory_cmd = (
                "source /opt/ros/jazzy/setup.bash && "
                "source /home/ho/ros2_ws/install/setup.bash && "
                f"python3 {ROOT / 'scripts' / 'map_trajectory_command.py'} "
                "--topic-prefix go2 "
                "--min-build-time 9.0 "
                "--min-base-z 0.28 "
                f"--trajectory-file {args.trajectory_file}"
            )
            trajectory_cmd_proc = launch_process(trajectory_cmd, run_dir / "trajectory_cmd.log")
            time.sleep(2.0)
        subprocess.run(["bash", "-lc", metrics_cmd], check=True)
    finally:
        if trajectory_cmd_proc is not None:
            stop_process(trajectory_cmd_proc)
        if controller is not None:
            stop_process(controller)
        if mujoco is not None:
            stop_process(mujoco)

    if args.mode == "uncertainty_v1":
        if UNCERTAINTY_CONTACT_METRICS_FILE.exists():
            (run_dir / "uncertainty_contact_metrics.log").write_text(
                UNCERTAINTY_CONTACT_METRICS_FILE.read_text(encoding="utf-8", errors="ignore"),
                encoding="utf-8",
            )
        merge_uncertainty_metrics(run_dir / "result.json")
    if args.mode == "height_only_v1":
        if HEIGHT_ONLY_REFERENCE_DEBUG_FILE.exists():
            (run_dir / "height_only_reference_debug.log").write_text(
                HEIGHT_ONLY_REFERENCE_DEBUG_FILE.read_text(encoding="utf-8", errors="ignore"),
                encoding="utf-8",
            )

    print(f"[done] {run_dir}")


if __name__ == "__main__":
    main()
