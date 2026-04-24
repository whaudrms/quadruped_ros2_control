#!/usr/bin/env python3

import argparse
import os
import re
import signal
import subprocess
import time
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent
RESULTS_DIR = ROOT / "results"
CONTROL_TOPICS = ("/control_input", "/cmd_vel")
READINESS_NODES = (
    "/controller_manager",
    "/ocs2_quadruped_controller",
    "/robot_state_publisher",
)


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


def stop_process(process: subprocess.Popen):
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def run_shell(command: str, *, timeout: float = 10.0):
    return subprocess.run(
        ["bash", "-lc", command],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def detect_first_existing(candidates):
    for candidate in candidates:
        path = Path(os.path.expanduser(str(candidate)))
        if path.exists():
            return path
    return None


def detect_install_setup(cli_value: str | None) -> Path:
    candidates = []
    if cli_value:
        candidates.append(cli_value)
    env_value = os.environ.get("PERCEPTIVE_INSTALL_SETUP")
    if env_value:
        candidates.append(env_value)
    candidates.extend(
        [
            "/tmp/perceptive_v2_install_patch/setup.bash",
            "/tmp/perceptive_v2_install_o0/setup.bash",
        ]
    )
    path = detect_first_existing(candidates)
    if path is None:
        raise FileNotFoundError("Install setup not found. Use --install-setup.")
    return path


def detect_scene_root(cli_value: str | None) -> Path:
    candidates = []
    if cli_value:
        candidates.append(cli_value)
    env_value = os.environ.get("UNITREE_SCENE_ROOT")
    if env_value:
        candidates.append(env_value)
    candidates.extend(
        [
            "~/unitree_mujoco_dev/unitree_robots/go2",
            "~/unitree_mujoco/unitree_robots/go2",
        ]
    )
    path = detect_first_existing(candidates)
    if path is None:
        raise FileNotFoundError("Scene root not found. Use --scene-root.")
    return path


def detect_mujoco_build_dir(cli_value: str | None) -> Path:
    candidates = []
    if cli_value:
        candidates.append(cli_value)
    env_value = os.environ.get("UNITREE_MUJOCO_BUILD_DIR")
    if env_value:
        candidates.append(env_value)
    candidates.extend(
        [
            "~/unitree_mujoco/simulate/build",
            "~/unitree_mujoco_dev/simulate/build",
        ]
    )
    path = detect_first_existing(candidates)
    if path is None:
        raise FileNotFoundError("MuJoCo build dir not found. Use --mujoco-build-dir.")
    return path


def make_ros_env_cmd(install_setup: Path) -> str:
    return f"source /opt/ros/jazzy/setup.bash && source {install_setup}"


def wait_for_nodes(ros_env_cmd: str, timeout_sec: float = 30.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        proc = run_shell(f"{ros_env_cmd} && ros2 node list", timeout=5.0)
        if proc.returncode == 0:
            nodes = set(line.strip() for line in proc.stdout.splitlines() if line.strip())
            if all(node in nodes for node in READINESS_NODES):
                return
        time.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for nodes: {READINESS_NODES}")


def subscriber_count(ros_env_cmd: str, topic_name: str) -> int:
    proc = run_shell(f"{ros_env_cmd} && ros2 topic info {topic_name}", timeout=5.0)
    if proc.returncode != 0:
        return 0
    match = re.search(r"Subscription count:\s*(\d+)", proc.stdout)
    return int(match.group(1)) if match else 0


def wait_for_topic_subscribers(ros_env_cmd: str, timeout_sec: float = 20.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        counts = {topic: subscriber_count(ros_env_cmd, topic) for topic in CONTROL_TOPICS}
        if all(counts[topic] > 0 for topic in CONTROL_TOPICS):
            return
        time.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for topic subscribers on {CONTROL_TOPICS}")


def wait_for_odom(ros_env_cmd: str, timeout_sec: float = 20.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        proc = run_shell(f"{ros_env_cmd} && ros2 topic echo /odom --once", timeout=6.0)
        if proc.returncode == 0 and "pose:" in proc.stdout:
            return
        time.sleep(0.5)
    raise TimeoutError("Timed out waiting for /odom")


def check_for_stale_nodes(ros_env_cmd: str):
    proc = run_shell(f"{ros_env_cmd} && ros2 node list", timeout=5.0)
    if proc.returncode != 0:
        return
    nodes = set(line.strip() for line in proc.stdout.splitlines() if line.strip())
    stale = [node for node in READINESS_NODES if node in nodes]
    if stale:
        raise RuntimeError(
            "Detected existing controller nodes before launch. "
            f"Stop them first and retry: {', '.join(stale)}"
        )


def resolve_scenario_path(scenario_arg: str) -> Path:
    scenario_path = Path(scenario_arg)
    if scenario_path.exists():
        return scenario_path
    candidate = ROOT / "scenarios" / scenario_arg
    if candidate.exists():
        return candidate
    candidate_yaml = ROOT / "scenarios" / f"{scenario_arg}.yaml"
    if candidate_yaml.exists():
        return candidate_yaml
    raise FileNotFoundError(f"Scenario file not found: {scenario_arg}")


def load_scene_path(scene_root: Path, scene_arg: str) -> str:
    scene = Path(scene_arg)
    if scene.is_absolute():
        return str(scene)
    return str(scene_root / scene_arg)


def main():
    parser = argparse.ArgumentParser(description="Run one automatic perceptive OCS2 + QP-WBC dev_v2 trial.")
    parser.add_argument("--terrain", default="basic_step.xml")
    parser.add_argument("--scenario", default="standing_trot_forward")
    parser.add_argument("--mode", choices=["pure_dev_v2", "perceptive_dev_v2"], default="perceptive_dev_v2")
    parser.add_argument("--tag", default="")
    parser.add_argument("--install-setup", default=None)
    parser.add_argument("--scene-root", default=None)
    parser.add_argument("--mujoco-build-dir", default=None)
    args = parser.parse_args()

    install_setup = detect_install_setup(args.install_setup)
    scene_root = detect_scene_root(args.scene_root)
    mujoco_build_dir = detect_mujoco_build_dir(args.mujoco_build_dir)
    ros_env_cmd = make_ros_env_cmd(install_setup)

    scenario_path = resolve_scenario_path(args.scenario)
    with open(scenario_path, "r", encoding="utf-8") as f:
        scenario_cfg = yaml.safe_load(f)

    scene_path = load_scene_path(scene_root, args.terrain)
    if not Path(scene_path).exists():
        raise FileNotFoundError(f"Scene file not found: {scene_path}")

    check_for_stale_nodes(ros_env_cmd)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_name = f"{timestamp}_{Path(scene_path).stem}_{args.mode}"
    if args.tag:
        run_name += f"_{args.tag}"
    run_dir = RESULTS_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    enable_perceptive = "true" if args.mode == "perceptive_dev_v2" else "false"

    mujoco_cmd = f"cd {mujoco_build_dir} && ./unitree_mujoco -r go2 -s {scene_path}"
    controller_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {install_setup} && "
        "ros2 launch ocs2_quadruped_controller mujoco.launch.py "
        "pkg_description:=go2_description "
        f"enable_perceptive:={enable_perceptive} "
        "publish_static_terrain:=true "
        f"terrain_scene_file:={Path(scene_path).name} "
    )
    metrics_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {install_setup} && "
        f"python3 {ROOT / 'auto_input_metrics.py'} "
        f"--scenario {scenario_path} "
        f"--result-json {run_dir / 'result.json'} "
        f"--append-csv {RESULTS_DIR / 'summary.csv'} "
        f"--terrain {Path(scene_path).stem} --mode {args.mode}"
    )

    mujoco = None
    controller = None
    try:
        mujoco = launch_process(mujoco_cmd, run_dir / "mujoco.log")
        time.sleep(float(scenario_cfg.get("mujoco_startup_wait_sec", 2.5)))
        controller = launch_process(controller_cmd, run_dir / "controller.log")
        wait_for_nodes(ros_env_cmd, timeout_sec=float(scenario_cfg.get("controller_startup_wait_sec", 20.0)))
        wait_for_topic_subscribers(ros_env_cmd, timeout_sec=15.0)
        wait_for_odom(ros_env_cmd, timeout_sec=15.0)
        subprocess.run(["bash", "-lc", metrics_cmd], check=True)
    finally:
        if controller is not None:
            stop_process(controller)
        if mujoco is not None:
            stop_process(mujoco)

    print(f"[done] {run_dir}")


if __name__ == "__main__":
    main()
