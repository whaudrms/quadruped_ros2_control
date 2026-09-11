#!/usr/bin/env python3
"""
Automatic perceptive OCS2 + WBC trial runner.

Ported from colleague's quadruped_temp/tools/perceptive_dev_v2/run_trial.py
(Jazzy) to our Humble branch at quadruped_ros2_control (humble-dev-perceptive).

Usage:
    python3 run_trial.py --terrain basic_step_short --robust on --tag test

Differences vs colleague:
- ROS distro: humble (was jazzy)
- Environment: sources ~/GO2_ws/setup_quadruped.sh (FastDDS) rather than
  /opt/ros/<distro>/setup.bash + custom install tree
- MuJoCo command: for basic_step scenes, adds `-k 0 -z 0.40` so the Go2
  spawns above box1 (our unitree_mujoco binary supports these flags).
  See ~/GO2_ws/unitree_mujoco/simulate/src/param.h.
"""

import argparse
import csv
import json
import math
import os
import re
import shlex
import signal
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
DEFAULT_RESULTS_DIR = ROOT / "results"
SOURCE_TASK_INFO_PATH = (
    PROJECT_ROOT / "descriptions/unitree/go2_description/config/ocs2/task.info"
)
INSTALLED_TASK_INFO_PATH = (
    WORKSPACE_ROOT / "install/go2_description/share/go2_description/config/ocs2/task.info"
)
# The controller loads the package-share copy. With --symlink-install this points
# at the source file; with a copied install tree we temporarily edit the active
# installed copy so CLI overrides still affect this trial.
TASK_INFO_PATH = (
    INSTALLED_TASK_INFO_PATH
    if INSTALLED_TASK_INFO_PATH.exists()
    else SOURCE_TASK_INFO_PATH
)
UNITREE_LIB_DIR = WORKSPACE_ROOT / "unitree_sdk2/install/lib"
COMMAND_SHELL = shutil.which("bash") or "/bin/bash"
CONTROL_TOPICS = ("/control_input", "/cmd_vel")
READINESS_NODES = (
    "/controller_manager",
    "/ocs2_quadruped_controller",
    "/robot_state_publisher",
)

# Default workspace env script (FastDDS + ROS2 humble + our install tree).
DEFAULT_WS_SETUP = WORKSPACE_ROOT / "setup_quadruped.sh"

# Process-name patterns that should never survive past the end of a trial.
# Used for both pre-run defensive cleanup and post-run nuke. The list catches
# processes that ros2 launch double-forks outside its own pgid.
LINGERING_PROCESS_PATTERNS = (
    "unitree_mujoco",
    "auto_input_metrics.py",
    "ros2_control_node",
    "ocs2_quadruped_controller",
    "ros2 launch ocs2",
    "planar_terrain",
    "rviz2",
    "robot_state_publisher",
    "controller_manager",
    "spawner",
)


def launch_process(command: str, log_path: Path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            [COMMAND_SHELL, "-lc", command],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    return process


def stream_matching_log_lines(
    log_path: Path,
    stop_event: threading.Event,
    patterns: tuple[str, ...],
):
    """Mirror selected lines from a process log to the runner's terminal."""
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
            while not stop_event.is_set():
                line = log_file.readline()
                if line:
                    if any(pattern in line for pattern in patterns):
                        print(line.rstrip(), flush=True)
                else:
                    stop_event.wait(0.1)

            # Drain lines already written before shutdown so the final timing
            # sample is not lost when the controller exits with the trial.
            for line in log_file:
                if any(pattern in line for pattern in patterns):
                    print(line.rstrip(), flush=True)
    except OSError as exc:
        print(f"[run_trial] could not stream {log_path}: {exc}", file=sys.stderr)


def _safe_killpg(pid: int, sig: int) -> bool:
    """Send a signal to a process group, swallowing zombie / not-found errors.

    Returns True if the signal was sent, False if the target was already gone.
    """
    try:
        pgid = os.getpgid(pid)
    except (ProcessLookupError, PermissionError):
        return False
    try:
        os.killpg(pgid, sig)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def stop_process(process: subprocess.Popen, label: str = "", force_cleanup: bool = False):
    """Stop a launched process group with bulletproof cleanup.

    1. SIGINT the pgid (politely ask for shutdown).
    2. wait up to 8s.
    3. If still alive, SIGKILL the pgid.
    4. If STILL alive after another 2s, optionally fall back to a global
       pattern-based cleanup when --force-cleanup was explicitly requested.

    All killpg calls are wrapped to swallow ProcessLookupError that can occur
    when the leader is already a zombie.
    """
    if process.poll() is not None:
        return
    pid = process.pid
    _safe_killpg(pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
        return
    except subprocess.TimeoutExpired:
        pass
    _safe_killpg(pid, signal.SIGKILL)
    try:
        process.wait(timeout=2)
        return
    except subprocess.TimeoutExpired:
        pass
    if force_cleanup:
        if label:
            print(f"[stop_process] {label}: SIGKILL did not reap; running forced cleanup")
        kill_lingering_processes(quiet=True)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        if label:
            print(f"[stop_process] {label}: still alive after process-group cleanup")


def kill_lingering_processes(quiet: bool = False):
    """SIGKILL matching processes after explicit --force-cleanup opt-in."""
    for pattern in LINGERING_PROCESS_PATTERNS:
        try:
            result = subprocess.run(
                ["pkill", "-9", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if not quiet and result.returncode == 0:
                print(f"[kill_lingering] killed leftover '{pattern}'")
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            if not quiet:
                print(f"[kill_lingering] pkill {pattern!r} failed: {exc}")
    # pkill only sends the signal; give the kernel a moment to close the DDS
    # sockets before another domain-1 participant starts probing RTPS ports.
    time.sleep(1.0)


def raise_if_process_exited(process: subprocess.Popen | None, label: str, log_path: Path):
    """Fail readiness on a dead launcher or a fatal child-process log entry."""
    lines = []
    selected = []
    try:
        with log_path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - 65536), os.SEEK_SET)
            lines = stream.read().decode("utf-8", errors="replace").splitlines()
        selected = [
            line for line in lines
            if "Failed to find a free participant index" in line
            or "DdsException" in line
            or ("ros2_control_node" in line and "process has died" in line)
        ]
    except OSError:
        pass

    if selected:
        raise RuntimeError(
            f"{label} reported a fatal startup error; inspect {log_path}\n"
            + "\n".join(selected[-4:])
        )
    if process is None or process.poll() is None:
        return

    detail = ""
    detail_lines = lines[-8:]
    if detail_lines:
        detail = "\n" + "\n".join(detail_lines)
    raise RuntimeError(
        f"{label} exited during startup with code {process.returncode}; "
        f"inspect {log_path}{detail}"
    )


def run_shell(command: str, *, timeout: float = 10.0):
    return subprocess.run(
        [COMMAND_SHELL, "-lc", command],
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


def detect_ws_setup(cli_value: str | None) -> Path:
    candidates = []
    if cli_value:
        candidates.append(cli_value)
    env_value = os.environ.get("PERCEPTIVE_WS_SETUP")
    if env_value:
        candidates.append(env_value)
    candidates.append(str(DEFAULT_WS_SETUP))
    path = detect_first_existing(candidates)
    if path is None:
        raise FileNotFoundError(
            f"Workspace setup script not found. Expected {DEFAULT_WS_SETUP}. "
            "Use --ws-setup or set PERCEPTIVE_WS_SETUP."
        )
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
            WORKSPACE_ROOT / "unitree_mujoco/unitree_robots/go2",
            "~/GO2_ws/unitree_mujoco/unitree_robots/go2",
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
            WORKSPACE_ROOT / "unitree_mujoco/simulate/build",
            "~/GO2_ws/unitree_mujoco/simulate/build",
            "~/unitree_mujoco/simulate/build",
        ]
    )
    path = detect_first_existing(candidates)
    if path is None:
        raise FileNotFoundError("MuJoCo build dir not found. Use --mujoco-build-dir.")
    return path


def make_ros_env_cmd(ws_setup: Path) -> str:
    # setup_quadruped.sh handles: ROS2 humble, FastDDS, domain 1, our install tree.
    return f"source {shlex.quote(str(ws_setup))}"


def wait_for_nodes(
    ros_env_cmd: str,
    timeout_sec: float = 30.0,
    watched_process: subprocess.Popen | None = None,
    watched_log: Path | None = None,
):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if watched_log is not None:
            raise_if_process_exited(watched_process, "controller launch", watched_log)
        proc = run_shell(f"{ros_env_cmd} && ros2 node list", timeout=5.0)
        if proc.returncode == 0:
            nodes = set(line.strip() for line in proc.stdout.splitlines() if line.strip())
            if all(node in nodes for node in READINESS_NODES):
                return
        time.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for nodes: {READINESS_NODES}")


def wait_for_controller_active(
    ros_env_cmd: str,
    timeout_sec: float = 180.0,
    watched_process: subprocess.Popen | None = None,
    watched_log: Path | None = None,
):
    """Wait through first-run CppAD compilation until the controller is active."""
    start = time.monotonic()
    deadline = start + timeout_sec
    next_notice = start
    while time.monotonic() < deadline:
        if watched_log is not None:
            raise_if_process_exited(watched_process, "controller launch", watched_log)
        try:
            proc = run_shell(
                f"{ros_env_cmd} && ros2 control list_controllers -c /controller_manager",
                timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            proc = None
        if proc is not None and proc.returncode == 0:
            for line in proc.stdout.splitlines():
                if "ocs2_quadruped_controller" in line and "active" in line:
                    return
        now = time.monotonic()
        if now >= next_notice:
            elapsed = now - start
            print(
                f"[run_trial] waiting for OCS2 controller to become active "
                f"({elapsed:.0f}/{timeout_sec:.0f}s; first run may compile CppAD models)"
            )
            next_notice = now + 10.0
        time.sleep(1.0)
    raise TimeoutError(
        f"Timed out after {timeout_sec:.0f}s waiting for ocs2_quadruped_controller "
        "to become active; inspect controller.log"
    )


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
    if not stale:
        return

    # Stage 8: ros2 daemon may still be reporting nodes whose host processes
    # were already killed by pre-run kill_lingering_processes. Restart the
    # daemon to flush its discovery cache, then retry once.
    print(f"[check_for_stale_nodes] stale nodes seen ({stale}); restarting ros2 daemon")
    try:
        run_shell(f"{ros_env_cmd} && ros2 daemon stop", timeout=8.0)
    except subprocess.TimeoutExpired:
        pass
    time.sleep(2.0)
    try:
        run_shell(f"{ros_env_cmd} && ros2 daemon start", timeout=8.0)
    except subprocess.TimeoutExpired:
        pass
    time.sleep(3.0)

    proc = run_shell(f"{ros_env_cmd} && ros2 node list", timeout=5.0)
    if proc.returncode != 0:
        return
    nodes = set(line.strip() for line in proc.stdout.splitlines() if line.strip())
    stale = [node for node in READINESS_NODES if node in nodes]
    if stale:
        raise RuntimeError(
            "Detected existing controller nodes before launch even after daemon restart. "
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
    if not scene.suffix:
        scene = scene.with_suffix(".xml")
    if scene.is_absolute():
        return str(scene)
    return str(scene_root / scene)


def mujoco_extra_args_for_scene(scene_name: str, override: str | None) -> str:
    """For basic_step scenes, spawn above the high box; else empty."""
    if override is not None:
        return override
    if "basic_step" in scene_name:
        return "-k 0 -z 0.40"
    return ""


def replace_task_value(text: str, key: str, value: str, block: str | None = None) -> str:
    """Replace exactly one task.info scalar, optionally restricted to a block."""
    search_text = text
    offset = 0
    if block is not None:
        block_match = re.search(
            rf"(^\s*{re.escape(block)}\s*$\s*^\s*\{{\s*$)(.*?)(^\s*\}}\s*$)",
            text,
            flags=re.MULTILINE | re.DOTALL,
        )
        if block_match is None:
            raise RuntimeError(f"Could not find '{block}' block in {TASK_INFO_PATH}")
        search_text = block_match.group(2)
        offset = block_match.start(2)

    pattern = re.compile(
        rf"^(\s*{re.escape(key)}\s+)(\S+)(\s*.*)$", re.MULTILINE
    )
    matches = list(pattern.finditer(search_text))
    if len(matches) != 1:
        scope = f" in block '{block}'" if block else ""
        raise RuntimeError(
            f"Expected exactly one '{key}' value{scope} in {TASK_INFO_PATH}; "
            f"found {len(matches)}"
        )
    match = matches[0]
    start = offset + match.start(2)
    end = offset + match.end(2)
    return text[:start] + value + text[end:]


def read_task_value(text: str, key: str, block: str | None = None, default: str | None = None) -> str:
    search_text = text
    if block is not None:
        block_match = re.search(
            rf"(^\s*{re.escape(block)}\s*$\s*^\s*\{{\s*$)(.*?)(^\s*\}}\s*$)",
            text,
            flags=re.MULTILINE | re.DOTALL,
        )
        if block_match is None:
            raise RuntimeError(f"Could not find '{block}' block in {TASK_INFO_PATH}")
        search_text = block_match.group(2)
    matches = list(
        re.finditer(rf"^\s*{re.escape(key)}\s+(\S+)", search_text, re.MULTILINE)
    )
    if not matches and default is not None:
        return default
    if len(matches) != 1:
        raise RuntimeError(f"Could not uniquely read '{key}' from {TASK_INFO_PATH}")
    return matches[0].group(1)


def effective_task_parameters(text: str) -> dict:
    robust_keys = (
        "enabled", "optimize_d", "d_min", "d_max", "t_a", "t_b", "d", "w_boundary", "w_v", "approach_barrier_mu",
        "approach_barrier_delta", "terrain_source", "terrain_z_M1",
        "foot_frame_offset", "hard_boundary_start", "hard_boundary_end",
        "slack_boundary_start", "slack_boundary_end",
        "slack_boundary_weight_start", "slack_boundary_weight_end",
        "enable_splice", "verbose_log",
    )
    robust = {
        key: read_task_value(text, key, "robustPhase") for key in robust_keys
    }
    for key in (
        "enabled", "optimize_d", "hard_boundary_start", "hard_boundary_end",
        "slack_boundary_start", "slack_boundary_end",
        "enable_splice", "verbose_log",
    ):
        if robust[key].lower() not in {"true", "false", "0", "1"}:
            raise ValueError(
                f"Invalid boolean robustPhase.{key}={robust[key]!r} in "
                f"{TASK_INFO_PATH}; expected true or false"
            )
    robust["w_d"] = read_task_value(text, "w_d", "robustPhase", default="0.0")
    weight = float(robust["w_d"])
    if not math.isfinite(weight) or weight < 0.0:
        raise ValueError("robustPhase.w_d must be finite and nonnegative")
    bool_value = lambda key: robust[key].lower() in {"true", "1"}
    if bool_value("hard_boundary_start") and bool_value("slack_boundary_start"):
        raise ValueError(
            "robustPhase start boundary cannot be both hard and slack"
        )
    if bool_value("hard_boundary_end") and bool_value("slack_boundary_end"):
        raise ValueError(
            "robustPhase end boundary cannot be both hard and slack"
        )
    for key in ("slack_boundary_weight_start", "slack_boundary_weight_end"):
        if float(robust[key]) <= 0.0:
            raise ValueError(f"robustPhase.{key} must be positive")
    dt = float(read_task_value(text, "dt", "sqp"))
    advance, delay, d = (float(robust[key]) for key in ("t_a", "t_b", "d"))
    if not all(math.isfinite(value) for value in (advance, delay, d)):
        raise ValueError("robustPhase.t_a, t_b and d must be finite")
    robust_window = advance + delay
    if advance < 0.0 or delay < 0.0 or not math.isfinite(robust_window) or robust_window <= 0.0:
        raise ValueError("robustPhase.t_a/t_b must be nonnegative with a positive finite sum")
    if d <= 0.0:
        raise ValueError("robustPhase.d must be positive")
    d_min, d_max = (float(robust[key]) for key in ("d_min", "d_max"))
    if not all(math.isfinite(value) for value in (d_min, d_max)) or not 0.0 < d_min <= d <= d_max:
        raise ValueError("robustPhase requires 0 < d_min <= d <= d_max (finite)")
    # Config-level values, before any runtime liftoff clamp. The controller
    # derives the actual per-window rate from its absolute endpoints.
    robust["T_robust_s"] = robust_window
    robust["v_max"] = 2.0 * d_max / robust_window
    if not math.isfinite(robust["v_max"]):
        raise ValueError("derived robustPhase v_max must be finite")
    robust["v_max_source"] = "2*d_max/(t_a+t_b), before runtime liftoff clamp"
    return {
        "mpcDesiredFrequency": read_task_value(text, "mpcDesiredFrequency"),
        "sqp.dt": dt,
        "sqp.sqpIteration": int(read_task_value(text, "sqpIteration", "sqp")),
        "swing_trajectory_config.swingHeight": float(
            read_task_value(text, "swingHeight", "swing_trajectory_config")
        ),
        "robustPhase": robust,
    }


def task_info_overrides(args) -> dict[str, str]:
    overrides: dict[str, str] = {}
    swing_height = getattr(args, "swing_height", None)
    if swing_height is not None:
        if not math.isfinite(swing_height) or swing_height <= 0.0:
            raise ValueError("--swing-height must be finite and positive")
        overrides["swing_trajectory_config.swingHeight"] = f"{swing_height:g}"
    if args.mpc_frequency is not None:
        if args.mpc_frequency <= 0.0:
            raise ValueError("--mpc-frequency must be positive")
        overrides["mpcDesiredFrequency"] = f"{args.mpc_frequency:g}"
    if args.sqp_iterations is not None:
        if args.sqp_iterations <= 0:
            raise ValueError("--sqp-iterations must be positive")
        overrides["sqp.sqpIteration"] = str(args.sqp_iterations)
    if args.robust != "keep":
        overrides["robustPhase.enabled"] = "true" if args.robust == "on" else "false"
    for option, key in (("robust_t_a", "t_a"), ("robust_t_b", "t_b")):
        value = getattr(args, option, None)
        if value is not None:
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"--{option.replace('_', '-')} must be finite and nonnegative")
            overrides[f"robustPhase.{key}"] = repr(value)
    if args.robust_d is not None:
        if not math.isfinite(args.robust_d) or args.robust_d <= 0.0:
            raise ValueError("--robust-d must be finite and positive")
        overrides["robustPhase.d"] = repr(args.robust_d)
    if args.robust_hard_boundary_start != "keep":
        overrides["robustPhase.hard_boundary_start"] = (
            "true" if args.robust_hard_boundary_start == "on" else "false"
        )
    if args.robust_hard_boundary_end != "keep":
        overrides["robustPhase.hard_boundary_end"] = (
            "true" if args.robust_hard_boundary_end == "on" else "false"
        )
    if args.robust_slack_boundary_start != "keep":
        overrides["robustPhase.slack_boundary_start"] = (
            "true" if args.robust_slack_boundary_start == "on" else "false"
        )
    if args.robust_slack_boundary_end != "keep":
        overrides["robustPhase.slack_boundary_end"] = (
            "true" if args.robust_slack_boundary_end == "on" else "false"
        )
    if args.robust_slack_weight_start is not None:
        if args.robust_slack_weight_start <= 0.0:
            raise ValueError("--robust-slack-weight-start must be positive")
        overrides["robustPhase.slack_boundary_weight_start"] = (
            f"{args.robust_slack_weight_start:g}"
        )
    if args.robust_slack_weight_end is not None:
        if args.robust_slack_weight_end <= 0.0:
            raise ValueError("--robust-slack-weight-end must be positive")
        overrides["robustPhase.slack_boundary_weight_end"] = (
            f"{args.robust_slack_weight_end:g}"
        )
    if args.robust_splice != "keep":
        overrides["robustPhase.enable_splice"] = (
            "true" if args.robust_splice == "on" else "false"
        )
    if args.robust_verbose != "keep":
        overrides["robustPhase.verbose_log"] = (
            "true" if args.robust_verbose == "on" else "false"
        )
    return overrides


def render_task_info(original_text: str, overrides: dict[str, str]) -> str:
    rendered = original_text
    for name, value in overrides.items():
        if name.startswith("swing_trajectory_config."):
            rendered = replace_task_value(
                rendered, name.removeprefix("swing_trajectory_config."),
                value, "swing_trajectory_config"
            )
        elif name.startswith("robustPhase."):
            rendered = replace_task_value(
                rendered, name.removeprefix("robustPhase."), value, "robustPhase"
            )
        elif name.startswith("sqp."):
            rendered = replace_task_value(
                rendered, name.removeprefix("sqp."), value, "sqp"
            )
        else:
            rendered = replace_task_value(rendered, name, value)
    return rendered


def append_trial_summary(summary_path: Path, result: dict, result_path: Path):
    experiment = result["experiment"]
    robust = experiment["effective_task_parameters"]["robustPhase"]
    fieldnames = [
        "trial", "tag", "scenario", "terrain", "mode", "robust_enabled",
        "robust_t_a", "robust_t_b", "robust_T_robust_s",
        "robust_d", "robust_optimize_d", "robust_d_min", "robust_d_max", "robust_w_d", "robust_v_max", "robust_splice",
        "mpc_frequency", "sqp_iterations", "sqp_dt", "terrain_z_offset", "success",
        "fall_reason", "duration_executed", "body_frame_forward_progress",
        "command_active_body_frame_forward_progress", "body_frame_lateral_progress",
        "roll_rms_deg", "pitch_rms_deg", "yaw_rms_deg", "min_base_z",
        "base_z_std", "result_json",
    ]
    row = {
        "trial": experiment["trial"],
        "tag": experiment["tag"],
        "scenario": result.get("scenario"),
        "terrain": experiment["terrain"],
        "mode": experiment["mode"],
        "robust_enabled": robust["enabled"],
        "robust_t_a": robust["t_a"],
        "robust_t_b": robust["t_b"],
        "robust_T_robust_s": robust["T_robust_s"],
        "robust_d": robust["d"],
        "robust_optimize_d": robust["optimize_d"],
        "robust_d_min": robust["d_min"],
        "robust_d_max": robust["d_max"],
        "robust_w_d": robust.get("w_d", "0.0"),
        "robust_v_max": robust["v_max"],
        "robust_splice": robust["enable_splice"],
        "mpc_frequency": experiment["effective_task_parameters"]["mpcDesiredFrequency"],
        "sqp_iterations": experiment["effective_task_parameters"]["sqp.sqpIteration"],
        "sqp_dt": experiment["effective_task_parameters"]["sqp.dt"],
        "terrain_z_offset": experiment["terrain_z_offset"],
        "success": result.get("success"),
        "fall_reason": result.get("fall_reason"),
        "duration_executed": result.get("duration_executed"),
        "body_frame_forward_progress": result.get("body_frame_forward_progress"),
        "command_active_body_frame_forward_progress": result.get(
            "command_active_body_frame_forward_progress"
        ),
        "body_frame_lateral_progress": result.get("body_frame_lateral_progress"),
        "roll_rms_deg": result.get("roll_rms_deg"),
        "pitch_rms_deg": result.get("pitch_rms_deg"),
        "yaw_rms_deg": result.get("yaw_rms_deg"),
        "min_base_z": result.get("min_base_z"),
        "base_z_std": result.get("base_z_std"),
        "result_json": str(result_path),
    }
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = summary_path.exists()
    if file_exists:
        with summary_path.open(newline="", encoding="utf-8") as stream:
            existing_fieldnames = next(csv.reader(stream), [])
        # Preserve the schema of an existing result set. New result folders
        # receive the extended schema, while old summaries remain appendable.
        if existing_fieldnames:
            fieldnames = existing_fieldnames
            row = {key: row.get(key, "") for key in fieldnames}
    with summary_path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def generate_trial_plots(run_dir: Path):
    for script_name in (
        "trial_metrics.py",
        "plot_robust_phase.py",
        "plot_trial_rmse.py",
        "plot_mpc_timing.py",
    ):
        script_path = ROOT / script_name
        try:
            proc = subprocess.run(
                [sys.executable, str(script_path), str(run_dir)],
                capture_output=True,
                text=True,
                timeout=120.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"[run_trial] plot generation failed for {script_name}: {exc}")
            continue
        if proc.returncode == 0:
            print(proc.stdout.strip())
        else:
            detail = proc.stderr.strip() or proc.stdout.strip()
            print(f"[run_trial] plot generation failed for {script_name}: {detail}")


def find_other_trial_runners() -> list[int]:
    pids = []
    proc_root = Path("/proc")
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            parts = [part.decode() for part in (entry / "cmdline").read_bytes().split(b"\0") if part]
        except (FileNotFoundError, PermissionError, ProcessLookupError, UnicodeDecodeError):
            continue
        if not parts or not Path(parts[0]).name.startswith("python"):
            continue
        for arg in parts[1:]:
            candidate = Path(arg)
            if candidate.name != "run_trial.py":
                continue
            try:
                if not candidate.is_absolute():
                    candidate = (entry / "cwd").resolve() / candidate
                if candidate.resolve() == Path(__file__).resolve():
                    pids.append(int(entry.name))
                    break
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
    return pids


def recover_interrupted_task_info(results_dir: Path):
    """Restore an override left by an interrupted prior trial when provable."""
    if not results_dir.is_dir() or not TASK_INFO_PATH.exists():
        return
    current = TASK_INFO_PATH.read_text(encoding="utf-8")
    configs = sorted(
        results_dir.glob("*/run_config.json"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    recovered_from = None
    interrupted_count = 0
    for config_path in configs:
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if config.get("status") != "starting":
            continue
        interrupted_count += 1
        run_dir = config_path.parent
        original_path = run_dir / "task.info.original"
        effective_path = run_dir / "task.info.effective"
        if original_path.exists() and effective_path.exists():
            original = original_path.read_text(encoding="utf-8")
            effective = effective_path.read_text(encoding="utf-8")
            if recovered_from is None and original != effective and current == effective:
                TASK_INFO_PATH.write_text(original, encoding="utf-8")
                current = original
                recovered_from = run_dir.name
        config["status"] = "interrupted"
        config["error"] = "Trial process ended before normal cleanup"
        config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    if recovered_from is not None:
        print(f"[run_trial] restored stale task.info override from {recovered_from}")
    if interrupted_count:
        print(f"[run_trial] marked {interrupted_count} stale trial(s) as interrupted")


def handle_termination_signal(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description="Run one automatic perceptive OCS2 + QP-WBC dev_v2 trial.")
    parser.add_argument("--terrain", default="basic_step.xml")
    parser.add_argument("--scenario", default="standing_trot_forward")
    parser.add_argument("--mode", choices=["pure_dev_v2", "perceptive_dev_v2"], default="perceptive_dev_v2")
    parser.add_argument("--tag", default="")
    parser.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR),
                        help=f"Trial output root (default: {DEFAULT_RESULTS_DIR})")
    parser.add_argument("--ws-setup", default=None,
                        help=f"Workspace setup script (default: {DEFAULT_WS_SETUP})")
    parser.add_argument("--scene-root", default=None)
    parser.add_argument("--mujoco-build-dir", default=None)
    parser.add_argument("--mujoco-extra-args", default=None,
                        help="Extra flags for unitree_mujoco (default: '-k 0 -z 0.40' for basic_step, else empty)")
    parser.add_argument("--terrain-z-offset", type=float, default=0.0,
                        help="Perception noise: shift non-floor box top z by this offset "
                             "(MuJoCo physics unchanged).")
    parser.add_argument("--terrain-z-offset-only-below-z", type=float, default=None,
                        help="Restrict terrain_z_offset to surfaces with true top z below this "
                             "threshold [m]. Default omits the launch arg → all non-floor surfaces "
                             "get the offset. Use 0.15 on basic_step_short to apply only to box2 "
                             "(z=0.10) and leave box1 (z=0.20) unchanged.")
    parser.add_argument("--foothold-plan-log", choices=["on", "off"], default="off",
                        help="Record every completed MPC FL optimized-policy and swing-reference snapshot in "
                             "foothold_plan_snapshots.csv. This preserves pre-contact plans "
                             "for plan-versus-actual analysis.")
    parser.add_argument("--swing-height", type=float, default=None,
                        help="Temporarily override swing_trajectory_config.swingHeight [m] "
                             "for this trial; restore the original value afterward")
    parser.add_argument("--mpc-frequency", type=float, default=None,
                        help="Override mpcDesiredFrequency in the active task.info for this trial. "
                             "The original file is restored after the trial. sqp.dt is unchanged.")
    parser.add_argument("--sqp-iterations", type=int, default=None,
                        help="Override sqp.sqpIteration for this trial")
    parser.add_argument("--robust", choices=["keep", "on", "off"], default="keep",
                        help="Temporarily set robustPhase.enabled (default: keep task.info value)")
    parser.add_argument("--robust-t-a", type=float, default=None,
                        help="Advance robust start before nominal touchdown [s]; default: task.info")
    parser.add_argument("--robust-t-b", type=float, default=None,
                        help="Delay robust end after nominal touchdown [s]; default: task.info")
    parser.add_argument("--robust-p", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--robust-d", type=float, default=None,
                        help="Temporarily set robustPhase.d [m]")
    parser.add_argument("--robust-v-max", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--robust-hard-boundary-start", choices=["keep", "on", "off"], default="keep",
                        help="Temporarily set robustPhase.hard_boundary_start (g(t_a) >= d)")
    parser.add_argument("--robust-hard-boundary-end", choices=["keep", "on", "off"], default="keep",
                        help="Temporarily set robustPhase.hard_boundary_end (g(t_b) <= -d)")
    parser.add_argument("--robust-slack-boundary-start", choices=["keep", "on", "off"], default="keep",
                        help="Temporarily enable quadratic slack for g(t_a) >= d")
    parser.add_argument("--robust-slack-boundary-end", choices=["keep", "on", "off"], default="keep",
                        help="Temporarily enable quadratic slack for g(t_b) <= -d")
    parser.add_argument("--robust-slack-weight-start", type=float, default=None,
                        help="Override robustPhase.slack_boundary_weight_start")
    parser.add_argument("--robust-slack-weight-end", type=float, default=None,
                        help="Override robustPhase.slack_boundary_weight_end")
    parser.add_argument("--robust-splice", choices=["keep", "on", "off"], default="keep",
                        help="Temporarily set robustPhase.enable_splice")
    parser.add_argument("--robust-verbose", choices=["keep", "on", "off"], default="keep",
                        help="Temporarily set robustPhase.verbose_log")
    parser.add_argument("--metrics-grace-sec", type=float, default=30.0,
                        help="How long after scenario timeout_sec to wait for "
                             "auto_input_metrics to exit cleanly before SIGKILL "
                             "(default 30 — protective vs the rclpy.shutdown hang). "
                             "Reduce to 5 for fast batch sweeps when you trust "
                             "auto_input_metrics shuts down promptly.")
    parser.add_argument("--controller-ready-timeout", type=float, default=None,
                        help="Seconds to wait for the OCS2 controller to finish loading. "
                             "Defaults to controller_startup_wait_sec in the scenario.")
    parser.add_argument("--post-trial-hold-sec", type=float, default=0.0,
                        help="Keep MuJoCo/controller alive after metrics finish. Use a positive "
                             "number of seconds, or -1 to wait until Ctrl+C (default: 0).")
    parser.add_argument("--force-cleanup", action="store_true",
                        help="Allow global pkill cleanup of matching ROS/MuJoCo processes. "
                             "Without this flag, only processes started by this trial are stopped.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate paths and print resolved commands/overrides without launching")
    args = parser.parse_args()

    if args.robust_p is not None or args.robust_v_max is not None:
        parser.error("--robust-p/--robust-v-max were removed; use --robust-t-a and --robust-t-b "
                     "in seconds. v_max is derived as 2*d_max/(window_end-window_start).")

    if args.robust == "on" and args.mode != "perceptive_dev_v2":
        parser.error("--robust on requires --mode perceptive_dev_v2")

    ws_setup = detect_ws_setup(args.ws_setup)
    scene_root = detect_scene_root(args.scene_root)
    mujoco_build_dir = detect_mujoco_build_dir(args.mujoco_build_dir)
    ros_env_cmd = make_ros_env_cmd(ws_setup)
    results_dir = Path(args.results_dir).expanduser().resolve()
    other_runners = find_other_trial_runners()
    if other_runners:
        raise RuntimeError(
            f"Another run_trial.py is already running (PID(s): {other_runners}); "
            "wait for it to finish before starting another trial"
        )
    recover_interrupted_task_info(results_dir)
    signal.signal(signal.SIGTERM, handle_termination_signal)

    scenario_path = resolve_scenario_path(args.scenario)
    with open(scenario_path, "r", encoding="utf-8") as f:
        scenario_cfg = yaml.safe_load(f)

    scene_path = load_scene_path(scene_root, args.terrain)
    if not Path(scene_path).exists():
        raise FileNotFoundError(f"Scene file not found: {scene_path}")
    if not TASK_INFO_PATH.exists():
        raise FileNotFoundError(f"task.info not found: {TASK_INFO_PATH}")
    if not UNITREE_LIB_DIR.is_dir():
        raise FileNotFoundError(f"unitree_sdk2 library directory not found: {UNITREE_LIB_DIR}")

    overrides = task_info_overrides(args)
    original_task_info = TASK_INFO_PATH.read_text(encoding="utf-8")
    effective_task_info = render_task_info(original_task_info, overrides)
    effective_parameters = effective_task_parameters(effective_task_info)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_name = f"{timestamp}_{Path(scene_path).stem}_{args.mode}"
    if args.tag:
        run_name += f"_{args.tag}"
    run_dir = results_dir / run_name

    enable_perceptive = "true" if args.mode == "perceptive_dev_v2" else "false"

    mujoco_extra = mujoco_extra_args_for_scene(args.terrain, args.mujoco_extra_args)
    # mujoco's bundled CycloneDDS (in unitree_sdk2/install/lib) crashes with
    # `dds_writecdr_impl_common: Assertion ... iox_pub == NULL ...` when the
    # ROS-loaded LD_LIBRARY_PATH brings in iceoryx-related libs that ABI-clash
    # with the bundled DDS. Strip the env down to just unitree_sdk2's libs +
    # /usr libs, and unset every ROS / RMW / DDS variable so the bundled
    # CycloneDDS uses its compiled-in defaults (no SHM).
    mujoco_cmd = (
        "unset RMW_IMPLEMENTATION CYCLONEDDS_URI ROS_DISTRO ROS_VERSION "
        "ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY CMAKE_PREFIX_PATH "
        "AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH "
        "PYTHONPATH && "
        f"export LD_LIBRARY_PATH={shlex.quote(str(UNITREE_LIB_DIR))}:/usr/local/lib:/usr/lib/x86_64-linux-gnu && "
        "export ROS_DOMAIN_ID=1 && "
        f"cd {shlex.quote(str(mujoco_build_dir))} && "
        f"./unitree_mujoco -r go2 -s {shlex.quote(str(scene_path))} {mujoco_extra}"
    ).strip()
    extra_args = []
    if abs(args.terrain_z_offset) > 0.0:
        extra_args.append(f"terrain_z_offset:={args.terrain_z_offset}")
    if args.terrain_z_offset_only_below_z is not None:
        extra_args.append(
            f"terrain_z_offset_only_below_z:={args.terrain_z_offset_only_below_z}")
    # Per-tick CSV log saved into the run_dir for offline analysis.
    tick_log_default = run_dir / "tick.csv"
    extra_args.append(f"tick_log_path:={tick_log_default}")
    foothold_plan_log_default = run_dir / "foothold_plan_snapshots.csv"
    if args.foothold_plan_log == "on":
        extra_args.append(f"foothold_plan_log_path:={foothold_plan_log_default}")
    ros_log_dir = run_dir / "ros_logs"
    trial_ros_env_cmd = (
        f"export ROS_LOG_DIR={shlex.quote(str(ros_log_dir))} && {ros_env_cmd}"
    )

    controller_cmd = (
        f"{trial_ros_env_cmd} && "
        "ros2 launch ocs2_quadruped_controller mujoco.launch.py "
        "pkg_description:=go2_description "
        f"enable_perceptive:={enable_perceptive} "
        "publish_static_terrain:=true "
        f"terrain_scene_file:={shlex.quote(Path(scene_path).name)} "
        + " ".join(shlex.quote(value) for value in extra_args)
    )
    metrics_cmd = (
        f"{trial_ros_env_cmd} && "
        f"python3 {shlex.quote(str(ROOT / 'auto_input_metrics.py'))} "
        f"--scenario {shlex.quote(str(scenario_path))} "
        f"--result-json {shlex.quote(str(run_dir / 'result.json'))} "
        f"--terrain {shlex.quote(Path(scene_path).stem)} --mode {shlex.quote(args.mode)}"
    )

    print(f"[run_trial] workspace : {WORKSPACE_ROOT}")
    print(f"[run_trial] scenario  : {scenario_path}")
    print(f"[run_trial] terrain   : {scene_path}")
    print(f"[run_trial] results   : {run_dir}")
    print(f"[run_trial] overrides : {overrides or {'task.info': 'unchanged'}}")
    print(f"[run_trial] effective : {effective_parameters}")
    if args.dry_run:
        print(f"[dry-run] mujoco    : {mujoco_cmd}")
        print(f"[dry-run] controller: {controller_cmd}")
        print(f"[dry-run] metrics   : {metrics_cmd}")
        return

    if args.force_cleanup:
        print("[run_trial] --force-cleanup: removing matching pre-existing processes")
        kill_lingering_processes(quiet=False)
    check_for_stale_nodes(ros_env_cmd)

    run_dir.mkdir(parents=True, exist_ok=False)
    shutil.copy2(scenario_path, run_dir / "scenario.yaml")
    (run_dir / "task.info.original").write_text(original_task_info, encoding="utf-8")
    (run_dir / "task.info.effective").write_text(effective_task_info, encoding="utf-8")
    run_config = {
        "status": "starting",
        "workspace": str(WORKSPACE_ROOT),
        "scenario": str(scenario_path),
        "terrain": str(scene_path),
        "mode": args.mode,
        "tag": args.tag,
        "task_info": str(TASK_INFO_PATH),
        "task_info_overrides": overrides,
        "effective_task_parameters": effective_parameters,
        "terrain_z_offset": args.terrain_z_offset,
        "terrain_z_offset_only_below_z": args.terrain_z_offset_only_below_z,
        "foothold_plan_log": args.foothold_plan_log,
        "post_trial_hold_sec": args.post_trial_hold_sec,
        "result_files": {
            "metrics": str(run_dir / "result.json"),
            "ticks": str(tick_log_default),
            "foothold_plan_snapshots": (
                str(foothold_plan_log_default)
                if args.foothold_plan_log == "on" else None
            ),
            "controller_log": str(run_dir / "controller.log"),
            "mujoco_log": str(run_dir / "mujoco.log"),
            "ros_log_dir": str(ros_log_dir),
            "summary": str(results_dir / "summary.csv"),
        },
    }
    run_config_path = run_dir / "run_config.json"
    run_config_path.write_text(json.dumps(run_config, indent=2), encoding="utf-8")

    mujoco = None
    controller = None
    metrics_proc = None
    controller_log_stream_stop = threading.Event()
    controller_log_stream = None
    task_info_changed = effective_task_info != original_task_info
    trial_status = "failed"
    trial_error = None
    try:
        if task_info_changed:
            TASK_INFO_PATH.write_text(effective_task_info, encoding="utf-8")
            print(f"[run_trial] applied temporary task.info overrides: {overrides}")
        mujoco_log_path = run_dir / "mujoco.log"
        controller_log_path = run_dir / "controller.log"
        mujoco = launch_process(mujoco_cmd, mujoco_log_path)
        time.sleep(float(scenario_cfg.get("mujoco_startup_wait_sec", 2.5)))
        raise_if_process_exited(mujoco, "MuJoCo", mujoco_log_path)
        controller = launch_process(controller_cmd, controller_log_path)
        controller_log_stream = threading.Thread(
            target=stream_matching_log_lines,
            args=(controller_log_path, controller_log_stream_stop, (
                "[MPC timing]",
                "[robust_event]",
                "[robust_contact_splice]",
                "[robust_stance_enter]",
            )),
            daemon=True,
        )
        controller_log_stream.start()
        ready_timeout = (
            args.controller_ready_timeout
            if args.controller_ready_timeout is not None
            else float(scenario_cfg.get("controller_startup_wait_sec", 180.0))
        )
        wait_for_nodes(
            ros_env_cmd,
            timeout_sec=ready_timeout,
            watched_process=controller,
            watched_log=controller_log_path,
        )
        wait_for_controller_active(
            ros_env_cmd,
            timeout_sec=ready_timeout,
            watched_process=controller,
            watched_log=controller_log_path,
        )
        wait_for_topic_subscribers(ros_env_cmd, timeout_sec=30.0)
        wait_for_odom(ros_env_cmd, timeout_sec=30.0)
        # Stage 8 fix: auto_input_metrics.py occasionally hangs in
        # rclpy.shutdown() after writing result.json on fall detection. That
        # would leave subprocess.run() blocked forever and prevent the
        # finally block from running -> all child processes (mujoco, rviz,
        # controller, …) survive past the trial. Wrap the call with a hard
        # timeout based on the scenario's timeout_sec plus generous slack.
        metrics_proc = subprocess.Popen(
            [COMMAND_SHELL, "-lc", metrics_cmd],
            preexec_fn=os.setsid,
        )
        metrics_timeout = float(scenario_cfg.get("timeout_sec", 16.0)) + args.metrics_grace_sec
        try:
            metrics_proc.wait(timeout=metrics_timeout)
        except subprocess.TimeoutExpired:
            print(f"[run_trial] auto_input_metrics did not exit within "
                  f"{metrics_timeout:.0f}s — SIGKILL its process group")
            try:
                os.killpg(os.getpgid(metrics_proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            metrics_proc.wait(timeout=2)
        if metrics_proc.returncode not in (0, -signal.SIGKILL):
            raise RuntimeError(f"auto_input_metrics exited with code {metrics_proc.returncode}")
        if not (run_dir / "result.json").exists():
            raise RuntimeError(f"metrics result was not created: {run_dir / 'result.json'}")
        result_path = run_dir / "result.json"
        result = json.loads(result_path.read_text(encoding="utf-8"))
        result["experiment"] = {
            "trial": run_name,
            "tag": args.tag,
            "terrain": Path(scene_path).stem,
            "mode": args.mode,
            "terrain_z_offset": args.terrain_z_offset,
            "terrain_z_offset_only_below_z": args.terrain_z_offset_only_below_z,
            "effective_task_parameters": effective_parameters,
        }
        result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        append_trial_summary(results_dir / "summary.csv", result, result_path)
        if args.post_trial_hold_sec != 0.0:
            try:
                if args.post_trial_hold_sec > 0.0:
                    print(
                        f"[run_trial] trial complete; keeping simulator alive for "
                        f"{args.post_trial_hold_sec:g}s"
                    )
                    time.sleep(args.post_trial_hold_sec)
                else:
                    print("[run_trial] trial complete; simulator remains alive until Ctrl+C")
                    while True:
                        time.sleep(1.0)
            except KeyboardInterrupt:
                print("[run_trial] Ctrl+C received; shutting down trial processes")
        trial_status = "completed"
    except BaseException as exc:
        trial_error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if metrics_proc is not None:
            stop_process(metrics_proc, label="metrics", force_cleanup=args.force_cleanup)
        if controller is not None:
            stop_process(controller, label="controller", force_cleanup=args.force_cleanup)
        controller_log_stream_stop.set()
        if controller_log_stream is not None:
            controller_log_stream.join(timeout=2.0)
        if mujoco is not None:
            stop_process(mujoco, label="mujoco", force_cleanup=args.force_cleanup)
        if args.force_cleanup:
            kill_lingering_processes(quiet=True)
        if task_info_changed:
            TASK_INFO_PATH.write_text(original_task_info, encoding="utf-8")
            print("[run_trial] restored original task.info")
        run_config["status"] = trial_status
        run_config["error"] = trial_error
        run_config_path.write_text(json.dumps(run_config, indent=2), encoding="utf-8")

    if trial_status == "completed":
        generate_trial_plots(run_dir)
    print(f"[done] {run_dir}")


if __name__ == "__main__":
    main()
