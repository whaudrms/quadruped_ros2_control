#!/usr/bin/env python3
"""
Automatic perceptive OCS2 + WBC trial runner.

Ported from colleague's quadruped_temp/tools/perceptive_dev_v2/run_trial.py
(Jazzy) to our Humble branch at quadruped_ros2_control (humble-dev-perceptive).

Usage:
    python3 run_trial.py --terrain basic_step.xml --mode perceptive_dev_v2 --tag test

Differences vs colleague:
- ROS distro: humble (was jazzy)
- Environment: sources ~/GO2_ws/setup_quadruped.sh (FastDDS) rather than
  /opt/ros/<distro>/setup.bash + custom install tree
- MuJoCo command: for basic_step scenes, adds `-k 0 -z 0.40` so the Go2
  spawns above box1 (our unitree_mujoco binary supports these flags).
  See ~/GO2_ws/unitree_mujoco/simulate/src/param.h.
"""

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

# Default workspace env script (FastDDS + ROS2 humble + our install tree).
DEFAULT_WS_SETUP = Path.home() / "GO2_ws" / "setup_quadruped.sh"

# Refiner daemon defaults (Stage 8).
REFINER_PYTHON = Path("/home/cora/miniforge3/envs/wb-mpc/bin/python")
REFINER_DAEMON_SCRIPT = Path("/home/cora/GO2_ws/contact_timing_uncertainty/robust_refine/refiner_daemon.py")

# Process-name patterns that should never survive past the end of a trial.
# Used for both pre-run defensive cleanup and post-run nuke.
# Stage 8 fix: rviz2/robot_state_publisher/spawner were missing — those
# processes survived run_trial's stop_process pgid-based cleanup (likely
# because ros2 launch double-forks them outside the launch's process group).
# Adding them here makes the post-run nuke + pre-run defensive sweep catch
# every leftover from the previous trial.
LINGERING_PROCESS_PATTERNS = (
    "unitree_mujoco",
    "ros2_control_node",
    "ocs2_quadruped_controller",
    "ros2 launch ocs2",
    "planar_terrain",
    "refiner_daemon.py",
    "rviz2",
    "robot_state_publisher",
    "controller_manager",
    "spawner",
)


def launch_process(command: str, log_path: Path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            ["zsh", "-lc", command],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    return process


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


def stop_process(process: subprocess.Popen, label: str = ""):
    """Stop a launched process group with bulletproof cleanup.

    1. SIGINT the pgid (politely ask for shutdown).
    2. wait up to 8s.
    3. If still alive, SIGKILL the pgid.
    4. If STILL alive after another 2s, fall back to a pkill -9 nuke pass that
       greps for the well-known lingering process patterns.

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
    # Last-ditch nuke pass — if a child of this group is still alive (e.g. a
    # ros2_control_node that didn't notice its parent died), pkill -9 it.
    if label:
        print(f"[stop_process] {label}: SIGKILL did not reap; running nuke pass")
    kill_lingering_processes(quiet=True)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        # Give up; the process is wedged in the kernel. The lingering-process
        # cleanup at the next run start will get it.
        if label:
            print(f"[stop_process] {label}: still wedged after nuke pass; abandoning")


def kill_lingering_processes(quiet: bool = False):
    """SIGKILL any leftover processes matching well-known names.

    Called both at the start of a run (defensive cleanup of garbage left by a
    previous failed run) and at the very end of a run (post-finally nuke).

    Uses pkill -9 -f. Errors are tolerated because pkill returns non-zero when
    nothing matched, which is the normal happy case.
    """
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


def run_shell(command: str, *, timeout: float = 10.0):
    return subprocess.run(
        ["zsh", "-lc", command],
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
    return f"source {ws_setup}"


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


def main():
    parser = argparse.ArgumentParser(description="Run one automatic perceptive OCS2 + QP-WBC dev_v2 trial.")
    parser.add_argument("--terrain", default="basic_step.xml")
    parser.add_argument("--scenario", default="standing_trot_forward")
    parser.add_argument("--mode", choices=["pure_dev_v2", "perceptive_dev_v2"], default="perceptive_dev_v2")
    parser.add_argument("--tag", default="")
    parser.add_argument("--ws-setup", default=None,
                        help=f"Workspace setup script (default: {DEFAULT_WS_SETUP})")
    parser.add_argument("--scene-root", default=None)
    parser.add_argument("--mujoco-build-dir", default=None)
    parser.add_argument("--mujoco-extra-args", default=None,
                        help="Extra flags for unitree_mujoco (default: '-k 0 -z 0.40' for basic_step, else empty)")
    parser.add_argument("--ocs2-dump-dir", default="",
                        help="Stage 7: if non-empty, dump OCS2 PrimalSolution CSVs into this directory")
    parser.add_argument("--ocs2-dump-max-cycles", type=int, default=0,
                        help="Stage 7: max MPC cycles to dump (0 = disabled)")
    parser.add_argument("--ocs2-dump-min-interval-sec", type=float, default=0.05,
                        help="Stage 7: min solver-init-time delta between dumps (s)")
    parser.add_argument("--terrain-z-offset", type=float, default=0.0,
                        help="Perception noise: shift all non-floor box top z by this offset "
                             "(MuJoCo physics unchanged). Used for robust_refine evaluation.")
    parser.add_argument("--enable-refiner-swap", action="store_true",
                        help="Stage 8: enable synchronous robust_refine IPC + WBC override.")
    parser.add_argument("--enable-refiner", action="store_true",
                        help="Stage 8: spawn the robust_refine Python daemon as a sidecar "
                             "and pass enable_refiner_swap:=true to the launch. Combines the "
                             "common case of '--enable-refiner-swap + start the daemon for me'.")
    parser.add_argument("--refiner-python", default=str(REFINER_PYTHON),
                        help=f"Python interpreter to run refiner_daemon.py (default: {REFINER_PYTHON})")
    parser.add_argument("--refiner-daemon-script", default=str(REFINER_DAEMON_SCRIPT),
                        help=f"Path to refiner_daemon.py (default: {REFINER_DAEMON_SCRIPT})")
    parser.add_argument("--refined-dir", default=None,
                        help="Stage 8: directory polled for refined plans "
                             "(default /dev/shm/robust_refine/out).")
    parser.add_argument("--refined-timeout-sec", type=float, default=None,
                        help="Stage 8: per-cycle timeout while waiting for the refined plan (s).")
    parser.add_argument("--no-wipe-refine-out", action="store_true",
                        help="Stage 8 offline-replay: do NOT wipe /dev/shm/robust_refine/out/ "
                             "before the run. Used when refined plans are pre-placed manually "
                             "(no live daemon).")
    parser.add_argument("--mpc-one-shot", action="store_true",
                        help="Stage 9: solve a single full-horizon OCP at OCS2 entry then "
                             "stop replanning. WBC tracks the cached policy for the rest of "
                             "the trial.")
    parser.add_argument("--mpc-one-shot-solves", type=int, default=10,
                        help="Stage 9: number of advanceMpc() iterations to run during the "
                             "one-shot bootstrap (each is one SQP iteration; warm-starts).")
    args = parser.parse_args()

    ws_setup = detect_ws_setup(args.ws_setup)
    scene_root = detect_scene_root(args.scene_root)
    mujoco_build_dir = detect_mujoco_build_dir(args.mujoco_build_dir)
    ros_env_cmd = make_ros_env_cmd(ws_setup)

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

    mujoco_extra = mujoco_extra_args_for_scene(args.terrain, args.mujoco_extra_args)
    # mujoco's bundled CycloneDDS (in unitree_sdk2/install/lib) crashes with
    # `dds_writecdr_impl_common: Assertion ... iox_pub == NULL ...` when the
    # ROS-loaded LD_LIBRARY_PATH brings in iceoryx-related libs that ABI-clash
    # with the bundled DDS. Strip the env down to just unitree_sdk2's libs +
    # /usr libs, and unset every ROS / RMW / DDS variable so the bundled
    # CycloneDDS uses its compiled-in defaults (no SHM).
    unitree_lib = "/home/cora/GO2_ws/unitree_sdk2/install/lib"
    mujoco_cmd = (
        "unset RMW_IMPLEMENTATION CYCLONEDDS_URI ROS_DISTRO ROS_VERSION "
        "ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY CMAKE_PREFIX_PATH "
        "AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH "
        "PYTHONPATH && "
        f"export LD_LIBRARY_PATH={unitree_lib}:/usr/local/lib:/usr/lib/x86_64-linux-gnu && "
        "export ROS_DOMAIN_ID=1 && "
        f"cd {mujoco_build_dir} && "
        f"./unitree_mujoco -r go2 -s {scene_path} {mujoco_extra}"
    ).strip()
    extra_args = []
    if args.ocs2_dump_dir:
        extra_args.append(f"ocs2_dump_dir:={args.ocs2_dump_dir}")
    if args.ocs2_dump_max_cycles > 0:
        extra_args.append(f"ocs2_dump_max_cycles:={args.ocs2_dump_max_cycles}")
        extra_args.append(f"ocs2_dump_min_interval_sec:={args.ocs2_dump_min_interval_sec}")
    if abs(args.terrain_z_offset) > 0.0:
        extra_args.append(f"terrain_z_offset:={args.terrain_z_offset}")
    # --enable-refiner is the convenience flag: it both enables the launch arg
    # and spawns the daemon below. --enable-refiner-swap remains for callers
    # that manage the daemon out-of-band.
    if args.enable_refiner_swap or args.enable_refiner:
        extra_args.append("enable_refiner_swap:=true")
    if args.refined_dir is not None:
        extra_args.append(f"refined_dir:={args.refined_dir}")
    if args.refined_timeout_sec is not None:
        extra_args.append(f"refined_timeout_sec:={args.refined_timeout_sec}")
    if args.mpc_one_shot:
        extra_args.append("mpc_one_shot:=true")
        extra_args.append(f"mpc_one_shot_solves:={args.mpc_one_shot_solves}")
    # Stage 8 analysis — per-tick CSV log saved into the run_dir.
    tick_log_default = str(run_dir / "tick.csv")
    extra_args.append(f"tick_log_path:={tick_log_default}")

    controller_cmd = (
        f"source {ws_setup} && "
        "ros2 launch ocs2_quadruped_controller mujoco.launch.py "
        "pkg_description:=go2_description "
        f"enable_perceptive:={enable_perceptive} "
        "publish_static_terrain:=true "
        f"terrain_scene_file:={Path(scene_path).name} "
        + " ".join(extra_args)
    )
    metrics_cmd = (
        f"source {ws_setup} && "
        f"python3 {ROOT / 'auto_input_metrics.py'} "
        f"--scenario {scenario_path} "
        f"--result-json {run_dir / 'result.json'} "
        f"--append-csv {RESULTS_DIR / 'summary.csv'} "
        f"--terrain {Path(scene_path).stem} --mode {args.mode}"
    )

    # Defensive cleanup before launching: nuke whatever previous run left
    # behind. Cheap and idempotent.
    kill_lingering_processes(quiet=True)

    # Wipe Stage 8 IPC dirs so stale files from a previous run can't trigger
    # phantom "override=active" log lines or be re-read by the new daemon.
    # Stage 8 offline-replay variant: --no-wipe-refine-out keeps pre-placed
    # refined plans in /dev/shm/.../out/ for the offline 1-shot comparison.
    if args.enable_refiner_swap or args.enable_refiner:
        wipe_subs = ["in"]
        if not args.no_wipe_refine_out:
            wipe_subs.append("out")
        for sub in wipe_subs:
            d = Path("/dev/shm/robust_refine") / sub
            if d.exists():
                for f in d.glob("*"):
                    try:
                        f.unlink()
                    except OSError:
                        pass

    mujoco = None
    controller = None
    refiner = None
    try:
        if args.enable_refiner:
            refiner_python = Path(args.refiner_python)
            refiner_script = Path(args.refiner_daemon_script)
            if not refiner_python.exists():
                raise FileNotFoundError(f"refiner python not found: {refiner_python}")
            if not refiner_script.exists():
                raise FileNotFoundError(f"refiner_daemon.py not found: {refiner_script}")
            # The wb-mpc conda env's pinocchio gets shadowed by /opt/ros/humble's
            # pinocchio if PYTHONPATH/AMENT_PREFIX_PATH/LD_LIBRARY_PATH inherit
            # from the user's ROS-sourced shell. Strip them so the daemon's
            # interpreter resolves imports purely against its own site-packages.
            refiner_cmd = (
                "unset PYTHONPATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX "
                "ROS_DISTRO ROS_PYTHON_VERSION ROS_VERSION CMAKE_PREFIX_PATH "
                "ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI "
                "COLCON_PREFIX_PATH LD_LIBRARY_PATH && "
                f"cd {refiner_script.parent} && "
                f"{refiner_python} {refiner_script.name}"
            )
            refiner = launch_process(refiner_cmd, run_dir / "refiner_daemon.log")
            # Give the daemon a moment to bind its IPC channel before the
            # controller starts polling for refined plans.
            time.sleep(1.0)

        mujoco = launch_process(mujoco_cmd, run_dir / "mujoco.log")
        time.sleep(float(scenario_cfg.get("mujoco_startup_wait_sec", 2.5)))
        controller = launch_process(controller_cmd, run_dir / "controller.log")
        wait_for_nodes(ros_env_cmd, timeout_sec=float(scenario_cfg.get("controller_startup_wait_sec", 20.0)))
        wait_for_topic_subscribers(ros_env_cmd, timeout_sec=15.0)
        wait_for_odom(ros_env_cmd, timeout_sec=15.0)
        # Stage 8 fix: auto_input_metrics.py occasionally hangs in
        # rclpy.shutdown() after writing result.json on fall detection. That
        # would leave subprocess.run() blocked forever and prevent the
        # finally block from running -> all child processes (mujoco, rviz,
        # controller, …) survive past the trial. Wrap the call with a hard
        # timeout based on the scenario's timeout_sec plus generous slack.
        metrics_proc = subprocess.Popen(
            ["zsh", "-lc", metrics_cmd],
            preexec_fn=os.setsid,
        )
        metrics_timeout = float(scenario_cfg.get("timeout_sec", 16.0)) + 30.0
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
    finally:
        if controller is not None:
            stop_process(controller, label="controller")
        if mujoco is not None:
            stop_process(mujoco, label="mujoco")
        if refiner is not None:
            stop_process(refiner, label="refiner")
        # Belt-and-suspenders: regardless of whether the per-process stop
        # succeeded, sweep the process table for any leftover noise. This is
        # what catches the zombie-pgid / 37-minute-hang case.
        kill_lingering_processes(quiet=True)

    print(f"[done] {run_dir}")


if __name__ == "__main__":
    main()
