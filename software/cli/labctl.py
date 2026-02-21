#!/usr/bin/env python3
"""SoftBot Lab command line entrypoint."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / "scripts"
FIRMWARE_DIR = REPO_ROOT / "firmware"
PROFILES_DIR = REPO_ROOT / "config" / "profiles"
EXAMPLES_DIR = REPO_ROOT / "software" / "ejemplos"
GUI_SCRIPT = REPO_ROOT / "software" / "gui" / "softbot_gui.py"
HARDWARE_GUI_SCRIPT = REPO_ROOT / "software" / "gui" / "hardware_mosfet_gui.py"
PUMP_EVAL_GUI_SCRIPT = REPO_ROOT / "software" / "gui" / "pump_eval_gui.py"
SMOKE_SCRIPT = REPO_ROOT / "software" / "tools" / "smoke_lab.py"
HARDWARE_TEST_SCRIPT = REPO_ROOT / "software" / "tools" / "hardware_component_tester.py"
HARDWARE_RUNTIME_CHECK_SCRIPT = REPO_ROOT / "software" / "tools" / "hardware_runtime_check.py"
PUMP_SWAP_BENCH_SCRIPT = REPO_ROOT / "software" / "tools" / "pump_swap_validation.py"
CONTROLLER_ID_CAPTURE_SCRIPT = REPO_ROOT / "software" / "tools" / "controller_id_capture.py"
ROOT_LOCUS_TUNER_SCRIPT = REPO_ROOT / "software" / "tools" / "root_locus_discrete_tuner.py"
CONTROLLER_VALIDATION_SCRIPT = REPO_ROOT / "software" / "tools" / "controller_validation_report.py"

OPS_DIR = REPO_ROOT / "experiments" / "logs" / "ops"
STATE_FILE = OPS_DIR / "labctl_state.json"
EVENT_FILE = OPS_DIR / "labctl_events.log"

DEFAULT_PROFILE_NAME = "default_lab.json"
MICROROS_CONTAINER_NAME = "softbot_microros_agent"


class LabCtlError(RuntimeError):
    """Domain exception for expected user-facing failures."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def ensure_ops_dir() -> None:
    OPS_DIR.mkdir(parents=True, exist_ok=True)


def event(message: str) -> None:
    ensure_ops_dir()
    with EVENT_FILE.open("a", encoding="utf-8") as handle:
        handle.write(f"{utc_now()} {message}\n")


def load_state() -> dict[str, Any]:
    ensure_ops_dir()
    if not STATE_FILE.exists():
        return {"processes": [], "containers": []}
    with STATE_FILE.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)
    if "processes" not in raw:
        raw["processes"] = []
    if "containers" not in raw:
        raw["containers"] = []
    return raw


def save_state(state: dict[str, Any]) -> None:
    ensure_ops_dir()
    with STATE_FILE.open("w", encoding="utf-8") as handle:
        json.dump(state, handle, indent=2)


def is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def cleanup_stale_processes(state: dict[str, Any]) -> None:
    alive: list[dict[str, Any]] = []
    for proc in state.get("processes", []):
        pid = int(proc.get("pid", -1))
        if pid > 0 and is_alive(pid):
            alive.append(proc)
    state["processes"] = alive


def record_process(name: str, pid: int, log_path: Path, command: str) -> None:
    state = load_state()
    cleanup_stale_processes(state)
    state["processes"].append(
        {
            "name": name,
            "pid": pid,
            "log": str(log_path.relative_to(REPO_ROOT)),
            "command": command,
            "started_at": utc_now(),
        }
    )
    save_state(state)


def record_container(name: str) -> None:
    state = load_state()
    containers = list(state.get("containers", []))
    if name not in containers:
        containers.append(name)
    state["containers"] = containers
    save_state(state)


def remove_container(name: str) -> None:
    state = load_state()
    state["containers"] = [c for c in state.get("containers", []) if c != name]
    save_state(state)


def resolve_profile_path(profile_arg: str | None) -> Path:
    if not profile_arg or profile_arg == "default":
        return PROFILES_DIR / DEFAULT_PROFILE_NAME

    candidate = Path(profile_arg)
    if candidate.is_absolute() and candidate.exists():
        return candidate

    if candidate.suffix != ".json":
        candidate = Path(f"{profile_arg}.json")

    from_profiles = PROFILES_DIR / candidate.name
    if from_profiles.exists():
        return from_profiles

    from_repo = (REPO_ROOT / candidate).resolve()
    if from_repo.exists():
        return from_repo

    raise LabCtlError(f"Profile not found: {profile_arg}")


def get_nested(data: dict[str, Any], keys: list[str]) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            raise KeyError(".".join(keys))
        current = current[key]
    return current


def validate_profile(profile: dict[str, Any]) -> list[str]:
    checks: list[tuple[list[str], type[Any]]] = [
        (["firmware", "board"], str),
        (["firmware", "upload_port"], str),
        (["firmware", "baud"], int),
        (["agent", "serial_dev"], str),
        (["agent", "baud"], int),
        (["safety", "max_kpa"], (int, float)),
        (["safety", "min_kpa"], (int, float)),
        (["locomotion", "default_strategy"], str),
        (["paths", "experiments_dir"], str),
    ]

    errors: list[str] = []
    for keys, expected_type in checks:
        try:
            value = get_nested(profile, keys)
        except KeyError:
            errors.append(f"Missing key: {'.'.join(keys)}")
            continue
        if not isinstance(value, expected_type):
            if isinstance(expected_type, tuple):
                name = " or ".join(t.__name__ for t in expected_type)
            else:
                name = expected_type.__name__
            errors.append(f"Invalid type for {'.'.join(keys)}: expected {name}")
    return errors


def load_profile(profile_arg: str | None) -> tuple[dict[str, Any], Path]:
    path = resolve_profile_path(profile_arg)
    with path.open("r", encoding="utf-8") as handle:
        profile = json.load(handle)
    errors = validate_profile(profile)
    if errors:
        details = "\n".join(f"- {msg}" for msg in errors)
        raise LabCtlError(f"Invalid profile {path}:\n{details}")
    return profile, path


def shell_join(parts: list[str]) -> str:
    return " ".join(shlex.quote(piece) for piece in parts)


def ros_wrapped(parts: list[str]) -> list[str]:
    ros_log_dir = OPS_DIR / "ros"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    cmd = shell_join(parts)
    full = (
        "source /opt/ros/humble/setup.bash && "
        f"export ROS_LOG_DIR={shlex.quote(str(ros_log_dir))} && {cmd}"
    )
    return ["bash", "-lc", full]


def run_command(
    parts: list[str],
    cwd: Path | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess:
    return subprocess.run(parts, cwd=cwd or REPO_ROOT, check=check)


def start_background(name: str, parts: list[str], use_ros: bool) -> int:
    ensure_ops_dir()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = OPS_DIR / f"{name}_{stamp}.log"

    command = ros_wrapped(parts) if use_ros else parts
    display = shell_join(command)

    with log_path.open("w", encoding="utf-8") as log_handle:
        proc = subprocess.Popen(
            command,
            cwd=REPO_ROOT,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    record_process(name=name, pid=proc.pid, log_path=log_path, command=display)
    event(f"spawn process name={name} pid={proc.pid} log={log_path}")
    print(f"Started {name} (pid={proc.pid})")
    print(f"Log: {log_path.relative_to(REPO_ROOT)}")
    return proc.pid


def resolve_pio_executable() -> str:
    if shutil.which("pio"):
        return "pio"
    bundled = REPO_ROOT / ".venv" / "bin" / "pio"
    if bundled.exists():
        return str(bundled)
    raise LabCtlError("PlatformIO not found. Run ./scripts/install_lab.sh first.")


def resolve_repo_python() -> str:
    bundled = REPO_ROOT / ".venv" / "bin" / "python"
    if bundled.exists():
        return str(bundled)

    detected = shutil.which("python3")
    if detected:
        return detected
    return sys.executable


def python_has_modules(python_bin: str, modules: list[str]) -> bool:
    if not modules:
        return True
    script = "import " + ", ".join(modules)
    result = subprocess.run(
        [python_bin, "-c", script],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def resolve_python_with_modules(modules: list[str]) -> str:
    candidates: list[str] = []
    preferred = resolve_repo_python()
    candidates.append(preferred)

    sys_py = shutil.which("python3")
    if sys_py and sys_py not in candidates:
        candidates.append(sys_py)

    for candidate in candidates:
        if python_has_modules(candidate, modules):
            return candidate

    names = ", ".join(modules)
    raise LabCtlError(f"Python with required modules not found ({names}).")


def board_to_env(board: str) -> str:
    mapping = {
        "esp32dev": "esp32dev",
        "esp32_devkit_v1": "esp32dev",
    }
    if board in mapping:
        return mapping[board]
    raise LabCtlError(f"Unsupported board profile: {board}")


def resolve_example(name: str) -> Path:
    candidate = Path(name)
    if candidate.is_absolute() and candidate.exists():
        return candidate

    if candidate.suffix != ".py":
        candidate = Path(f"{name}.py")

    by_name = EXAMPLES_DIR / candidate.name
    if by_name.exists():
        return by_name

    repo_path = (REPO_ROOT / candidate).resolve()
    if repo_path.exists():
        return repo_path

    available = sorted(p.name for p in EXAMPLES_DIR.glob("*.py"))
    raise LabCtlError(f"Example not found: {name}. Available: {', '.join(available)}")


def ensure_docker_access() -> None:
    if shutil.which("docker") is None:
        raise LabCtlError("docker command not found")

    result = subprocess.run(
        ["docker", "info"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode == 0:
        return

    stderr = (result.stderr or "").strip()
    hint = (
        "Docker daemon not reachable or permission denied. "
        "Try: sudo systemctl enable --now docker && sudo usermod -aG docker $USER "
        "then open a new login shell (or run: exec su -l $USER)."
    )
    if stderr:
        raise LabCtlError(f"{hint}\nDocker error: {stderr}")
    raise LabCtlError(hint)


def cmd_doctor(args: argparse.Namespace) -> int:
    profile, profile_path = load_profile(getattr(args, "profile", None))

    checks: list[tuple[str, bool, str]] = []

    checks.append(("Linux host", sys.platform.startswith("linux"), sys.platform))

    py_ok = sys.version_info >= (3, 10)
    checks.append(("Python >= 3.10", py_ok, sys.version.split()[0]))

    ros_path = Path("/opt/ros/humble/setup.bash")
    checks.append(("ROS 2 Humble setup", ros_path.exists(), str(ros_path)))

    docker_path = shutil.which("docker") is not None
    checks.append(("docker command", docker_path, shutil.which("docker") or "missing"))

    pio_bin = None
    try:
        pio_bin = resolve_pio_executable()
        checks.append(("PlatformIO", True, pio_bin))
    except LabCtlError as exc:
        checks.append(("PlatformIO", False, str(exc)))

    checks.append(("Profile loaded", True, str(profile_path.relative_to(REPO_ROOT))))

    serial_candidates = sorted(Path("/dev").glob("ttyUSB*")) + sorted(Path("/dev").glob("ttyACM*"))
    serial_msg = ", ".join(p.name for p in serial_candidates) if serial_candidates else "none"
    checks.append(("Serial devices", len(serial_candidates) > 0, serial_msg))

    docker_daemon_ok = False
    docker_daemon_msg = "docker not available"
    if docker_path:
        result = subprocess.run(
            ["docker", "info"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        docker_daemon_ok = result.returncode == 0
        docker_daemon_msg = "reachable" if docker_daemon_ok else "not reachable"
    checks.append(("Docker daemon", docker_daemon_ok, docker_daemon_msg))

    profile_errors = validate_profile(profile)
    checks.append(("Profile schema", not profile_errors, "; ".join(profile_errors) or "ok"))

    print("SoftBot Lab doctor")
    print(f"Profile: {profile_path}")
    for label, ok, detail in checks:
        status = "OK" if ok else "FAIL"
        print(f"[{status}] {label}: {detail}")

    critical_labels = {
        "Python >= 3.10",
        "ROS 2 Humble setup",
        "docker command",
        "PlatformIO",
        "Profile schema",
    }
    failed_critical = [label for label, ok, _ in checks if (label in critical_labels and not ok)]

    if failed_critical:
        event(f"doctor failed critical={failed_critical}")
        return 1

    event("doctor success")
    return 0


def cmd_setup(args: argparse.Namespace) -> int:
    installer = SCRIPTS_DIR / "install_lab.sh"
    if not installer.exists():
        raise LabCtlError("scripts/install_lab.sh not found")

    if args.offline:
        command = [str(installer), "--offline", args.offline]
    else:
        command = [str(installer), "--online"]

    event(f"setup run mode={'offline' if args.offline else 'online'}")
    run_command(command)
    return 0


def cmd_firmware_build(args: argparse.Namespace) -> int:
    profile, profile_path = load_profile(args.profile)
    pio = resolve_pio_executable()
    env_name = board_to_env(str(profile["firmware"]["board"]))

    command = [pio, "run", "-d", str(FIRMWARE_DIR), "-e", env_name]
    event(f"firmware build profile={profile_path} env={env_name}")
    run_command(command)
    return 0


def cmd_firmware_flash(args: argparse.Namespace) -> int:
    profile, profile_path = load_profile(args.profile)
    pio = resolve_pio_executable()
    env_name = board_to_env(str(profile["firmware"]["board"]))
    upload_port = args.port or str(profile["firmware"]["upload_port"])

    command = [
        pio,
        "run",
        "-d",
        str(FIRMWARE_DIR),
        "-e",
        env_name,
        "--target",
        "upload",
        "--upload-port",
        upload_port,
    ]
    event(f"firmware flash profile={profile_path} env={env_name} port={upload_port}")
    run_command(command)
    return 0


def cmd_agent_start(args: argparse.Namespace) -> int:
    profile, _ = load_profile(args.profile)
    serial_dev = args.port or str(profile["agent"]["serial_dev"])
    baud = int(args.baud or profile["agent"]["baud"])

    ensure_docker_access()

    subprocess.run(["docker", "rm", "-f", MICROROS_CONTAINER_NAME], check=False)

    command = [
        "docker",
        "run",
        "-d",
        "--name",
        MICROROS_CONTAINER_NAME,
        "--restart",
        "unless-stopped",
        "-v",
        "/dev:/dev",
        "--privileged",
        "--net=host",
        "microros/micro-ros-agent:humble",
        "serial",
        "--dev",
        serial_dev,
        "-b",
        str(baud),
    ]

    run_command(command)
    record_container(MICROROS_CONTAINER_NAME)
    event(f"agent start container={MICROROS_CONTAINER_NAME} dev={serial_dev} baud={baud}")
    print(f"micro-ROS agent started as container '{MICROROS_CONTAINER_NAME}'")
    return 0


def cmd_gui_start(args: argparse.Namespace) -> int:
    if not GUI_SCRIPT.exists():
        raise LabCtlError(f"GUI script missing: {GUI_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [python_bin, str(GUI_SCRIPT)]
    if args.foreground:
        event("gui start foreground")
        run_command(ros_wrapped(command))
        return 0

    start_background(name="gui", parts=command, use_ros=True)
    print(
        "GUI started in background. If no window appears, "
        "run: ./scripts/labctl gui start --foreground"
    )
    return 0


def cmd_gui_pump_eval(args: argparse.Namespace) -> int:
    if not PUMP_EVAL_GUI_SCRIPT.exists():
        raise LabCtlError(f"Pump eval GUI script missing: {PUMP_EVAL_GUI_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [python_bin, str(PUMP_EVAL_GUI_SCRIPT)]
    if args.foreground:
        event("gui pump-eval foreground")
        run_command(ros_wrapped(command))
        return 0

    start_background(name="gui_pump_eval", parts=command, use_ros=True)
    print(
        "Pump eval GUI started in background. If no window appears, "
        "run: ./scripts/labctl gui pump-eval --foreground"
    )
    return 0


def cmd_example_run(args: argparse.Namespace) -> int:
    example = resolve_example(args.name)
    python_bin = resolve_repo_python()
    command = [python_bin, str(example)]

    if args.foreground:
        event(f"example run foreground script={example}")
        run_command(ros_wrapped(command))
        return 0

    start_background(name=f"example_{example.stem}", parts=command, use_ros=True)
    return 0


def cmd_smoke(args: argparse.Namespace) -> int:
    profile_path = resolve_profile_path(args.profile)
    if not SMOKE_SCRIPT.exists():
        raise LabCtlError(f"Smoke script missing: {SMOKE_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [
        python_bin,
        str(SMOKE_SCRIPT),
        "--profile",
        str(profile_path),
        "--chamber",
        str(args.chamber),
        "--inflate-kpa",
        str(args.inflate_kpa),
        "--hold-s",
        str(args.hold_s),
    ]

    event(f"smoke run profile={profile_path}")
    run_command(ros_wrapped(command))
    return 0


def cmd_hardware_test(args: argparse.Namespace) -> int:
    if not HARDWARE_TEST_SCRIPT.exists():
        raise LabCtlError(f"Hardware test script missing: {HARDWARE_TEST_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [
        python_bin,
        str(HARDWARE_TEST_SCRIPT),
        "--component",
        args.component,
        "--pwm",
        str(args.pwm),
        "--duration-s",
        str(args.duration_s),
        "--repeat",
        str(args.repeat),
    ]
    event(f"hardware test component={args.component} pwm={args.pwm} repeat={args.repeat}")
    run_command(ros_wrapped(command))
    return 0


def cmd_hardware_panel(args: argparse.Namespace) -> int:
    if not HARDWARE_TEST_SCRIPT.exists():
        raise LabCtlError(f"Hardware test script missing: {HARDWARE_TEST_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [python_bin, str(HARDWARE_TEST_SCRIPT), "--interactive", "--pwm", str(args.pwm)]
    event(f"hardware panel pwm={args.pwm}")
    run_command(ros_wrapped(command))
    return 0


def cmd_hardware_off(args: argparse.Namespace) -> int:
    del args
    if not HARDWARE_TEST_SCRIPT.exists():
        raise LabCtlError(f"Hardware test script missing: {HARDWARE_TEST_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [python_bin, str(HARDWARE_TEST_SCRIPT), "--off"]
    event("hardware off")
    run_command(ros_wrapped(command))
    return 0


def cmd_hardware_gui(args: argparse.Namespace) -> int:
    if not HARDWARE_GUI_SCRIPT.exists():
        raise LabCtlError(f"Hardware GUI script missing: {HARDWARE_GUI_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [python_bin, str(HARDWARE_GUI_SCRIPT)]
    if args.foreground:
        event("hardware gui foreground")
        run_command(ros_wrapped(command))
        return 0

    start_background(name="hardware_gui", parts=command, use_ros=True)
    print(
        "Hardware GUI started in background. If no window appears, "
        "run: ./scripts/labctl hardware gui --foreground"
    )
    return 0


def cmd_hardware_verify(args: argparse.Namespace) -> int:
    if not HARDWARE_RUNTIME_CHECK_SCRIPT.exists():
        raise LabCtlError(f"Hardware runtime check script missing: {HARDWARE_RUNTIME_CHECK_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [
        python_bin,
        str(HARDWARE_RUNTIME_CHECK_SCRIPT),
        "--timeout-s",
        str(args.timeout_s),
        "--sample-timeout-s",
        str(args.sample_timeout_s),
        "--node-name",
        str(args.node_name),
    ]
    if args.no_system_debug_sample:
        command.append("--no-system-debug-sample")

    event(
        "hardware verify "
        f"timeout_s={args.timeout_s} sample_timeout_s={args.sample_timeout_s} "
        f"node={args.node_name} sample={'off' if args.no_system_debug_sample else 'on'}"
    )
    run_command(ros_wrapped(command))
    return 0


def cmd_benchmark_pumps(args: argparse.Namespace) -> int:
    if not PUMP_SWAP_BENCH_SCRIPT.exists():
        raise LabCtlError(f"Pump benchmark script missing: {PUMP_SWAP_BENCH_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [
        python_bin,
        str(PUMP_SWAP_BENCH_SCRIPT),
        "--pump-label",
        str(args.pump_label),
        "--mode",
        str(args.mode),
        "--target-kpa",
        str(args.target_kpa),
        "--chamber",
        str(args.chamber),
        "--runs",
        str(args.runs),
        "--timeout-s",
        str(args.timeout_s),
        "--sample-ms",
        str(args.sample_ms),
        "--vent-s",
        str(args.vent_s),
        "--rest-s",
        str(args.rest_s),
        "--registry-csv",
        str(args.registry_csv),
    ]
    if args.tag:
        command.extend(["--tag", str(args.tag)])
    if args.no_registry:
        command.append("--no-registry")

    event(
        "benchmark pumps "
        f"label={args.pump_label} mode={args.mode} chamber={args.chamber} "
        f"target={args.target_kpa} runs={args.runs}"
    )
    run_command(ros_wrapped(command))
    return 0


def cmd_tuning_capture_id(args: argparse.Namespace) -> int:
    if not CONTROLLER_ID_CAPTURE_SCRIPT.exists():
        raise LabCtlError(f"Controller ID script missing: {CONTROLLER_ID_CAPTURE_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [
        python_bin,
        str(CONTROLLER_ID_CAPTURE_SCRIPT),
        "--pump-label",
        str(args.pump_label),
        "--chamber",
        str(args.chamber),
        "--pwm-levels",
        str(args.pwm_levels),
        "--step-duration-s",
        str(args.step_duration_s),
        "--sample-ms",
        str(args.sample_ms),
        "--vent-s",
        str(args.vent_s),
        "--rest-s",
        str(args.rest_s),
        "--repeats",
        str(args.repeats),
        "--directions",
        str(args.directions),
    ]
    if args.tag:
        command.extend(["--tag", str(args.tag)])
    if args.demo:
        command.append("--demo")

    event(
        "tuning capture-id "
        f"label={args.pump_label} chamber={args.chamber} repeats={args.repeats} "
        f"directions={args.directions} demo={int(bool(args.demo))}"
    )
    if args.demo:
        run_command(command)
    else:
        run_command(ros_wrapped(command))
    return 0


def cmd_tuning_root_locus(args: argparse.Namespace) -> int:
    if not ROOT_LOCUS_TUNER_SCRIPT.exists():
        raise LabCtlError(f"Root-locus tuner script missing: {ROOT_LOCUS_TUNER_SCRIPT}")

    python_bin = resolve_python_with_modules(["numpy", "matplotlib"])
    command = [
        python_bin,
        str(ROOT_LOCUS_TUNER_SCRIPT),
        "--id-csv",
        *[str(path) for path in args.id_csv],
        "--sample-ms",
        str(args.sample_ms),
        "--target-pressure-kpa",
        str(args.target_pressure_kpa),
        "--target-vacuum-kpa",
        str(args.target_vacuum_kpa),
        "--settle-band-kpa",
        str(args.settle_band_kpa),
        "--settle-hold-s",
        str(args.settle_hold_s),
        "--max-overshoot-kpa",
        str(args.max_overshoot_kpa),
        "--sim-horizon-s",
        str(args.sim_horizon_s),
        "--z0-min",
        str(args.z0_min),
        "--z0-max",
        str(args.z0_max),
        "--z0-points",
        str(args.z0_points),
        "--k-min",
        str(args.k_min),
        "--k-max",
        str(args.k_max),
        "--k-points",
        str(args.k_points),
        "--max-pole-mag",
        str(args.max_pole_mag),
        "--seed",
        str(args.seed),
    ]
    if args.reference_raw_csv:
        command.extend(["--reference-raw-csv", *[str(path) for path in args.reference_raw_csv]])
    if args.tag:
        command.extend(["--tag", str(args.tag)])

    event(
        "tuning root-locus "
        f"id_csv={len(args.id_csv)} refs={len(args.reference_raw_csv)} "
        f"target_p={args.target_pressure_kpa} target_v={args.target_vacuum_kpa}"
    )
    run_command(command)
    return 0


def cmd_tuning_validate(args: argparse.Namespace) -> int:
    if not CONTROLLER_VALIDATION_SCRIPT.exists():
        raise LabCtlError(f"Controller validation script missing: {CONTROLLER_VALIDATION_SCRIPT}")

    python_bin = resolve_repo_python()
    command = [
        python_bin,
        str(CONTROLLER_VALIDATION_SCRIPT),
        "--baseline-summary",
        str(args.baseline_summary),
        "--candidate-summary",
        str(args.candidate_summary),
        "--settle-band-kpa",
        str(args.settle_band_kpa),
        "--settle-hold-s",
        str(args.settle_hold_s),
        "--max-time-target-s",
        str(args.max_time_target_s),
        "--max-overshoot-kpa",
        str(args.max_overshoot_kpa),
        "--max-settling-s",
        str(args.max_settling_s),
    ]
    if args.baseline_raw:
        command.extend(["--baseline-raw", str(args.baseline_raw)])
    if args.candidate_raw:
        command.extend(["--candidate-raw", str(args.candidate_raw)])
    if args.tag:
        command.extend(["--tag", str(args.tag)])

    event(
        "tuning validate "
        f"baseline={args.baseline_summary} candidate={args.candidate_summary}"
    )
    run_command(command)
    return 0


def terminate_process(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return


def cmd_stop(args: argparse.Namespace) -> int:
    del args
    state = load_state()
    cleanup_stale_processes(state)

    for proc in state.get("processes", []):
        pid = int(proc.get("pid", -1))
        name = proc.get("name", "unknown")
        if pid > 0:
            terminate_process(pid)
            event(f"stop process name={name} pid={pid}")
            print(f"Stopped process {name} (pid={pid})")

    for container in state.get("containers", []):
        subprocess.run(["docker", "rm", "-f", container], check=False)
        remove_container(container)
        event(f"stop container name={container}")
        print(f"Stopped container {container}")

    state["processes"] = []
    state["containers"] = []
    save_state(state)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="labctl", description="SoftBot Lab platform CLI")
    sub = parser.add_subparsers(dest="command", required=True)

    doctor = sub.add_parser("doctor", help="Validate local environment and profile")
    doctor.add_argument("--profile", default="default")
    doctor.set_defaults(func=cmd_doctor)

    setup = sub.add_parser("setup", help="Run platform bootstrap installer")
    setup.add_argument("--online", action="store_true", help="Force online install")
    setup.add_argument("--offline", type=str, help="Install from offline bundle")
    setup.set_defaults(func=cmd_setup)

    firmware = sub.add_parser("firmware", help="Firmware build/flash commands")
    firmware_sub = firmware.add_subparsers(dest="firmware_cmd", required=True)

    fw_build = firmware_sub.add_parser("build", help="Build ESP32 firmware")
    fw_build.add_argument("--profile", default="default")
    fw_build.set_defaults(func=cmd_firmware_build)

    fw_flash = firmware_sub.add_parser("flash", help="Build and flash ESP32 firmware")
    fw_flash.add_argument("--profile", default="default")
    fw_flash.add_argument("--port", type=str, help="Override serial port")
    fw_flash.set_defaults(func=cmd_firmware_flash)

    agent = sub.add_parser("agent", help="micro-ROS agent commands")
    agent_sub = agent.add_subparsers(dest="agent_cmd", required=True)

    agent_start = agent_sub.add_parser("start", help="Start micro-ROS agent Docker container")
    agent_start.add_argument("--profile", default="default")
    agent_start.add_argument("--port", type=str, help="Override serial device")
    agent_start.add_argument("--baud", type=int, help="Override serial baud")
    agent_start.set_defaults(func=cmd_agent_start)

    gui = sub.add_parser("gui", help="GUI commands")
    gui_sub = gui.add_subparsers(dest="gui_cmd", required=True)

    gui_start = gui_sub.add_parser("start", help="Start SoftBot GUI")
    gui_start.add_argument("--foreground", action="store_true", help="Run attached to terminal")
    gui_start.set_defaults(func=cmd_gui_start)

    gui_pump_eval = gui_sub.add_parser(
        "pump-eval",
        help="Start dedicated pump evaluation GUI",
    )
    gui_pump_eval.add_argument("--foreground", action="store_true", help="Run attached to terminal")
    gui_pump_eval.set_defaults(func=cmd_gui_pump_eval)

    example = sub.add_parser("example", help="Run example scripts")
    example_sub = example.add_subparsers(dest="example_cmd", required=True)

    example_run = example_sub.add_parser("run", help="Run example script by name or path")
    example_run.add_argument("name", help="Example filename (e.g. 01_Locomocion_Gusano)")
    example_run.add_argument(
        "--foreground",
        action="store_true",
        help="Run attached to terminal instead of background",
    )
    example_run.set_defaults(func=cmd_example_run)

    smoke = sub.add_parser("smoke", help="Execute safe smoke test sequence")
    smoke.add_argument("--profile", default="default")
    smoke.add_argument("--chamber", type=int, default=7, choices=[1, 2, 3, 4, 5, 6, 7])
    smoke.add_argument("--inflate-kpa", type=float, default=5.0)
    smoke.add_argument("--hold-s", type=float, default=1.0)
    smoke.set_defaults(func=cmd_smoke)

    benchmark = sub.add_parser("benchmark", help="Benchmark routines for competition testing")
    benchmark_sub = benchmark.add_subparsers(dest="benchmark_cmd", required=True)

    bench_pumps = benchmark_sub.add_parser(
        "pumps",
        help="Benchmark pump setup speed (time to target pressure)",
    )
    bench_pumps.add_argument(
        "--pump-label",
        required=True,
        type=str,
        help="Pump setup label (examples: actuales, nuevas_v1)",
    )
    bench_pumps.add_argument(
        "--mode",
        type=str,
        choices=["pid"],
        default="pid",
        help="Control mode for the benchmark run",
    )
    bench_pumps.add_argument("--target-kpa", type=float, default=35.0)
    bench_pumps.add_argument("--chamber", type=int, default=7, choices=[1, 2, 3, 4, 5, 6, 7])
    bench_pumps.add_argument("--runs", type=int, default=5)
    bench_pumps.add_argument("--timeout-s", type=float, default=3.0)
    bench_pumps.add_argument("--sample-ms", type=int, default=20)
    bench_pumps.add_argument("--vent-s", type=float, default=0.6)
    bench_pumps.add_argument("--rest-s", type=float, default=0.4)
    bench_pumps.add_argument("--tag", type=str, default="")
    bench_pumps.add_argument(
        "--registry-csv",
        type=str,
        default="experiments/pump_benchmark_registry.csv",
    )
    bench_pumps.add_argument(
        "--no-registry",
        action="store_true",
        help="Skip writing benchmark result into cumulative registry CSV",
    )
    bench_pumps.set_defaults(func=cmd_benchmark_pumps)

    tuning = sub.add_parser("tuning", help="Controller tuning pipeline commands")
    tuning_sub = tuning.add_subparsers(dest="tuning_cmd", required=True)

    tuning_capture = tuning_sub.add_parser(
        "capture-id",
        help="Capture open-loop PWM identification dataset",
    )
    tuning_capture.add_argument("--pump-label", type=str, default="ab")
    tuning_capture.add_argument("--chamber", type=int, default=3, choices=[1, 2, 3, 4, 5, 6, 7])
    tuning_capture.add_argument("--pwm-levels", type=str, default="80,120,160,200")
    tuning_capture.add_argument("--step-duration-s", type=float, default=2.5)
    tuning_capture.add_argument("--sample-ms", type=int, default=20)
    tuning_capture.add_argument("--vent-s", type=float, default=0.6)
    tuning_capture.add_argument("--rest-s", type=float, default=0.4)
    tuning_capture.add_argument("--repeats", type=int, default=3)
    tuning_capture.add_argument("--directions", type=str, default="inflate,suction")
    tuning_capture.add_argument("--tag", type=str, default="ab")
    tuning_capture.add_argument("--demo", action="store_true")
    tuning_capture.set_defaults(func=cmd_tuning_capture_id)

    tuning_root = tuning_sub.add_parser(
        "root-locus",
        help="Fit ARX models and propose PI gains via discrete root locus",
    )
    tuning_root.add_argument("--id-csv", nargs="+", required=True)
    tuning_root.add_argument("--reference-raw-csv", nargs="*", default=[])
    tuning_root.add_argument("--sample-ms", type=int, default=20)
    tuning_root.add_argument("--target-pressure-kpa", type=float, default=28.0)
    tuning_root.add_argument("--target-vacuum-kpa", type=float, default=-15.0)
    tuning_root.add_argument("--settle-band-kpa", type=float, default=1.0)
    tuning_root.add_argument("--settle-hold-s", type=float, default=0.4)
    tuning_root.add_argument("--max-overshoot-kpa", type=float, default=2.0)
    tuning_root.add_argument("--sim-horizon-s", type=float, default=3.0)
    tuning_root.add_argument("--z0-min", type=float, default=0.60)
    tuning_root.add_argument("--z0-max", type=float, default=0.98)
    tuning_root.add_argument("--z0-points", type=int, default=40)
    tuning_root.add_argument("--k-min", type=float, default=1e-2)
    tuning_root.add_argument("--k-max", type=float, default=1e4)
    tuning_root.add_argument("--k-points", type=int, default=120)
    tuning_root.add_argument("--max-pole-mag", type=float, default=0.995)
    tuning_root.add_argument("--seed", type=int, default=7)
    tuning_root.add_argument("--tag", type=str, default="ab")
    tuning_root.set_defaults(func=cmd_tuning_root_locus)

    tuning_validate = tuning_sub.add_parser(
        "validate",
        help="Generate baseline vs candidate validation report",
    )
    tuning_validate.add_argument("--baseline-summary", required=True)
    tuning_validate.add_argument("--candidate-summary", required=True)
    tuning_validate.add_argument("--baseline-raw", type=str, default="")
    tuning_validate.add_argument("--candidate-raw", type=str, default="")
    tuning_validate.add_argument("--settle-band-kpa", type=float, default=1.0)
    tuning_validate.add_argument("--settle-hold-s", type=float, default=0.4)
    tuning_validate.add_argument("--max-time-target-s", type=float, default=0.35)
    tuning_validate.add_argument("--max-overshoot-kpa", type=float, default=2.0)
    tuning_validate.add_argument("--max-settling-s", type=float, default=1.0)
    tuning_validate.add_argument("--tag", type=str, default="ab")
    tuning_validate.set_defaults(func=cmd_tuning_validate)

    hardware = sub.add_parser("hardware", help="Hardware component diagnostics")
    hardware_sub = hardware.add_subparsers(dest="hardware_cmd", required=True)

    hw_test = hardware_sub.add_parser("test", help="Run timed test for one hardware component")
    hw_test.add_argument(
        "--component",
        required=True,
        choices=[
            "inflate_main",
            "inflate_aux",
            "suction_main",
            "suction_aux",
            "valve_inflate",
            "valve_suction",
            "valve_chamber_c",
            "mux_a",
            "mux_b",
        ],
    )
    hw_test.add_argument("--pwm", type=int, default=120)
    hw_test.add_argument("--duration-s", type=float, default=1.0)
    hw_test.add_argument("--repeat", type=int, default=1)
    hw_test.set_defaults(func=cmd_hardware_test)

    hw_panel = hardware_sub.add_parser("panel", help="Interactive hardware diagnostics panel")
    hw_panel.add_argument("--pwm", type=int, default=120)
    hw_panel.set_defaults(func=cmd_hardware_panel)

    hw_off = hardware_sub.add_parser("off", help="Force all hardware outputs OFF")
    hw_off.set_defaults(func=cmd_hardware_off)

    hw_gui = hardware_sub.add_parser("gui", help="Start dedicated MOSFET hardware diagnostics GUI")
    hw_gui.add_argument("--foreground", action="store_true", help="Run attached to terminal")
    hw_gui.set_defaults(func=cmd_hardware_gui)

    hw_verify = hardware_sub.add_parser(
        "verify",
        help="Verify firmware ROS node, command/telemetry topics, and /system_debug traffic",
    )
    hw_verify.add_argument("--timeout-s", type=float, default=8.0)
    hw_verify.add_argument("--sample-timeout-s", type=float, default=3.0)
    hw_verify.add_argument("--node-name", type=str, default="soft_robot_node")
    hw_verify.add_argument(
        "--no-system-debug-sample",
        action="store_true",
        help="Skip waiting for a /system_debug message payload",
    )
    hw_verify.set_defaults(func=cmd_hardware_verify)

    stop = sub.add_parser("stop", help="Stop processes/containers started via labctl")
    stop.set_defaults(func=cmd_stop)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        return int(args.func(args))
    except LabCtlError as exc:
        print(f"labctl error: {exc}", file=sys.stderr)
        event(f"error {exc}")
        return 1
    except subprocess.CalledProcessError as exc:
        print(f"Command failed with code {exc.returncode}: {exc.cmd}", file=sys.stderr)
        event(f"subprocess_failed code={exc.returncode} cmd={exc.cmd}")
        return exc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
