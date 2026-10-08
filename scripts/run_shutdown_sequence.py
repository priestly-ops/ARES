#!/usr/bin/env python3
"""Run one minimal ARES GPU-LiDAR shutdown-order experiment."""

import argparse
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
TOKENS = ("gz sim -", "gz-sim-main", "/ros_gz_bridge/parameter_bridge")


def process_conflicts():
    rows = subprocess.check_output(["ps", "-eo", "pid,args"], text=True).splitlines()
    own_pid = os.getpid()
    return [
        row
        for row in rows
        if str(own_pid) not in row and any(token in row for token in TOKENS)
    ]


def dmesg_lines():
    result = subprocess.run(["dmesg", "--ctime"], text=True, capture_output=True)
    return result.stdout.splitlines(), result.stderr, result.returncode


def wait_exit(process, timeout=15):
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        return None


def tail(path, lines=50):
    if not path.exists():
        return []
    return path.read_text(errors="replace").splitlines()[-lines:]


def sensor_profile(model):
    sensor = model.find(".//sensor[@name='ares_lidar']")
    horizontal = sensor.find("lidar/scan/horizontal")
    lidar_range = sensor.find("lidar/range")
    noise = sensor.find("lidar/noise")
    return {
        "type": sensor.get("type"),
        "update_rate_hz": float(sensor.findtext("update_rate")),
        "samples": int(horizontal.findtext("samples")),
        "resolution": float(horizontal.findtext("resolution")),
        "min_angle_rad": float(horizontal.findtext("min_angle")),
        "max_angle_rad": float(horizontal.findtext("max_angle")),
        "range_min_m": float(lidar_range.findtext("min")),
        "range_max_m": float(lidar_range.findtext("max")),
        "range_resolution_m": float(lidar_range.findtext("resolution")),
        "noise_type": noise.findtext("type"),
        "noise_mean": float(noise.findtext("mean")),
        "noise_stddev": float(noise.findtext("stddev")),
        "topic": sensor.findtext("topic"),
        "frame": sensor.findtext("frame_id"),
        "always_on": sensor.findtext("always_on"),
    }


parser = argparse.ArgumentParser()
parser.add_argument("--sequence", required=True, choices=tuple("ABCDEF"))
parser.add_argument("--name", required=True)
parser.add_argument("--lazy-bridge", action="store_true")
parser.add_argument("--subscriber-driven-lidar", action="store_true")
parser.add_argument("--software-rendering", action="store_true")
parser.add_argument("--gallium-driver", choices=("llvmpipe",))
parser.add_argument("--capture-bridge-core", action="store_true")
parser.add_argument(
    "--full-bridge-config",
    action="store_true",
    help="Use the unmodified Week 1 bridge_baseline.yaml instead of scan-only",
)
parser.add_argument(
    "--consumer-settle-s",
    type=float,
    default=5.0,
    help="Sequence B wait after the ROS scan subscriber exits",
)
parser.add_argument(
    "--preload-rmw",
    action="store_true",
    help="Preload the selected Fast DDS RMW only in parameter_bridge",
)
parser.add_argument(
    "--fastdds-builtin-transports",
    choices=("DEFAULT", "UDPv4"),
    help="Controlled Fast DDS transport diagnostic for parameter_bridge",
)
parser.add_argument(
    "--output-root", default="results/week1/wsl_shutdown_order_20260924"
)
args = parser.parse_args()

out = ROOT / args.output_root / args.name
out.mkdir(parents=True, exist_ok=False)

before_conflicts = process_conflicts()
if before_conflicts:
    raise RuntimeError(f"Contaminated baseline: {before_conflicts!r}")

core_limit_before = resource.getrlimit(resource.RLIMIT_CORE)
resource.setrlimit(resource.RLIMIT_CORE, (0, core_limit_before[1]))
core_limit_during = resource.getrlimit(resource.RLIMIT_CORE)
before_dmesg, before_dmesg_stderr, before_dmesg_rc = dmesg_lines()

env = os.environ.copy()
env.update(
    ROS_DOMAIN_ID=str(120 + os.getpid() % 80),
    GZ_PARTITION="ares_shutdown_sequence_" + str(os.getpid()),
    ROS_LOG_DIR=str(out / "ros_logs"),
)
if args.software_rendering:
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
if args.gallium_driver:
    env["GALLIUM_DRIVER"] = args.gallium_driver

bridge_env = env.copy()
if args.preload_rmw:
    bridge_env["LD_PRELOAD"] = "/opt/ros/lyrical/lib/librmw_fastrtps_cpp.so"
if args.fastdds_builtin_transports:
    bridge_env["FASTDDS_BUILTIN_TRANSPORTS"] = args.fastdds_builtin_transports

renderer_probe = subprocess.run(
    ["glxinfo", "-B"], env=env, text=True, capture_output=True
)

world_tree = ET.parse(ROOT / "src/ares_simulation/worlds/ares_test_world.sdf")
world = world_tree.getroot().find("world")
for actor in world.findall("actor"):
    world.remove(actor)
include = world.find("include")
model = ET.parse(
    ROOT / "src/ares_simulation/models/ares_jackal/model.sdf"
).getroot().find("model")
model.find("pose").text = include.findtext("pose")
world.remove(include)
camera_link = model.find("link[@name='camera_link']")
for camera_sensor in camera_link.findall("sensor"):
    camera_link.remove(camera_sensor)
if args.subscriber_driven_lidar:
    model.find(".//sensor[@name='ares_lidar']/always_on").text = "false"
profile = sensor_profile(model)
expected_profile = {
    "type": "gpu_lidar",
    "update_rate_hz": 5.0,
    "samples": 360,
    "resolution": 1.0,
    "min_angle_rad": -3.141592653589793,
    "max_angle_rad": 3.141592653589793,
    "range_min_m": 0.12,
    "range_max_m": 20.0,
    "range_resolution_m": 0.01,
    "noise_type": "gaussian",
    "noise_mean": 0.0,
    "noise_stddev": 0.01,
    "topic": "/ares/scan",
    "frame": "lidar_link",
    "always_on": "false" if args.subscriber_driven_lidar else "true",
}
for key, expected in expected_profile.items():
    actual = profile[key]
    if isinstance(expected, float):
        matches = abs(actual - expected) <= 1e-11
    else:
        matches = actual == expected
    if not matches:
        raise RuntimeError(
            f"LiDAR profile mismatch for {key}: expected {expected!r}, got {actual!r}"
        )
world.append(model)
world_path = out / "world.sdf"
world_tree.write(world_path, encoding="unicode")

bridge_source_path = ROOT / "src/ares_simulation/config/bridge_baseline.yaml"
bridge_source = yaml.safe_load(bridge_source_path.read_text())
scan_bridge = [
    item for item in bridge_source if item.get("ros_topic_name") == "/ares/scan"
]
if len(scan_bridge) != 1:
    raise RuntimeError(f"Expected one /ares/scan bridge entry, found {scan_bridge!r}")
if args.lazy_bridge:
    scan_bridge[0]["lazy"] = True
bridge_path = out / "bridge_scan_only.yaml"
bridge_config = bridge_source if args.full_bridge_config else scan_bridge
if args.full_bridge_config:
    bridge_path.write_bytes(bridge_source_path.read_bytes())
else:
    bridge_path.write_text(yaml.safe_dump(bridge_config, sort_keys=False))

processes = {}
logs = {}
timeline = []
forced_cleanup = []


def start(name, command, process_env=None):
    log_path = out / f"{name}.log"
    log = log_path.open("w")
    logs[name] = log
    process = subprocess.Popen(
        command,
        cwd=out if args.capture_bridge_core and name == "bridge" else ROOT,
        env=process_env or env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        preexec_fn=(
            lambda: resource.setrlimit(
                resource.RLIMIT_CORE,
                (resource.RLIM_INFINITY, resource.RLIM_INFINITY),
            )
        )
        if args.capture_bridge_core and name == "bridge"
        else None,
    )
    processes[name] = process
    timeline.append(
        {"event": "start", "process": name, "pid": process.pid, "wall": time.time()}
    )
    return process


def signal_and_wait(name, sig, delay_after=0.0):
    process = processes[name]
    if process.poll() is None:
        os.killpg(process.pid, sig)
        timeline.append(
            {
                "event": "signal",
                "process": name,
                "signal": signal.Signals(sig).name,
                "wall": time.time(),
            }
        )
    code = wait_exit(process)
    timeline.append(
        {"event": "exit", "process": name, "exit_code": code, "wall": time.time()}
    )
    if delay_after:
        timeline.append(
            {
                "event": "stabilization_wait",
                "after": name,
                "seconds": delay_after,
                "wall": time.time(),
            }
        )
        time.sleep(delay_after)


def simultaneous(sig):
    for name in ("scan_subscriber", "bridge", "gazebo"):
        process = processes[name]
        if process.poll() is None:
            os.killpg(process.pid, sig)
            timeline.append(
                {
                    "event": "signal",
                    "process": name,
                    "signal": signal.Signals(sig).name,
                    "wall": time.time(),
                }
            )
    for name in ("scan_subscriber", "bridge", "gazebo"):
        code = wait_exit(processes[name])
        timeline.append(
            {"event": "exit", "process": name, "exit_code": code, "wall": time.time()}
        )


try:
    gazebo = start(
        "gazebo", ["gz", "sim", "-s", "-r", "--seed", "42", str(world_path)]
    )
    time.sleep(3)
    bridge = start(
        "bridge",
        [
            "/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge",
            "--ros-args",
            "-r",
            "__node:=ros_gz_bridge",
            "-p",
            "config_file:=" + str(bridge_path),
            "-p",
            "use_sim_time:=true",
        ],
        bridge_env,
    )
    time.sleep(3)
    subscriber = start(
        "scan_subscriber", ["ros2", "topic", "hz", "/ares/scan", "--window", "20"]
    )
    time.sleep(8)

    startup = {
        name: {"pid": process.pid, "poll": process.poll()}
        for name, process in processes.items()
    }
    if any(item["poll"] is not None for item in startup.values()):
        raise RuntimeError(f"Process exited before sequence: {startup!r}")
    for name, process in processes.items():
        try:
            (out / f"{name}.maps").write_text(
                Path(f"/proc/{process.pid}/maps").read_text(errors="replace")
            )
        except (OSError, PermissionError, ProcessLookupError) as exc:
            (out / f"{name}.maps").write_text(f"UNAVAILABLE: {exc!r}\n")

    scan_log_before = tail(out / "scan_subscriber.log", 200)
    scan_received = any("average rate:" in line for line in scan_log_before)
    if not scan_received:
        raise RuntimeError("No scan-rate evidence before shutdown sequence")

    if args.sequence == "A":
        # Stop all ROS-side scan consumption, then the bridge, then Gazebo.
        signal_and_wait("scan_subscriber", signal.SIGINT)
        signal_and_wait("bridge", signal.SIGINT, 2.0)
        signal_and_wait("gazebo", signal.SIGINT)
    elif args.sequence == "B":
        # Give subscriber removal and bridge removal separate settling windows.
        signal_and_wait("scan_subscriber", signal.SIGINT)
        timeline.append(
            {
                "event": "sensor_deactivation_wait",
                "seconds": args.consumer_settle_s,
                "wall": time.time(),
            }
        )
        time.sleep(args.consumer_settle_s)
        topic_info = subprocess.run(
            ["gz", "topic", "-i", "-t", "/ares/scan"],
            cwd=ROOT,
            env=env,
            text=True,
            capture_output=True,
        )
        timeline.append(
            {
                "event": "gz_topic_info_after_consumer_removal",
                "returncode": topic_info.returncode,
                "stdout": topic_info.stdout,
                "stderr": topic_info.stderr,
                "wall": time.time(),
            }
        )
        signal_and_wait("bridge", signal.SIGINT, 3.0)
        signal_and_wait("gazebo", signal.SIGINT)
    elif args.sequence == "C":
        signal_and_wait("bridge", signal.SIGINT, 2.0)
        signal_and_wait("gazebo", signal.SIGINT)
        signal_and_wait("scan_subscriber", signal.SIGINT)
    elif args.sequence == "D":
        signal_and_wait("gazebo", signal.SIGINT, 2.0)
        signal_and_wait("bridge", signal.SIGINT)
        signal_and_wait("scan_subscriber", signal.SIGINT)
    elif args.sequence == "E":
        simultaneous(signal.SIGINT)
    else:
        simultaneous(signal.SIGTERM)
finally:
    for name in ("scan_subscriber", "bridge", "gazebo"):
        process = processes.get(name)
        if process is None or process.poll() is not None:
            continue
        os.killpg(process.pid, signal.SIGTERM)
        code = wait_exit(process, timeout=5)
        forced_cleanup.append(
            {"process": name, "signal": "SIGTERM", "exit_code": code}
        )
        if code is None:
            os.killpg(process.pid, signal.SIGKILL)
            code = process.wait()
            forced_cleanup.append(
                {"process": name, "signal": "SIGKILL", "exit_code": code}
            )
    for log in logs.values():
        log.close()

after_dmesg, after_dmesg_stderr, after_dmesg_rc = dmesg_lines()
new_dmesg = (
    after_dmesg[len(before_dmesg) :]
    if after_dmesg[: len(before_dmesg)] == before_dmesg
    else []
)
after_conflicts = process_conflicts()
exit_codes = {name: process.returncode for name, process in processes.items()}
clean = (
    all(code == 0 for code in exit_codes.values())
    and not forced_cleanup
    and not after_conflicts
    and not any(code == -11 for code in exit_codes.values())
)
summary = {
    "schema_version": 1,
    "name": args.name,
    "sequence": args.sequence,
    "sensor_profile": profile,
    "lazy_bridge": args.lazy_bridge,
    "full_bridge_config": args.full_bridge_config,
    "consumer_settle_s": args.consumer_settle_s,
    "preload_rmw": args.preload_rmw,
    "fastdds_builtin_transports": args.fastdds_builtin_transports,
    "subscriber_driven_lidar": args.subscriber_driven_lidar,
    "software_rendering": args.software_rendering,
    "gallium_driver_override": args.gallium_driver,
    "capture_bridge_core": args.capture_bridge_core,
    "renderer_environment": {
        key: env[key]
        for key in ("LIBGL_ALWAYS_SOFTWARE", "GALLIUM_DRIVER", "MESA_LOADER_DRIVER_OVERRIDE")
        if key in env
    },
    "renderer_probe": {
        "command": "glxinfo -B",
        "returncode": renderer_probe.returncode,
        "stdout": renderer_probe.stdout,
        "stderr": renderer_probe.stderr,
    },
    "ros_domain_id": env["ROS_DOMAIN_ID"],
    "gz_partition": env["GZ_PARTITION"],
    "rmw_implementation": bridge_env.get("RMW_IMPLEMENTATION"),
    "bridge_environment": {
        key: bridge_env[key]
        for key in (
            "ROS_DOMAIN_ID", "GZ_PARTITION", "RMW_IMPLEMENTATION",
            "FASTDDS_BUILTIN_TRANSPORTS", "FASTRTPS_DEFAULT_PROFILES_FILE",
            "LD_PRELOAD",
        )
        if key in bridge_env
    },
    "core_limit_before": list(core_limit_before),
    "core_limit_during": list(core_limit_during),
    "scan_received": scan_received,
    "startup": startup,
    "timeline": timeline,
    "exit_codes": exit_codes,
    "forced_cleanup": forced_cleanup,
    "stale_processes_after": after_conflicts,
    "clean_shutdown": clean,
    "crash": any(code == -11 for code in exit_codes.values()),
    "dmesg_before_returncode": before_dmesg_rc,
    "dmesg_before_stderr": before_dmesg_stderr,
    "dmesg_after_returncode": after_dmesg_rc,
    "dmesg_after_stderr": after_dmesg_stderr,
    "dmesg_new": new_dmesg,
    "log_tails": {
        name: tail(out / f"{name}.log") for name in processes
    },
}
(out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps({
    "name": args.name,
    "sequence": args.sequence,
    "exit_codes": exit_codes,
    "clean_shutdown": clean,
    "crash": summary["crash"],
    "forced_cleanup": forced_cleanup,
    "stale_processes_after": after_conflicts,
}))
