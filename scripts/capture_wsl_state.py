#!/usr/bin/env python3
"""Capture read-only WSL/host resource state for ARES experiments."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import time


parser = argparse.ArgumentParser()
parser.add_argument("--output", required=True)
args = parser.parse_args()

root = Path(__file__).resolve().parents[1]
output = (root / args.output).resolve()
output.mkdir(parents=True, exist_ok=True)


def run(command):
    result = subprocess.run(
        command, cwd=root, text=True, capture_output=True, errors="replace"
    )
    return {
        "command": command,
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


def read(path):
    try:
        return Path(path).read_text(errors="replace")
    except (OSError, PermissionError) as exc:
        return f"UNAVAILABLE: {exc!r}\n"


def process_snapshot():
    rows = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                errors="replace"
            )
            status = {}
            for line in (entry / "status").read_text(errors="replace").splitlines():
                if ":" not in line:
                    continue
                key, value = line.split(":", 1)
                if key in {
                    "Name",
                    "State",
                    "Threads",
                    "VmRSS",
                    "VmSize",
                    "Cpus_allowed_list",
                    "voluntary_ctxt_switches",
                    "nonvoluntary_ctxt_switches",
                }:
                    status[key] = value.strip()
            rows.append({"pid": int(entry.name), "cmdline": cmdline, "status": status})
        except (OSError, PermissionError, ProcessLookupError):
            continue
    return sorted(rows, key=lambda item: item["pid"])


graphics_keys = (
    "DISPLAY",
    "WAYLAND",
    "MESA",
    "LIBGL",
    "GALLIUM",
    "WLR",
    "QT",
    "OGRE",
    "GZ",
)

commands = {
    "uname": run(["uname", "-a"]),
    "nproc": run(["nproc"]),
    "lscpu": run(["lscpu"]),
    "free": run(["free", "-h"]),
    "df": run(["df", "-h"]),
    "uptime": run(["uptime"]),
    "glxinfo": run(["glxinfo", "-B"]),
    "vmstat": run(["vmstat", "1", "5"]),
    "ps": run(
        [
            "ps",
            "-eo",
            "pid=,ppid=,psr=,ni=,cls=,pri=,stat=,pcpu=,pmem=,rss=,vsz=,nlwp=,etimes=,comm=,args=",
            "--sort=-pcpu",
        ]
    ),
    "gazebo_version": run(["gz", "sim", "--versions"]),
    "ros_environment": run(["bash", "-lc", "source /opt/ros/lyrical/setup.bash && env"]),
}

state = {
    "captured_wall_epoch": time.time(),
    "captured_local": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    "cwd": str(root),
    "pid": os.getpid(),
    "collector_affinity": sorted(os.sched_getaffinity(0)),
    "commands": commands,
    "proc_version": read("/proc/version"),
    "proc_meminfo": read("/proc/meminfo"),
    "proc_loadavg": read("/proc/loadavg"),
    "proc_pressure": {
        name: read(f"/proc/pressure/{name}") for name in ("cpu", "memory", "io")
    },
    "proc_swaps": read("/proc/swaps"),
    "cgroup_cpu_max": read("/sys/fs/cgroup/cpu.max"),
    "cgroup_memory_pressure": read("/sys/fs/cgroup/memory.pressure"),
    "cpu_online": read("/sys/devices/system/cpu/online"),
    "cpu_possible": read("/sys/devices/system/cpu/possible"),
    "cpu_frequencies": {
        str(path): read(path)
        for path in sorted(
            Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/*freq*")
        )
    },
    "graphics_environment": {
        key: value
        for key, value in sorted(os.environ.items())
        if any(token in key for token in graphics_keys)
    },
    "processes": process_snapshot(),
}

ares_tokens = (
    "gz sim",
    "gz-sim",
    "parameter_bridge",
    "robot_state_publisher",
    "ekf_node",
    "map_server",
    "amcl",
    "planner_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
    "lifecycle_manager",
    "cmd_vel_adapter",
    "ros2 bag",
    "health_check",
    "healthy_mission",
)
state["stale_ares_processes"] = [
    process
    for process in state["processes"]
    if any(token in process["cmdline"] for token in ares_tokens)
    and process["pid"] != os.getpid()
]

(output / "wsl_state_before.json").write_text(json.dumps(state, indent=2) + "\n")

with (output / "wsl_state_before.txt").open("w") as stream:
    stream.write(f"captured_local: {state['captured_local']}\n")
    stream.write(f"collector_affinity: {state['collector_affinity']}\n")
    stream.write(f"stale_ares_process_count: {len(state['stale_ares_processes'])}\n\n")
    stream.write("/proc/version\n" + state["proc_version"] + "\n")
    for name in (
        "uname",
        "nproc",
        "lscpu",
        "free",
        "df",
        "uptime",
        "vmstat",
        "glxinfo",
        "gazebo_version",
    ):
        item = commands[name]
        stream.write(f"[{name}] rc={item['returncode']}\n")
        stream.write(item["stdout"])
        stream.write(item["stderr"])
        stream.write("\n")
    stream.write("[/proc/loadavg]\n" + state["proc_loadavg"] + "\n")
    stream.write("[/proc/meminfo]\n" + state["proc_meminfo"] + "\n")
    stream.write("[/proc/swaps]\n" + state["proc_swaps"] + "\n")
    for name, value in state["proc_pressure"].items():
        stream.write(f"[/proc/pressure/{name}]\n{value}\n")
    stream.write("[graphics environment]\n")
    for key, value in state["graphics_environment"].items():
        stream.write(f"{key}={value}\n")
    stream.write("\n[stale ARES processes]\n")
    for process in state["stale_ares_processes"]:
        stream.write(json.dumps(process, sort_keys=True) + "\n")
    stream.write("\n[ps]\n" + commands["ps"]["stdout"])

print(
    json.dumps(
        {
            "output": str(output),
            "captured_local": state["captured_local"],
            "stale_ares_process_count": len(state["stale_ares_processes"]),
            "collector_affinity": state["collector_affinity"],
        }
    )
)
