#!/usr/bin/env python3
"""Correlate retained multi-run ARES RTF results with host instrumentation."""

import argparse
import json
import os
from pathlib import Path
import re
import statistics


parser = argparse.ArgumentParser()
parser.add_argument("--diagnostic", required=True)
parser.add_argument("--output", required=True)
args = parser.parse_args()

root = Path(__file__).resolve().parents[1]
source = (root / args.diagnostic).resolve()
output = (root / args.output).resolve()
output.parent.mkdir(parents=True, exist_ok=True)
summary = json.loads((source / "summary.json").read_text())
samples = [
    json.loads(line)
    for line in (source / "samples.jsonl").read_text().splitlines()
    if line.strip()
]
clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])


def number_series(values):
    values = [value for value in values if value is not None]
    return {
        "min": min(values) if values else None,
        "mean": statistics.mean(values) if values else None,
        "max": max(values) if values else None,
    }


def psi_avg10(text, kind="some"):
    match = re.search(rf"^{kind} avg10=([0-9.]+)", text or "", re.MULTILINE)
    return float(match.group(1)) if match else None


def meminfo(text, key):
    match = re.search(rf"^{re.escape(key)}:\s+(\d+)", text or "", re.MULTILINE)
    return int(match.group(1)) if match else None


def process_cpu(run_samples, token):
    observations = {}
    for sample in run_samples:
        for process in sample.get("processes", []):
            if token not in process.get("cmdline", ""):
                continue
            ticks = process.get("cpu_ticks")
            if not ticks:
                continue
            observations.setdefault(process["pid"], []).append((
                sample["monotonic"], ticks["utime"] + ticks["stime"]
            ))
    total = 0.0
    found = False
    for values in observations.values():
        if len(values) < 2:
            continue
        elapsed = values[-1][0] - values[0][0]
        if elapsed <= 0:
            continue
        total += (values[-1][1] - values[0][1]) / clock_ticks / elapsed * 100.0
        found = True
    return total if found else None


def activation_seconds(launch_path):
    text = launch_path.read_text(errors="replace")
    stamps = [float(value) for value in re.findall(r"\[INFO\] \[([0-9]+\.[0-9]+)\]", text)]
    active = re.search(
        r"\[INFO\] \[([0-9]+\.[0-9]+)\].*Managed nodes are active", text
    )
    return float(active.group(1)) - min(stamps) if stamps and active else None


rows = []
for index, result in enumerate(summary.get("run_results", []), start=1):
    metrics_path = root / result["metrics_path"]
    run_name = metrics_path.parent.name
    run_samples = []
    for sample in samples:
        processes = sample.get("processes", [])
        if any(run_name in process.get("cmdline", "") for process in processes):
            run_samples.append(sample)
    runnable = []
    load_one = []
    for sample in run_samples:
        fields = (sample.get("loadavg") or "").split()
        if fields:
            load_one.append(float(fields[0]))
        if len(fields) >= 4 and "/" in fields[3]:
            runnable.append(int(fields[3].split("/", 1)[0]))
    swap_used = []
    for sample in run_samples:
        total = meminfo(sample.get("meminfo"), "SwapTotal")
        free = meminfo(sample.get("meminfo"), "SwapFree")
        if total is not None and free is not None:
            swap_used.append(total - free)
    launch_path = metrics_path.parent / "launch.log"
    rows.append({
        "run": index,
        "run_name": run_name,
        "effective_rtf": result.get("effective_rtf"),
        "success": result.get("success"),
        "sample_count": len(run_samples),
        "load_one": number_series(load_one),
        "runnable_tasks": number_series(runnable),
        "cpu_psi_some_avg10": number_series([
            psi_avg10(sample.get("pressure", {}).get("cpu")) for sample in run_samples
        ]),
        "memory_psi_some_avg10": number_series([
            psi_avg10(sample.get("pressure", {}).get("memory")) for sample in run_samples
        ]),
        "memory_available_kib": number_series([
            meminfo(sample.get("meminfo"), "MemAvailable") for sample in run_samples
        ]),
        "swap_used_kib": number_series(swap_used),
        "gazebo_cpu_percent": process_cpu(run_samples, "gz-sim-main"),
        "bridge_cpu_percent": process_cpu(run_samples, "/ros_gz_bridge/parameter_bridge"),
        "agent_proxy_cpu_percent": process_cpu(run_samples, "headroom.cli proxy --port"),
        "bag_cpu_percent": process_cpu(run_samples, "ros2 bag record"),
        "nav2_activation_wall_s": activation_seconds(launch_path),
    })

low = min(rows, key=lambda row: row["effective_rtf"])
nearest_high = min(
    (row for row in rows if row["effective_rtf"] > 0.7),
    key=lambda row: abs(row["run"] - low["run"]),
)
analysis = {
    "source": str(source.relative_to(root)),
    "rows": rows,
    "lowest_rtf_run": low,
    "nearest_high_rtf_run": nearest_high,
    "not_retained": [
        "DDS discovery completion timestamps",
        "point-in-time scan subscription count",
        "point-in-time ROS graph size",
        "Windows host scheduler and GPU load outside Linux",
    ],
}
output.write_text(json.dumps(analysis, indent=2) + "\n")

report = output.with_suffix(".md")
report.write_text(
    "# Retained 20-run RTF variability analysis\n\n"
    f"The lowest run was {low['run']} at RTF {low['effective_rtf']:.3f}; the "
    f"nearest >0.7 run was {nearest_high['run']} at RTF "
    f"{nearest_high['effective_rtf']:.3f}.\n\n"
    "| Run | RTF | load1 mean | runnable mean/max | CPU PSI avg10 mean/max | "
    "Gazebo CPU | bridge CPU | proxy CPU | Nav2 activation wall s |\n"
    "|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n"
    + "\n".join(
        "| {run} | {rtf:.3f} | {load:.2f} | {runmean:.1f}/{runmax:.0f} | "
        "{psimean:.2f}/{psimax:.2f} | {gz:.1f} | {bridge:.1f} | {proxy:.1f} | "
        "{activation:.2f} |".format(
            run=row["run"], rtf=row["effective_rtf"],
            load=row["load_one"]["mean"] or 0.0,
            runmean=row["runnable_tasks"]["mean"] or 0.0,
            runmax=row["runnable_tasks"]["max"] or 0.0,
            psimean=row["cpu_psi_some_avg10"]["mean"] or 0.0,
            psimax=row["cpu_psi_some_avg10"]["max"] or 0.0,
            gz=row["gazebo_cpu_percent"] or 0.0,
            bridge=row["bridge_cpu_percent"] or 0.0,
            proxy=row["agent_proxy_cpu_percent"] or 0.0,
            activation=row["nav2_activation_wall_s"] or 0.0,
        ) for row in (low, nearest_high)
    )
    + "\n\nThe low run is retained as a real failure. Fields that were not captured by "
    "the original suite are listed explicitly in the JSON and are not inferred.\n"
)
print(json.dumps({"output": str(output), "low": low, "nearest_high": nearest_high}))
