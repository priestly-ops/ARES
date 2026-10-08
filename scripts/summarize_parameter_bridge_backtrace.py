#!/usr/bin/env python3
"""Summarize the captured parameter_bridge teardown core."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = (
    ROOT
    / "results/week1/wsl_teardown_crash_debug_20260923"
    / "parameter_bridge_core_20260924_0725"
)
CORE = OUT / "core.56753"
GDB = OUT / "gdb_backtrace.txt"
SOURCE = (
    ROOT
    / "results/week1/diagnostics/phase4_parameter_bridge_core_health_002"
    / "summary.json"
)

gdb_text = GDB.read_text(errors="replace")
source = json.loads(SOURCE.read_text())
fact = {
    "schema_version": 1,
    "executable": "/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge",
    "signal": "SIGSEGV",
    "exit_code": -11,
    "crashing_thread": {
        "core_lwp": 56919,
        "kernel_comm": "dds.shm.37175",
        "program_counter": "0x7f4945bb774f",
    },
    "top_frames": [
        {"frame": 0, "address": "0x7f4945bb774f", "symbol": None},
        {"frame": 1, "address": "0x7f4945d89c47", "symbol": None},
        {"frame": 2, "address": "0x7f493d7f90e0", "symbol": None},
    ],
    "evidence": {
        "core": str(CORE.relative_to(ROOT)),
        "gdb_backtrace": str(GDB.relative_to(ROOT)),
        "source_summary": str(SOURCE.relative_to(ROOT)),
        "core_size_bytes": CORE.stat().st_size,
        "gdb_lists_ros_gz_bridge": "libros_gz_bridge.so" in gdb_text,
        "gdb_lists_rclcpp": "librclcpp.so" in gdb_text,
        "gdb_lists_rmw_loader": "librmw_implementation.so" in gdb_text,
        "gdb_lists_fastdds_runtime": "libfastdds.so" in gdb_text,
        "gdb_lists_rmw_fastrtps_runtime": "librmw_fastrtps" in gdb_text,
        "fault_pc_has_loaded_mapping": False,
        "fault_pc_disassembly_available": False,
        "process_exit_codes": source.get("process_exit_codes"),
        "launch_child_exit_codes": source.get("launch_child_exit_codes"),
    },
    "classification": {
        "proven": [
            "parameter_bridge received SIGSEGV and exited -11 during its dedicated SIGINT shutdown stage.",
            "The crashing kernel thread was named dds.shm.37175; core LWP 56919 stopped at 0x7f4945bb774f.",
            "The fault PC and its immediate caller are absent from the surviving loaded-object mappings, and GDB cannot disassemble the PC.",
            "The core retains Fast DDS /dev/shm mappings, but neither libfastdds.so nor librmw_fastrtps*.so remains in GDB's post-fault shared-library list.",
            "Gazebo exited cleanly in the same llvmpipe trial; the failing child was parameter_bridge only.",
        ],
        "likely": [
            "A Fast DDS/RMW teardown lifetime race allowed a DDS worker to execute after its runtime code or callback target had been unloaded.",
            "This is distinct from the Gazebo D3D12 teardown fault and is not a libgcc_s root cause.",
        ],
        "unknown": [
            "The exact Fast DDS/RMW function because the crashing PC belongs to an unmapped region and the surviving stack is corrupted beyond frame 1.",
        ],
    },
}
(OUT / "backtrace.json").write_text(json.dumps(fact, indent=2) + "\n")

report = """# parameter_bridge Native Teardown Backtrace

## Result

`parameter_bridge` received `SIGSEGV` and exited `-11` during its dedicated SIGINT shutdown stage. Gazebo exited cleanly in the same llvmpipe trial.

## PROVEN

- Executable: `/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge`.
- Crashing thread: core LWP `56919`, reported by the kernel as `dds.shm.37175`.
- Fault PC: `0x7f4945bb774f`; immediate caller: `0x7f4945d89c47`.
- Neither address belongs to a surviving loaded-object mapping, and GDB cannot disassemble the fault PC.
- The core retains numerous `/dev/shm/fastdds_*` mappings. GDB still lists `libros_gz_bridge.so`, `librclcpp.so`, and the RMW loader, but no longer lists `libfastdds.so` or `librmw_fastrtps*.so`.
- This crash is separate from the Gazebo D3D12 core: the renderer was llvmpipe and Gazebo exited `0` here.

## LIKELY

The evidence is consistent with a Fast DDS/RMW teardown lifetime or unload-order race: a DDS worker remained runnable after its runtime code or callback target was unmapped. The unusable frames are themselves evidence of execution through stale/unmapped state; they are not evidence that `libgcc_s` caused the fault.

## UNKNOWN

The exact Fast DDS/RMW symbol cannot be recovered because the fault address is unmapped in the core and the worker's remaining stack is corrupted beyond the immediate caller.

## Artifacts

- `core.56753`: native ELF core.
- `gdb_backtrace.txt`: all-thread backtrace, shared libraries, and mappings.
- `backtrace.json`: machine-readable classification.
"""
(OUT / "BACKTRACE_REPORT.md").write_text(report)
print(json.dumps({"output": str(OUT), "signal": "SIGSEGV", "exit_code": -11}))
