#!/usr/bin/env python3
"""Capture bounded, correlated ISR timing for trace-off/10 Hz/50 Hz runs."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import subprocess
import sys
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--target-rpm", type=float, default=582.0)
    parser.add_argument("--rates", default="0,10,50")
    parser.add_argument("--closed-loop", action="store_true")
    parser.add_argument("--allow-motor-run", action="store_true")
    parser.add_argument("--output", type=pathlib.Path, required=True)
    return parser.parse_args()


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def read_available(port: serial.Serial, raw: list[str], prefix: str) -> list[str]:
    lines: list[str] = []
    while port.in_waiting:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if line:
            raw.append(f"[{prefix}] {line}")
            lines.append(line)
    return lines


def collect_for(port: serial.Serial, seconds: float, raw: list[str], prefix: str) -> list[str]:
    deadline = time.monotonic() + seconds
    lines: list[str] = []
    while time.monotonic() < deadline:
        lines.extend(read_available(port, raw, prefix))
        time.sleep(0.005)
    lines.extend(read_available(port, raw, prefix))
    return lines


TIMING_FIELDS = (
    "version",
    "samples",
    "invalid",
    "step",
    "total",
    "pre",
    "control",
    "post",
    "trace",
    "sampled",
    "peak_pre",
    "peak_control",
    "peak_post",
    "deadline",
    "misses",
    "errors",
    "fault",
    "flags",
)


def parse_timing(line: str) -> dict[str, int] | None:
    marker = line.find("FTIMING,")
    if marker < 0:
        return None
    try:
        raw_values = line[marker + len("FTIMING,") :].split(",")
        values = [int(value, 0) for value in raw_values]
    except ValueError:
        return None
    return dict(zip(TIMING_FIELDS, values)) if len(values) == len(TIMING_FIELDS) else None


def git_commit(project: pathlib.Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=project,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def run_scenario(
    port: serial.Serial,
    rate_hz: int,
    duration: float,
    target_rpm: float,
    raw: list[str],
) -> dict[str, object]:
    label = f"trace-{rate_hz}Hz" if rate_hz else "trace-off"
    raw.append(f"[{label}] BEGIN")
    send(port, "foc_stop")
    send(port, "foc_trace stop")
    collect_for(port, 0.2, raw, label)
    if rate_hz:
        send(port, f"foc_trace start {rate_hz}")
        collect_for(port, 0.2, raw, label)

    send(port, f"foc_start {target_rpm:g}")
    start_lines = collect_for(port, 0.5, raw, label)
    if any("FOC start REFUSED" in line for line in start_lines):
        raise RuntimeError(f"{label}: controller refused to start")
    if not any("FOC ARMED" in line for line in start_lines):
        raise RuntimeError(f"{label}: no FOC ARMED acknowledgement")

    collect_for(port, duration, raw, label)
    send(port, "foc_stop")
    collect_for(port, 0.2, raw, label)
    send(port, "foc_status")
    timing_lines = collect_for(port, 0.5, raw, label)
    timing = next(
        (parsed for line in reversed(timing_lines) if (parsed := parse_timing(line)) is not None),
        None,
    )
    send(port, "foc_trace stop")
    collect_for(port, 0.1, raw, label)
    if timing is None:
        raise RuntimeError(f"{label}: no FTIMING record received")
    issues: list[str] = []
    if timing["samples"] == 0:
        issues.append("no realtime timing samples")
    if timing["invalid"] != 0:
        issues.append("invalid correlated timing sample")
    if timing["pre"] + timing["control"] + timing["post"] != timing["total"]:
        issues.append("timing segments are not from one ISR sample")
    if timing["trace"] != (1 if rate_hz else 0):
        issues.append("trace mode mismatch in WCET sample")
    if timing["misses"] != 0:
        issues.append("ISR deadline missed")
    if timing["errors"] != 0 or timing["fault"] != 0:
        issues.append("control loop faulted")
    raw.append(f"[{label}] END")
    return {
        "trace_rate_hz": rate_hz,
        "duration_s": duration,
        "target_speed_rpm": target_rpm,
        "passed": not issues,
        "issues": issues,
        "timing": timing,
    }


def main() -> int:
    args = parse_args()
    rates = [int(item.strip()) for item in args.rates.split(",") if item.strip()]
    if not args.allow_motor_run:
        raise SystemExit("refusing to energize the motor without --allow-motor-run")
    if args.duration <= 0 or not rates or any(rate not in (0, 10, 50) for rate in rates):
        raise SystemExit("duration must be positive; rates must be selected from 0,10,50")

    project = pathlib.Path(__file__).resolve().parents[1]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    raw: list[str] = []
    scenarios: list[dict[str, object]] = []

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0) as port:
        time.sleep(0.25)
        port.reset_input_buffer()
        send(port, "foc_stop")
        send(port, "foc_trace stop")
        if args.closed_loop:
            send(port, "foc_cfg closedloop 1")
        collect_for(port, 0.25, raw, "setup")
        try:
            for rate_hz in rates:
                scenarios.append(
                    run_scenario(port, rate_hz, args.duration, args.target_rpm, raw)
                )
        finally:
            send(port, "foc_stop")
            send(port, "foc_trace stop")
            if args.closed_loop:
                send(port, "foc_cfg closedloop 0")
            collect_for(port, 0.25, raw, "cleanup")

    document = {
        "schema_version": 1,
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "git_commit": git_commit(project),
        "port": args.port,
        "baud": args.baud,
        "closed_loop_requested": args.closed_loop,
        "scenarios": scenarios,
    }
    args.output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    log_path = args.output.with_suffix(".log")
    log_path.write_text("\n".join(raw) + "\n", encoding="utf-8")
    print(f"FOC_TIMING_BASELINE={args.output}")
    print(f"FOC_TIMING_RAW_LOG={log_path}")
    failed = sum(not bool(scenario["passed"]) for scenario in scenarios)
    if failed:
        print(f"FOC_TIMING_CAPTURE_FAIL scenarios={len(scenarios)} failed={failed}")
        return 1
    print(f"FOC_TIMING_CAPTURE_PASS scenarios={len(scenarios)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(str(error)) from error
