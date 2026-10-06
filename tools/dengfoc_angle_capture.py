#!/usr/bin/env python3
"""Capture and summarize FluxRT's USB-only DengFOC AS5600 telemetry."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import time
from pathlib import Path
from typing import Iterable


LINE_PATTERN = re.compile(
    r"axis=(?P<axis>\d+),valid=(?P<valid>\d+),seq=(?P<sequence>\d+),"
    r"mech=(?P<mechanical>[-+0-9.eE]+),multi=(?P<multiturn>[-+0-9.eE]+),"
    r"vel=(?P<velocity>[-+0-9.eE]+),elec=(?P<electrical>[-+0-9.eE]+),"
    r"fail=(?P<failures>\d+)"
)


def parse_telemetry_line(line: str, host_time_s: float = 0.0) -> dict[str, int | float] | None:
    match = LINE_PATTERN.fullmatch(line.strip())
    if match is None:
        return None
    groups = match.groupdict()
    return {
        "host_time_s": host_time_s,
        "axis": int(groups["axis"]),
        "valid": int(groups["valid"]),
        "sequence": int(groups["sequence"]),
        "mechanical_position_rad": float(groups["mechanical"]),
        "multi_turn_position_rad": float(groups["multiturn"]),
        "mechanical_velocity_rad_s": float(groups["velocity"]),
        "electrical_angle_rad": float(groups["electrical"]),
        "read_failures": int(groups["failures"]),
    }


def summarize(rows: Iterable[dict[str, int | float]]) -> dict[str, int | float | str]:
    all_rows = list(rows)
    valid_rows = [row for row in all_rows if row["valid"] == 1]
    summary: dict[str, int | float | str] = {
        "sample_count": len(all_rows),
        "valid_sample_count": len(valid_rows),
        "verdict": "NO_VALID_SAMPLES",
    }
    if not valid_rows:
        return summary

    mechanical = [float(row["mechanical_position_rad"]) for row in valid_rows]
    multiturn = [float(row["multi_turn_position_rad"]) for row in valid_rows]
    velocity = [float(row["mechanical_velocity_rad_s"]) for row in valid_rows]
    failures = [int(row["read_failures"]) for row in valid_rows]
    differences = [right - left for left, right in zip(multiturn, multiturn[1:])]
    positive_motion = sum(value for value in differences if value > 0.0)
    negative_motion = -sum(value for value in differences if value < 0.0)
    wrap_events = sum(
        1
        for left, right in zip(mechanical, mechanical[1:])
        if abs(right - left) > math.pi
    )
    summary.update(
        {
            "first_sequence": int(valid_rows[0]["sequence"]),
            "last_sequence": int(valid_rows[-1]["sequence"]),
            "failure_count_max": max(failures),
            "mechanical_min_rad": min(mechanical),
            "mechanical_max_rad": max(mechanical),
            "mechanical_span_rad": max(mechanical) - min(mechanical),
            "mechanical_std_rad": statistics.pstdev(mechanical),
            "multiturn_min_rad": min(multiturn),
            "multiturn_max_rad": max(multiturn),
            "net_motion_rad": multiturn[-1] - multiturn[0],
            "total_motion_rad": sum(abs(value) for value in differences),
            "positive_motion_rad": positive_motion,
            "negative_motion_rad": negative_motion,
            "dominant_motion_rad": max(positive_motion, negative_motion),
            "reverse_motion_rad": min(positive_motion, negative_motion),
            "wrap_events": wrap_events,
            "velocity_min_rad_s": min(velocity),
            "velocity_max_rad_s": max(velocity),
            "velocity_rms_rad_s": math.sqrt(
                sum(value * value for value in velocity) / len(velocity)
            ),
            "verdict": "VALID",
        }
    )
    return summary


def requirements_pass(summary: dict[str, int | float | str], args: argparse.Namespace) -> bool:
    if summary["verdict"] != "VALID" or int(summary["failure_count_max"]) != 0:
        return False
    return (
        float(summary["total_motion_rad"]) >= args.require_motion_rad
        and float(summary["dominant_motion_rad"]) >= args.require_one_direction_rad
        and float(summary["reverse_motion_rad"]) <= args.max_reverse_rad
        and int(summary["wrap_events"]) >= args.require_wraps
    )


def capture(port: str, baud: int, duration_s: float) -> tuple[list[dict[str, int | float]], list[str]]:
    import serial

    rows: list[dict[str, int | float]] = []
    boot_lines: list[str] = []
    started = time.monotonic()
    with serial.Serial(port, baud, timeout=0.2) as stream:
        stream.dtr = False
        stream.rts = False
        while time.monotonic() - started < duration_s:
            line = stream.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            row = parse_telemetry_line(line, time.monotonic() - started)
            if row is None:
                boot_lines.append(line)
            else:
                rows.append(row)
    return rows, boot_lines


def write_capture(output: Path, rows: list[dict[str, int | float]], summary: dict[str, object]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else [
        "host_time_s",
        "axis",
        "valid",
        "sequence",
        "mechanical_position_rad",
        "multi_turn_position_rad",
        "mechanical_velocity_rad_s",
        "electrical_angle_rad",
        "read_failures",
    ]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    output.with_suffix(".summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-motion-rad", type=float, default=0.0)
    parser.add_argument("--require-one-direction-rad", type=float, default=0.0)
    parser.add_argument("--max-reverse-rad", type=float, default=float("inf"))
    parser.add_argument("--require-wraps", type=int, default=0)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.baud <= 0:
        parser.error("duration and baud must be positive")

    rows, boot_lines = capture(args.port, args.baud, args.duration)
    summary = summarize(rows)
    summary.update(
        {
            "port": args.port,
            "baud": args.baud,
            "duration_s": args.duration,
            "boot_line_count": len(boot_lines),
            "requirements_pass": requirements_pass(summary, args),
        }
    )
    write_capture(args.output, rows, summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if bool(summary["requirements_pass"]) else 2


if __name__ == "__main__":
    raise SystemExit(main())
