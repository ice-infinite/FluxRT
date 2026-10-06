#!/usr/bin/env python3
"""Capture and validate the no-power G431 AS5600 FENC stream."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path


COUNTS_PER_REVOLUTION = 4096
FENC_FIELD_COUNT = 14


def parse_fenc(line: str) -> dict[str, int] | None:
    fields = line.strip().split(",")
    if len(fields) != FENC_FIELD_COUNT or fields[0] != "FENC":
        return None
    try:
        return {
            "index": int(fields[1]),
            "ok": int(fields[2]),
            "target_time_ms": int(fields[3]),
            "raw_count": int(fields[4]),
            "angle_mdeg": int(fields[5]),
            "present": int(fields[6]),
            "read_count": int(fields[7]),
            "error_count": int(fields[8]),
            "hal_status": int(fields[9]),
            "hal_error": int(fields[10], 16),
            "i2c_isr": int(fields[11]),
            "scl_high": int(fields[12]),
            "sda_high": int(fields[13]),
        }
    except ValueError:
        return None


def unwrap_counts(rows: list[dict[str, int]]) -> list[int]:
    if not rows:
        return []
    unwrapped = [rows[0]["raw_count"]]
    previous = rows[0]["raw_count"]
    for row in rows[1:]:
        current = row["raw_count"]
        delta = current - previous
        if delta > COUNTS_PER_REVOLUTION // 2:
            delta -= COUNTS_PER_REVOLUTION
        elif delta < -(COUNTS_PER_REVOLUTION // 2):
            delta += COUNTS_PER_REVOLUTION
        unwrapped.append(unwrapped[-1] + delta)
        previous = current
    return unwrapped


def summarize(rows: list[dict[str, int]]) -> dict[str, int | float | bool | str]:
    valid = [row for row in rows if row["ok"] == 1 and row["present"] == 1]
    result: dict[str, int | float | bool | str] = {
        "sample_count": len(rows),
        "valid_sample_count": len(valid),
        "verdict": "NO_VALID_SAMPLES",
        "requirements_pass": False,
    }
    if not valid:
        return result
    positions = unwrap_counts(valid)
    deltas = [right - left for left, right in zip(positions, positions[1:])]
    positive = sum(delta for delta in deltas if delta > 0)
    negative = -sum(delta for delta in deltas if delta < 0)
    wraps = sum(
        1
        for left, right in zip(valid, valid[1:])
        if abs(right["raw_count"] - left["raw_count"]) >
        COUNTS_PER_REVOLUTION // 2
    )
    maximum_gap_ms = max(
        (right["target_time_ms"] - left["target_time_ms"]
         for left, right in zip(valid, valid[1:])),
        default=0,
    )
    healthy = (
        len(valid) == len(rows)
        and max(row["error_count"] for row in valid) == 0
        and max(row["hal_status"] for row in valid) == 0
        and max(row["hal_error"] for row in valid) == 0
        and min(row["scl_high"] for row in valid) == 1
        and min(row["sda_high"] for row in valid) == 1
    )
    result.update(
        {
            "first_raw_count": valid[0]["raw_count"],
            "last_raw_count": valid[-1]["raw_count"],
            "raw_min_count": min(row["raw_count"] for row in valid),
            "raw_max_count": max(row["raw_count"] for row in valid),
            "net_motion_count": positions[-1] - positions[0],
            "positive_motion_count": positive,
            "negative_motion_count": negative,
            "positive_motion_rev": positive / COUNTS_PER_REVOLUTION,
            "negative_motion_rev": negative / COUNTS_PER_REVOLUTION,
            "total_motion_rev": (positive + negative) / COUNTS_PER_REVOLUTION,
            "wrap_event_count": wraps,
            "maximum_sample_gap_ms": maximum_gap_ms,
            "maximum_error_count": max(row["error_count"] for row in valid),
            "bus_healthy": healthy,
            "verdict": "VALID" if healthy else "BUS_OR_SAMPLE_ERROR",
        }
    )
    return result


def send_line(stream, command: str) -> None:
    for byte in command.encode("ascii"):
        stream.write(bytes([byte]))
        time.sleep(0.002)
    stream.write(b"\r")
    stream.flush()


def read_until(stream, duration_s: float) -> list[str]:
    deadline = time.monotonic() + duration_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        raw = stream.readline()
        if raw:
            lines.append(raw.decode("utf-8", "replace").strip())
    return lines


def capture(port: str, baud: int, segments: int, samples: int, period_ms: int) -> tuple[list[dict[str, int]], list[str]]:
    import serial

    rows: list[dict[str, int]] = []
    other_lines: list[str] = []
    with serial.Serial(port, baud, timeout=0.1) as stream:
        stream.dtr = False
        stream.rts = False
        time.sleep(0.25)
        stream.reset_input_buffer()
        send_line(stream, "foc_stop")
        other_lines.extend(read_until(stream, 0.35))
        for _ in range(segments):
            send_line(stream, f"foc_encoder_watch {samples} {period_ms}")
            expected_s = samples * period_ms / 1000.0 + 0.8
            for line in read_until(stream, expected_s):
                parsed = parse_fenc(line)
                if parsed is None:
                    other_lines.append(line)
                else:
                    rows.append(parsed)
    return rows, other_lines


def write_outputs(output: Path, rows: list[dict[str, int]], summary: dict[str, object]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else [
        "index", "ok", "target_time_ms", "raw_count", "angle_mdeg",
        "present", "read_count", "error_count", "hal_status", "hal_error",
        "i2c_isr", "scl_high", "sda_high",
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
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--segments", type=int, default=2)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--period-ms", type=int, default=100)
    parser.add_argument("--min-positive-rev", type=float, default=0.75)
    parser.add_argument("--min-negative-rev", type=float, default=0.75)
    parser.add_argument("--min-total-rev", type=float, default=0.25)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.segments <= 0 or not 1 <= args.samples <= 100:
        parser.error("segments must be positive and samples must be in 1..100")
    if not 5 <= args.period_ms <= 1000 or args.baud <= 0:
        parser.error("period-ms must be in 5..1000 and baud must be positive")

    rows, other_lines = capture(
        args.port, args.baud, args.segments, args.samples, args.period_ms
    )
    summary = summarize(rows)
    motion_pass = (
        float(summary.get("positive_motion_rev", 0.0)) >= args.min_positive_rev
        and float(summary.get("negative_motion_rev", 0.0)) >= args.min_negative_rev
        and float(summary.get("total_motion_rev", 0.0)) >= args.min_total_rev
    )
    summary.update(
        {
            "port": args.port,
            "baud": args.baud,
            "segments": args.segments,
            "samples_per_segment": args.samples,
            "period_ms": args.period_ms,
            "other_line_count": len(other_lines),
            "motion_requirements_pass": motion_pass,
            "minimum_total_rev": args.min_total_rev,
            "requirements_pass": bool(summary.get("bus_healthy", False)) and motion_pass,
        }
    )
    write_outputs(args.output, rows, summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["requirements_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
