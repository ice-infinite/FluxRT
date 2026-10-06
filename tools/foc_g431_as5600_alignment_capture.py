#!/usr/bin/env python3
"""Run and evaluate the one-shot G431/AS5600 alignment candidate."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path


TAU = 2.0 * math.pi
COUNTS_PER_REVOLUTION = 4096
ALIGNMENT_TRIAL_ACTIVE = 3
ALIGNMENT_TRIAL_COMPLETE = 4
ALIGNMENT_TRIAL_RESULT_OK = 1
EXPECTED_TICKS = 12000
HISTORICAL_OFFSET_RAD = 0.9658693323147879


def parse_row(line: str) -> dict[str, int] | None:
    fields = line.strip().split(",")
    if len(fields) != 6 or fields[0] != "FALR":
        return None
    try:
        return {
            "index": int(fields[1]),
            "elapsed_ms": int(fields[2]),
            "read_ok": int(fields[3]),
            "raw_count": int(fields[4]),
            "trial_state": int(fields[5]),
        }
    except ValueError:
        return None


def parse_end(line: str) -> dict[str, int] | None:
    fields = line.strip().split(",")
    if len(fields) != 16 or fields[0] != "FALEND":
        return None
    try:
        return {
            "preflight_raw": int(fields[1]),
            "runtime_status": int(fields[2]),
            "platform_status": int(fields[3]),
            "start_status": int(fields[4]),
            "trial_state": int(fields[5]),
            "trial_result": int(fields[6]),
            "total_ticks": int(fields[7]),
            "committed_alignment_ticks": int(fields[8]),
            "diagnostic_flags": int(fields[9], 16),
            "deadline_miss_count": int(fields[10]),
            "realtime_error_count": int(fields[11]),
            "peak_current_delta_counts": int(fields[12]),
            "reported_sample_count": int(fields[13]),
            "reported_valid_sample_count": int(fields[14]),
            "restore_status_packed": int(fields[15]),
        }
    except ValueError:
        return None


def wrap_pi(value: float) -> float:
    return (value + math.pi) % TAU - math.pi


def circular_mean(values: list[float]) -> float:
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.hypot(sine, cosine) <= 1.0e-12:
        raise ValueError("ambiguous circular mean")
    return math.atan2(sine, cosine) % TAU


def summarize(
    rows: list[dict[str, int]],
    end: dict[str, int] | None,
    tail_count: int = 15,
    maximum_deviation_rad: float = 0.05,
    maximum_historical_delta_rad: float = 0.15,
) -> dict[str, object]:
    active = [
        row
        for row in rows
        if row["read_ok"] == 1
        and row["trial_state"] == ALIGNMENT_TRIAL_ACTIVE
        and 0 <= row["raw_count"] < COUNTS_PER_REVOLUTION
    ]
    result: dict[str, object] = {
        "status": "alignment-offset-candidate-not-approved",
        "sample_count": len(rows),
        "active_valid_sample_count": len(active),
        "selected_tail_sample_count": 0,
        "requirements_pass": False,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
    }
    if end is None or len(active) < tail_count:
        return result
    selected = active[-tail_count:]
    offsets = [
        (-7.0 * (row["raw_count"] * TAU / COUNTS_PER_REVOLUTION)) % TAU
        for row in selected
    ]
    mean = circular_mean(offsets)
    deviations = [abs(wrap_pi(value - mean)) for value in offsets]
    historical_delta = abs(wrap_pi(mean - HISTORICAL_OFFSET_RAD))
    reported_sample_count = end["reported_sample_count"]
    reported_valid_sample_count = end["reported_valid_sample_count"]
    row_indices = [row["index"] for row in rows]
    expected_indices = list(range(reported_sample_count))
    transport_capture_complete = (
        len(rows) == reported_sample_count
        and row_indices == expected_indices
        and reported_valid_sample_count ==
            sum(row["read_ok"] == 1 for row in rows)
    )
    device_report_consistent = (
        reported_sample_count >= tail_count + 1
        and reported_valid_sample_count == reported_sample_count
        and len(row_indices) > 0
        and max(row_indices) == reported_sample_count - 1
    )
    selected_indices = [row["index"] for row in selected]
    final_row_observed = any(
        row["index"] == reported_sample_count - 1
        and row["read_ok"] == 1
        and row["trial_state"] == ALIGNMENT_TRIAL_COMPLETE
        for row in rows
    )
    tail_capture_complete = (
        selected_indices == list(
            range(selected_indices[0], selected_indices[0] + tail_count)
        )
        and selected_indices[-1] == reported_sample_count - 2
        and final_row_observed
    )
    transaction_ok = (
        end["runtime_status"] == 0
        and end["platform_status"] == 0
        and end["start_status"] == 0
        and end["trial_state"] == ALIGNMENT_TRIAL_COMPLETE
        and end["trial_result"] == ALIGNMENT_TRIAL_RESULT_OK
        and end["total_ticks"] == EXPECTED_TICKS
        and end["committed_alignment_ticks"] == EXPECTED_TICKS
        and end["deadline_miss_count"] == 0
        and end["realtime_error_count"] == 0
        and end["restore_status_packed"] == 0
    )
    stable = max(deviations) <= maximum_deviation_rad
    repeats_history = historical_delta <= maximum_historical_delta_rad
    result.update(
        {
            "selected_tail_sample_count": len(selected),
            "selected_first_index": selected[0]["index"],
            "selected_last_index": selected[-1]["index"],
            "additive_time_map_offset_rad": mean,
            "as5600_subtractive_offset_rad": (-mean) % TAU,
            "maximum_candidate_deviation_rad": max(deviations),
            "historical_offset_rad": HISTORICAL_OFFSET_RAD,
            "historical_delta_rad": historical_delta,
            "transaction_ok": transaction_ok,
            "device_report_consistent": device_report_consistent,
            "tail_capture_complete": tail_capture_complete,
            "transport_capture_complete": transport_capture_complete,
            "stable_tail": stable,
            "repeats_historical_candidate": repeats_history,
            "requirements_pass": (
                transaction_ok
                and device_report_consistent
                and tail_capture_complete
                and stable
                and repeats_history
            ),
            "end": end,
        }
    )
    return result


def load_existing_capture(output: Path):
    fields = ["index", "elapsed_ms", "read_ok", "raw_count", "trial_state"]
    with output.open(newline="", encoding="utf-8") as stream:
        rows = [
            {field: int(row[field]) for field in fields}
            for row in csv.DictReader(stream)
        ]
    previous = json.loads(output.with_suffix(".summary.json").read_text(
        encoding="utf-8"
    ))
    return rows, previous.get("end"), int(previous.get("other_line_count", 0))


def send_line(stream, command: str) -> None:
    for byte in command.encode("ascii"):
        stream.write(bytes([byte]))
        time.sleep(0.002)
    stream.write(b"\r")
    stream.flush()


def capture(port: str, baud: int, timeout_s: float):
    import serial

    rows: list[dict[str, int]] = []
    end = None
    other_lines: list[str] = []
    with serial.Serial(port, baud, timeout=0.1) as stream:
        stream.dtr = False
        stream.rts = False
        time.sleep(0.25)
        stream.reset_input_buffer()
        send_line(stream, "foc_encoder_align P55-ALIGN1")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = stream.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            parsed_row = parse_row(line)
            parsed_end = parse_end(line)
            if parsed_row is not None:
                rows.append(parsed_row)
            elif parsed_end is not None:
                end = parsed_end
                break
            else:
                other_lines.append(line)
    return rows, end, other_lines


def write_outputs(output: Path, rows, summary) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = ["index", "elapsed_ms", "read_ok", "raw_count", "trial_state"]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
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
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--reanalyze-existing",
        action="store_true",
        help="reanalyze the existing CSV/summary without opening the serial port",
    )
    args = parser.parse_args()
    if args.reanalyze_existing:
        rows, end, other_line_count = load_existing_capture(args.output)
    else:
        rows, end, other = capture(args.port, args.baud, args.timeout_s)
        other_line_count = len(other)
    summary = summarize(rows, end)
    summary.update(
        {
            "port": args.port,
            "baud": args.baud,
            "other_line_count": other_line_count,
        }
    )
    write_outputs(args.output, rows, summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["requirements_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
