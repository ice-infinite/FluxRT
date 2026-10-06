#!/usr/bin/env python3
"""Capture verified G431 PB7/DengFOC H3 anchors without touching motor power.

The live path only writes the whitelisted ``foc_stop``, ``foc_status`` and
``foc_h3_sync_*`` shell commands.  It refuses to emit a frame unless the G431
reports disabled/zero duty/no fault and DengFOC reports ``driver_disabled=1``.
The produced anchors use the PB7 input-capture DWT tick, not the planned PB6
compare timestamp.  Output is evidence only and never grants a capability.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path


CONFIRMATION = "NO_POWER_H3_CAPTURE"
TX_ABI_VERSION = 0x00030000
REQUIRED_FLAGS = 0x7

TRUTH_PATTERN = re.compile(
    r"SYNC_TRUTH,sequence=(?P<sequence>\d+),truth_tick_us=(?P<tick>\d+),"
    r"mechanical_angle_rad=(?P<angle>[-+0-9.eE]+),valid=(?P<valid>[01]),"
    r"read_failures=(?P<failures>\d+)"
)


class CaptureError(ValueError):
    """Raised when board evidence is incomplete or unsafe."""


@dataclass(frozen=True)
class G431Anchor:
    session_id: int
    edge_sequence: int
    edge_tag: int
    planned_cycle_tick: int
    control_tick: int
    flags: int
    loopback_timer_tick: int
    loopback_cycle_tick: int
    loopback_delta_cycles: int


@dataclass(frozen=True)
class DengAnchor:
    session_id: int
    edge_sequence: int
    edge_tag: int
    truth_tick_us: int
    flags: int


def _key_values(line: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for field in line.split(",")[1:]:
        if "=" in field:
            key, value = field.split("=", 1)
            values[key] = value
    return values


def parse_g431_status(line: str) -> G431Anchor | None:
    fields = line.strip().split(",")
    if not fields or fields[0] != "FH3_SYNC_STATUS":
        return None
    if len(fields) != 20:
        raise CaptureError(
            f"G431 H3 status field count is {len(fields)}, expected 20 for ABI v3: "
            f"{line[:240]}"
        )
    values = [int(value, 10) for value in fields[1:]]
    result, version, state = values[:3]
    if result != 1 or version != TX_ABI_VERSION:
        raise CaptureError("G431 H3 status is not ABI v3")
    # A newly opened Shell can still contain an explicit READY snapshot from a
    # preceding preflight.  It carries no edge identity and must neither become
    # an anchor nor poison a later COMPLETE snapshot.  BUSY is handled the same
    # way: if no COMPLETE row follows, pair_anchors() fails closed on count.
    if state in (1, 2):
        return None
    if state != 3:
        raise CaptureError("G431 H3 status is not a completed frame")
    flags = values[10]
    if (flags & REQUIRED_FLAGS) != REQUIRED_FLAGS:
        raise CaptureError("G431 H3 status lacks generated/control/loopback validity")
    return G431Anchor(
        session_id=values[11],
        edge_sequence=values[12],
        edge_tag=values[13],
        planned_cycle_tick=values[8],
        control_tick=values[9],
        flags=flags,
        loopback_timer_tick=values[16],
        loopback_cycle_tick=values[17],
        loopback_delta_cycles=values[18],
    )


def parse_deng_anchor(line: str) -> DengAnchor | None:
    if not line.startswith("SYNC_ANCHOR_RX,"):
        return None
    values = _key_values(line)
    required = {"session", "edge_sequence", "edge_tag", "truth_tick_us", "flags"}
    if required - values.keys():
        raise CaptureError("DengFOC anchor is missing fields")
    flags = int(values["flags"], 0)
    if (flags & 0x3) != 0x3:
        raise CaptureError("DengFOC anchor identity is not verified")
    return DengAnchor(
        session_id=int(values["session"]),
        edge_sequence=int(values["edge_sequence"]),
        edge_tag=int(values["edge_tag"]),
        truth_tick_us=int(values["truth_tick_us"]),
        flags=flags,
    )


def parse_truth(line: str) -> dict[str, int | float] | None:
    match = TRUTH_PATTERN.fullmatch(line.strip())
    if match is None:
        return None
    values = match.groupdict()
    return {
        "sequence": int(values["sequence"]),
        "truth_tick": int(values["tick"]),
        "mechanical_angle_rad": float(values["angle"]),
        "valid": int(values["valid"]),
        "read_failures": int(values["failures"]),
    }


def pair_anchors(
    g431: list[G431Anchor], deng: list[DengAnchor]
) -> list[tuple[G431Anchor, DengAnchor]]:
    g_by_sequence = {row.edge_sequence: row for row in g431}
    d_by_sequence = {row.edge_sequence: row for row in deng}
    if len(g_by_sequence) != len(g431) or len(d_by_sequence) != len(deng):
        raise CaptureError("duplicate H3 anchor sequence")
    if set(g_by_sequence) != set(d_by_sequence):
        raise CaptureError("G431 and DengFOC anchor sequences differ")
    sequences = sorted(g_by_sequence)
    if any(right != left + 1 for left, right in zip(sequences, sequences[1:])):
        raise CaptureError("H3 anchor sequences are not contiguous")
    result = []
    for sequence in sequences:
        left = g_by_sequence[sequence]
        right = d_by_sequence[sequence]
        if (left.session_id, left.edge_tag) != (right.session_id, right.edge_tag):
            raise CaptureError("H3 anchor identity mismatch")
        result.append((left, right))
    return result


def build_summary(
    pairs: list[tuple[G431Anchor, DengAnchor]],
    truth: list[dict[str, int | float]],
) -> dict[str, object]:
    if len(pairs) < 3:
        raise CaptureError("at least three paired H3 anchors are required")
    deltas = [left.loopback_delta_cycles for left, _ in pairs]
    if any(left.loopback_timer_tick != 100 for left, _ in pairs):
        raise CaptureError("PB7 input capture did not observe the 100 us first edge")
    if any(int(row["valid"]) != 1 or int(row["read_failures"]) != 0 for row in truth):
        raise CaptureError("DengFOC truth contains an invalid sample")
    reference = [pairs[0][0].loopback_cycle_tick]
    for (previous, _), (current, _) in zip(pairs, pairs[1:]):
        delta = (current.loopback_cycle_tick - previous.loopback_cycle_tick) & 0xFFFFFFFF
        if delta == 0 or delta >= 0x80000000:
            raise CaptureError("PB7 cycle ticks are duplicate or not forward ordered")
        reference.append(reference[-1] + delta)
    truth_ticks = [right.truth_tick_us for _, right in pairs]
    if any(right <= left for left, right in zip(truth_ticks, truth_ticks[1:])):
        raise CaptureError("DengFOC anchor ticks are not forward ordered")
    truth_mean = sum(truth_ticks) / len(truth_ticks)
    reference_mean = sum(reference) / len(reference)
    denominator = sum((value - truth_mean) ** 2 for value in truth_ticks)
    if denominator <= 0.0:
        raise CaptureError("H3 anchor time span is zero")
    slope = sum(
        (truth_tick - truth_mean) * (reference_tick - reference_mean)
        for truth_tick, reference_tick in zip(truth_ticks, reference)
    ) / denominator
    intercept = reference_mean - slope * truth_mean
    residuals = [
        reference_tick - (slope * truth_tick + intercept)
        for truth_tick, reference_tick in zip(truth_ticks, reference)
    ]
    residual_max = max(abs(value) for value in residuals)
    residual_rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    return {
        "contract": "fluxrt-h3-board-capture",
        "version": 1,
        "status": "captured-not-approved",
        "anchor_count": len(pairs),
        "first_edge_sequence": pairs[0][0].edge_sequence,
        "last_edge_sequence": pairs[-1][0].edge_sequence,
        "truth_sample_count": len(truth),
        "loopback": {
            "valid_count": len(pairs),
            "timer_tick": 100,
            "delta_cycles_min": min(deltas),
            "delta_cycles_max": max(deltas),
            "delta_cycles_mean": statistics.mean(deltas),
            "delta_cycles_median": statistics.median(deltas),
            "absolute_max_us": max(abs(value) for value in deltas) / 170.0,
        },
        "clock_mapping": {
            "reference_tick_hz": 170_000_000,
            "truth_tick_hz": 1_000_000,
            "slope_reference_ticks_per_truth_tick": slope,
            "clock_drift_ppm": (slope / 170.0 - 1.0) * 1_000_000.0,
            "maximum_anchor_residual_ticks": residual_max,
            "rms_anchor_residual_ticks": residual_rms,
            "maximum_anchor_residual_us": residual_max / 170.0,
            "rms_anchor_residual_us": residual_rms / 170.0,
        },
        "motor_power_enabled": False,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def write_raw_checkpoint(
    output: Path,
    raw_lines: list[tuple[str, str]],
    status: str,
    error: str | None = None,
) -> None:
    """Persist no-power serial evidence before schema analysis."""
    output.mkdir(parents=True, exist_ok=False)
    raw_path = output / "raw-lines.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "raw_line"])
        writer.writerows(raw_lines)
    summary: dict[str, object] = {
        "contract": "fluxrt-h3-board-capture",
        "version": 1,
        "status": status,
        "motor_power_enabled": False,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
        "artifacts": {raw_path.name: _sha256(raw_path)},
    }
    if error is not None:
        summary["error"] = error
    (output / "capture.summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def mark_checkpoint_failed(output: Path, error: str) -> None:
    summary_path = output / "capture.summary.json"
    if not summary_path.is_file():
        return
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["status"] = "capture-failed"
    summary["error"] = error
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def write_artifacts(
    output: Path,
    pairs: list[tuple[G431Anchor, DengAnchor]],
    truth: list[dict[str, int | float]],
    raw_lines: list[tuple[str, str]],
    summary: dict[str, object],
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    anchors_path = output / "anchors.csv"
    with anchors_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["edge_sequence", "reference_control_tick", "truth_tick"],
        )
        writer.writeheader()
        for left, right in pairs:
            writer.writerow(
                {
                    "edge_sequence": left.edge_sequence,
                    "reference_control_tick": left.loopback_cycle_tick,
                    "truth_tick": right.truth_tick_us,
                }
            )
    raw_anchor_path = output / "raw-anchors.csv"
    fields = [f"g431_{name}" for name in asdict(pairs[0][0])] + [
        f"deng_{name}" for name in asdict(pairs[0][1])
    ]
    with raw_anchor_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for left, right in pairs:
            writer.writerow(
                {**{f"g431_{key}": value for key, value in asdict(left).items()},
                 **{f"deng_{key}": value for key, value in asdict(right).items()}}
            )
    truth_path = output / "truth.csv"
    truth_fields = ["sequence", "truth_tick", "mechanical_angle_rad", "valid", "read_failures"]
    with truth_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=truth_fields)
        writer.writeheader()
        writer.writerows(truth)
    raw_path = output / "raw-lines.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "raw_line"])
        writer.writerows(raw_lines)
    stored = dict(summary)
    stored["artifacts"] = {
        path.name: _sha256(path)
        for path in (anchors_path, raw_anchor_path, truth_path, raw_path)
    }
    (output / "capture.summary.json").write_text(
        json.dumps(stored, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def _open_serial(port: str, baud: int):
    import serial

    stream = serial.Serial()
    stream.port = port
    stream.baudrate = baud
    stream.timeout = 0.02
    stream.dtr = False
    stream.rts = False
    stream.open()
    return stream


def live_capture(args: argparse.Namespace) -> tuple[
    list[tuple[G431Anchor, DengAnchor]],
    list[dict[str, int | float]],
    list[tuple[str, str]],
]:
    if args.confirm_no_power != CONFIRMATION:
        raise CaptureError(f"--confirm-no-power must equal {CONFIRMATION}")
    if args.g431_port.casefold() == args.dengfoc_port.casefold():
        raise CaptureError("G431 and DengFOC ports must differ")
    if args.count < 3 or args.count > 256 or args.interval < 0.20:
        raise CaptureError("count must be 3..256 and interval must be >= 0.20 s")
    if args.output.exists():
        raise CaptureError("output directory already exists")
    g431_stream = _open_serial(args.g431_port, args.g431_baud)
    deng_stream = _open_serial(args.dengfoc_port, args.dengfoc_baud)
    stop = threading.Event()
    lock = threading.Lock()
    raw_lines: list[tuple[str, str]] = []
    reader_errors: list[str] = []
    capture_started = False

    def reader(source: str, stream) -> None:
        try:
            while not stop.is_set():
                payload = stream.readline()
                if payload:
                    line = payload.decode("utf-8", "replace").strip()
                    if line:
                        with lock:
                            raw_lines.append((source, line))
        except Exception as error:  # Serial disconnects are evidence unless stopping.
            if not stop.is_set():
                with lock:
                    reader_errors.append(f"{source}: {error}")

    workers = [
        threading.Thread(target=reader, args=("g431", g431_stream), daemon=True),
        threading.Thread(target=reader, args=("dengfoc", deng_stream), daemon=True),
    ]
    for worker in workers:
        worker.start()

    def send(command: str, delay: float) -> None:
        for character in command + "\r\n":
            g431_stream.write(character.encode("ascii"))
            time.sleep(0.003)
        g431_stream.flush()
        time.sleep(delay)

    try:
        time.sleep(0.8)
        send("foc_stop", 0.25)
        send("foc_status", 0.45)
        time.sleep(0.5)
        with lock:
            snapshot = [line for _, line in raw_lines]
        safe_status = any(
            line.startswith("FOC st=0 rf=00000000 duty=0/0/0") for line in snapshot
        )
        safe_fault = any(
            line.startswith("FFAULT,00000000,00000000,0,0,ARM,0,0000")
            for line in snapshot
        )
        safe_deng = any(
            line.startswith("SYNC_STATUS,") and "driver_disabled=1" in line
            for line in snapshot
        )
        if not (safe_status and safe_fault and safe_deng):
            raise CaptureError("no-power preflight did not prove both targets disabled")
        capture_started = True
        send("foc_h3_sync_init", 0.05)
        # Reset leaves PB6 high impedance while DengFOC's former SDA input is
        # pulled high.  Once init drives PB6 low, allow the 500 us RMT idle
        # completion and several receiver management loops to re-arm before
        # the first evidence frame.
        time.sleep(0.02)
        with lock:
            init_snapshot = [line for _, line in raw_lines]
        init_ok = any(
            line.startswith("FH3_SYNC_INIT,1,196608,1,")
            for line in init_snapshot
        )
        if not init_ok:
            raise CaptureError("H3 transmitter init did not reach ABI v3 READY")
        for offset in range(args.count):
            sequence = args.start_sequence + offset
            tag = args.start_tag + offset
            send(
                f"foc_h3_sync_send {args.session} {sequence} {tag}",
                args.interval,
            )
            send("foc_h3_sync_status", 0.20)
        time.sleep(0.8)
    finally:
        if capture_started:
            try:
                send("foc_h3_sync_abort", 0.10)
            except Exception as error:
                with lock:
                    reader_errors.append(f"g431 abort: {error}")
        stop.set()
        g431_stream.close()
        deng_stream.close()
        for worker in workers:
            worker.join(timeout=0.5)
        if capture_started:
            write_raw_checkpoint(
                args.output,
                raw_lines,
                "captured-pending-analysis",
            )
    if reader_errors:
        raise CaptureError("; ".join(reader_errors))

    g431 = []
    deng = []
    truth = []
    for _, line in raw_lines:
        parsed_g431 = parse_g431_status(line)
        if parsed_g431 is not None and args.start_sequence <= parsed_g431.edge_sequence < args.start_sequence + args.count:
            g431.append(parsed_g431)
        parsed_deng = parse_deng_anchor(line)
        if parsed_deng is not None and args.start_sequence <= parsed_deng.edge_sequence < args.start_sequence + args.count:
            deng.append(parsed_deng)
        parsed_truth = parse_truth(line)
        if parsed_truth is not None:
            truth.append(parsed_truth)
    pairs = pair_anchors(g431, deng)
    if len(pairs) != args.count:
        raise CaptureError(f"expected {args.count} paired anchors, captured {len(pairs)}")
    return pairs, truth, raw_lines


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g431-port", required=True)
    parser.add_argument("--g431-baud", type=int, default=115200)
    parser.add_argument("--dengfoc-port", required=True)
    parser.add_argument("--dengfoc-baud", type=int, default=921600)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.30)
    parser.add_argument("--session", type=int, required=True)
    parser.add_argument("--start-sequence", type=int, required=True)
    parser.add_argument("--start-tag", type=int, required=True)
    parser.add_argument("--confirm-no-power", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        pairs, truth, raw_lines = live_capture(args)
        summary = build_summary(pairs, truth)
        write_artifacts(args.output, pairs, truth, raw_lines, summary)
    except (CaptureError, OSError) as error:
        mark_checkpoint_failed(args.output, str(error))
        parser.error(str(error))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
