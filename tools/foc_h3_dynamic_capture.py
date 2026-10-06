#!/usr/bin/env python3
"""Capture one fixed reverse P5.5 H3 trial with same-rotor AS5600 truth.

The target owns the 14 s total timeout and the final 1,200 accepted control
ticks.  This host tool cannot select speed, current, feature mask or duration;
it only supplies verified H3 identity fields and always sends ``foc_stop``.
The firmware accepts only the reverse token and this tool performs exactly one
attempt: it has no retry loop.  Outputs are evidence with approval denied.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time


CONFIRMATION = "P55_H3_REVERSE_12V3_2A_UNLOADED"
COMMAND_TOKEN = "P55-H3-REV"
COMMAND_DIRECTION = -1
COMMAND_TARGET_SPEED_RPM = -582.0
MIN_POWERED_BUS_MV = 7000
MAX_POWERED_BUS_MV = 18000
MAX_VALID_TRUTH_GAP_US = 10_000
ROOT = Path(__file__).resolve().parents[1]
BOARD_TOOL = ROOT / "tools/foc_h3_board_capture.py"
SPEC = importlib.util.spec_from_file_location("foc_h3_board_capture_dynamic", BOARD_TOOL)
assert SPEC is not None and SPEC.loader is not None
board = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = board
SPEC.loader.exec_module(board)


class DynamicCaptureError(ValueError):
    """Raised when powered evidence is incomplete or unsafe."""


def prepare_truth_stream(
    truth: list[dict[str, int | float]],
    maximum_allowed_gap_us: int = MAX_VALID_TRUTH_GAP_US,
) -> tuple[list[dict[str, int | float]], dict[str, int]]:
    """Remove angle-less tracker rebase markers under a bounded quality gate.

    The DengFOC tracker publishes ``valid=0`` when a sample interval exceeds
    its 5 ms velocity limit.  That row has no usable angle, but it is not an
    I2C failure.  Keep the surrounding positional samples only when the
    cumulative failure counter stays zero and their largest gap is bounded.
    """
    if maximum_allowed_gap_us <= 0:
        raise DynamicCaptureError("AS5600 truth gap limit must be positive")
    if any(int(row["read_failures"]) != 0 for row in truth):
        raise DynamicCaptureError("AS5600 truth reports an I2C read failure")
    if any(int(row["valid"]) not in (0, 1) for row in truth):
        raise DynamicCaptureError("AS5600 truth validity is not boolean")
    valid = [row for row in truth if int(row["valid"]) == 1]
    if len(valid) < 2:
        raise DynamicCaptureError("AS5600 truth is incomplete")
    ticks = [int(row["truth_tick"]) for row in valid]
    if any(right <= left for left, right in zip(ticks, ticks[1:])):
        raise DynamicCaptureError("AS5600 truth ticks are not forward ordered")
    maximum_gap_us = max(right - left for left, right in zip(ticks, ticks[1:]))
    if maximum_gap_us > maximum_allowed_gap_us:
        raise DynamicCaptureError(
            f"AS5600 valid truth gap {maximum_gap_us} us exceeds "
            f"{maximum_allowed_gap_us} us"
        )
    return valid, {
        "raw_sample_count": len(truth),
        "valid_sample_count": len(valid),
        "rebase_sample_count": len(truth) - len(valid),
        "maximum_valid_gap_us": maximum_gap_us,
        "maximum_allowed_gap_us": maximum_allowed_gap_us,
    }


def write_raw_checkpoint(
    output: Path,
    raw: list[tuple[str, str]],
    status: str,
    error: str | None = None,
) -> None:
    """Persist the live serial stream before any schema analysis.

    A powered attempt is evidence even when parsing fails.  The checkpoint is
    deliberately explicit about its non-approved state and is overwritten by
    ``write_artifacts`` only after every analysis gate passes.
    """
    output.mkdir(parents=True, exist_ok=False)
    raw_path = output / "raw-lines.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "raw_line"])
        writer.writerows(raw)
    summary: dict[str, object] = {
        "contract": "fluxrt-h3-dynamic-board-capture",
        "version": 1,
        "status": status,
        "motor_power_enabled": True,
        "command_token": COMMAND_TOKEN,
        "commanded_mechanical_direction": COMMAND_DIRECTION,
        "commanded_target_speed_rpm": COMMAND_TARGET_SPEED_RPM,
        "attempt_limit": 1,
        "automatic_retry": False,
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
    """Turn an existing raw checkpoint into an explicit failed capture."""
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


def parse_dynamic_anchor(line: str):
    if not line.startswith("FH3_DYNAMIC_ANCHOR,"):
        return None
    return board.parse_g431_status(
        "FH3_SYNC_STATUS,1," + line.split(",", 1)[1]
    )


def parse_ftr_query(line: str) -> dict[str, int | float] | None:
    if not line.startswith("FTR,"):
        return None
    fields = line.split(",")
    if len(fields) != 32:
        raise DynamicCaptureError("FTR row is not V2/31 fields")
    try:
        step = int(fields[1], 10)
        controller_state = int(fields[2], 10)
        control_angle_mrad = int(fields[16], 10)
        forced_angle_mrad = int(fields[17], 10)
        observer_angle_mrad = int(fields[18], 10)
        reliable = int(fields[20], 10)
    except ValueError as error:
        raise DynamicCaptureError("FTR row contains a non-integer field") from error
    if reliable not in (0, 1):
        raise DynamicCaptureError("FTR observer reliability is not boolean")
    return {
        "sequence": 0,
        "reference_control_tick": step,
        "controller_state": controller_state,
        "control_electrical_angle_rad": control_angle_mrad / 1000.0,
        "forced_electrical_angle_rad": forced_angle_mrad / 1000.0,
        "estimated_electrical_angle_rad": observer_angle_mrad / 1000.0,
        "reliable": reliable,
    }


def parse_fadc_bus_voltage_mv(line: str) -> int | None:
    if not line.startswith("FADC,"):
        return None
    fields = line.split(",")
    if len(fields) != 6:
        raise DynamicCaptureError("FADC row schema mismatch")
    try:
        raw = int(fields[3], 10)
        millivolts = int(fields[4], 10)
    except ValueError as error:
        raise DynamicCaptureError("FADC bus fields are not integers") from error
    if raw < 0 or millivolts < 0:
        raise DynamicCaptureError("FADC bus fields are negative")
    return millivolts


def validate_reverse_completion(raw: list[tuple[str, str]]) -> int:
    """Require one successful, deadline-clean reverse trial completion."""
    rows = [
        line for source, line in raw
        if source == "g431" and line.startswith("FADVP,state=")
    ]
    if len(rows) != 1:
        raise DynamicCaptureError("capture must contain exactly one H3 completion")
    values: dict[str, str] = {}
    for item in rows[0].split(",")[1:]:
        if "=" not in item:
            raise DynamicCaptureError("H3 completion schema mismatch")
        key, value = item.split("=", 1)
        values[key] = value
    required = {
        "state", "result", "ticks", "epoch", "miss", "features", "status",
        "snap", "orel", "closed", "speed", "lossus", "diagmiss", "finish",
    }
    if not required.issubset(values):
        raise DynamicCaptureError("H3 completion lacks fixed-envelope fields")
    try:
        active_ticks, _ = (int(value, 10) for value in values["ticks"].split("/"))
        epoch_before, epoch_after = (
            int(value, 10) for value in values["epoch"].split("/")
        )
        miss_before, miss_after = (
            int(value, 10) for value in values["miss"].split("/")
        )
        speed = int(values["speed"], 10)
        valid = (
            int(values["state"], 10) == 4
            and int(values["result"], 10) == 1
            and active_ticks == 1200
            and epoch_before == epoch_after
            and miss_before == 0
            and miss_after == 0
            and int(values["features"], 16) == 0
            and int(values["status"], 16) == 0
            and int(values["snap"], 10) == 1
            and int(values["orel"], 10) == 1
            and int(values["closed"], 10) == 1
            and int(values["lossus"], 10) == 0
            and int(values["diagmiss"], 10) == 0
            and int(values["finish"], 10) == 0
        )
    except (ValueError, TypeError) as error:
        raise DynamicCaptureError("H3 completion contains invalid fields") from error
    if not valid:
        raise DynamicCaptureError("H3 completion is outside the fixed safe envelope")
    if speed >= 0:
        raise DynamicCaptureError("H3 completion did not prove reverse speed")
    return speed


def validate_powered_preflight(lines: list[str]) -> int:
    safe_g431 = any(
        line.startswith("FOC st=0 rf=00000000 duty=0/0/0") for line in lines
    ) and any(
        line.startswith("FFAULT,00000000,00000000,0,0,ARM,0,0000")
        for line in lines
    )
    safe_deng = any(
        line.startswith("SYNC_STATUS,") and
        "driver_disabled=1" in line and
        "sync_level=0" in line and
        "capture_failure=0" in line and
        "overflow=0" in line and
        "truth_tx_fail=0" in line and
        "session_rollovers=" in line
        for line in lines
    )
    bus_values = [
        value
        for line in lines
        if (value := parse_fadc_bus_voltage_mv(line)) is not None
    ]
    if not (safe_g431 and safe_deng):
        raise DynamicCaptureError("powered preflight did not prove both targets safe")
    if not bus_values:
        raise DynamicCaptureError("powered preflight has no FADC bus voltage")
    bus_mv = bus_values[-1]
    if not (MIN_POWERED_BUS_MV <= bus_mv <= MAX_POWERED_BUS_MV):
        raise DynamicCaptureError(
            f"powered preflight bus voltage {bus_mv} mV is outside "
            f"{MIN_POWERED_BUS_MV}..{MAX_POWERED_BUS_MV} mV"
        )
    return bus_mv


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


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def live_capture(args: argparse.Namespace):
    if args.confirm_powered != CONFIRMATION:
        raise DynamicCaptureError(f"--confirm-powered must equal {CONFIRMATION}")
    if args.g431_port.casefold() == args.dengfoc_port.casefold():
        raise DynamicCaptureError("G431 and DengFOC ports must differ")
    if min(args.session, args.start_sequence, args.start_tag) <= 0:
        raise DynamicCaptureError("H3 identity values must be positive")
    if args.output.exists():
        raise DynamicCaptureError("output directory already exists")

    g431 = _open_serial(args.g431_port, args.g431_baud)
    deng = _open_serial(args.dengfoc_port, args.dengfoc_baud)
    stop = threading.Event()
    lock = threading.Lock()
    raw: list[tuple[str, str]] = []
    errors: list[str] = []
    query_sent = False

    def reader(source: str, stream) -> None:
        try:
            while not stop.is_set():
                payload = stream.readline()
                if payload:
                    line = payload.decode("utf-8", "replace").strip()
                    if line:
                        with lock:
                            raw.append((source, line))
        except Exception as error:
            if not stop.is_set():
                with lock:
                    errors.append(f"{source}: {error}")

    workers = [
        threading.Thread(target=reader, args=("g431", g431), daemon=True),
        threading.Thread(target=reader, args=("dengfoc", deng), daemon=True),
    ]
    for worker in workers:
        worker.start()

    def send(command: str, delay: float = 0.25) -> None:
        for character in command + "\r\n":
            g431.write(character.encode("ascii"))
            time.sleep(0.003)
        g431.flush()
        time.sleep(delay)

    try:
        time.sleep(0.8)
        send("foc_stop")
        send("foc_status", 0.6)
        time.sleep(0.5)
        with lock:
            snapshot = [line for _, line in raw]
        validate_powered_preflight(snapshot)
        send(
            f"foc_advanced_trial {COMMAND_TOKEN} {args.session} "
            f"{args.start_sequence} {args.start_tag}",
            0.0,
        )
        query_sent = True
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            with lock:
                if any(line.startswith("FH3R,") for _, line in raw):
                    break
                rejection = next((
                    line for source, line in raw
                    if source == "g431" and
                    (line.startswith("FADVP,start=") or
                     line.startswith("FADVP,h3-") or
                     line.startswith("FADVP,h3-active"))
                ), None)
            if rejection is not None:
                raise DynamicCaptureError(
                    f"target rejected bounded H3 trial: {rejection}"
                )
            time.sleep(0.05)
        else:
            raise DynamicCaptureError("bounded H3 trial did not report completion")
        time.sleep(0.8)
    finally:
        try:
            send("foc_stop", 0.15)
            send("foc_status", 0.45)
        finally:
            stop.set()
            g431.close()
            deng.close()
            for worker in workers:
                worker.join(timeout=0.5)
            if query_sent:
                write_raw_checkpoint(
                    args.output,
                    raw,
                    "captured-pending-analysis",
                )
    if errors:
        raise DynamicCaptureError("; ".join(errors))
    return raw


def analyse(raw: list[tuple[str, str]]):
    validate_reverse_completion(raw)
    left = []
    right = []
    truth = []
    queries = []
    result_count = None
    result_position = None
    safe_positions = []
    malformed_truth_sample_count = 0
    terminal_partial_truth_sample_count = 0
    for position, (source, line) in enumerate(raw):
        if source == "g431":
            anchor = parse_dynamic_anchor(line)
            if anchor is not None:
                left.append(anchor)
            query = parse_ftr_query(line)
            if query is not None:
                queries.append(query)
            if line.startswith("FH3R,"):
                fields = line.split(",")
                if len(fields) != 4 or fields[2:] != ["50", "1"]:
                    raise DynamicCaptureError("H3 result schema mismatch")
                result_count = int(fields[1], 10)
                result_position = position
            if line.startswith("FOC st=0 rf=00000000 duty=0/0/0"):
                safe_positions.append(position)
        else:
            anchor = board.parse_deng_anchor(line)
            if anchor is not None:
                right.append(anchor)
            sample = board.parse_truth(line)
            if sample is not None:
                truth.append(sample)
            elif source == "dengfoc" and line.startswith("SYNC_TRUTH,"):
                if position == (len(raw) - 1):
                    terminal_partial_truth_sample_count += 1
                else:
                    malformed_truth_sample_count += 1
    pairs = board.pair_anchors(left, right)
    if result_count != len(pairs) or len(pairs) < 3:
        raise DynamicCaptureError("reported and paired H3 anchor counts differ")
    if result_position is None or not any(
        position > result_position for position in safe_positions
    ):
        terminal_states = [
            line
            for position, (source, line) in enumerate(raw)
            if source == "g431"
            and position > (result_position if result_position is not None else -1)
            and line.startswith("FOC st=0 rf=")
            and line.endswith("duty=0/0/0")
        ]
        if terminal_states:
            raise DynamicCaptureError(
                f"final G431 state is fail-closed: {terminal_states[-1]}"
            )
        raise DynamicCaptureError("final G431 disabled state is missing")
    if malformed_truth_sample_count != 0:
        raise DynamicCaptureError(
            "AS5600 truth contains "
            f"{malformed_truth_sample_count} malformed sample(s)"
        )
    truth, truth_quality = prepare_truth_stream(truth)
    truth_quality["malformed_truth_sample_count"] = (
        malformed_truth_sample_count
    )
    truth_quality["terminal_partial_truth_sample_count"] = (
        terminal_partial_truth_sample_count
    )
    lower = min(item.control_tick for item, _ in pairs)
    upper = max(item.control_tick for item, _ in pairs)
    queries = [
        row for row in queries
        if lower <= int(row["reference_control_tick"]) <= upper
    ]
    if not queries:
        raise DynamicCaptureError("no FTR query is bracketed by H3 anchors")
    for sequence, row in enumerate(queries):
        row["sequence"] = sequence
    return pairs, truth, queries, truth_quality


def write_artifacts(
    output: Path, raw, pairs, truth, queries, truth_quality
) -> dict[str, object]:
    output.mkdir(parents=True, exist_ok=True)
    anchors_path = output / "anchors.csv"
    with anchors_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["edge_sequence", "reference_control_tick", "truth_tick"],
        )
        writer.writeheader()
        for left, right in pairs:
            writer.writerow({
                "edge_sequence": left.edge_sequence,
                "reference_control_tick": left.control_tick,
                "truth_tick": right.truth_tick_us,
            })
    truth_path = output / "truth.csv"
    with truth_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["sequence", "truth_tick", "mechanical_angle_rad", "valid", "read_failures"],
        )
        writer.writeheader()
        writer.writerows(truth)
    query_path = output / "queries.csv"
    with query_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "sequence",
            "reference_control_tick",
            "controller_state",
            "control_electrical_angle_rad",
            "forced_electrical_angle_rad",
            "estimated_electrical_angle_rad",
            "reliable",
        ])
        writer.writeheader()
        writer.writerows(queries)
    raw_path = output / "raw-lines.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "raw_line"])
        writer.writerows(raw)
    summary = {
        "contract": "fluxrt-h3-dynamic-board-capture",
        "version": 1,
        "status": "captured-not-approved",
        "anchor_count": len(pairs),
        "truth_sample_count": len(truth),
        "truth_quality": truth_quality,
        "query_count": len(queries),
        "motor_power_enabled": True,
        "power_window": "target-owned-bounded",
        "command_token": COMMAND_TOKEN,
        "commanded_mechanical_direction": COMMAND_DIRECTION,
        "commanded_target_speed_rpm": COMMAND_TARGET_SPEED_RPM,
        "attempt_limit": 1,
        "automatic_retry": False,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
    }
    summary["artifacts"] = {
        path.name: _sha256(path)
        for path in (anchors_path, truth_path, query_path, raw_path)
    }
    (output / "capture.summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g431-port", required=True)
    parser.add_argument("--g431-baud", type=int, default=115200)
    parser.add_argument("--dengfoc-port", required=True)
    parser.add_argument("--dengfoc-baud", type=int, default=921600)
    parser.add_argument("--session", type=int, required=True)
    parser.add_argument("--start-sequence", type=int, required=True)
    parser.add_argument("--start-tag", type=int, required=True)
    parser.add_argument("--confirm-powered", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        raw = live_capture(args)
        pairs, truth, queries, truth_quality = analyse(raw)
        summary = write_artifacts(
            args.output, raw, pairs, truth, queries, truth_quality
        )
    except (DynamicCaptureError, board.CaptureError, OSError) as error:
        mark_checkpoint_failed(args.output, str(error))
        parser.error(str(error))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
