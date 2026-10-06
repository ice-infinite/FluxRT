#!/usr/bin/env python3
"""Capture the one fixed no-power H3 frame from the dynamic-query image.

This tool cannot choose identity, timing or frame count.  It invokes only the
target-owned ``foc_h3_no_power_probe`` after proving both boards disabled and
Vbus below 0.5 V, persists raw lines before analysis, and always sends
``foc_stop`` plus a final status request.
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


CONFIRMATION = "NO_POWER_H3_FIXED_PROBE"
MAX_NO_POWER_BUS_MV = 500
ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


board = _load("foc_h3_board_capture_fixed", ROOT / "tools/foc_h3_board_capture.py")
dynamic = _load(
    "foc_h3_dynamic_capture_fixed", ROOT / "tools/foc_h3_dynamic_capture.py"
)


class FixedProbeError(ValueError):
    """Raised when fixed no-power evidence is incomplete or unsafe."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def _write_raw(output: Path, raw: list[tuple[str, str]]) -> None:
    output.mkdir(parents=True, exist_ok=False)
    raw_path = output / "raw-lines.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "raw_line"])
        writer.writerows(raw)
    summary = {
        "contract": "fluxrt-h3-fixed-no-power-probe",
        "version": 1,
        "status": "captured-pending-analysis",
        "motor_power_enabled": False,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
        "artifacts": {raw_path.name: _sha256(raw_path)},
    }
    (output / "capture.summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _finish_summary(output: Path, status: str, **fields: object) -> dict[str, object]:
    path = output / "capture.summary.json"
    summary = json.loads(path.read_text(encoding="utf-8"))
    summary["status"] = status
    summary.update(fields)
    path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def validate_preflight(lines: list[str]) -> int:
    safe_g431 = any(
        line.startswith("FOC st=0 rf=00000000 duty=0/0/0") for line in lines
    ) and any(
        line.startswith("FFAULT,00000000,00000000,0,0,ARM,0,0000")
        for line in lines
    )
    safe_deng = any(
        line.startswith("SYNC_STATUS,") and
        "driver_disabled=1" in line and
        "capture_failure=0" in line and
        "overflow=0" in line and
        "truth_tx_fail=0" in line and
        "session_rollovers=" in line
        for line in lines
    )
    bus_values = [
        value
        for line in lines
        if (value := dynamic.parse_fadc_bus_voltage_mv(line)) is not None
    ]
    if not (safe_g431 and safe_deng):
        raise FixedProbeError("no-power preflight did not prove both targets disabled")
    if not bus_values:
        raise FixedProbeError("no-power preflight has no FADC bus voltage")
    bus_mv = bus_values[-1]
    if bus_mv > MAX_NO_POWER_BUS_MV:
        raise FixedProbeError(
            f"no-power preflight bus voltage {bus_mv} mV exceeds "
            f"{MAX_NO_POWER_BUS_MV} mV"
        )
    return bus_mv


def analyse(raw: list[tuple[str, str]]) -> dict[str, object]:
    left = []
    right = []
    result_position = None
    safe_positions = []
    for position, (source, line) in enumerate(raw):
        if source == "g431":
            anchor = dynamic.parse_dynamic_anchor(line)
            if anchor is not None:
                left.append(anchor)
            if line == "FH3_NOPWR,1":
                result_position = position
            if line.startswith("FOC st=0 rf=00000000 duty=0/0/0"):
                safe_positions.append(position)
        else:
            anchor = board.parse_deng_anchor(line)
            if anchor is not None:
                right.append(anchor)
    if len(left) != 1 or len(right) != 1:
        raise FixedProbeError(
            f"expected one G431 and one DengFOC anchor, got {len(left)}/{len(right)}"
        )
    pairs = board.pair_anchors(left, right)
    if len(pairs) != 1:
        raise FixedProbeError("fixed probe identity did not pair")
    if result_position is None:
        raise FixedProbeError("target did not report FH3_NOPWR success")
    if not any(position > result_position for position in safe_positions):
        raise FixedProbeError("final G431 disabled state is missing")
    g431, deng = pairs[0]
    return {
        "anchor_count": 1,
        "session_id": g431.session_id,
        "edge_sequence": g431.edge_sequence,
        "edge_tag": g431.edge_tag,
        "reference_control_tick": g431.control_tick,
        "truth_tick_us": deng.truth_tick_us,
        "g431_flags": g431.flags,
        "deng_flags": deng.flags,
        "loopback_delta_cycles": g431.loopback_delta_cycles,
    }


def live_capture(args: argparse.Namespace) -> tuple[list[tuple[str, str]], int]:
    if args.confirm_no_power != CONFIRMATION:
        raise FixedProbeError(f"--confirm-no-power must equal {CONFIRMATION}")
    if args.g431_port.casefold() == args.dengfoc_port.casefold():
        raise FixedProbeError("G431 and DengFOC ports must differ")
    if args.output.exists():
        raise FixedProbeError("output directory already exists")

    g431 = dynamic._open_serial(args.g431_port, args.g431_baud)
    deng = dynamic._open_serial(args.dengfoc_port, args.dengfoc_baud)
    stop = threading.Event()
    lock = threading.Lock()
    raw: list[tuple[str, str]] = []
    errors: list[str] = []
    probe_sent = False

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

    def send(command: str, delay: float) -> None:
        for character in command + "\r\n":
            g431.write(character.encode("ascii"))
            time.sleep(0.003)
        g431.flush()
        time.sleep(delay)

    bus_mv = -1
    try:
        time.sleep(0.8)
        send("foc_stop", 0.25)
        send("foc_status", 0.6)
        time.sleep(0.5)
        with lock:
            snapshot = [line for _, line in raw]
        bus_mv = validate_preflight(snapshot)
        send("foc_h3_no_power_probe", 0.0)
        probe_sent = True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with lock:
                if any(line.startswith("FH3_NOPWR,") for _, line in raw):
                    break
            time.sleep(0.02)
        else:
            raise FixedProbeError("fixed no-power probe did not report completion")
        time.sleep(0.5)
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
            if probe_sent:
                _write_raw(args.output, raw)
    if errors:
        raise FixedProbeError("; ".join(errors))
    return raw, bus_mv


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g431-port", required=True)
    parser.add_argument("--g431-baud", type=int, default=115200)
    parser.add_argument("--dengfoc-port", required=True)
    parser.add_argument("--dengfoc-baud", type=int, default=921600)
    parser.add_argument("--confirm-no-power", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        raw, bus_mv = live_capture(args)
        result = analyse(raw)
        summary = _finish_summary(
            args.output,
            "captured-not-approved",
            bus_voltage_mv=bus_mv,
            **result,
        )
    except (FixedProbeError, board.CaptureError, OSError) as error:
        if args.output.is_dir():
            _finish_summary(args.output, "capture-failed", error=str(error))
        parser.error(str(error))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
