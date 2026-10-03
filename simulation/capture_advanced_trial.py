#!/usr/bin/env python3
"""Capture exactly one bounded P5.4 Advanced trial without losing its final line.

The command and token are intentionally fixed.  A real run requires both
``--allow-motor-run`` and ``--confirm-safe-setup``; every exit path sends two
``foc_stop`` commands, requests final status and writes the raw log.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import re
import subprocess
import sys
import time
from typing import Callable


TRIAL_COMMAND = "foc_advanced_trial P54-BASIC-100MS"
FIRMWARE_TIMEOUT_S = 16.0
DEFAULT_CAPTURE_TIMEOUT_S = 18.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_CAPTURE_TIMEOUT_S)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--allow-motor-run", action="store_true")
    parser.add_argument(
        "--confirm-safe-setup",
        action="store_true",
        help="confirm 12.3 V, <=2 A, unloaded/free rotor and immediate power cut",
    )
    return parser.parse_args()


def send(port: object, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def read_available(port: object, raw: list[str], phase: str) -> list[str]:
    lines: list[str] = []
    while port.in_waiting:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if line:
            raw.append(f"[{phase}] {line}")
            lines.append(line)
    return lines


def collect_for(
    port: object,
    seconds: float,
    raw: list[str],
    phase: str,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> list[str]:
    deadline = clock() + seconds
    lines: list[str] = []
    while clock() < deadline:
        lines.extend(read_available(port, raw, phase))
        sleeper(0.005)
    lines.extend(read_available(port, raw, phase))
    return lines


def parse_preflight(lines: list[str]) -> dict[str, int | bool]:
    text = "\n".join(lines)
    fadc = next((line for line in reversed(lines) if "FADC," in line), "")
    try:
        adc_fields = fadc[fadc.index("FADC,") + 5 :].split(",")
        bus_raw = int(adc_fields[-3], 0)
        bus_mv = int(adc_fields[-2], 0)
    except (ValueError, IndexError):
        bus_raw = -1
        bus_mv = -1
    safe = (
        "FSTAT,disabled," in text
        and "FOC st=0 rf=00000000 duty=0/0/0" in text
        and "FFAULT,00000000,00000000,0,0" in text
        and re.search(r"\bmiss=0(?:\s|$)", text) is not None
        and 10000 <= bus_mv <= 15000
    )
    return {"safe": safe, "bus_raw": bus_raw, "bus_mv": bus_mv}


def parse_completion(line: str) -> dict[str, str] | None:
    marker = line.find("FADVP,")
    if marker < 0:
        return None
    fields = line[marker + len("FADVP,") :].split(",")
    values: dict[str, str] = {}
    for field in fields:
        if "=" not in field:
            return None
        key, value = field.split("=", 1)
        values[key] = value
    if "result" not in values:
        return None
    return values


def shutdown_output_safe(lines: list[str]) -> bool:
    return any(
        re.search(r"\bFOC st=0 rf=[0-9A-Fa-f]{8} duty=0/0/0(?:\s|$)", line)
        is not None
        for line in lines
    )


def collect_completion(
    port: object,
    timeout_s: float,
    raw: list[str],
    *,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> tuple[str, dict[str, str]]:
    deadline = clock() + timeout_s
    while clock() < deadline:
        for line in read_available(port, raw, "trial"):
            completion = parse_completion(line)
            if completion is not None:
                return line, completion
        sleeper(0.005)
    for line in read_available(port, raw, "trial"):
        completion = parse_completion(line)
        if completion is not None:
            return line, completion
    raise RuntimeError(f"no FADVP completion within {timeout_s:g} s")


def safe_stop_and_status(
    port: object,
    raw: list[str],
    *,
    collector: Callable[[object, float, list[str], str], list[str]] = collect_for,
) -> list[str]:
    send(port, "foc_stop")
    collector(port, 0.25, raw, "cleanup-stop-1")
    send(port, "foc_stop")
    collector(port, 0.25, raw, "cleanup-stop-2")
    send(port, "foc_status")
    return collector(port, 1.0, raw, "cleanup-status")


def git_commit(project: pathlib.Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=project,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def write_outputs(
    output: pathlib.Path,
    raw: list[str],
    document: dict[str, object],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    output.with_suffix(".log").write_text("\n".join(raw) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if not args.allow_motor_run or not args.confirm_safe_setup:
        raise SystemExit(
            "refusing motor run without --allow-motor-run and --confirm-safe-setup"
        )
    if not FIRMWARE_TIMEOUT_S + 1.0 <= args.timeout_s <= 30.0:
        raise SystemExit("timeout must be within 17..30 s")

    try:
        import serial
    except ModuleNotFoundError as error:
        raise RuntimeError("pyserial is required for a hardware capture") from error

    project = pathlib.Path(__file__).resolve().parents[1]
    raw: list[str] = []
    completion_line = ""
    completion: dict[str, str] | None = None
    error_text = ""
    preflight: dict[str, int | bool] = {"safe": False, "bus_raw": -1, "bus_mv": -1}
    final_status: list[str] = []

    try:
        with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0) as port:
            time.sleep(0.25)
            port.reset_input_buffer()
            try:
                send(port, "foc_stop")
                collect_for(port, 0.25, raw, "preflight-stop")
                send(port, "foc_status")
                preflight_lines = collect_for(port, 1.0, raw, "preflight-status")
                preflight = parse_preflight(preflight_lines)
                if not bool(preflight["safe"]):
                    raise RuntimeError(f"unsafe preflight: {preflight}")
                send(port, TRIAL_COMMAND)
                completion_line, completion = collect_completion(
                    port, args.timeout_s, raw
                )
            finally:
                final_status = safe_stop_and_status(port, raw)
    except (RuntimeError, serial.SerialException) as error:
        error_text = str(error)

    shutdown_safe = shutdown_output_safe(final_status)
    if not error_text and not shutdown_safe:
        error_text = "cleanup status did not prove st=0 and duty=0/0/0"

    document: dict[str, object] = {
        "schema_version": 1,
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "git_commit": git_commit(project),
        "port": args.port,
        "baud": args.baud,
        "timeout_s": args.timeout_s,
        "command": TRIAL_COMMAND,
        "preflight": preflight,
        "completion_line": completion_line,
        "completion": completion,
        "final_status": final_status,
        "shutdown_output_safe": shutdown_safe,
        "error": error_text,
    }
    write_outputs(args.output, raw, document)
    print(f"FOC_ADVANCED_TRIAL={args.output}")
    print(f"FOC_ADVANCED_TRIAL_RAW={args.output.with_suffix('.log')}")
    if error_text or completion is None:
        print(f"FOC_ADVANCED_TRIAL_CAPTURE_FAIL error={error_text}")
        return 1
    print("FOC_ADVANCED_TRIAL_CAPTURE_PASS")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
