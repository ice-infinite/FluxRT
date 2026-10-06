#!/usr/bin/env python3
"""Read-only, common-clock capture for G431 logs and DengFOC AS5600 truth.

This tool deliberately has no serial write path.  It timestamps complete lines
from both ports against one ``time.monotonic_ns()`` origin and only packages
evidence for later H2 analysis.  It cannot arm or start either controller.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

from dengfoc_angle_capture import parse_telemetry_line


CONTRACT = "fluxrt-dual-serial-capture"
VERSION = 1


class ReadableSerial(Protocol):
    def readline(self) -> bytes: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class SourceConfig:
    name: str
    port: str
    baud: int


@dataclass(frozen=True)
class CapturedLine:
    source: str
    line_sequence: int
    host_time_ns: int
    host_time_s: float
    raw_line: str


def validate_sources(g431: SourceConfig, dengfoc: SourceConfig, duration_s: float) -> None:
    if duration_s <= 0.0:
        raise ValueError("duration must be positive")
    if g431.baud <= 0 or dengfoc.baud <= 0:
        raise ValueError("baud must be positive")
    if not g431.port.strip() or not dengfoc.port.strip():
        raise ValueError("both serial ports are required")
    if g431.port.casefold() == dengfoc.port.casefold():
        raise ValueError("G431 and DengFOC must use different serial ports")


def _open_read_only_serial(config: SourceConfig) -> ReadableSerial:
    import serial

    # Configure modem-control outputs before opening so opening the port does
    # not intentionally pulse reset/boot pins.  No write method is called by
    # this module.
    stream = serial.Serial()
    stream.port = config.port
    stream.baudrate = config.baud
    stream.timeout = 0.10
    stream.write_timeout = 0
    stream.dtr = False
    stream.rts = False
    stream.open()
    return stream


def _reader(
    config: SourceConfig,
    stream: ReadableSerial,
    origin_ns: int,
    stop: threading.Event,
    records: list[CapturedLine],
    errors: list[str],
    clock_ns: Callable[[], int],
) -> None:
    sequence = 0
    try:
        while not stop.is_set():
            payload = stream.readline()
            timestamp_ns = clock_ns()
            if not payload:
                continue
            line = payload.decode("utf-8", "replace").strip()
            if not line:
                continue
            sequence += 1
            records.append(
                CapturedLine(
                    source=config.name,
                    line_sequence=sequence,
                    host_time_ns=timestamp_ns,
                    host_time_s=(timestamp_ns - origin_ns) / 1_000_000_000.0,
                    raw_line=line,
                )
            )
    except Exception as error:  # Serial failures are evidence, not silent loss.
        if not stop.is_set():
            errors.append(f"{config.name}: {error}")


def capture(
    g431: SourceConfig,
    dengfoc: SourceConfig,
    duration_s: float,
    opener: Callable[[SourceConfig], ReadableSerial] = _open_read_only_serial,
    clock_ns: Callable[[], int] = time.monotonic_ns,
) -> tuple[int, int, list[CapturedLine], list[str]]:
    validate_sources(g431, dengfoc, duration_s)
    streams: list[tuple[SourceConfig, ReadableSerial]] = []
    try:
        streams.append((g431, opener(g431)))
        streams.append((dengfoc, opener(dengfoc)))
    except Exception:
        for _, stream in streams:
            stream.close()
        raise

    origin_ns = clock_ns()
    records: list[CapturedLine] = []
    errors: list[str] = []
    stop = threading.Event()
    workers = [
        threading.Thread(
            target=_reader,
            args=(config, stream, origin_ns, stop, records, errors, clock_ns),
            name=f"capture-{config.name}",
            daemon=True,
        )
        for config, stream in streams
    ]
    for worker in workers:
        worker.start()
    stop.wait(duration_s)
    stop.set()
    for _, stream in streams:
        stream.close()
    for worker in workers:
        worker.join(timeout=1.0)
        if worker.is_alive():
            errors.append(f"{worker.name}: reader did not stop")
    end_ns = clock_ns()
    records.sort(key=lambda row: (row.host_time_ns, row.source, row.line_sequence))
    return origin_ns, end_ns, records, errors


def _source_bounds(records: list[CapturedLine], source: str) -> dict[str, int | float | None]:
    selected = [row for row in records if row.source == source]
    return {
        "line_count": len(selected),
        "first_host_time_s": selected[0].host_time_s if selected else None,
        "last_host_time_s": selected[-1].host_time_s if selected else None,
    }


def build_summary(
    g431: SourceConfig,
    dengfoc: SourceConfig,
    origin_ns: int,
    end_ns: int,
    records: list[CapturedLine],
    errors: list[str],
) -> dict[str, object]:
    angle_rows = []
    g431_raw = []
    for row in records:
        if row.source == dengfoc.name:
            parsed = parse_telemetry_line(row.raw_line, row.host_time_s)
            if parsed is not None:
                angle_rows.append(parsed)
        elif row.source == g431.name and row.raw_line.startswith("FLSI_RAW,"):
            g431_raw.append(row)

    status = "capture-complete" if not errors else "capture-failed"
    return {
        "contract": CONTRACT,
        "version": VERSION,
        "status": status,
        "read_only": True,
        "commands_sent": 0,
        "clock": {
            "source": "time.monotonic_ns",
            "shared_origin": True,
            "origin_ns": origin_ns,
            "end_ns": end_ns,
            "elapsed_s": (end_ns - origin_ns) / 1_000_000_000.0,
        },
        "sources": {
            g431.name: {
                "port": g431.port,
                "baud": g431.baud,
                **_source_bounds(records, g431.name),
                "flsi_raw_line_count": len(g431_raw),
            },
            dengfoc.name: {
                "port": dengfoc.port,
                "baud": dengfoc.baud,
                **_source_bounds(records, dengfoc.name),
                "parsed_angle_sample_count": len(angle_rows),
            },
        },
        "static_h2_association": {
            "kind": "common-host-static-window-only",
            "dynamic_alignment_valid": False,
            "g431_raw_window_start_s": g431_raw[0].host_time_s if g431_raw else None,
            "g431_raw_window_end_s": g431_raw[-1].host_time_s if g431_raw else None,
        },
        "errors": errors,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
    }


def write_artifacts(
    output: Path,
    records: list[CapturedLine],
    summary: dict[str, object],
    dengfoc_source: str = "dengfoc",
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    combined_path = output / "combined-lines.csv"
    with combined_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["source", "line_sequence", "host_time_ns", "host_time_s", "raw_line"],
        )
        writer.writeheader()
        writer.writerows(
            {
                "source": row.source,
                "line_sequence": row.line_sequence,
                "host_time_ns": row.host_time_ns,
                "host_time_s": f"{row.host_time_s:.9f}",
                "raw_line": row.raw_line,
            }
            for row in records
        )

    angle_rows: list[dict[str, int | float]] = []
    for row in records:
        if row.source != dengfoc_source:
            continue
        parsed = parse_telemetry_line(row.raw_line, row.host_time_s)
        if parsed is not None:
            angle_rows.append(parsed)
    angle_fields = [
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
    angle_path = output / "dengfoc-angle.csv"
    with angle_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=angle_fields)
        writer.writeheader()
        writer.writerows(angle_rows)
    stored_summary = dict(summary)
    stored_summary["artifacts"] = {
        "combined_lines_path": combined_path.name,
        "combined_lines_sha256": hashlib.sha256(combined_path.read_bytes()).hexdigest().upper(),
        "angle_csv_path": angle_path.name,
        "angle_csv_sha256": hashlib.sha256(angle_path.read_bytes()).hexdigest().upper(),
    }
    (output / "capture.summary.json").write_text(
        json.dumps(stored_summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g431-port", required=True)
    parser.add_argument("--g431-baud", type=int, default=115200)
    parser.add_argument("--dengfoc-port", required=True)
    parser.add_argument("--dengfoc-baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    g431 = SourceConfig("g431", args.g431_port, args.g431_baud)
    dengfoc = SourceConfig("dengfoc", args.dengfoc_port, args.dengfoc_baud)
    try:
        origin_ns, end_ns, records, errors = capture(g431, dengfoc, args.duration)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    summary = build_summary(g431, dengfoc, origin_ns, end_ns, records, errors)
    write_artifacts(args.output, records, summary, dengfoc.name)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["status"] == "capture-complete" else 2


if __name__ == "__main__":
    raise SystemExit(main())
