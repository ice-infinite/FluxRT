#!/usr/bin/env python3
"""Capture a stopped FluxRT phase-voltage window with explicit divider mode."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import time

RAW_FIELDS = [
    "sequence",
    "phase_u_raw",
    "phase_v_raw",
    "phase_w_raw",
    "current_u_raw",
    "current_v_raw",
    "bus_last_raw",
]
CSV_FIELDS = ["divider_mode", *RAW_FIELDS]
MODEL_INTEGER_FIELDS = {
    "vref_mv",
    "adc_max",
    "upper_ohm",
    "lower_ohm",
    "full_scale_mv",
    "uv_per_count",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument(
        "--divider",
        choices=("on", "off"),
        default="on",
        help="PC9 divider state during the stopped capture (default: on)",
    )
    return parser.parse_args()


def read_for(port: object, seconds: float, raw: list[str]) -> list[str]:
    deadline = time.monotonic() + seconds
    result: list[str] = []
    while time.monotonic() < deadline:
        if port.in_waiting:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if line:
                raw.append(line)
                result.append(line)
        else:
            time.sleep(0.01)
    return result


def command(
    port: object, text: str, wait_s: float, raw: list[str]
) -> list[str]:
    port.write((text + "\r\n").encode("ascii"))
    port.flush()
    return read_for(port, wait_s, raw)


def parse_sample(line: str) -> dict[str, int] | None:
    marker = line.find("FPV,")
    if marker < 0:
        return None
    values = line[marker:].split(",")[1:]
    if len(values) != len(RAW_FIELDS):
        return None
    try:
        return dict(zip(RAW_FIELDS, (int(value, 0) for value in values)))
    except ValueError:
        return None


def parse_model(line: str) -> dict[str, int | str] | None:
    """Parse the firmware-emitted nominal/calibrated model metadata."""
    marker = line.find("FPV_MODEL,")
    if marker < 0:
        return None
    result: dict[str, int | str] = {}
    for field in line[marker:].split(",")[1:]:
        if "=" not in field:
            return None
        key, value = field.split("=", 1)
        if key in MODEL_INTEGER_FIELDS:
            try:
                result[key] = int(value, 0)
            except ValueError:
                return None
        else:
            result[key] = value
    required = MODEL_INTEGER_FIELDS | {"source", "quality", "observer"}
    return result if required.issubset(result) else None


def main() -> int:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required for board capture") from error

    args = parse_args()
    raw: list[str] = []
    samples: list[dict[str, int | str]] = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix(".log")
    metadata_path = args.output.with_suffix(".json")

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        try:
            read_for(port, 0.4, raw)
            command(port, "foc_stop", 0.25, raw)
            command(port, "foc_phase_capture stop", 0.15, raw)
            start_lines = command(
                port, f"foc_phase_capture start {args.divider}", 0.25, raw
            )
            if not any(
                "phase capture armed" in line and f"divider={args.divider}" in line
                for line in start_lines
            ):
                raise RuntimeError("firmware refused phase-voltage capture")
            status_lines = command(port, "foc_phase_capture status", 0.25, raw)
            if not any(
                "state=complete" in line
                and "samples=256/256" in line
                and f"divider={args.divider}" in line
                for line in status_lines
            ):
                raise RuntimeError("capture did not complete 256 samples")
            dump_lines = command(port, "foc_phase_capture dump", 3.0, raw)
            if not any(f"FPV_META,divider={args.divider}" in line for line in dump_lines):
                raise RuntimeError("capture dump mode does not match the request")
            models = [model for line in dump_lines if (model := parse_model(line))]
            if len(models) != 1:
                raise RuntimeError("firmware did not report exactly one phase-voltage model")
            phase_voltage_model = models[0]
            if phase_voltage_model["observer"] != "disabled":
                raise RuntimeError("unapproved phase-voltage model is observer eligible")
            command(port, "foc_status", 0.5, raw)
        finally:
            command(port, "foc_phase_capture stop", 0.1, raw)
            command(port, "foc_stop", 0.2, raw)

    # A final FPV line can arrive immediately after the fixed dump wait and be
    # consumed by the following status/stop read.  Parse the complete session
    # buffer only after the fail-safe stop commands have drained the port.
    for line in raw:
        sample = parse_sample(line)
        if sample is not None:
            sample["divider_mode"] = args.divider
            samples.append(sample)

    log_path.write_text("\n".join(raw) + "\n", encoding="utf-8")
    if len(samples) != 256:
        raise RuntimeError(f"expected 256 samples, got {len(samples)}")
    if [sample["sequence"] for sample in samples] != list(range(256)):
        raise RuntimeError("sample sequence is not contiguous 0..255")
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(samples)

    summary = {
        "format_version": 1,
        "capture_kind": "stopped-phase-voltage-window",
        "divider_mode": args.divider,
        "sample_rate_hz": 12000,
        "samples": len(samples),
        "sequence_first": samples[0]["sequence"],
        "sequence_last": samples[-1]["sequence"],
        "output": str(args.output),
        "output_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "log": str(log_path),
        "log_sha256": hashlib.sha256(log_path.read_bytes()).hexdigest(),
        "phase_voltage_model": phase_voltage_model,
    }
    for phase in ("phase_u_raw", "phase_v_raw", "phase_w_raw"):
        values = [sample[phase] for sample in samples]
        summary[phase] = {
            "min": min(values),
            "mean": sum(values) / len(values),
            "max": max(values),
        }
    metadata_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    summary["metadata"] = str(metadata_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
