#!/usr/bin/env python3
"""Compare independent Rust and MATLAB full-speed sensorless angle chains."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


TRACE_COLUMNS = [
    "case_id", "tick", "time_s", "true_angle_rad", "electrical_speed_rad_s",
    "estimated_angle_rad", "angle_error_rad", "source", "reliable",
    "fallback_required", "stage", "hfi_valid", "bemf_valid",
    "injection_alpha_v", "injection_beta_v",
]
METRIC_COLUMNS = [
    "case_id", "rms_angle_error_rad", "maximum_angle_error_rad",
    "maximum_angle_step_rad", "unreliable_fraction", "source_mask", "transition_mask",
    "fallback_seen", "final_reliable",
]
EXACT_TRACE = {
    "case_id", "tick", "source", "reliable", "fallback_required", "stage",
    "hfi_valid", "bemf_valid",
}
EXACT_METRIC = {"case_id", "source_mask", "transition_mask", "fallback_seen", "final_reliable"}
TRACE_TOLERANCE = {
    "time_s": (1e-6, 1e-6),
    "true_angle_rad": (0.02, 2e-3),
    "electrical_speed_rad_s": (2e-3, 1e-5),
    "estimated_angle_rad": (0.03, 3e-3),
    "angle_error_rad": (0.03, 3e-3),
    "injection_alpha_v": (0.02, 2e-3),
    "injection_beta_v": (0.02, 2e-3),
}
METRIC_TOLERANCE = {
    "rms_angle_error_rad": (0.02, 2e-2),
    "maximum_angle_error_rad": (0.03, 2e-2),
    "maximum_angle_step_rad": (0.03, 2e-2),
    "unreliable_fraction": (2e-3, 1e-2),
}
EXPECTED_CASES = [
    "zero_to_forward", "zero_to_reverse", "forward_accel_decel",
    "bidirectional_reversal", "reverse_accel_decel", "bemf_loss_overlap",
    "hfi_loss_overlap", "bemf_loss_high_speed", "hfi_loss_low_speed",
]


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            raise ValueError(f"invalid metadata line: {line!r}")
        key, value = line.split("=", 1)
        if not key or not value or key in result:
            raise ValueError(f"duplicate/empty metadata field: {line!r}")
        result[key] = value
    required = {
        "contract", "version", "engine", "workspace_revision", "case_count",
        "row_count", "result", "model_scope",
    }
    if set(result) != required:
        raise ValueError(f"metadata fields differ: {set(result) ^ required}")
    if result["contract"] != "fluxrt-sensorless-full-speed-chain" or result["version"] != "1":
        raise ValueError("metadata identity mismatch")
    if result["result"] != "PASS" or result["model_scope"] != "design-not-hardware-truth":
        raise ValueError("metadata evidence scope mismatch")
    return result


def read_csv(path: Path, columns: list[str]) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != columns:
            raise ValueError(f"column mismatch in {path}: {reader.fieldnames}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty result: {path}")
    for index, row in enumerate(rows):
        for name in columns:
            if row[name] == "":
                raise ValueError(f"empty {name} in row {index}")
            if name != "case_id" and not math.isfinite(float(row[name])):
                raise ValueError(f"non-finite {name} in row {index}")
    return rows


def close(left: str, right: str, tolerance: tuple[float, float]) -> bool:
    a, b = float(left), float(right)
    absolute, relative = tolerance
    return abs(a - b) <= absolute + relative * max(abs(a), abs(b))


def compare(
    left: list[dict[str, str]], right: list[dict[str, str]], columns: list[str],
    exact: set[str], tolerances: dict[str, tuple[float, float]], label: str,
) -> None:
    if len(left) != len(right):
        raise ValueError(f"{label} row count mismatch: {len(left)} != {len(right)}")
    for index, (a, b) in enumerate(zip(left, right, strict=True)):
        for name in columns:
            if name in exact:
                if a[name] != b[name]:
                    raise ValueError(
                        f"{label} exact mismatch row={index} field={name}: "
                        f"{a[name]!r} != {b[name]!r}"
                    )
            elif not close(a[name], b[name], tolerances[name]):
                raise ValueError(
                    f"{label} numeric mismatch row={index} field={name}: "
                    f"{a[name]} != {b[name]} tolerance={tolerances[name]}"
                )


def verify_semantics(metrics: list[dict[str, str]]) -> None:
    if [row["case_id"] for row in metrics] != EXPECTED_CASES:
        raise ValueError("case order/set mismatch")
    for row in metrics:
        if float(row["rms_angle_error_rad"]) > 0.20:
            raise ValueError(f"RMS angle gate failed: {row}")
        if float(row["maximum_angle_step_rad"]) > 0.50:
            raise ValueError(f"angle continuity gate failed: {row}")
        if row["final_reliable"] != "1":
            raise ValueError(f"case did not recover by final sample: {row}")
    for case in (
        "zero_to_forward", "zero_to_reverse", "forward_accel_decel",
        "bidirectional_reversal", "reverse_accel_decel",
    ):
        row = next(value for value in metrics if value["case_id"] == case)
        if row["source_mask"] != "7" or row["fallback_seen"] != "0":
            raise ValueError(f"nominal source/fallback semantics failed: {row}")
    for case in ("bemf_loss_overlap", "hfi_loss_overlap"):
        row = next(value for value in metrics if value["case_id"] == case)
        if row["fallback_seen"] != "0":
            raise ValueError(f"single-channel overlap loss caused fallback: {row}")
    for case in ("bemf_loss_high_speed", "hfi_loss_low_speed"):
        row = next(value for value in metrics if value["case_id"] == case)
        if row["fallback_seen"] != "1":
            raise ValueError(f"fault case never reached fallback: {row}")


def main() -> None:
    parser = argparse.ArgumentParser()
    for name in ("rust-metadata", "rust-trace", "rust-metrics", "matlab-metadata", "matlab-trace", "matlab-metrics"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args()
    rust_meta = read_metadata(args.rust_metadata)
    matlab_meta = read_metadata(args.matlab_metadata)
    for key in ("workspace_revision", "case_count", "row_count"):
        if rust_meta[key] != matlab_meta[key]:
            raise ValueError(f"metadata mismatch {key}: {rust_meta[key]} != {matlab_meta[key]}")
    rust_trace = read_csv(args.rust_trace, TRACE_COLUMNS)
    matlab_trace = read_csv(args.matlab_trace, TRACE_COLUMNS)
    rust_metrics = read_csv(args.rust_metrics, METRIC_COLUMNS)
    matlab_metrics = read_csv(args.matlab_metrics, METRIC_COLUMNS)
    verify_semantics(rust_metrics)
    verify_semantics(matlab_metrics)
    compare(rust_trace, matlab_trace, TRACE_COLUMNS, EXACT_TRACE, TRACE_TOLERANCE, "trace")
    compare(rust_metrics, matlab_metrics, METRIC_COLUMNS, EXACT_METRIC, METRIC_TOLERANCE, "metrics")
    print(f"SENSORLESS_FULL_SPEED_DUAL_SIM_PASS cases={len(rust_metrics)} rows={len(rust_trace)}")


if __name__ == "__main__":
    main()
