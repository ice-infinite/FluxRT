#!/usr/bin/env python3
"""Compare independent Rust and MATLAB full-speed dynamic design models."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


TRACE_COLUMNS = [
    "case_id", "tick", "time_s", "target_rpm", "speed_rpm", "id_a", "iq_a",
    "id_ref_a", "iq_ref_a", "vd_v", "vq_v", "dc_bus_v", "source_current_a",
    "load_torque_nm", "region", "active_features", "voltage_limited",
]
METRIC_COLUMNS = [
    "case_id", "final_target_rpm", "final_speed_rpm", "final_speed_error_rpm",
    "peak_current_a", "min_bus_v", "max_bus_v", "voltage_limited_fraction",
]
EXACT_TRACE = {"case_id", "tick", "region", "active_features", "voltage_limited"}
TRACE_TOLERANCE = {
    "time_s": (1e-7, 1e-6), "target_rpm": (1e-5, 1e-6),
    "speed_rpm": (0.5, 2e-3), "id_a": (0.01, 5e-3), "iq_a": (0.01, 5e-3),
    "id_ref_a": (0.01, 5e-3), "iq_ref_a": (0.01, 5e-3),
    "vd_v": (0.05, 5e-3), "vq_v": (0.05, 5e-3),
    "dc_bus_v": (0.02, 2e-3), "source_current_a": (0.05, 5e-3),
    "load_torque_nm": (1e-7, 1e-6),
}
METRIC_TOLERANCE = {
    "final_target_rpm": (1e-5, 1e-6), "final_speed_rpm": (0.5, 2e-3),
    "final_speed_error_rpm": (0.5, 2e-3), "peak_current_a": (0.02, 1e-2),
    "min_bus_v": (0.02, 2e-3), "max_bus_v": (0.02, 2e-3),
    "voltage_limited_fraction": (2e-3, 1e-2),
}
EXPECTED_CASES = [
    "forward_base", "forward_high_fw", "reverse_base", "load_step", "bus_sag",
    "regen_reversal",
]


def metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or "=" not in line:
            raise ValueError(f"invalid metadata line in {path}: {line!r}")
        key, value = line.split("=", 1)
        if key in result or not key or not value:
            raise ValueError(f"duplicate/empty metadata line in {path}: {line!r}")
        result[key] = value
    required = {
        "contract", "version", "engine", "workspace_revision", "case_count",
        "row_count", "result", "model_scope",
    }
    if set(result) != required:
        raise ValueError(f"metadata key mismatch in {path}: {set(result) ^ required}")
    if result["contract"] != "fluxrt-full-speed-closed-loop" or result["version"] != "1":
        raise ValueError(f"contract identity mismatch in {path}")
    if result["result"] != "PASS" or result["model_scope"] != "design-not-hardware-truth":
        raise ValueError(f"engine did not preserve scope/pass in {path}")
    return result


def read_csv(path: Path, columns: list[str]) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != columns:
            raise ValueError(f"column mismatch in {path}: {reader.fieldnames}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty result: {path}")
    for row_index, row in enumerate(rows):
        if any(row[name] == "" for name in columns):
            raise ValueError(f"empty field in {path} row {row_index}")
        for name in columns:
            if name != "case_id" and not math.isfinite(float(row[name])):
                raise ValueError(f"non-finite {name} in {path} row {row_index}")
    return rows


def close(left: str, right: str, tolerance: tuple[float, float]) -> bool:
    a, b = float(left), float(right)
    absolute, relative = tolerance
    return abs(a - b) <= absolute + relative * max(abs(a), abs(b))


def compare_rows(
    rust: list[dict[str, str]], matlab: list[dict[str, str]], columns: list[str],
    exact: set[str], tolerances: dict[str, tuple[float, float]], label: str,
) -> None:
    if len(rust) != len(matlab):
        raise ValueError(f"{label} row count mismatch: {len(rust)} != {len(matlab)}")
    for index, (left, right) in enumerate(zip(rust, matlab, strict=True)):
        for name in columns:
            if name in exact:
                if left[name] != right[name]:
                    raise ValueError(
                        f"{label} exact mismatch row={index} field={name}: "
                        f"{left[name]!r} != {right[name]!r}"
                    )
            elif not close(left[name], right[name], tolerances[name]):
                raise ValueError(
                    f"{label} numeric mismatch row={index} field={name}: "
                    f"{left[name]} != {right[name]} tolerance={tolerances[name]}"
                )


def verify_semantics(trace: list[dict[str, str]], metrics: list[dict[str, str]]) -> None:
    case_order = list(dict.fromkeys(row["case_id"] for row in trace))
    if case_order != EXPECTED_CASES or [row["case_id"] for row in metrics] != EXPECTED_CASES:
        raise ValueError(f"case order/set mismatch: {case_order}")
    grouped = {case: [row for row in trace if row["case_id"] == case] for case in EXPECTED_CASES}
    if float(grouped["forward_base"][-1]["speed_rpm"]) <= 0:
        raise ValueError("forward case did not rotate forward")
    if float(grouped["reverse_base"][-1]["speed_rpm"]) >= 0:
        raise ValueError("reverse case did not rotate in reverse")
    if min(float(row["dc_bus_v"]) for row in grouped["bus_sag"]) >= 12.9:
        raise ValueError("bus-sag case did not exercise the dynamic DC link")
    if max(float(row["dc_bus_v"]) for row in grouped["regen_reversal"]) <= 13.0:
        raise ValueError("regeneration case did not raise a non-sinking DC link")
    for metric in metrics:
        target = abs(float(metric["final_target_rpm"]))
        allowed_error = max(50.0, 0.10 * target)
        error = abs(float(metric["final_speed_error_rpm"]))
        if error > allowed_error:
            raise ValueError(
                f"{metric['case_id']} did not settle within the design gate: "
                f"error={error:.6f} rpm allowed={allowed_error:.6f} rpm"
            )
    for row in trace:
        if not 6.0 <= float(row["dc_bus_v"]) <= 20.0:
            raise ValueError(f"DC bus escaped configured rails: {row}")
        if math.hypot(float(row["id_ref_a"]), float(row["iq_ref_a"])) > 0.801:
            raise ValueError(f"current reference escaped configured circle: {row}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-metadata", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--rust-metrics", type=Path, required=True)
    parser.add_argument("--matlab-metadata", type=Path, required=True)
    parser.add_argument("--matlab-trace", type=Path, required=True)
    parser.add_argument("--matlab-metrics", type=Path, required=True)
    args = parser.parse_args()
    rust_meta, matlab_meta = metadata(args.rust_metadata), metadata(args.matlab_metadata)
    for key in ("workspace_revision", "case_count", "row_count"):
        if rust_meta[key] != matlab_meta[key]:
            raise ValueError(f"D0 metadata mismatch {key}: {rust_meta[key]} != {matlab_meta[key]}")
    rust_trace = read_csv(args.rust_trace, TRACE_COLUMNS)
    matlab_trace = read_csv(args.matlab_trace, TRACE_COLUMNS)
    rust_metrics = read_csv(args.rust_metrics, METRIC_COLUMNS)
    matlab_metrics = read_csv(args.matlab_metrics, METRIC_COLUMNS)
    verify_semantics(rust_trace, rust_metrics)
    verify_semantics(matlab_trace, matlab_metrics)
    compare_rows(rust_trace, matlab_trace, TRACE_COLUMNS, EXACT_TRACE, TRACE_TOLERANCE, "trace")
    compare_rows(
        rust_metrics, matlab_metrics, METRIC_COLUMNS, {"case_id"}, METRIC_TOLERANCE, "metrics"
    )
    print(
        f"FULL_SPEED_DUAL_SIM_PASS cases={len(rust_metrics)} rows={len(rust_trace)} "
        "scope=design-not-hardware-truth"
    )


if __name__ == "__main__":
    main()
