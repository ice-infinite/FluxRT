#!/usr/bin/env python3
"""Strict D0/D2/D3/D4 comparison for independent Rust and MATLAB feedback models."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


ENGINE_IDS = {
    "fluxrt.rust.feedback_sensor.v1",
    "fluxrt.matlab.feedback_sensor.v1",
}
EXACT_COLUMNS = {
    "case_id",
    "control_tick",
    "feedback_primary_available",
    "feedback_backup_available",
    "feedback_active_mode",
    "feedback_route_state",
    "feedback_valid_flags",
    "feedback_quality_flags",
    "feedback_event",
}
NUMERIC_COLUMNS = {
    "time_s",
    "truth_position_rad",
    "truth_velocity_rad_s",
    "feedback_encoder_position_rad",
    "feedback_hall_electrical_angle_rad",
    "feedback_selected_position_rad",
    "feedback_selected_velocity_rad_s",
    "feedback_selected_electrical_angle_rad",
    "feedback_sample_age_s",
}


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or "=" not in line:
            raise ValueError(f"invalid metadata line in {path}: {line!r}")
        key, value = line.split("=", 1)
        if not key or key in result:
            raise ValueError(f"duplicate/empty metadata key in {path}: {key!r}")
        result[key] = value
    return result


def read_trace(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or set(reader.fieldnames) != EXACT_COLUMNS | NUMERIC_COLUMNS:
            raise ValueError(f"unexpected feedback trace columns in {path}: {reader.fieldnames}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty feedback trace: {path}")
    keys = [(row["case_id"], row["control_tick"]) for row in rows]
    if len(keys) != len(set(keys)):
        raise ValueError(f"duplicate case/tick row in {path}")
    return rows


def load_tolerances(path: Path) -> dict[str, tuple[float, float]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    result: dict[str, tuple[float, float]] = {}
    for item in document["numeric_tolerances"]:
        result[item["channel"]] = (float(item["absolute"]), float(item["relative"]))
    missing = {
        name
        for name in NUMERIC_COLUMNS
        if name.startswith("feedback_") and name not in result
    }
    if missing:
        raise ValueError(f"feedback numeric tolerances missing: {sorted(missing)}")
    result.setdefault("time_s", (1.0e-9, 1.0e-9))
    result.setdefault("truth_position_rad", (1.0e-9, 1.0e-9))
    result.setdefault("truth_velocity_rad_s", (1.0e-9, 1.0e-9))
    return result


def numeric_equal(left: str, right: str, absolute: float, relative: float) -> bool:
    if not left or not right:
        return left == right
    a = float(left)
    b = float(right)
    if not math.isfinite(a) or not math.isfinite(b):
        return False
    return abs(a - b) <= absolute + relative * max(abs(a), abs(b))


def verify_semantics(rows: list[dict[str, str]]) -> None:
    by_case: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_case.setdefault(row["case_id"], []).append(row)
    required = {
        "feature_off",
        "nominal",
        "encoder_drop_fallback",
        "encoder_index_missing",
        "hall_wrong_order_and_primary_drop",
    }
    if set(by_case) != required:
        raise ValueError(f"feedback case set mismatch: {sorted(by_case)}")
    for case_rows in by_case.values():
        ticks = [int(row["control_tick"]) for row in case_rows]
        if ticks != list(range(len(case_rows))):
            raise ValueError("feedback ticks must be contiguous and start at zero")

    for row in by_case["feature_off"]:
        if row["feedback_route_state"] != "Bypassed" or row["feedback_active_mode"] != "Sensorless":
            raise ValueError("feature-off route is not the legacy Sensorless bypass")
        if row["feedback_selected_position_rad"] != row["truth_position_rad"]:
            raise ValueError("feature-off position is not exactly equivalent")
        if row["feedback_selected_velocity_rad_s"] != row["truth_velocity_rad_s"]:
            raise ValueError("feature-off velocity is not exactly equivalent")

    drop_events = {
        (int(row["control_tick"]), row["feedback_event"])
        for row in by_case["encoder_drop_fallback"]
        if row["feedback_event"] != "none"
    }
    if (7, "fallback_entered") not in drop_events or (12, "primary_recovered") not in drop_events:
        raise ValueError(f"fallback/recovery timing mismatch: {sorted(drop_events)}")
    if not any(
        row["feedback_route_state"] == "Fallback"
        and row["feedback_active_mode"] == "Hall"
        for row in by_case["encoder_index_missing"]
    ):
        raise ValueError("missing encoder Index did not force Hall fallback")
    if not any(
        row["feedback_route_state"] == "Lost"
        and row["feedback_valid_flags"] == "0"
        for row in by_case["hall_wrong_order_and_primary_drop"]
    ):
        raise ValueError("combined primary/backup fault did not fail closed")


def compare(
    rust_metadata_path: Path,
    rust_trace_path: Path,
    matlab_metadata_path: Path,
    matlab_trace_path: Path,
    comparison_path: Path,
) -> None:
    rust_metadata = read_metadata(rust_metadata_path)
    matlab_metadata = read_metadata(matlab_metadata_path)
    if {rust_metadata.get("engine_id"), matlab_metadata.get("engine_id")} != ENGINE_IDS:
        raise ValueError("Rust/MATLAB feedback engines are not distinct canonical implementations")
    rust_identity = {key: value for key, value in rust_metadata.items() if key != "engine_id"}
    matlab_identity = {key: value for key, value in matlab_metadata.items() if key != "engine_id"}
    if rust_identity != matlab_identity:
        differing = sorted(
            key
            for key in set(rust_identity) | set(matlab_identity)
            if rust_identity.get(key) != matlab_identity.get(key)
        )
        raise ValueError(f"D0/summary metadata mismatch: {differing}")
    for gate in ("d0", "d2", "d3", "d4", "feature_off"):
        if rust_identity.get(gate) != "PASS":
            raise ValueError(f"gate did not pass: {gate}")

    rust_rows = read_trace(rust_trace_path)
    matlab_rows = read_trace(matlab_trace_path)
    if len(rust_rows) != len(matlab_rows) or int(rust_identity["row_count"]) != len(rust_rows):
        raise ValueError("feedback row count mismatch")
    tolerances = load_tolerances(comparison_path)
    for index, (rust_row, matlab_row) in enumerate(zip(rust_rows, matlab_rows, strict=True)):
        for column in EXACT_COLUMNS:
            if rust_row[column] != matlab_row[column]:
                raise ValueError(
                    f"D2/D4 mismatch row={index} column={column}: "
                    f"{rust_row[column]!r} != {matlab_row[column]!r}"
                )
        for column in NUMERIC_COLUMNS:
            absolute, relative = tolerances[column]
            if not numeric_equal(rust_row[column], matlab_row[column], absolute, relative):
                raise ValueError(
                    f"D3 mismatch row={index} column={column}: "
                    f"{rust_row[column]!r} != {matlab_row[column]!r}"
                )
    verify_semantics(rust_rows)
    verify_semantics(matlab_rows)
    print(
        "FLUXRT_FEEDBACK_DUAL_PASS D0=PASS D2=PASS D3=PASS D4=PASS "
        f"feature_off=PASS rows={len(rust_rows)}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-metadata", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--matlab-metadata", type=Path, required=True)
    parser.add_argument("--matlab-trace", type=Path, required=True)
    parser.add_argument("--comparison", type=Path, required=True)
    args = parser.parse_args()
    compare(
        args.rust_metadata,
        args.rust_trace,
        args.matlab_metadata,
        args.matlab_trace,
        args.comparison,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
