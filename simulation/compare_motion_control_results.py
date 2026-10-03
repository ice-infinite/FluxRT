#!/usr/bin/env python3
"""Strict D0-D4 comparison for independent Rust and MATLAB motion models."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any


ENGINE_IDS = {
    "fluxrt.rust.motion.v1",
    "fluxrt.matlab.motion.v1",
}
TRACE_COLUMNS = [
    "case_id",
    "control_tick",
    "time_s",
    "enabled",
    "control_mode",
    "input_mode",
    "status",
    "command_position_rad",
    "command_velocity_rad_s",
    "command_torque_nm",
    "feedback_position_rad",
    "feedback_velocity_rad_s",
    "feedback_iq_a",
    "planned_position_rad",
    "planned_velocity_rad_s",
    "planned_torque_nm",
    "output_velocity_ref_rad_s",
    "output_torque_nm",
    "output_id_a",
    "output_iq_a",
    "planner_flags",
    "cascade_flags",
    "velocity_integrator_nm",
]
EXACT_COLUMNS = {
    "case_id",
    "control_tick",
    "enabled",
    "control_mode",
    "input_mode",
    "status",
    "planner_flags",
    "cascade_flags",
}
NUMERIC_COLUMNS = set(TRACE_COLUMNS) - EXACT_COLUMNS
REQUIRED_METADATA_KEYS = {
    "contract_gate_result_version",
    "engine_id",
    "workspace_revision",
    "bundle_id",
    "bundle_sha256",
    "profile_id",
    "profile_revision",
    "profile_sha256",
    "base_model_revision",
    "trace_schema_id",
    "trace_schema_version",
    "trace_schema_sha256",
    "comparison_id",
    "comparison_version",
    "comparison_sha256",
    "scenario_contract",
    "scenario_version",
    "motion_scenario_id",
    "motion_scenario_revision",
    "motion_scenario_sha256",
    "motion_model_revision",
    "case_count",
    "row_count",
    "d0",
    "d1",
    "d2",
    "d3",
    "d4",
    "e2_acceptance",
    "feature_off",
}
REQUIRED_CASES = [
    "feature_off",
    "torque_ramp",
    "velocity_ramp",
    "position_filter",
    "trapezoidal_trajectory",
    "mode_switch",
    "command_limit",
    "invalid_feedback",
    "accept_torque_low_power",
    "accept_velocity_low_power",
    "accept_position_low_power",
]
PLANNER_TRANSITION = 1 << 0
PLANNER_LIMIT_MASK = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4)
CASCADE_TRANSITION_PRELOADED = 1 << 2
CASCADE_LIMIT_MASK = (1 << 3) | (1 << 4) | (1 << 5)
FAIL_CLOSED_COLUMNS = {
    "planned_position_rad",
    "planned_velocity_rad_s",
    "planned_torque_nm",
    "output_velocity_ref_rad_s",
    "output_torque_nm",
    "output_id_a",
    "output_iq_a",
    "velocity_integrator_nm",
}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line or "=" not in line:
            raise ValueError(f"invalid metadata line {line_number} in {path}: {line!r}")
        key, value = line.split("=", 1)
        if not key or not value or key in result:
            raise ValueError(f"duplicate/empty metadata entry in {path}: {line!r}")
        result[key] = value
    if set(result) != REQUIRED_METADATA_KEYS:
        missing = sorted(REQUIRED_METADATA_KEYS - set(result))
        extra = sorted(set(result) - REQUIRED_METADATA_KEYS)
        raise ValueError(f"unexpected metadata keys in {path}: missing={missing} extra={extra}")
    return result


def read_trace(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != TRACE_COLUMNS:
            raise ValueError(f"unexpected motion trace columns in {path}: {reader.fieldnames}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty motion trace: {path}")
    for index, row in enumerate(rows):
        if set(row) != set(TRACE_COLUMNS) or any(row[name] == "" for name in TRACE_COLUMNS):
            raise ValueError(f"missing or empty trace value in {path} row {index}")
        for name in NUMERIC_COLUMNS:
            value = float(row[name])
            if not math.isfinite(value):
                raise ValueError(f"non-finite {name} in {path} row {index}")
        parse_integer(row["control_tick"], "control_tick")
        parse_integer(row["planner_flags"], "planner_flags")
        parse_integer(row["cascade_flags"], "cascade_flags")
        if row["enabled"] not in {"0", "1"}:
            raise ValueError(f"enabled must be 0 or 1 in {path} row {index}")
    keys = [(row["case_id"], row["control_tick"]) for row in rows]
    if len(keys) != len(set(keys)):
        raise ValueError(f"duplicate case/tick row in {path}")
    return rows


def parse_integer(value: str, field: str) -> int:
    try:
        return int(value, 0)
    except ValueError as error:
        raise ValueError(f"{field} is not an integer: {value!r}") from error


def load_scenario(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("contract") != "fluxrt-motion-control-scenario" or document.get("version") != 1:
        raise ValueError("unsupported motion scenario contract")
    cases = document.get("cases")
    if not isinstance(cases, list) or [case.get("case_id") for case in cases] != REQUIRED_CASES:
        raise ValueError("motion scenario case order/set mismatch")
    return document


def load_tolerances(path: Path) -> tuple[dict[str, tuple[float, float]], dict[str, Any]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("contract") != "fluxrt-comparison-gates" or document.get("version") != 1:
        raise ValueError("unsupported comparison contract")
    entries = document.get("motion_numeric_tolerances")
    if not isinstance(entries, list):
        raise ValueError("motion_numeric_tolerances is missing")
    result: dict[str, tuple[float, float]] = {}
    for item in entries:
        channel = item.get("channel")
        if channel in result:
            raise ValueError(f"duplicate motion tolerance: {channel!r}")
        absolute = float(item.get("absolute"))
        relative = float(item.get("relative"))
        if channel not in NUMERIC_COLUMNS or absolute < 0.0 or relative < 0.0:
            raise ValueError(f"invalid motion tolerance: {item!r}")
        result[channel] = (absolute, relative)
    if set(result) != NUMERIC_COLUMNS:
        raise ValueError(f"motion tolerance columns mismatch: {sorted(set(result) ^ NUMERIC_COLUMNS)}")
    return result, document


def numeric_equal(left: str, right: str, absolute: float, relative: float) -> bool:
    a = float(left)
    b = float(right)
    return abs(a - b) <= absolute + relative * max(abs(a), abs(b))


def group_rows(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    grouped: dict[str, list[dict[str, str]]] = {}
    seen_closed: set[str] = set()
    previous = ""
    for row in rows:
        case_id = row["case_id"]
        if case_id != previous:
            if case_id in seen_closed:
                raise ValueError(f"case rows are not contiguous: {case_id}")
            if previous:
                seen_closed.add(previous)
            previous = case_id
        grouped.setdefault(case_id, []).append(row)
    if list(grouped) != REQUIRED_CASES:
        raise ValueError(f"motion trace case order/set mismatch: {list(grouped)}")
    return grouped


def validate_identity(
    metadata: dict[str, str],
    scenario: dict[str, Any],
    scenario_path: Path,
    comparison: dict[str, Any],
    comparison_path: Path,
) -> None:
    expected = {
        "scenario_contract": str(scenario["contract"]),
        "scenario_version": str(scenario["version"]),
        "motion_scenario_id": str(scenario["scenario_id"]),
        "motion_scenario_revision": str(scenario["revision"]),
        "motion_scenario_sha256": sha256_file(scenario_path),
        "profile_id": str(scenario["profile_id"]),
        "profile_revision": str(scenario["profile_revision"]),
        "motion_model_revision": str(scenario["model_revision"]),
        "comparison_id": str(comparison["comparison_id"]),
        "comparison_version": str(comparison["version"]),
        "comparison_sha256": sha256_file(comparison_path),
        "contract_gate_result_version": "1",
        "case_count": str(len(scenario["cases"])),
    }
    differing = sorted(key for key, value in expected.items() if metadata.get(key) != value)
    if differing:
        raise ValueError(f"D0 identity does not match contracts: {differing}")
    for gate in ("d0", "d1", "d2", "d3", "d4", "e2_acceptance", "feature_off"):
        if metadata[gate] != "PASS":
            raise ValueError(f"engine gate did not pass: {gate}={metadata[gate]!r}")


def verify_semantics(rows: list[dict[str, str]], scenario: dict[str, Any]) -> None:
    grouped = group_rows(rows)
    cases = {case["case_id"]: case for case in scenario["cases"]}
    for case_id, case_rows in grouped.items():
        case = cases[case_id]
        expected_ticks = int(case["evaluation_ticks"])
        ticks = [parse_integer(row["control_tick"], "control_tick") for row in case_rows]
        if ticks != list(range(expected_ticks)):
            raise ValueError(f"{case_id}: ticks must be contiguous 0..{expected_ticks - 1}")
        expected_enabled = "1" if case["enabled"] else "0"
        if any(row["enabled"] != expected_enabled for row in case_rows):
            raise ValueError(f"{case_id}: enabled column disagrees with scenario")

        if case["enabled"]:
            by_tick = {parse_integer(row["control_tick"], "control_tick"): row for row in case_rows}
            for command in case["commands"]:
                tick = int(command["start_tick"])
                row = by_tick[tick]
                if row["status"] != "ok":
                    raise ValueError(f"{case_id}: transition tick {tick} did not succeed")
                if parse_integer(row["planner_flags"], "planner_flags") & PLANNER_TRANSITION == 0:
                    raise ValueError(f"{case_id}: transition flag missing at tick {tick}")
                if (
                    parse_integer(row["cascade_flags"], "cascade_flags")
                    & CASCADE_TRANSITION_PRELOADED
                    == 0
                ):
                    raise ValueError(f"{case_id}: cascade preload flag missing at tick {tick}")

    for row in grouped["feature_off"]:
        if (
            row["status"] != "planner_disabled"
            or row["planner_flags"] != "0"
            or row["cascade_flags"] != "0"
        ):
            raise ValueError("feature_off did not report a disabled, flag-free result")
        if any(float(row[name]) != 0.0 for name in FAIL_CLOSED_COLUMNS):
            raise ValueError("feature_off did not fail closed to zero motion output")

    invalid_case = cases["invalid_feedback"]
    fault_ticks = {
        tick
        for fault in invalid_case["feedback_faults"]
        for tick in range(int(fault["start_tick"]), int(fault["end_tick"]))
    }
    if not any(
        parse_integer(row["control_tick"], "control_tick") in fault_ticks
        and row["status"] != "ok"
        for row in grouped["invalid_feedback"]
    ):
        raise ValueError("invalid_feedback case did not report an error during its fault window")

    if not any(
        parse_integer(row["planner_flags"], "planner_flags") & PLANNER_LIMIT_MASK
        or parse_integer(row["cascade_flags"], "cascade_flags") & CASCADE_LIMIT_MASK
        for row in grouped["command_limit"]
    ):
        raise ValueError("command_limit case did not exercise a planner/cascade limit flag")


def verify_acceptance(
    rows: list[dict[str, str]], scenario: dict[str, Any]
) -> list[str]:
    grouped = group_rows(rows)
    runtime = scenario["runtime"]
    torque_constant = float(runtime["torque_constant_nm_per_a"])
    summaries: list[str] = []
    for case in scenario["cases"]:
        if "acceptance" not in case:
            continue
        case_id = str(case["case_id"])
        case_rows = grouped[case_id]
        if any(row["status"] != "ok" for row in case_rows):
            raise ValueError(f"{case_id}: acceptance trace contains a failed tick")
        acceptance = case["acceptance"]
        command = case["commands"][-1]
        final = case_rows[-1]
        peak_iq = max(
            max(abs(float(row["feedback_iq_a"])), abs(float(row["output_iq_a"])))
            for row in case_rows
        )
        peak_abs_velocity = max(abs(float(row["feedback_velocity_rad_s"])) for row in case_rows)
        if peak_iq > float(acceptance["maximum_absolute_iq_a"]):
            raise ValueError(f"{case_id}: peak iq {peak_iq} exceeds acceptance")
        if peak_abs_velocity > float(acceptance["maximum_absolute_velocity_rad_s"]):
            raise ValueError(f"{case_id}: peak velocity {peak_abs_velocity} exceeds acceptance")

        mode = acceptance["mode"]
        if mode == "Torque":
            final_value = float(final["feedback_iq_a"]) * torque_constant
            target = float(command["torque_ref_nm"])
            final_error = abs(final_value - target)
            progress = float(final["feedback_velocity_rad_s"])
            if final_error > float(acceptance["maximum_final_torque_error_nm"]):
                raise ValueError(f"{case_id}: final torque error {final_error} exceeds acceptance")
            if progress < float(acceptance["minimum_final_velocity_rad_s"]):
                raise ValueError(f"{case_id}: final velocity {progress} is below acceptance")
            summaries.append(
                f"Torque(final={final_value:.6f}Nm,error={final_error:.6f}Nm,"
                f"peak_iq={peak_iq:.4f}A,velocity={progress:.4f}rad/s)"
            )
        elif mode == "Velocity":
            target = float(command["velocity_ref_rad_s"])
            final_value = float(final["feedback_velocity_rad_s"])
            final_error = abs(final_value - target)
            overshoot = max(
                0.0,
                max(float(row["feedback_velocity_rad_s"]) for row in case_rows) - target,
            )
            if final_error > float(acceptance["maximum_final_velocity_error_rad_s"]):
                raise ValueError(f"{case_id}: final velocity error {final_error} exceeds acceptance")
            if overshoot > float(acceptance["maximum_velocity_overshoot_rad_s"]):
                raise ValueError(f"{case_id}: velocity overshoot {overshoot} exceeds acceptance")
            summaries.append(
                f"Velocity(final={final_value:.5f}rad/s,error={final_error:.5f},"
                f"overshoot={overshoot:.5f},peak_iq={peak_iq:.4f}A)"
            )
        elif mode == "Position":
            target = float(command["position_ref_rad"])
            final_value = float(final["feedback_position_rad"])
            final_velocity = abs(float(final["feedback_velocity_rad_s"]))
            final_error = abs(final_value - target)
            overshoot = max(
                0.0,
                max(float(row["feedback_position_rad"]) for row in case_rows) - target,
            )
            if final_error > float(acceptance["maximum_final_position_error_rad"]):
                raise ValueError(f"{case_id}: final position error {final_error} exceeds acceptance")
            if overshoot > float(acceptance["maximum_position_overshoot_rad"]):
                raise ValueError(f"{case_id}: position overshoot {overshoot} exceeds acceptance")
            if final_velocity > float(acceptance["maximum_final_absolute_velocity_rad_s"]):
                raise ValueError(f"{case_id}: final velocity {final_velocity} exceeds acceptance")
            summaries.append(
                f"Position(final={final_value:.5f}rad,error={final_error:.5f},"
                f"overshoot={overshoot:.5f},velocity={final_velocity:.5f}rad/s,"
                f"peak_iq={peak_iq:.4f}A)"
            )
        else:
            raise ValueError(f"{case_id}: unsupported acceptance mode {mode!r}")
    if len(summaries) != 3:
        raise ValueError(f"expected three E2 acceptance cases, got {len(summaries)}")
    return summaries


def compare(
    rust_metadata_path: Path,
    rust_trace_path: Path,
    matlab_metadata_path: Path,
    matlab_trace_path: Path,
    scenario_path: Path,
    comparison_path: Path,
) -> None:
    scenario = load_scenario(scenario_path)
    tolerances, comparison = load_tolerances(comparison_path)
    rust_metadata = read_metadata(rust_metadata_path)
    matlab_metadata = read_metadata(matlab_metadata_path)
    if {rust_metadata["engine_id"], matlab_metadata["engine_id"]} != ENGINE_IDS:
        raise ValueError("Rust/MATLAB motion engines are not the two canonical implementations")
    validate_identity(rust_metadata, scenario, scenario_path, comparison, comparison_path)
    validate_identity(matlab_metadata, scenario, scenario_path, comparison, comparison_path)
    rust_identity = {key: value for key, value in rust_metadata.items() if key != "engine_id"}
    matlab_identity = {key: value for key, value in matlab_metadata.items() if key != "engine_id"}
    if rust_identity != matlab_identity:
        differing = sorted(
            key
            for key in rust_identity
            if rust_identity.get(key) != matlab_identity.get(key)
        )
        raise ValueError(f"D0/summary metadata mismatch: {differing}")

    rust_rows = read_trace(rust_trace_path)
    matlab_rows = read_trace(matlab_trace_path)
    if len(rust_rows) != len(matlab_rows) or int(rust_metadata["row_count"]) != len(rust_rows):
        raise ValueError("motion row count mismatch")
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
    verify_semantics(rust_rows, scenario)
    verify_semantics(matlab_rows, scenario)
    summaries = verify_acceptance(rust_rows, scenario)
    verify_acceptance(matlab_rows, scenario)
    print(
        "FLUXRT_MOTION_DUAL_PASS D0=PASS D2=PASS D3=PASS D4=PASS "
        f"E2=PASS feature_off=PASS rows={len(rust_rows)}"
    )
    for summary in summaries:
        print(f"FLUXRT_MOTION_E2 {summary}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-metadata", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--matlab-metadata", type=Path, required=True)
    parser.add_argument("--matlab-trace", type=Path, required=True)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--comparison", type=Path, required=True)
    args = parser.parse_args()
    compare(
        args.rust_metadata,
        args.rust_trace,
        args.matlab_metadata,
        args.matlab_trace,
        args.scenario,
        args.comparison,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
