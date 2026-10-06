#!/usr/bin/env python3
"""Validate and aggregate H2 static Ls(I) runs with AS5600 angle windows.

This is an offline evidence tool.  It never opens a serial port, never sends a
target command and never changes an approved motor profile.  A complete matrix
is still screening evidence; parameter approval remains a separate H2-6 step.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Any


TAU = 2.0 * math.pi
AS5600_COUNTS = 4096
PULSE_POSITIVE = 1 << 5
PULSE_NEGATIVE = 1 << 6
DRIVE_ACTIVE = 1 << 4
FAULT_MASK = (1 << 8) | (1 << 9)
SHA256_LENGTH = 64


class MatrixError(ValueError):
    """Raised when an evidence bundle is incomplete, ambiguous or unsafe."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def checked_project_file(root: Path, relative: Any, digest: Any, label: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise MatrixError(f"{label} path must be a non-empty string")
    path = Path(relative)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise MatrixError(f"{label} path must be project-relative without traversal")
    if not isinstance(digest, str) or len(digest) != SHA256_LENGTH:
        raise MatrixError(f"{label} SHA-256 must contain 64 hex digits")
    try:
        int(digest, 16)
    except ValueError as error:
        raise MatrixError(f"{label} SHA-256 is not hexadecimal") from error
    resolved = root.joinpath(path)
    if not resolved.is_file():
        raise MatrixError(f"{label} file does not exist: {relative}")
    actual = sha256(resolved)
    if actual != digest.upper():
        raise MatrixError(f"{label} SHA-256 mismatch: expected {digest}, got {actual}")
    return resolved


def finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise MatrixError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise MatrixError(f"{label} must be a finite number") from error
    if not math.isfinite(result):
        raise MatrixError(f"{label} must be a finite number")
    return result


def positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise MatrixError(f"{label} must be a positive integer")
    return value


def nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise MatrixError(f"{label} must be a non-negative integer")
    return value


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MatrixError(f"cannot read {label}: {error}") from error
    if not isinstance(value, dict):
        raise MatrixError(f"{label} root must be an object")
    return value


def validate_common_clock_capture(
    summary_path: Path,
    combined_path: Path,
    angle_path: Path,
    run_id: str,
    angle_window_start_s: float,
    angle_window_end_s: float,
) -> dict[str, Any]:
    summary = load_json(summary_path, f"run {run_id} capture summary")
    if (
        summary.get("contract") != "fluxrt-dual-serial-capture"
        or summary.get("version") != 1
        or summary.get("status") != "capture-complete"
    ):
        raise MatrixError(f"run {run_id} capture summary is not complete V1 evidence")
    if summary.get("read_only") is not True or summary.get("commands_sent") != 0:
        raise MatrixError(f"run {run_id} capture was not read-only")
    clock = summary.get("clock")
    association = summary.get("static_h2_association")
    sources = summary.get("sources")
    if (
        not isinstance(clock, dict)
        or clock.get("source") != "time.monotonic_ns"
        or clock.get("shared_origin") is not True
    ):
        raise MatrixError(f"run {run_id} capture has no shared monotonic clock")
    if (
        not isinstance(association, dict)
        or association.get("kind") != "common-host-static-window-only"
        or association.get("dynamic_alignment_valid") is not False
    ):
        raise MatrixError(f"run {run_id} capture has invalid static H2 semantics")
    if not isinstance(sources, dict) or not all(name in sources for name in ("g431", "dengfoc")):
        raise MatrixError(f"run {run_id} capture is missing a serial source")
    if summary.get("errors") != []:
        raise MatrixError(f"run {run_id} capture contains reader errors")
    artifacts = summary.get("artifacts")
    if not isinstance(artifacts, dict):
        raise MatrixError(f"run {run_id} capture has no artifact binding")
    if (
        artifacts.get("combined_lines_path") != combined_path.name
        or str(artifacts.get("combined_lines_sha256", "")).upper() != sha256(combined_path)
        or artifacts.get("angle_csv_path") != angle_path.name
        or str(artifacts.get("angle_csv_sha256", "")).upper() != sha256(angle_path)
    ):
        raise MatrixError(f"run {run_id} capture artifact binding mismatch")
    raw_start = finite(
        association.get("g431_raw_window_start_s"),
        f"run {run_id} g431_raw_window_start_s",
    )
    raw_end = finite(
        association.get("g431_raw_window_end_s"),
        f"run {run_id} g431_raw_window_end_s",
    )
    if raw_start < 0.0 or raw_end < raw_start:
        raise MatrixError(f"run {run_id} G431 raw window is invalid")
    if not (angle_window_start_s <= raw_start <= raw_end <= angle_window_end_s):
        raise MatrixError(f"run {run_id} angle window does not contain the G431 raw window")
    return summary


def load_trace(path: Path) -> list[dict[str, float | int]]:
    required = {
        "control_tick",
        "current_u_a",
        "current_v_a",
        "current_w_a",
        "applied_phase_u_v",
        "flags",
    }
    rows: list[dict[str, float | int]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise MatrixError(f"trace is missing columns: {sorted(missing)}")
        for line, raw in enumerate(reader, start=2):
            try:
                row: dict[str, float | int] = {
                    "control_tick": int(raw["control_tick"]),
                    "current_u_a": float(raw["current_u_a"]),
                    "current_v_a": float(raw["current_v_a"]),
                    "current_w_a": float(raw["current_w_a"]),
                    "applied_phase_u_v": float(raw["applied_phase_u_v"]),
                    "flags": int(raw["flags"], 0),
                }
            except (TypeError, ValueError) as error:
                raise MatrixError(f"invalid trace value at row {line}") from error
            if not all(
                math.isfinite(float(row[name]))
                for name in (
                    "current_u_a",
                    "current_v_a",
                    "current_w_a",
                    "applied_phase_u_v",
                )
            ):
                raise MatrixError(f"non-finite trace value at row {line}")
            rows.append(row)
    if len(rows) < 3:
        raise MatrixError("trace must contain at least three samples")
    if any(
        int(right["control_tick"]) != int(left["control_tick"]) + 1
        for left, right in zip(rows, rows[1:])
    ):
        raise MatrixError("trace control ticks are not contiguous")
    if any(int(row["flags"]) & FAULT_MASK for row in rows):
        raise MatrixError("trace contains a hardware or software trip flag")
    return rows


def load_angle_window(
    path: Path,
    axis: int,
    start_s: float,
    end_s: float,
    minimum_samples: int,
    maximum_span_counts: int,
) -> dict[str, float | int]:
    if start_s < 0.0 or end_s <= start_s:
        raise MatrixError("angle window must have an increasing non-negative time range")
    rows: list[dict[str, float | int]] = []
    required = {
        "host_time_s",
        "axis",
        "valid",
        "sequence",
        "mechanical_position_rad",
        "read_failures",
    }
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise MatrixError(f"angle CSV is missing columns: {sorted(missing)}")
        for line, raw in enumerate(reader, start=2):
            try:
                host_time = float(raw["host_time_s"])
                row = {
                    "host_time_s": host_time,
                    "axis": int(raw["axis"]),
                    "valid": int(raw["valid"]),
                    "sequence": int(raw["sequence"]),
                    "mechanical_position_rad": float(raw["mechanical_position_rad"]),
                    "read_failures": int(raw["read_failures"]),
                }
            except (TypeError, ValueError) as error:
                raise MatrixError(f"invalid angle value at row {line}") from error
            if not all(
                math.isfinite(float(row[name]))
                for name in ("host_time_s", "mechanical_position_rad")
            ):
                raise MatrixError(f"non-finite angle value at row {line}")
            if start_s <= host_time <= end_s and int(row["axis"]) == axis:
                rows.append(row)
    if len(rows) < minimum_samples:
        raise MatrixError(
            f"angle window has {len(rows)} samples; at least {minimum_samples} required"
        )
    if any(int(row["valid"]) != 1 for row in rows):
        raise MatrixError("angle window contains an invalid AS5600 sample")
    if any(int(row["read_failures"]) != 0 for row in rows):
        raise MatrixError("angle window contains a non-zero AS5600 failure count")
    sequences = [int(row["sequence"]) for row in rows]
    if any(((right - left) & 0xFFFFFFFF) != 1 for left, right in zip(sequences, sequences[1:])):
        raise MatrixError("angle window sequence is not contiguous")

    angles = [float(row["mechanical_position_rad"]) % TAU for row in rows]
    counts = sorted(int(round(angle / TAU * AS5600_COUNTS)) % AS5600_COUNTS for angle in angles)
    gaps = [right - left for left, right in zip(counts, counts[1:])]
    gaps.append(counts[0] + AS5600_COUNTS - counts[-1])
    span_counts = AS5600_COUNTS - max(gaps)
    if span_counts > maximum_span_counts:
        raise MatrixError(
            f"angle window spans {span_counts} counts; maximum is {maximum_span_counts}"
        )
    mean_angle = math.atan2(
        sum(math.sin(angle) for angle in angles),
        sum(math.cos(angle) for angle in angles),
    ) % TAU
    return {
        "angle_sample_count": len(rows),
        "angle_span_counts": span_counts,
        "mechanical_angle_rad": mean_angle,
        "first_angle_sequence": sequences[0],
        "last_angle_sequence": sequences[-1],
    }


def pulse_metrics(rows: list[dict[str, float | int]]) -> dict[str, float | int]:
    positive: list[float] = []
    negative: list[float] = []
    bias_values: list[float] = []
    for left, right in zip(rows, rows[1:]):
        left_flags = int(left["flags"])
        right_flags = int(right["flags"])
        delta = float(right["current_u_a"]) - float(left["current_u_a"])
        if (left_flags & PULSE_POSITIVE) and (right_flags & PULSE_POSITIVE):
            positive.append(abs(delta))
        elif (left_flags & PULSE_NEGATIVE) and (right_flags & PULSE_NEGATIVE):
            negative.append(abs(delta))
        if (left_flags & DRIVE_ACTIVE) and not (
            left_flags & (PULSE_POSITIVE | PULSE_NEGATIVE)
        ):
            bias_values.append(float(left["current_u_a"]))
    if len(positive) < 2 or len(negative) < 2:
        raise MatrixError("trace does not contain enough positive and negative transitions")
    positive_mean = statistics.fmean(positive)
    negative_mean = statistics.fmean(negative)
    response_floor = max(positive_mean, negative_mean)
    asymmetry = (
        abs(positive_mean - negative_mean) / response_floor if response_floor > 0.0 else math.inf
    )
    noise = statistics.pstdev(bias_values) if len(bias_values) >= 2 else 0.0
    response_to_noise = min(positive_mean, negative_mean) / max(noise, 1.0e-12)
    maximum_current = max(
        max(
            abs(float(row["current_u_a"])),
            abs(float(row["current_v_a"])),
            abs(float(row["current_w_a"])),
        )
        for row in rows
    )
    return {
        "positive_transition_count": len(positive),
        "negative_transition_count": len(negative),
        "positive_mean_abs_delta_a": positive_mean,
        "negative_mean_abs_delta_a": negative_mean,
        "positive_negative_asymmetry": asymmetry,
        "bias_noise_std_a": noise,
        "response_to_noise_ratio": response_to_noise,
        "maximum_phase_current_a": maximum_current,
    }


def analyse_manifest(manifest_path: Path, project_root: Path) -> dict[str, Any]:
    manifest = load_json(manifest_path, "H2 matrix manifest")
    if manifest.get("contract") != "fluxrt-lsi-h2-matrix" or manifest.get("version") != 1:
        raise MatrixError("unsupported H2 matrix identity")
    motor_id = manifest.get("motor_id")
    firmware_sha = manifest.get("firmware_sha256")
    if not isinstance(motor_id, str) or not motor_id:
        raise MatrixError("motor_id must be a non-empty string")
    if not isinstance(firmware_sha, str) or len(firmware_sha) != SHA256_LENGTH:
        raise MatrixError("firmware_sha256 must contain 64 hex digits")
    try:
        int(firmware_sha, 16)
    except ValueError as error:
        raise MatrixError("firmware_sha256 is not hexadecimal") from error
    pole_pairs = positive_int(manifest.get("pole_pairs"), "pole_pairs")
    requirements = manifest.get("requirements")
    runs = manifest.get("runs")
    if not isinstance(requirements, dict) or not isinstance(runs, list) or not runs:
        raise MatrixError("requirements must be an object and runs must be non-empty")

    minimum_runs = positive_int(requirements.get("minimum_runs"), "minimum_runs")
    angle_bin_count = positive_int(requirements.get("angle_bin_count"), "angle_bin_count")
    minimum_angle_bins = positive_int(
        requirements.get("minimum_angle_bins"), "minimum_angle_bins"
    )
    minimum_angle_samples = positive_int(
        requirements.get("minimum_angle_samples"), "minimum_angle_samples"
    )
    maximum_angle_span_counts = positive_int(
        requirements.get("maximum_angle_span_counts"), "maximum_angle_span_counts"
    )
    maximum_phase_current_a = finite(
        requirements.get("maximum_phase_current_a"), "maximum_phase_current_a"
    )
    maximum_asymmetry = finite(
        requirements.get("maximum_positive_negative_asymmetry"),
        "maximum_positive_negative_asymmetry",
    )
    minimum_response_to_noise = finite(
        requirements.get("minimum_response_to_noise_ratio"),
        "minimum_response_to_noise_ratio",
    )
    if minimum_angle_bins > angle_bin_count:
        raise MatrixError("minimum_angle_bins cannot exceed angle_bin_count")
    if maximum_phase_current_a <= 0.0:
        raise MatrixError("maximum_phase_current_a must be positive")
    if not 0.0 <= maximum_asymmetry <= 1.0:
        raise MatrixError("maximum_positive_negative_asymmetry must be in [0,1]")
    if minimum_response_to_noise < 0.0:
        raise MatrixError("minimum_response_to_noise_ratio cannot be negative")

    results: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for index, run in enumerate(runs):
        if not isinstance(run, dict):
            raise MatrixError(f"run {index} must be an object")
        run_id = run.get("id")
        if not isinstance(run_id, str) or not run_id or run_id in seen_ids:
            raise MatrixError(f"run {index} has an empty or duplicate id")
        seen_ids.add(run_id)
        if run.get("motor_id") != motor_id:
            raise MatrixError(f"run {run_id} motor_id mismatch")
        if str(run.get("firmware_sha256", "")).upper() != firmware_sha.upper():
            raise MatrixError(f"run {run_id} firmware SHA mismatch")

        trace_path = checked_project_file(
            project_root,
            run.get("trace_csv_path"),
            run.get("trace_csv_sha256"),
            f"run {run_id} trace",
        )
        analysis_path = checked_project_file(
            project_root,
            run.get("analysis_json_path"),
            run.get("analysis_json_sha256"),
            f"run {run_id} analysis",
        )
        capture_summary_path = checked_project_file(
            project_root,
            run.get("capture_summary_path"),
            run.get("capture_summary_sha256"),
            f"run {run_id} capture summary",
        )
        combined_path = checked_project_file(
            project_root,
            run.get("combined_lines_path"),
            run.get("combined_lines_sha256"),
            f"run {run_id} combined serial lines",
        )
        angle_path = checked_project_file(
            project_root,
            run.get("angle_csv_path"),
            run.get("angle_csv_sha256"),
            f"run {run_id} angle",
        )
        analysis = load_json(analysis_path, f"run {run_id} analysis")
        if analysis.get("analysis_kind") != "lsi-powered-one-shot-screen":
            raise MatrixError(f"run {run_id} has the wrong analysis kind")
        if analysis.get("decision", {}).get("hardware_execution") != "pass":
            raise MatrixError(f"run {run_id} did not pass bounded hardware execution")
        if analysis.get("raw_contract", {}).get("contract_ok") is False:
            raise MatrixError(f"run {run_id} raw sample contract failed")
        if analysis.get("safety", {}).get("bounded_run_pass") is False:
            raise MatrixError(f"run {run_id} safety result failed")
        if analysis.get("decision", {}).get("parameter_approval") != "not-granted":
            raise MatrixError(f"run {run_id} analysis illegally claims parameter approval")
        source_firmware = str(analysis.get("source_metadata", {}).get("hex_sha256", ""))
        if source_firmware.upper() != firmware_sha.upper():
            raise MatrixError(f"run {run_id} analysis firmware SHA mismatch")

        angle_window_start_s = finite(
            run.get("angle_window_start_s"), "angle_window_start_s"
        )
        angle_window_end_s = finite(
            run.get("angle_window_end_s"), "angle_window_end_s"
        )
        validate_common_clock_capture(
            capture_summary_path,
            combined_path,
            angle_path,
            run_id,
            angle_window_start_s,
            angle_window_end_s,
        )
        trace_rows = load_trace(trace_path)
        pulse = pulse_metrics(trace_rows)
        angle = load_angle_window(
            angle_path,
            nonnegative_int(run.get("angle_axis"), "angle_axis"),
            angle_window_start_s,
            angle_window_end_s,
            minimum_angle_samples,
            maximum_angle_span_counts,
        )
        if float(pulse["maximum_phase_current_a"]) >= maximum_phase_current_a:
            raise MatrixError(f"run {run_id} reached the matrix current limit")
        if float(pulse["positive_negative_asymmetry"]) > maximum_asymmetry:
            raise MatrixError(f"run {run_id} positive/negative response is asymmetric")
        if float(pulse["response_to_noise_ratio"]) < minimum_response_to_noise:
            raise MatrixError(f"run {run_id} response is not above the noise requirement")

        mechanical_angle = float(angle["mechanical_angle_rad"])
        electrical_angle = (mechanical_angle * pole_pairs) % TAU
        angle_bin = min(
            angle_bin_count - 1,
            int(electrical_angle / TAU * angle_bin_count),
        )
        fit = analysis.get("fit", {}).get("applied_voltage_fixed_rs", {})
        results.append(
            {
                "run_id": run_id,
                "motor_id": motor_id,
                "mechanical_angle_rad": mechanical_angle,
                "electrical_angle_rad": electrical_angle,
                "electrical_angle_bin": angle_bin,
                **angle,
                **pulse,
                "fit_inductance_h": finite(fit.get("inductance_h"), "fit.inductance_h"),
                "fit_rmse_a": finite(fit.get("rmse_a"), "fit.rmse_a"),
                "fit_maximum_residual_a": finite(
                    fit.get("maximum_absolute_residual_a"),
                    "fit.maximum_absolute_residual_a",
                ),
            }
        )

    bins = sorted({int(run["electrical_angle_bin"]) for run in results})
    matrix_complete = len(results) >= minimum_runs and len(bins) >= minimum_angle_bins
    return {
        "contract": "fluxrt-lsi-h2-matrix-result",
        "version": 1,
        "motor_id": motor_id,
        "firmware_sha256": firmware_sha.upper(),
        "run_count": len(results),
        "covered_electrical_angle_bins": bins,
        "matrix_complete": matrix_complete,
        "result": "screening-complete" if matrix_complete else "collecting",
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
        "runs": results,
    }


def write_outputs(result: dict[str, Any], json_output: Path, csv_output: Path) -> None:
    json_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    csv_output.parent.mkdir(parents=True, exist_ok=True)
    rows = result["runs"]
    with csv_output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    parser.add_argument("--json-output", type=Path, required=True)
    parser.add_argument("--csv-output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = analyse_manifest(args.manifest, args.project_root.resolve())
        write_outputs(result, args.json_output, args.csv_output)
    except MatrixError as error:
        parser.error(str(error))
    print(
        f"H2_MATRIX_{result['result'].upper().replace('-', '_')} "
        f"runs={result['run_count']} bins={len(result['covered_electrical_angle_bins'])} "
        "approval=NOT_GRANTED"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
