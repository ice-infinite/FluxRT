#!/usr/bin/env python3
"""Validate H3 hardware-sync anchors and align AS5600 truth to FOC ticks.

UART receive timestamps are intentionally absent from the mapping inputs.  Each
anchor must represent one physical synchronization edge timestamped by both
MCUs.  The result is offline evidence only and never grants target capability.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any


TAU = 2.0 * math.pi
U32_MODULUS = 1 << 32
U32_HALF = 1 << 31


class TimeMapError(ValueError):
    """Raised when dynamic time evidence is incomplete or ambiguous."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise TimeMapError(f"{label} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise TimeMapError(f"{label} must be finite") from error
    if not math.isfinite(result):
        raise TimeMapError(f"{label} must be finite")
    return result


def integer(value: Any, label: str, minimum: int = 0) -> int:
    if isinstance(value, bool):
        raise TimeMapError(f"{label} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise TimeMapError(f"{label} must be an integer") from error
    if str(result) != str(value).strip() or result < minimum:
        raise TimeMapError(f"{label} must be an integer >= {minimum}")
    return result


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise TimeMapError(f"cannot read {label}: {error}") from error
    if not isinstance(value, dict):
        raise TimeMapError(f"{label} root must be an object")
    return value


def checked_file(root: Path, relative: Any, digest: Any, label: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise TimeMapError(f"{label} path must be a non-empty string")
    path = Path(relative)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise TimeMapError(f"{label} path must be project-relative without traversal")
    if not isinstance(digest, str) or len(digest) != 64:
        raise TimeMapError(f"{label} SHA-256 must contain 64 hex digits")
    try:
        int(digest, 16)
    except ValueError as error:
        raise TimeMapError(f"{label} SHA-256 is not hexadecimal") from error
    resolved = root.joinpath(path)
    if not resolved.is_file():
        raise TimeMapError(f"{label} file does not exist: {relative}")
    actual = sha256(resolved)
    if actual != digest.upper():
        raise TimeMapError(f"{label} SHA-256 mismatch: expected {digest}, got {actual}")
    return resolved


def unwrap_u32(values: list[int], label: str) -> list[int]:
    if not values:
        raise TimeMapError(f"{label} is empty")
    if any(value < 0 or value >= U32_MODULUS for value in values):
        raise TimeMapError(f"{label} contains a value outside u32")
    unwrapped = [values[0]]
    for previous_raw, current_raw in zip(values, values[1:]):
        delta = (current_raw - previous_raw) & 0xFFFFFFFF
        if delta == 0 or delta >= U32_HALF:
            raise TimeMapError(f"{label} is duplicate or not forward ordered")
        unwrapped.append(unwrapped[-1] + delta)
    return unwrapped


def unwrap_near(raw: int, anchor_raw: int, anchor_unwrapped: int) -> int:
    delta = (raw - anchor_raw) & 0xFFFFFFFF
    if delta >= U32_HALF:
        delta -= U32_MODULUS
    return anchor_unwrapped + delta


def require_contiguous_u32(values: list[int], label: str) -> None:
    if any(((right - left) & 0xFFFFFFFF) != 1 for left, right in zip(values, values[1:])):
        raise TimeMapError(f"{label} is not contiguous")


def load_anchors(path: Path) -> list[dict[str, int]]:
    required = {"edge_sequence", "reference_control_tick", "truth_tick"}
    rows: list[dict[str, int]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise TimeMapError(f"anchors CSV is missing columns: {sorted(missing)}")
        for line, raw in enumerate(reader, start=2):
            rows.append(
                {
                    "edge_sequence": integer(raw["edge_sequence"], f"anchor line {line} sequence"),
                    "reference_control_tick": integer(
                        raw["reference_control_tick"], f"anchor line {line} reference tick"
                    ),
                    "truth_tick": integer(raw["truth_tick"], f"anchor line {line} truth tick"),
                }
            )
    require_contiguous_u32([row["edge_sequence"] for row in rows], "anchor edge sequence")
    truth_ticks = [row["truth_tick"] for row in rows]
    if any(right <= left for left, right in zip(truth_ticks, truth_ticks[1:])):
        raise TimeMapError("anchor truth ticks are duplicate or not forward ordered")
    return rows


def load_truth(
    path: Path,
    maximum_gap_ticks: int,
    maximum_missing_samples: int = 0,
) -> tuple[list[dict[str, int | float]], int]:
    required = {
        "sequence",
        "truth_tick",
        "mechanical_angle_rad",
        "valid",
        "read_failures",
    }
    rows: list[dict[str, int | float]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise TimeMapError(f"truth CSV is missing columns: {sorted(missing)}")
        for line, raw in enumerate(reader, start=2):
            row = {
                "sequence": integer(raw["sequence"], f"truth line {line} sequence"),
                "truth_tick": integer(raw["truth_tick"], f"truth line {line} tick"),
                "mechanical_angle_rad": finite(
                    raw["mechanical_angle_rad"], f"truth line {line} angle"
                ),
                "valid": integer(raw["valid"], f"truth line {line} valid"),
                "read_failures": integer(
                    raw["read_failures"], f"truth line {line} failures"
                ),
            }
            rows.append(row)
    if len(rows) < 2:
        raise TimeMapError("truth CSV needs at least two samples")
    sequences = [int(row["sequence"]) for row in rows]
    missing_samples = 0
    for left, right in zip(sequences, sequences[1:]):
        delta = (right - left) & 0xFFFFFFFF
        if delta == 0 or delta >= 0x80000000:
            raise TimeMapError("truth sequence is duplicate or not forward ordered")
        missing_samples += delta - 1
    if missing_samples > maximum_missing_samples:
        raise TimeMapError(
            "truth sequence missing-sample count exceeds the configured bound"
        )
    if any(int(row["valid"]) != 1 for row in rows):
        raise TimeMapError("truth contains an invalid AS5600 sample")
    if any(int(row["read_failures"]) != 0 for row in rows):
        raise TimeMapError("truth contains a non-zero AS5600 failure count")
    ticks = [int(row["truth_tick"]) for row in rows]
    gaps = [right - left for left, right in zip(ticks, ticks[1:])]
    if any(gap <= 0 for gap in gaps):
        raise TimeMapError("truth ticks are duplicate or not forward ordered")
    if any(gap > maximum_gap_ticks for gap in gaps):
        raise TimeMapError("truth sample gap exceeds the configured bound")
    return rows, missing_samples


def load_queries(path: Path) -> list[dict[str, int | float]]:
    required = {
        "sequence",
        "reference_control_tick",
        "estimated_electrical_angle_rad",
        "reliable",
    }
    rows: list[dict[str, int | float]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        missing = required - fields
        if missing:
            raise TimeMapError(f"queries CSV is missing columns: {sorted(missing)}")
        context_fields = {
            "controller_state",
            "control_electrical_angle_rad",
            "forced_electrical_angle_rad",
        }
        present_context = context_fields & fields
        if present_context and present_context != context_fields:
            raise TimeMapError("queries CSV has incomplete controller context")
        for line, raw in enumerate(reader, start=2):
            row = {
                    "sequence": integer(raw["sequence"], f"query line {line} sequence"),
                    "reference_control_tick": integer(
                        raw["reference_control_tick"], f"query line {line} reference tick"
                    ),
                    "estimated_electrical_angle_rad": finite(
                        raw["estimated_electrical_angle_rad"], f"query line {line} angle"
                    ),
                    "reliable": integer(raw["reliable"], f"query line {line} reliable"),
                }
            if present_context:
                row.update({
                    "controller_state": integer(
                        raw["controller_state"], f"query line {line} controller state"
                    ),
                    "control_electrical_angle_rad": finite(
                        raw["control_electrical_angle_rad"],
                        f"query line {line} control angle",
                    ),
                    "forced_electrical_angle_rad": finite(
                        raw["forced_electrical_angle_rad"],
                        f"query line {line} forced angle",
                    ),
                })
            rows.append(row)
    if not rows:
        raise TimeMapError("queries CSV is empty")
    require_contiguous_u32([int(row["sequence"]) for row in rows], "query sequence")
    if any(int(row["reliable"]) not in (0, 1) for row in rows):
        raise TimeMapError("query reliable must be 0 or 1")
    unwrap_u32([int(row["reference_control_tick"]) for row in rows], "query control ticks")
    return rows


def fit_affine_mapping(
    anchors: list[dict[str, int]], reference_hz: int, truth_hz: int
) -> dict[str, float | int | list[int]]:
    reference_raw = [row["reference_control_tick"] for row in anchors]
    reference = unwrap_u32(reference_raw, "anchor reference ticks")
    truth = [row["truth_tick"] for row in anchors]
    x_mean = sum(truth) / len(truth)
    y_mean = sum(reference) / len(reference)
    denominator = sum((value - x_mean) ** 2 for value in truth)
    if denominator <= 0.0:
        raise TimeMapError("anchor time span is zero")
    slope = sum(
        (x - x_mean) * (y - y_mean) for x, y in zip(truth, reference)
    ) / denominator
    if slope <= 0.0:
        raise TimeMapError("clock mapping slope is not positive")
    intercept = y_mean - slope * x_mean
    residuals = [y - (slope * x + intercept) for x, y in zip(truth, reference)]
    nominal_slope = reference_hz / truth_hz
    drift_ppm = (slope / nominal_slope - 1.0) * 1_000_000.0
    return {
        "slope_reference_ticks_per_truth_tick": slope,
        "intercept_reference_ticks": intercept,
        "clock_drift_ppm": drift_ppm,
        "maximum_anchor_residual_ticks": max(abs(value) for value in residuals),
        "rms_anchor_residual_ticks": math.sqrt(
            sum(value * value for value in residuals) / len(residuals)
        ),
        "reference_raw_anchor": reference_raw[0],
        "reference_unwrapped_anchor": reference[0],
        "reference_anchor_min_tick": reference[0],
        "reference_anchor_max_tick": reference[-1],
        "reference_unwrapped": reference,
    }


def wrap_tau(value: float) -> float:
    return value % TAU


def wrap_pi(value: float) -> float:
    return (value + math.pi) % TAU - math.pi


def interpolate_angle(
    truth: list[dict[str, int | float]], truth_tick: float
) -> tuple[float, int, int]:
    ticks = [int(row["truth_tick"]) for row in truth]
    if truth_tick < ticks[0] or truth_tick > ticks[-1]:
        raise TimeMapError("query requires dynamic truth extrapolation")
    right = bisect.bisect_left(ticks, truth_tick)
    if right < len(ticks) and ticks[right] == truth_tick:
        angle = float(truth[right]["mechanical_angle_rad"])
        sequence = int(truth[right]["sequence"])
        return angle, sequence, sequence
    if right == 0 or right == len(ticks):
        raise TimeMapError("query cannot be bracketed by truth samples")
    left = right - 1
    fraction = (truth_tick - ticks[left]) / (ticks[right] - ticks[left])
    left_angle = float(truth[left]["mechanical_angle_rad"])
    delta = wrap_pi(float(truth[right]["mechanical_angle_rad"]) - left_angle)
    return (
        wrap_tau(left_angle + fraction * delta),
        int(truth[left]["sequence"]),
        int(truth[right]["sequence"]),
    )


def analyse_manifest(manifest_path: Path, project_root: Path) -> dict[str, Any]:
    manifest = load_json(manifest_path, "H3 time-map manifest")
    if manifest.get("contract") != "fluxrt-h3-time-map" or manifest.get("version") != 1:
        raise TimeMapError("unsupported H3 time-map identity")
    if manifest.get("parameter_approval") != "not-granted" or manifest.get(
        "target_capability_approval"
    ) != "not-granted":
        raise TimeMapError("H3 time-map input must not claim approval")
    motor_id = manifest.get("motor_id")
    if not isinstance(motor_id, str) or not motor_id:
        raise TimeMapError("motor_id must be a non-empty string")
    for name in ("reference_firmware_sha256", "truth_firmware_sha256"):
        digest = manifest.get(name)
        if not isinstance(digest, str) or len(digest) != 64:
            raise TimeMapError(f"{name} must contain 64 hex digits")
        try:
            int(digest, 16)
        except ValueError as error:
            raise TimeMapError(f"{name} is not hexadecimal") from error
    pole_pairs = integer(manifest.get("pole_pairs"), "pole_pairs", 1)
    direction = integer(manifest.get("mechanical_direction"), "mechanical_direction", -1)
    if direction not in (-1, 1):
        raise TimeMapError("mechanical_direction must be -1 or 1")
    offset = finite(manifest.get("electrical_offset_rad"), "electrical_offset_rad")
    reference_hz = integer(manifest.get("reference_tick_hz"), "reference_tick_hz", 1)
    truth_hz = integer(manifest.get("truth_tick_hz"), "truth_tick_hz", 1)
    requirements = manifest.get("requirements")
    if not isinstance(requirements, dict):
        raise TimeMapError("requirements must be an object")
    minimum_anchors = integer(requirements.get("minimum_anchor_count"), "minimum anchors", 3)
    maximum_residual = finite(
        requirements.get("maximum_anchor_residual_ticks"), "maximum anchor residual"
    )
    maximum_drift = finite(requirements.get("maximum_clock_drift_ppm"), "maximum drift")
    maximum_gap = integer(requirements.get("maximum_truth_gap_ticks"), "maximum truth gap", 1)
    maximum_missing = integer(
        requirements.get("maximum_truth_missing_samples", 0),
        "maximum truth missing samples",
        0,
    )
    maximum_outside_anchors = integer(
        requirements.get("maximum_anchors_outside_truth_window", 0),
        "maximum anchors outside truth window",
        0,
    )
    maximum_outside_queries = integer(
        requirements.get("maximum_queries_outside_anchor_span", 0),
        "maximum queries outside anchor span",
        0,
    )
    if maximum_residual < 0.0 or maximum_drift < 0.0:
        raise TimeMapError("residual and drift bounds must be non-negative")

    anchors_path = checked_file(
        project_root, manifest.get("anchors_csv_path"), manifest.get("anchors_csv_sha256"), "anchors"
    )
    truth_path = checked_file(
        project_root, manifest.get("truth_csv_path"), manifest.get("truth_csv_sha256"), "truth"
    )
    queries_path = checked_file(
        project_root, manifest.get("queries_csv_path"), manifest.get("queries_csv_sha256"), "queries"
    )
    anchors = load_anchors(anchors_path)
    truth, missing_truth_samples = load_truth(
        truth_path, maximum_gap, maximum_missing
    )
    truth_min = int(truth[0]["truth_tick"])
    truth_max = int(truth[-1]["truth_tick"])
    excluded_anchors = [
        row for row in anchors
        if int(row["truth_tick"]) < truth_min or int(row["truth_tick"]) > truth_max
    ]
    if len(excluded_anchors) > maximum_outside_anchors:
        raise TimeMapError("anchor outside captured truth window exceeds configured bound")
    anchors = [
        row for row in anchors
        if truth_min <= int(row["truth_tick"]) <= truth_max
    ]
    if len(anchors) < minimum_anchors:
        raise TimeMapError("not enough hardware synchronization anchors")
    queries = load_queries(queries_path)
    mapping = fit_affine_mapping(anchors, reference_hz, truth_hz)
    if abs(float(mapping["clock_drift_ppm"])) > maximum_drift:
        raise TimeMapError("clock drift exceeds the configured bound")
    if float(mapping["maximum_anchor_residual_ticks"]) > maximum_residual:
        raise TimeMapError("anchor residual exceeds the configured bound")

    reference_raw_anchor = int(mapping["reference_raw_anchor"])
    reference_unwrapped_anchor = int(mapping["reference_unwrapped_anchor"])
    slope = float(mapping["slope_reference_ticks_per_truth_tick"])
    intercept = float(mapping["intercept_reference_ticks"])
    anchor_min = int(mapping["reference_anchor_min_tick"])
    anchor_max = int(mapping["reference_anchor_max_tick"])
    aligned: list[dict[str, int | float]] = []
    outside_query_sequences: list[int] = []
    for query in queries:
        reference_tick = unwrap_near(
            int(query["reference_control_tick"]),
            reference_raw_anchor,
            reference_unwrapped_anchor,
        )
        if reference_tick < anchor_min or reference_tick > anchor_max:
            outside_query_sequences.append(int(query["sequence"]))
            continue
        truth_tick = (reference_tick - intercept) / slope
        mechanical, left_sequence, right_sequence = interpolate_angle(truth, truth_tick)
        electrical = wrap_tau(direction * pole_pairs * mechanical + offset)
        estimate = wrap_tau(float(query["estimated_electrical_angle_rad"]))
        aligned_row = {
                "sequence": int(query["sequence"]),
                "reference_control_tick": int(query["reference_control_tick"]),
                "reference_control_tick_unwrapped": reference_tick,
                "mapped_truth_tick": truth_tick,
                "truth_sequence_left": left_sequence,
                "truth_sequence_right": right_sequence,
                "truth_mechanical_angle_rad": mechanical,
                "truth_electrical_angle_rad": electrical,
                "estimated_electrical_angle_rad": estimate,
                "electrical_angle_error_rad": wrap_pi(estimate - electrical),
                "reliable": int(query["reliable"]),
            }
        for name in (
            "controller_state",
            "control_electrical_angle_rad",
            "forced_electrical_angle_rad",
        ):
            if name in query:
                aligned_row[name] = query[name]
        aligned.append(aligned_row)
    if len(outside_query_sequences) > maximum_outside_queries:
        raise TimeMapError("query outside hardware-sync anchor span exceeds configured bound")
    if not aligned:
        raise TimeMapError("no query is inside the hardware-sync anchor span")
    reliable_errors = [
        abs(float(row["electrical_angle_error_rad"]))
        for row in aligned
        if int(row["reliable"]) == 1
    ]
    return {
        "contract": "fluxrt-h3-time-map-result",
        "version": 1,
        "status": "aligned-offline" if reliable_errors else "aligned-no-reliable-estimate",
        "motor_id": motor_id,
        "mapping": {
            key: value for key, value in mapping.items() if key != "reference_unwrapped"
        },
        "aligned_sample_count": len(aligned),
        "input_anchor_count": len(anchors) + len(excluded_anchors),
        "used_anchor_count": len(anchors),
        "excluded_anchor_count": len(excluded_anchors),
        "excluded_anchor_sequences": [
            int(row["edge_sequence"]) for row in excluded_anchors
        ],
        "input_query_count": len(queries),
        "excluded_query_count": len(outside_query_sequences),
        "excluded_query_sequences": outside_query_sequences,
        "reliable_sample_count": len(reliable_errors),
        "truth_missing_sample_count": missing_truth_samples,
        "maximum_truth_missing_samples": maximum_missing,
        "reliable_rms_angle_error_rad": (
            math.sqrt(sum(value * value for value in reliable_errors) / len(reliable_errors))
            if reliable_errors
            else None
        ),
        "reliable_maximum_angle_error_rad": max(reliable_errors) if reliable_errors else None,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
        "aligned": aligned,
    }


def write_outputs(result: dict[str, Any], json_output: Path, csv_output: Path) -> None:
    json_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    csv_output.parent.mkdir(parents=True, exist_ok=True)
    rows = result["aligned"]
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
    except TimeMapError as error:
        parser.error(str(error))
    print(
        f"H3_TIME_MAP_{result['status'].upper().replace('-', '_')} "
        f"samples={result['aligned_sample_count']} approval=NOT_GRANTED"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
