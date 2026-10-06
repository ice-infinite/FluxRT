#!/usr/bin/env python3
"""Derive a non-approving electrical-offset candidate from H3 alignment data.

The calculation deliberately uses only the controller's forced alignment angle
and independently time-aligned AS5600 mechanical truth.  Observer angle and
observer reliability are not inputs to the candidate, avoiding circular proof.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any


TAU = 2.0 * math.pi


class AlignmentOffsetError(ValueError):
    """Raised when alignment evidence is incomplete or fails its bounds."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise AlignmentOffsetError(f"{label} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise AlignmentOffsetError(f"{label} must be finite") from error
    if not math.isfinite(result):
        raise AlignmentOffsetError(f"{label} must be finite")
    return result


def integer(value: Any, label: str, minimum: int = 0) -> int:
    if isinstance(value, bool):
        raise AlignmentOffsetError(f"{label} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise AlignmentOffsetError(f"{label} must be an integer") from error
    if str(result) != str(value).strip() or result < minimum:
        raise AlignmentOffsetError(f"{label} must be an integer >= {minimum}")
    return result


def wrap_tau(value: float) -> float:
    return value % TAU


def wrap_pi(value: float) -> float:
    return (value + math.pi) % TAU - math.pi


def circular_mean(values: list[float], label: str) -> float:
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.hypot(sine, cosine) <= 1.0e-12:
        raise AlignmentOffsetError(f"{label} circular mean is ambiguous")
    return wrap_tau(math.atan2(sine, cosine))


def checked_file(root: Path, relative: Any, digest: Any) -> Path:
    if not isinstance(relative, str) or not relative:
        raise AlignmentOffsetError("aligned CSV path must be a non-empty string")
    path = Path(relative)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise AlignmentOffsetError("aligned CSV path must be project-relative without traversal")
    if not isinstance(digest, str) or len(digest) != 64:
        raise AlignmentOffsetError("aligned CSV SHA-256 must contain 64 hex digits")
    try:
        int(digest, 16)
    except ValueError as error:
        raise AlignmentOffsetError("aligned CSV SHA-256 is not hexadecimal") from error
    resolved = root / path
    if not resolved.is_file():
        raise AlignmentOffsetError(f"aligned CSV does not exist: {relative}")
    actual = sha256(resolved)
    if actual != digest.upper():
        raise AlignmentOffsetError(
            f"aligned CSV SHA-256 mismatch: expected {digest}, got {actual}"
        )
    return resolved


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AlignmentOffsetError(f"cannot read alignment manifest: {error}") from error
    if not isinstance(value, dict):
        raise AlignmentOffsetError("alignment manifest root must be an object")
    if value.get("contract") != "fluxrt-h3-alignment-offset" or value.get("version") != 1:
        raise AlignmentOffsetError("unsupported alignment-offset identity")
    if value.get("parameter_approval") != "not-granted" or value.get(
        "target_capability_approval"
    ) != "not-granted":
        raise AlignmentOffsetError("alignment input must not claim approval")
    return value


def load_aligned(path: Path) -> list[dict[str, int | float]]:
    required = {
        "sequence",
        "reference_control_tick_unwrapped",
        "truth_mechanical_angle_rad",
        "controller_state",
        "forced_electrical_angle_rad",
    }
    rows: list[dict[str, int | float]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise AlignmentOffsetError(f"aligned CSV is missing columns: {sorted(missing)}")
        for line, raw in enumerate(reader, start=2):
            rows.append({
                "sequence": integer(raw["sequence"], f"aligned line {line} sequence"),
                "reference_control_tick_unwrapped": integer(
                    raw["reference_control_tick_unwrapped"],
                    f"aligned line {line} control tick",
                ),
                "truth_mechanical_angle_rad": finite(
                    raw["truth_mechanical_angle_rad"],
                    f"aligned line {line} mechanical angle",
                ),
                "controller_state": integer(
                    raw["controller_state"], f"aligned line {line} controller state"
                ),
                "forced_electrical_angle_rad": finite(
                    raw["forced_electrical_angle_rad"],
                    f"aligned line {line} forced angle",
                ),
            })
    if not rows:
        raise AlignmentOffsetError("aligned CSV is empty")
    sequences = [int(row["sequence"]) for row in rows]
    if any(((right - left) & 0xFFFFFFFF) != 1 for left, right in zip(sequences, sequences[1:])):
        raise AlignmentOffsetError("aligned query sequence is not contiguous")
    ticks = [int(row["reference_control_tick_unwrapped"]) for row in rows]
    if any(right <= left for left, right in zip(ticks, ticks[1:])):
        raise AlignmentOffsetError("aligned control ticks are duplicate or not forward ordered")
    return rows


def analyse_manifest(manifest_path: Path, project_root: Path) -> dict[str, Any]:
    manifest = load_manifest(manifest_path)
    motor_id = manifest.get("motor_id")
    if not isinstance(motor_id, str) or not motor_id:
        raise AlignmentOffsetError("motor_id must be a non-empty string")
    pole_pairs = integer(manifest.get("pole_pairs"), "pole_pairs", 1)
    direction = integer(manifest.get("mechanical_direction"), "mechanical_direction", -1)
    if direction not in (-1, 1):
        raise AlignmentOffsetError("mechanical_direction must be -1 or 1")
    alignment_state = integer(manifest.get("alignment_state"), "alignment_state")
    requirements = manifest.get("requirements")
    if not isinstance(requirements, dict):
        raise AlignmentOffsetError("requirements must be an object")
    minimum = integer(
        requirements.get("minimum_alignment_samples"), "minimum alignment samples", 2
    )
    tail_count = integer(
        requirements.get("settled_tail_sample_count"), "settled tail sample count", 2
    )
    maximum_candidate_deviation = finite(
        requirements.get("maximum_candidate_deviation_rad"),
        "maximum candidate deviation",
    )
    maximum_forced_deviation = finite(
        requirements.get("maximum_forced_angle_deviation_rad"),
        "maximum forced angle deviation",
    )
    if maximum_candidate_deviation < 0.0 or maximum_forced_deviation < 0.0:
        raise AlignmentOffsetError("alignment deviation bounds must be non-negative")
    if tail_count < minimum:
        raise AlignmentOffsetError("settled tail count must cover the minimum sample count")

    aligned_path = checked_file(
        project_root,
        manifest.get("aligned_csv_path"),
        manifest.get("aligned_csv_sha256"),
    )
    rows = load_aligned(aligned_path)
    alignment = [row for row in rows if int(row["controller_state"]) == alignment_state]
    if len(alignment) < minimum:
        raise AlignmentOffsetError("not enough alignment-state samples")
    if len(alignment) < tail_count:
        raise AlignmentOffsetError("alignment state does not cover the settled tail count")
    selected = alignment[-tail_count:]

    forced_values = [float(row["forced_electrical_angle_rad"]) for row in selected]
    forced_mean = circular_mean(forced_values, "forced angle")
    forced_deviations = [abs(wrap_pi(value - forced_mean)) for value in forced_values]
    if max(forced_deviations) > maximum_forced_deviation:
        raise AlignmentOffsetError("forced alignment angle is not stable")

    additive_values = [
        wrap_tau(
            float(row["forced_electrical_angle_rad"])
            - direction * pole_pairs * float(row["truth_mechanical_angle_rad"])
        )
        for row in selected
    ]
    additive_mean = circular_mean(additive_values, "offset candidate")
    candidate_deviations = [
        abs(wrap_pi(value - additive_mean)) for value in additive_values
    ]
    if max(candidate_deviations) > maximum_candidate_deviation:
        raise AlignmentOffsetError("alignment offset candidate is not stable")

    return {
        "contract": "fluxrt-h3-alignment-offset-result",
        "version": 1,
        "status": "alignment-offset-candidate-not-approved",
        "motor_id": motor_id,
        "alignment_state": alignment_state,
        "alignment_sample_count": len(alignment),
        "selected_tail_sample_count": len(selected),
        "selected_first_sequence": int(selected[0]["sequence"]),
        "selected_last_sequence": int(selected[-1]["sequence"]),
        "forced_electrical_angle_mean_rad": forced_mean,
        "maximum_forced_angle_deviation_rad": max(forced_deviations),
        "additive_time_map_offset_rad": additive_mean,
        "as5600_subtractive_offset_rad": wrap_tau(-additive_mean),
        "maximum_candidate_deviation_rad": max(candidate_deviations),
        "calculation_inputs": [
            "forced_electrical_angle_rad",
            "truth_mechanical_angle_rad",
            "controller_state",
            "pole_pairs",
            "mechanical_direction",
        ],
        "observer_angle_used": False,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    parser.add_argument("--json-output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = analyse_manifest(args.manifest, args.project_root.resolve())
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    except AlignmentOffsetError as error:
        parser.error(str(error))
    print(
        "H3_ALIGNMENT_OFFSET_CANDIDATE_NOT_APPROVED "
        f"samples={result['selected_tail_sample_count']} approval=NOT_GRANTED"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
