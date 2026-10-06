#!/usr/bin/env python3
"""Compare fixed forward/reverse H3 evidence without granting approval.

Every referenced artifact is SHA-256 bound.  The tool proves only that two
already captured, fail-closed trials satisfy an explicitly supplied consistency
envelope.  It never opens a serial port, starts a target, retries a trial, edits
parameters or changes a platform capability bit.
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


class BidirectionalCompareError(ValueError):
    """Raised when bound H3 evidence is incomplete or inconsistent."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise BidirectionalCompareError(f"{label} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise BidirectionalCompareError(f"{label} must be finite") from error
    if not math.isfinite(result):
        raise BidirectionalCompareError(f"{label} must be finite")
    return result


def integer(value: Any, label: str, minimum: int | None = None) -> int:
    if isinstance(value, bool):
        raise BidirectionalCompareError(f"{label} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise BidirectionalCompareError(f"{label} must be an integer") from error
    if str(result) != str(value).strip() or (
        minimum is not None and result < minimum
    ):
        suffix = "" if minimum is None else f" >= {minimum}"
        raise BidirectionalCompareError(f"{label} must be an integer{suffix}")
    return result


def hexadecimal(value: Any, label: str) -> int:
    if not isinstance(value, str) or not value:
        raise BidirectionalCompareError(f"{label} must be hexadecimal")
    try:
        return int(value, 16)
    except ValueError as error:
        raise BidirectionalCompareError(f"{label} must be hexadecimal") from error


def circular_difference(left: float, right: float) -> float:
    return abs((left - right + math.pi) % TAU - math.pi)


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise BidirectionalCompareError(f"cannot read {label}: {error}") from error
    if not isinstance(value, dict):
        raise BidirectionalCompareError(f"{label} root must be an object")
    return value


def checked_json(
    root: Path, relative: Any, digest: Any, label: str
) -> tuple[Path, dict[str, Any]]:
    if not isinstance(relative, str) or not relative:
        raise BidirectionalCompareError(f"{label} path must be non-empty")
    path = Path(relative)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise BidirectionalCompareError(
            f"{label} path must be project-relative without traversal"
        )
    if not isinstance(digest, str) or len(digest) != 64:
        raise BidirectionalCompareError(f"{label} SHA-256 must contain 64 hex digits")
    try:
        int(digest, 16)
    except ValueError as error:
        raise BidirectionalCompareError(f"{label} SHA-256 is not hexadecimal") from error
    resolved = root / path
    if not resolved.is_file():
        raise BidirectionalCompareError(f"{label} does not exist: {relative}")
    actual = sha256(resolved)
    if actual != digest.upper():
        raise BidirectionalCompareError(
            f"{label} SHA-256 mismatch: expected {digest}, got {actual}"
        )
    return resolved, load_json(resolved, label)


def require_non_approval(value: dict[str, Any], label: str) -> None:
    if value.get("parameter_approval") != "not-granted" or value.get(
        "target_capability_approval"
    ) != "not-granted":
        raise BidirectionalCompareError(f"{label} must explicitly deny approval")


def parse_pair(value: str, label: str) -> tuple[int, int]:
    fields = value.split("/")
    if len(fields) != 2:
        raise BidirectionalCompareError(f"{label} is not a pair")
    return integer(fields[0], label), integer(fields[1], label)


def parse_trial_raw(summary_path: Path, summary: dict[str, Any], direction: int) -> dict[str, int]:
    artifacts = summary.get("artifacts")
    if not isinstance(artifacts, dict) or "raw-lines.csv" not in artifacts:
        raise BidirectionalCompareError("capture summary lacks bound raw-lines.csv")
    raw_path = summary_path.parent / "raw-lines.csv"
    if not raw_path.is_file() or sha256(raw_path) != str(artifacts["raw-lines.csv"]).upper():
        raise BidirectionalCompareError("capture raw-lines.csv hash mismatch")
    rows: list[tuple[str, str]] = []
    with raw_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if set(reader.fieldnames or []) != {"source", "raw_line"}:
            raise BidirectionalCompareError("capture raw-lines.csv schema mismatch")
        for row in reader:
            rows.append((row["source"], row["raw_line"]))
    trials = [
        (index, line)
        for index, (source, line) in enumerate(rows)
        if source == "g431" and line.startswith("FADVP,state=")
    ]
    results = [
        (index, line)
        for index, (source, line) in enumerate(rows)
        if source == "g431" and line.startswith("FH3R,")
    ]
    if len(trials) != 1 or len(results) != 1:
        raise BidirectionalCompareError("capture must contain exactly one completed H3 attempt")
    trial_index, trial_line = trials[0]
    result_index, result_line = results[0]
    if result_index <= trial_index:
        raise BidirectionalCompareError("H3 result precedes the completed trial record")
    fields: dict[str, str] = {}
    for item in trial_line.split(",")[1:]:
        if "=" not in item:
            raise BidirectionalCompareError("FADVP completion schema mismatch")
        key, value = item.split("=", 1)
        fields[key] = value
    required = {
        "state", "result", "ticks", "epoch", "miss", "features", "status",
        "snap", "orel", "closed", "speed", "lossus", "diagmiss", "finish",
    }
    if not required.issubset(fields):
        raise BidirectionalCompareError("FADVP completion is missing safety fields")
    ticks, _ = parse_pair(fields["ticks"], "FADVP ticks")
    epoch_before, epoch_after = parse_pair(fields["epoch"], "FADVP epoch")
    miss_before, miss_after = parse_pair(fields["miss"], "FADVP miss")
    speed = integer(fields["speed"], "FADVP speed")
    if (
        integer(fields["state"], "FADVP state") != 4
        or integer(fields["result"], "FADVP result") != 1
        or ticks != 1200
        or epoch_before != epoch_after
        or miss_before != 0
        or miss_after != 0
        or hexadecimal(fields["features"], "FADVP features") != 0
        or hexadecimal(fields["status"], "FADVP status") != 0
        or integer(fields["snap"], "FADVP snap") != 1
        or integer(fields["orel"], "FADVP observer reliability") != 1
        or integer(fields["closed"], "FADVP closed loop") != 1
        or integer(fields["lossus"], "FADVP observer loss") != 0
        or integer(fields["diagmiss"], "FADVP diagnostic miss") != 0
        or integer(fields["finish"], "FADVP finish") != 0
    ):
        raise BidirectionalCompareError("FADVP completion is outside the fixed safe envelope")
    if speed == 0 or (1 if speed > 0 else -1) != direction:
        raise BidirectionalCompareError("measured terminal speed sign disagrees with command")
    result_fields = result_line.split(",")
    if len(result_fields) != 4 or result_fields[2:] != ["50", "1"]:
        raise BidirectionalCompareError("FH3R completion schema mismatch")
    if integer(result_fields[1], "FH3R anchor count", 3) != integer(
        summary.get("anchor_count"), "capture anchor_count", 3
    ):
        raise BidirectionalCompareError("FH3R and capture anchor counts differ")
    terminal_safe = any(
        source == "g431" and line == "FOC st=0 rf=00000000 duty=0/0/0"
        for source, line in rows[result_index + 1 :]
    )
    if not terminal_safe:
        raise BidirectionalCompareError("capture lacks final disabled zero-duty state")
    return {"terminal_speed_rpm": speed, "active_ticks": ticks}


def analyse_side(
    root: Path,
    side: dict[str, Any],
    expected_direction: int,
    requirements: dict[str, Any],
) -> dict[str, Any]:
    direction = integer(side.get("commanded_direction"), "commanded_direction")
    if direction != expected_direction:
        raise BidirectionalCompareError("forward/reverse direction declaration is wrong")
    capture_path, capture = checked_json(
        root, side.get("capture_summary_path"), side.get("capture_summary_sha256"),
        "capture summary",
    )
    _, time_manifest = checked_json(
        root, side.get("time_map_manifest_path"), side.get("time_map_manifest_sha256"),
        "time-map manifest",
    )
    _, time_result = checked_json(
        root, side.get("time_map_result_path"), side.get("time_map_result_sha256"),
        "time-map result",
    )
    _, alignment = checked_json(
        root,
        side.get("alignment_offset_result_path"),
        side.get("alignment_offset_result_sha256"),
        "alignment-offset result",
    )
    for label, value in (
        ("capture", capture), ("time-map manifest", time_manifest),
        ("time-map result", time_result), ("alignment offset", alignment),
    ):
        require_non_approval(value, label)
    if capture.get("contract") != "fluxrt-h3-dynamic-board-capture" or capture.get(
        "status"
    ) != "captured-not-approved":
        raise BidirectionalCompareError("capture is not complete non-approved H3 evidence")
    if time_manifest.get("contract") != "fluxrt-h3-time-map" or time_result.get(
        "contract"
    ) != "fluxrt-h3-time-map-result" or time_result.get("status") != "aligned-offline":
        raise BidirectionalCompareError("time-map evidence identity/status mismatch")
    if alignment.get("contract") != "fluxrt-h3-alignment-offset-result" or alignment.get(
        "status"
    ) != "alignment-offset-candidate-not-approved":
        raise BidirectionalCompareError("alignment-offset evidence identity/status mismatch")
    motor_ids = {
        time_manifest.get("motor_id"), time_result.get("motor_id"), alignment.get("motor_id")
    }
    if len(motor_ids) != 1 or not next(iter(motor_ids), None):
        raise BidirectionalCompareError("evidence motor identities differ")
    declared = capture.get("commanded_mechanical_direction")
    if declared is not None and integer(declared, "capture direction") != direction:
        raise BidirectionalCompareError("capture direction metadata mismatch")
    raw = parse_trial_raw(capture_path, capture, direction)
    mapping = time_result.get("mapping")
    if not isinstance(mapping, dict):
        raise BidirectionalCompareError("time-map result lacks mapping")
    metrics = {
        "motor_id": next(iter(motor_ids)),
        "pole_pairs": integer(time_manifest.get("pole_pairs"), "pole_pairs", 1),
        "mechanical_direction": integer(
            time_manifest.get("mechanical_direction"), "mechanical_direction"
        ),
        "commanded_direction": direction,
        "terminal_speed_rpm": raw["terminal_speed_rpm"],
        "anchor_count": integer(capture.get("anchor_count"), "anchor_count", 3),
        "query_count": integer(capture.get("query_count"), "query_count", 1),
        "reliable_sample_count": integer(
            time_result.get("reliable_sample_count"), "reliable_sample_count", 1
        ),
        "clock_drift_ppm": finite(mapping.get("clock_drift_ppm"), "clock drift"),
        "maximum_anchor_residual_ticks": finite(
            mapping.get("maximum_anchor_residual_ticks"), "anchor residual"
        ),
        "rms_angle_error_rad": finite(
            time_result.get("reliable_rms_angle_error_rad"), "RMS angle error"
        ),
        "maximum_angle_error_rad": finite(
            time_result.get("reliable_maximum_angle_error_rad"), "maximum angle error"
        ),
        "alignment_offset_rad": finite(
            alignment.get("additive_time_map_offset_rad"), "alignment offset"
        ),
        "alignment_candidate_deviation_rad": finite(
            alignment.get("maximum_candidate_deviation_rad"),
            "alignment candidate deviation",
        ),
    }
    scalar_bounds = (
        ("anchor_count", "minimum_anchor_count", lambda a, b: a >= b),
        ("query_count", "minimum_query_count", lambda a, b: a >= b),
        ("reliable_sample_count", "minimum_reliable_sample_count", lambda a, b: a >= b),
        ("clock_drift_ppm", "maximum_absolute_clock_drift_ppm", lambda a, b: abs(a) <= b),
        ("maximum_anchor_residual_ticks", "maximum_anchor_residual_ticks", lambda a, b: a <= b),
        ("rms_angle_error_rad", "maximum_rms_angle_error_rad", lambda a, b: a <= b),
        ("maximum_angle_error_rad", "maximum_angle_error_rad", lambda a, b: a <= b),
        ("alignment_candidate_deviation_rad", "maximum_alignment_candidate_deviation_rad", lambda a, b: a <= b),
    )
    for metric, bound, predicate in scalar_bounds:
        limit = finite(requirements.get(bound), bound)
        if limit < 0 or not predicate(metrics[metric], limit):
            raise BidirectionalCompareError(f"{metric} exceeds bidirectional envelope")
    return metrics


def analyse_manifest(path: Path, project_root: Path) -> dict[str, Any]:
    manifest = load_json(path, "bidirectional manifest")
    if manifest.get("contract") != "fluxrt-h3-bidirectional-consistency" or manifest.get(
        "version"
    ) != 1:
        raise BidirectionalCompareError("unsupported bidirectional manifest identity")
    require_non_approval(manifest, "bidirectional manifest")
    requirements = manifest.get("requirements")
    forward = manifest.get("forward")
    reverse = manifest.get("reverse")
    if not all(isinstance(value, dict) for value in (requirements, forward, reverse)):
        raise BidirectionalCompareError("manifest sides and requirements must be objects")
    forward_metrics = analyse_side(project_root, forward, 1, requirements)
    reverse_metrics = analyse_side(project_root, reverse, -1, requirements)
    if forward_metrics["motor_id"] != reverse_metrics["motor_id"]:
        raise BidirectionalCompareError("forward/reverse motor identities differ")
    if forward_metrics["pole_pairs"] != reverse_metrics["pole_pairs"]:
        raise BidirectionalCompareError("forward/reverse pole-pair counts differ")
    if forward_metrics["mechanical_direction"] != reverse_metrics["mechanical_direction"]:
        raise BidirectionalCompareError("forward/reverse sensor direction conventions differ")
    comparisons = {
        "alignment_offset_delta_rad": circular_difference(
            float(forward_metrics["alignment_offset_rad"]),
            float(reverse_metrics["alignment_offset_rad"]),
        ),
        "terminal_speed_magnitude_delta_rpm": abs(
            abs(int(forward_metrics["terminal_speed_rpm"]))
            - abs(int(reverse_metrics["terminal_speed_rpm"]))
        ),
        "rms_angle_error_delta_rad": abs(
            float(forward_metrics["rms_angle_error_rad"])
            - float(reverse_metrics["rms_angle_error_rad"])
        ),
        "maximum_angle_error_delta_rad": abs(
            float(forward_metrics["maximum_angle_error_rad"])
            - float(reverse_metrics["maximum_angle_error_rad"])
        ),
    }
    comparison_bounds = {
        "alignment_offset_delta_rad": "maximum_alignment_offset_delta_rad",
        "terminal_speed_magnitude_delta_rpm": "maximum_terminal_speed_magnitude_delta_rpm",
        "rms_angle_error_delta_rad": "maximum_rms_angle_error_delta_rad",
        "maximum_angle_error_delta_rad": "maximum_angle_error_delta_rad",
    }
    for metric, bound in comparison_bounds.items():
        limit = finite(requirements.get(bound), bound)
        if limit < 0 or float(comparisons[metric]) > limit:
            raise BidirectionalCompareError(f"{metric} exceeds bidirectional envelope")
    return {
        "contract": "fluxrt-h3-bidirectional-consistency-result",
        "version": 1,
        "status": "bidirectional-consistency-candidate-not-approved",
        "motor_id": forward_metrics["motor_id"],
        "forward": forward_metrics,
        "reverse": reverse_metrics,
        "comparisons": comparisons,
        "automatic_retry": False,
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
    except BidirectionalCompareError as error:
        parser.error(str(error))
    print("H3_BIDIRECTIONAL_CONSISTENCY_CANDIDATE_NOT_APPROVED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
