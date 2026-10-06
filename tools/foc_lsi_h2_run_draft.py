#!/usr/bin/env python3
"""Bind one real H2 capture bundle into an unapproved matrix-run draft."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


class DraftError(ValueError):
    """Raised when the evidence bundle cannot be bound without ambiguity."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def project_file(root: Path, value: Path, label: str) -> tuple[Path, str]:
    resolved = value.resolve() if value.is_absolute() else root.joinpath(value).resolve()
    try:
        relative = resolved.relative_to(root)
    except ValueError as error:
        raise DraftError(f"{label} must stay inside project root") from error
    if not resolved.is_file():
        raise DraftError(f"{label} does not exist: {relative.as_posix()}")
    return resolved, relative.as_posix()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise DraftError(f"cannot read {label}: {error}") from error
    if not isinstance(value, dict):
        raise DraftError(f"{label} root must be an object")
    return value


def checked_sha(value: str, label: str) -> str:
    if len(value) != 64:
        raise DraftError(f"{label} must contain 64 hex digits")
    try:
        int(value, 16)
    except ValueError as error:
        raise DraftError(f"{label} is not hexadecimal") from error
    return value.upper()


def finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise DraftError(f"{label} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise DraftError(f"{label} must be finite") from error
    if not math.isfinite(result):
        raise DraftError(f"{label} must be finite")
    return result


def validate_capture_summary(
    summary: dict[str, Any], combined_path: Path, angle_path: Path
) -> tuple[float, float]:
    if (
        summary.get("contract") != "fluxrt-dual-serial-capture"
        or summary.get("version") != 1
        or summary.get("status") != "capture-complete"
    ):
        raise DraftError("capture summary is not complete V1 evidence")
    if summary.get("read_only") is not True or summary.get("commands_sent") != 0:
        raise DraftError("capture summary is not read-only")
    if summary.get("errors") != []:
        raise DraftError("capture summary contains reader errors")
    clock = summary.get("clock")
    association = summary.get("static_h2_association")
    sources = summary.get("sources")
    if (
        not isinstance(clock, dict)
        or clock.get("source") != "time.monotonic_ns"
        or clock.get("shared_origin") is not True
    ):
        raise DraftError("capture summary has no shared monotonic clock")
    if (
        not isinstance(association, dict)
        or association.get("kind") != "common-host-static-window-only"
        or association.get("dynamic_alignment_valid") is not False
    ):
        raise DraftError("capture summary is not static H2 evidence")
    if not isinstance(sources, dict) or not all(name in sources for name in ("g431", "dengfoc")):
        raise DraftError("capture summary is missing one serial source")
    if int(sources["g431"].get("flsi_raw_line_count", 0)) <= 0:
        raise DraftError("capture summary contains no G431 FLSI_RAW lines")
    if int(sources["dengfoc"].get("parsed_angle_sample_count", 0)) <= 0:
        raise DraftError("capture summary contains no AS5600 samples")
    artifacts = summary.get("artifacts")
    if not isinstance(artifacts, dict):
        raise DraftError("capture summary has no artifact binding")
    if (
        artifacts.get("combined_lines_path") != combined_path.name
        or str(artifacts.get("combined_lines_sha256", "")).upper() != sha256(combined_path)
        or artifacts.get("angle_csv_path") != angle_path.name
        or str(artifacts.get("angle_csv_sha256", "")).upper() != sha256(angle_path)
    ):
        raise DraftError("capture artifact binding mismatch")
    raw_start = finite(association.get("g431_raw_window_start_s"), "raw window start")
    raw_end = finite(association.get("g431_raw_window_end_s"), "raw window end")
    if raw_start < 0.0 or raw_end < raw_start:
        raise DraftError("capture summary G431 raw window is invalid")
    return raw_start, raw_end


def build_draft(
    project_root: Path,
    run_id: str,
    motor_id: str,
    firmware_sha256: str,
    capture_dir: Path,
    trace_csv: Path,
    analysis_json: Path,
    angle_axis: int,
    window_padding_s: float,
) -> dict[str, Any]:
    root = project_root.resolve()
    if not run_id or not motor_id:
        raise DraftError("run_id and motor_id must be non-empty")
    if angle_axis < 0 or window_padding_s <= 0.0 or not math.isfinite(window_padding_s):
        raise DraftError("angle_axis must be non-negative and window padding must be positive")
    firmware = checked_sha(firmware_sha256, "firmware_sha256")
    capture = capture_dir.resolve() if capture_dir.is_absolute() else root.joinpath(capture_dir).resolve()
    try:
        capture.relative_to(root)
    except ValueError as error:
        raise DraftError("capture directory must stay inside project root") from error

    summary_path, summary_relative = project_file(root, capture / "capture.summary.json", "capture summary")
    combined_path, combined_relative = project_file(root, capture / "combined-lines.csv", "combined lines")
    angle_path, angle_relative = project_file(root, capture / "dengfoc-angle.csv", "angle CSV")
    trace_path, trace_relative = project_file(root, trace_csv, "trace CSV")
    analysis_path, analysis_relative = project_file(root, analysis_json, "analysis JSON")

    summary = load_json(summary_path, "capture summary")
    raw_start, raw_end = validate_capture_summary(summary, combined_path, angle_path)
    analysis = load_json(analysis_path, "analysis JSON")
    if analysis.get("analysis_kind") != "lsi-powered-one-shot-screen":
        raise DraftError("analysis has the wrong kind")
    if analysis.get("decision", {}).get("parameter_approval") != "not-granted":
        raise DraftError("analysis must not claim parameter approval")
    if str(analysis.get("source_metadata", {}).get("hex_sha256", "")).upper() != firmware:
        raise DraftError("analysis firmware SHA does not match the requested firmware")

    start_s = max(0.0, raw_start - window_padding_s)
    end_s = raw_end + window_padding_s
    matrix_run = {
        "id": run_id,
        "motor_id": motor_id,
        "firmware_sha256": firmware,
        "trace_csv_path": trace_relative,
        "trace_csv_sha256": sha256(trace_path),
        "analysis_json_path": analysis_relative,
        "analysis_json_sha256": sha256(analysis_path),
        "capture_summary_path": summary_relative,
        "capture_summary_sha256": sha256(summary_path),
        "combined_lines_path": combined_relative,
        "combined_lines_sha256": sha256(combined_path),
        "angle_csv_path": angle_relative,
        "angle_csv_sha256": sha256(angle_path),
        "angle_axis": angle_axis,
        "angle_window_start_s": start_s,
        "angle_window_end_s": end_s,
    }
    return {
        "contract": "fluxrt-lsi-h2-run-draft",
        "version": 1,
        "status": "evidence-bound-unapproved",
        "matrix_run": matrix_run,
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--motor-id", required=True)
    parser.add_argument("--firmware-sha256", required=True)
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--analysis-json", type=Path, required=True)
    parser.add_argument("--angle-axis", type=int, default=0)
    parser.add_argument("--window-padding-s", type=float, default=0.25)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        draft = build_draft(
            args.project_root,
            args.run_id,
            args.motor_id,
            args.firmware_sha256,
            args.capture_dir,
            args.trace_csv,
            args.analysis_json,
            args.angle_axis,
            args.window_padding_s,
        )
    except DraftError as error:
        parser.error(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(draft, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"H2_RUN_DRAFT_BOUND id={args.run_id} approval=NOT_GRANTED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
