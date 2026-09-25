#!/usr/bin/env python3
"""Create, extend, fit, and validate FluxRT phase-voltage calibration records."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import sys


FORMAT_VERSION = 1
SHA256_RE = re.compile(r"[0-9a-f]{64}")
UTC_RE = re.compile(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z")
CHANNEL_MAP = {
    "u": ("PC0", "ADC12_IN6"),
    "v": ("PC3", "ADC12_IN9"),
    "w": ("PC1", "ADC12_IN7"),
}
REQUIRED_EVIDENCE = {"raw-measurements", "meter-reference", "review"}
TOP_KEYS = {"format_version", "record_id", "state", "board", "firmware", "adc", "divider", "acceptance", "channels", "evidence", "approval"}
BOARD_KEYS = {"board_id", "serial", "hardware_revision"}
FIRMWARE_KEYS = {"build_profile", "bin_sha256"}
ADC_KEYS = {"reference_voltage_v", "resolution_bits", "samples_per_point"}
DIVIDER_KEYS = {"enable_pin", "active_level", "upper_ohm", "lower_ohm"}
ACCEPTANCE_KEYS = {"maximum_abs_error_v", "minimum_r_squared"}
CHANNEL_KEYS = {"pin", "adc_input", "points", "fit"}
POINT_KEYS = {"divider_mode", "applied_voltage_v", "raw_mean_counts", "raw_stddev_counts", "samples", "temperature_c"}
FIT_KEYS = {"slope_v_per_count", "intercept_v", "maximum_abs_error_v", "r_squared"}
EVIDENCE_KEYS = {"kind", "path", "sha256"}
APPROVAL_KEYS = {"reviewer", "approved_at_utc"}


class CalibrationError(ValueError):
    """The record is malformed or cannot be trusted."""


def exact_object(value: object, keys: set[str], context: str) -> dict:
    if not isinstance(value, dict):
        raise CalibrationError(f"{context} must be an object")
    missing = sorted(keys - set(value))
    extra = sorted(set(value) - keys)
    if missing or extra:
        raise CalibrationError(f"{context} keys mismatch: missing={missing}, extra={extra}")
    return value


def finite_number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationError(f"{context} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise CalibrationError(f"{context} must be finite")
    return result


def positive(value: object, context: str) -> float:
    result = finite_number(value, context)
    if result <= 0.0:
        raise CalibrationError(f"{context} must be positive")
    return result


def validate_point(raw: object, context: str, minimum_samples: int) -> tuple[float, str]:
    point = exact_object(raw, POINT_KEYS, context)
    divider_mode = point["divider_mode"]
    if divider_mode not in ("on", "off"):
        raise CalibrationError(f"{context}.divider_mode must be on or off")
    applied = finite_number(point["applied_voltage_v"], f"{context}.applied_voltage_v")
    mean = finite_number(point["raw_mean_counts"], f"{context}.raw_mean_counts")
    deviation = finite_number(point["raw_stddev_counts"], f"{context}.raw_stddev_counts")
    finite_number(point["temperature_c"], f"{context}.temperature_c")
    samples = point["samples"]
    if applied < 0.0 or not 0.0 <= mean <= 4095.0 or deviation < 0.0:
        raise CalibrationError(f"{context} contains an out-of-range measurement")
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < minimum_samples:
        raise CalibrationError(f"{context}.samples must be >= adc.samples_per_point")
    return applied, divider_mode


def validate_fit(raw: object, context: str, acceptance: dict) -> None:
    fit = exact_object(raw, FIT_KEYS, context)
    positive(fit["slope_v_per_count"], f"{context}.slope_v_per_count")
    finite_number(fit["intercept_v"], f"{context}.intercept_v")
    error = finite_number(fit["maximum_abs_error_v"], f"{context}.maximum_abs_error_v")
    r_squared = finite_number(fit["r_squared"], f"{context}.r_squared")
    if error < 0.0 or error > acceptance["maximum_abs_error_v"]:
        raise CalibrationError(f"{context} exceeds maximum_abs_error_v")
    if not acceptance["minimum_r_squared"] <= r_squared <= 1.0:
        raise CalibrationError(f"{context} does not meet minimum_r_squared")


def checked_evidence_path(project_root: Path, relative: object, context: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise CalibrationError(f"{context}.path is required")
    candidate = Path(relative)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise CalibrationError(f"{context}.path must stay inside the project")
    root = project_root.resolve()
    resolved = (root / candidate).resolve()
    if resolved != root and root not in resolved.parents:
        raise CalibrationError(f"{context}.path escapes the project")
    return resolved


def validate_record(record: object, project_root: Path) -> dict:
    record = exact_object(record, TOP_KEYS, "record")
    if record["format_version"] != FORMAT_VERSION:
        raise CalibrationError("format_version must be 1")
    if not isinstance(record["record_id"], str) or re.fullmatch(r"[a-z0-9][a-z0-9._-]{2,63}", record["record_id"]) is None:
        raise CalibrationError("record_id has an invalid form")
    if record["state"] not in ("draft", "approved"):
        raise CalibrationError("state must be draft or approved")

    board = exact_object(record["board"], BOARD_KEYS, "board")
    if board["board_id"] != "NUCLEO-G431RB+X-NUCLEO-IHM16M1":
        raise CalibrationError("board.board_id does not match the A18 hardware")
    if not all(isinstance(board[key], str) and board[key].strip() for key in BOARD_KEYS):
        raise CalibrationError("board identity fields must be non-empty strings")

    firmware = exact_object(record["firmware"], FIRMWARE_KEYS, "firmware")
    if firmware["build_profile"] != "calibration":
        raise CalibrationError("firmware.build_profile must be calibration")
    if not isinstance(firmware["bin_sha256"], str) or SHA256_RE.fullmatch(firmware["bin_sha256"]) is None:
        raise CalibrationError("firmware.bin_sha256 must be lowercase SHA-256")

    adc = exact_object(record["adc"], ADC_KEYS, "adc")
    positive(adc["reference_voltage_v"], "adc.reference_voltage_v")
    if adc["resolution_bits"] != 12:
        raise CalibrationError("adc.resolution_bits must be 12")
    minimum_samples = adc["samples_per_point"]
    if isinstance(minimum_samples, bool) or not isinstance(minimum_samples, int) or minimum_samples < 16:
        raise CalibrationError("adc.samples_per_point must be >= 16")

    divider = exact_object(record["divider"], DIVIDER_KEYS, "divider")
    if divider["enable_pin"] != "PC9" or divider["active_level"] != "low":
        raise CalibrationError("divider must use active-low PC9")
    positive(divider["upper_ohm"], "divider.upper_ohm")
    positive(divider["lower_ohm"], "divider.lower_ohm")

    acceptance = exact_object(record["acceptance"], ACCEPTANCE_KEYS, "acceptance")
    acceptance["maximum_abs_error_v"] = positive(acceptance["maximum_abs_error_v"], "acceptance.maximum_abs_error_v")
    acceptance["minimum_r_squared"] = finite_number(acceptance["minimum_r_squared"], "acceptance.minimum_r_squared")
    if not 0.0 <= acceptance["minimum_r_squared"] <= 1.0:
        raise CalibrationError("acceptance.minimum_r_squared must be in 0..1")

    channels = exact_object(record["channels"], set(CHANNEL_MAP), "channels")
    for name, expected_mapping in CHANNEL_MAP.items():
        channel = exact_object(channels[name], CHANNEL_KEYS, f"channels.{name}")
        if (channel["pin"], channel["adc_input"]) != expected_mapping:
            raise CalibrationError(f"channels.{name} mapping does not match firmware")
        if not isinstance(channel["points"], list):
            raise CalibrationError(f"channels.{name}.points must be an array")
        validated_points = [
            validate_point(item, f"channels.{name}.points[{index}]", minimum_samples)
            for index, item in enumerate(channel["points"])
        ]
        voltages = [applied for applied, mode in validated_points if mode == "on"]
        if channel["fit"] is not None:
            validate_fit(channel["fit"], f"channels.{name}.fit", acceptance)
        if record["state"] == "approved":
            if len(set(voltages)) < 3:
                raise CalibrationError(f"channels.{name} needs at least three distinct voltage points")
            if channel["fit"] is None:
                raise CalibrationError(f"channels.{name}.fit is required for approval")

    if not isinstance(record["evidence"], list):
        raise CalibrationError("evidence must be an array")
    observed_kinds: set[str] = set()
    for index, raw_item in enumerate(record["evidence"]):
        context = f"evidence[{index}]"
        item = exact_object(raw_item, EVIDENCE_KEYS, context)
        if item["kind"] not in REQUIRED_EVIDENCE:
            raise CalibrationError(f"{context}.kind is unknown")
        if not isinstance(item["sha256"], str) or SHA256_RE.fullmatch(item["sha256"]) is None:
            raise CalibrationError(f"{context}.sha256 must be lowercase SHA-256")
        path = checked_evidence_path(project_root, item["path"], context)
        if not path.is_file():
            raise CalibrationError(f"{context}.path does not exist")
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != item["sha256"]:
            raise CalibrationError(f"{context} SHA-256 mismatch")
        observed_kinds.add(item["kind"])

    if record["state"] == "draft":
        if record["approval"] is not None:
            raise CalibrationError("draft record must not carry approval")
    else:
        missing = sorted(REQUIRED_EVIDENCE - observed_kinds)
        if missing:
            raise CalibrationError(f"approved record is missing evidence kinds: {missing}")
        approval = exact_object(record["approval"], APPROVAL_KEYS, "approval")
        if not isinstance(approval["reviewer"], str) or not approval["reviewer"].strip():
            raise CalibrationError("approval.reviewer is required")
        if not isinstance(approval["approved_at_utc"], str) or UTC_RE.fullmatch(approval["approved_at_utc"]) is None:
            raise CalibrationError("approval.approved_at_utc must be an explicit UTC timestamp")
    return record


def load_and_validate(path: Path, project_root: Path) -> dict:
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CalibrationError(f"cannot read record: {error}") from error
    return validate_record(record, project_root)


def sha256_file(path: Path) -> str:
    if not path.is_file():
        raise CalibrationError(f"file does not exist: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_record(path: Path, record: dict, project_root: Path) -> None:
    validate_record(record, project_root)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def new_draft_record(args: argparse.Namespace) -> dict:
    return {
        "format_version": FORMAT_VERSION,
        "record_id": args.record_id,
        "state": "draft",
        "board": {
            "board_id": "NUCLEO-G431RB+X-NUCLEO-IHM16M1",
            "serial": args.serial,
            "hardware_revision": args.hardware_revision,
        },
        "firmware": {
            "build_profile": "calibration",
            "bin_sha256": sha256_file(args.firmware_bin),
        },
        "adc": {
            "reference_voltage_v": args.reference_voltage_v,
            "resolution_bits": 12,
            "samples_per_point": args.samples_per_point,
        },
        "divider": {
            "enable_pin": "PC9",
            "active_level": "low",
            "upper_ohm": args.upper_ohm,
            "lower_ohm": args.lower_ohm,
        },
        "acceptance": {
            "maximum_abs_error_v": args.maximum_abs_error_v,
            "minimum_r_squared": args.minimum_r_squared,
        },
        "channels": {
            name: {"pin": pin, "adc_input": adc_input, "points": [], "fit": None}
            for name, (pin, adc_input) in CHANNEL_MAP.items()
        },
        "evidence": [],
        "approval": None,
    }


def read_capture_csv(path: Path) -> tuple[str, int, dict[str, tuple[float, float]]]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    except OSError as error:
        raise CalibrationError(f"cannot read capture CSV: {error}") from error
    required = {"divider_mode", "sequence", "phase_u_raw", "phase_v_raw", "phase_w_raw"}
    if not rows or not required.issubset(rows[0]):
        raise CalibrationError("capture CSV is empty or lacks divider/raw columns")
    modes = {row["divider_mode"] for row in rows}
    if len(modes) != 1 or next(iter(modes)) not in ("on", "off"):
        raise CalibrationError("capture CSV must contain one explicit divider_mode")
    try:
        sequences = [int(row["sequence"], 0) for row in rows]
        values = {
            name: [int(row[f"phase_{name}_raw"], 0) for row in rows]
            for name in CHANNEL_MAP
        }
    except (TypeError, ValueError) as error:
        raise CalibrationError("capture CSV contains a non-integer raw value") from error
    if sequences != list(range(len(rows))):
        raise CalibrationError("capture CSV sequence must be contiguous from zero")
    if any(not 0 <= value <= 4095 for series in values.values() for value in series):
        raise CalibrationError("capture CSV contains an out-of-range ADC value")
    statistics_by_channel = {
        name: (statistics.fmean(series), statistics.pstdev(series))
        for name, series in values.items()
    }
    return next(iter(modes)), len(rows), statistics_by_channel


def project_relative_path(path: Path, project_root: Path) -> str:
    try:
        return path.resolve().relative_to(project_root.resolve()).as_posix()
    except ValueError as error:
        raise CalibrationError("capture CSV must stay inside the project") from error


def add_capture_point(args: argparse.Namespace) -> dict:
    record = load_and_validate(args.record, args.project_root)
    if record["state"] != "draft":
        raise CalibrationError("only a draft record may receive capture points")
    divider_mode, sample_count, stats = read_capture_csv(args.capture_csv)
    mean, deviation = stats[args.channel]
    record["channels"][args.channel]["points"].append(
        {
            "divider_mode": divider_mode,
            "applied_voltage_v": args.applied_voltage_v,
            "raw_mean_counts": mean,
            "raw_stddev_counts": deviation,
            "samples": sample_count,
            "temperature_c": args.temperature_c,
        }
    )
    record["channels"][args.channel]["fit"] = None
    relative_path = project_relative_path(args.capture_csv, args.project_root)
    evidence = {
        "kind": "raw-measurements",
        "path": relative_path,
        "sha256": sha256_file(args.capture_csv),
    }
    record["evidence"] = [
        item for item in record["evidence"]
        if not (item["kind"] == evidence["kind"] and item["path"] == relative_path)
    ]
    record["evidence"].append(evidence)
    validate_record(record, args.project_root)
    return record


def linear_fit(points: list[dict], context: str) -> dict:
    enabled = [point for point in points if point["divider_mode"] == "on"]
    if len({point["applied_voltage_v"] for point in enabled}) < 3:
        raise CalibrationError(f"{context} needs three divider-on voltage points")
    x = [float(point["raw_mean_counts"]) for point in enabled]
    y = [float(point["applied_voltage_v"]) for point in enabled]
    mean_x = statistics.fmean(x)
    mean_y = statistics.fmean(y)
    denominator = sum((value - mean_x) ** 2 for value in x)
    if denominator <= 0.0:
        raise CalibrationError(f"{context} raw means do not span a usable range")
    slope = sum((raw - mean_x) * (voltage - mean_y) for raw, voltage in zip(x, y)) / denominator
    intercept = mean_y - slope * mean_x
    predictions = [slope * raw + intercept for raw in x]
    residuals = [predicted - actual for predicted, actual in zip(predictions, y)]
    total = sum((actual - mean_y) ** 2 for actual in y)
    if slope <= 0.0 or total <= 0.0:
        raise CalibrationError(f"{context} fit is not physically usable")
    residual_sum = sum(error * error for error in residuals)
    return {
        "slope_v_per_count": slope,
        "intercept_v": intercept,
        "maximum_abs_error_v": max(abs(error) for error in residuals),
        "r_squared": 1.0 - residual_sum / total,
    }


def fit_record(record: dict, project_root: Path) -> dict:
    if record["state"] != "draft":
        raise CalibrationError("only a draft record may be fitted")
    for name, channel in record["channels"].items():
        channel["fit"] = linear_fit(channel["points"], f"channels.{name}")
    validate_record(record, project_root)
    return record


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    commands = {"validate", "init-draft", "add-point", "fit"}
    if arguments and arguments[0] not in commands and arguments[0] not in ("-h", "--help"):
        arguments.insert(0, "validate")
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    default_root = Path(__file__).resolve().parents[1]

    validate_parser = subparsers.add_parser("validate", help="validate a record")
    validate_parser.add_argument("record", type=Path)
    validate_parser.add_argument("--project-root", type=Path, default=default_root)

    init_parser = subparsers.add_parser("init-draft", help="create an empty draft")
    init_parser.add_argument("output", type=Path)
    init_parser.add_argument("--record-id", required=True)
    init_parser.add_argument("--serial", required=True)
    init_parser.add_argument("--hardware-revision", required=True)
    init_parser.add_argument("--firmware-bin", type=Path, required=True)
    init_parser.add_argument("--reference-voltage-v", type=float, default=3.3)
    init_parser.add_argument("--samples-per-point", type=int, default=256)
    init_parser.add_argument("--upper-ohm", type=float, default=10000.0)
    init_parser.add_argument("--lower-ohm", type=float, default=2200.0)
    init_parser.add_argument("--maximum-abs-error-v", type=float, default=0.05)
    init_parser.add_argument("--minimum-r-squared", type=float, default=0.999)
    init_parser.add_argument("--project-root", type=Path, default=default_root)

    add_parser = subparsers.add_parser("add-point", help="append one CSV-derived point")
    add_parser.add_argument("record", type=Path)
    add_parser.add_argument("--channel", choices=tuple(CHANNEL_MAP), required=True)
    add_parser.add_argument("--applied-voltage-v", type=float, required=True)
    add_parser.add_argument("--capture-csv", type=Path, required=True)
    add_parser.add_argument("--temperature-c", type=float, required=True)
    add_parser.add_argument("--project-root", type=Path, default=default_root)

    fit_parser = subparsers.add_parser("fit", help="fit all divider-on points")
    fit_parser.add_argument("record", type=Path)
    fit_parser.add_argument("--project-root", type=Path, default=default_root)
    args = parser.parse_args(arguments)
    try:
        if args.command == "validate":
            record = load_and_validate(args.record, args.project_root)
        elif args.command == "init-draft":
            if args.output.exists():
                raise CalibrationError(f"refusing to overwrite existing record: {args.output}")
            record = new_draft_record(args)
            write_record(args.output, record, args.project_root)
        elif args.command == "add-point":
            record = add_capture_point(args)
            write_record(args.record, record, args.project_root)
        else:
            record = fit_record(load_and_validate(args.record, args.project_root), args.project_root)
            write_record(args.record, record, args.project_root)
    except CalibrationError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(
        f"PASS: command={args.command} record={record['record_id']} "
        f"state={record['state']} contract=v{FORMAT_VERSION}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
