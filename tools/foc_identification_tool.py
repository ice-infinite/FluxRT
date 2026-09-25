#!/usr/bin/env python3
"""Validate a bounded motor-identification session and produce a screening report.

The current FluxRT Diagnostic trace contains controller-command voltages at at
most 50 Hz. That is enough to screen the standstill resistance trend, but not to
approve Rs, identify a sub-millisecond inductance, or identify PM flux without
independent speed/angle truth. This tool enforces that boundary.
"""

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


PROJECT_ROOT = Path(__file__).resolve().parents[1]

TOP_KEYS = {
    "format_version",
    "session_id",
    "created_at_utc",
    "purpose",
    "target",
    "conditions",
    "acquisition",
    "capabilities",
    "safety",
}
TARGET_KEYS = {
    "board_id",
    "board_name",
    "motor_id",
    "motor_name",
    "pole_pairs",
    "profile_revision",
    "runtime_config_crc32",
    "firmware_profile",
    "firmware_image_path",
    "firmware_sha256",
}
CONDITION_KEYS = {
    "supply_voltage_nominal_v",
    "supply_current_limit_a",
    "load",
    "software_current_trip_a",
    "closed_loop_enabled",
    "ambient_temperature_c",
    "winding_temperature_c",
}
ACQUISITION_KEYS = {
    "trace_csv_path",
    "trace_csv_sha256",
    "raw_log_path",
    "raw_log_sha256",
    "control_frequency_hz",
    "trace_rate_hz",
    "duration_s",
    "target_speed_rpm",
}
CAPABILITY_KEYS = {
    "actual_phase_voltage_measured",
    "independent_speed_truth",
    "high_rate_current_capture",
    "winding_temperature_measured",
}
SAFETY_KEYS = {
    "allow_motor_run_confirmed",
    "max_allowed_phase_current_a",
    "minimum_bus_voltage_v",
    "maximum_bus_voltage_v",
    "emergency_stop_command",
}

REQUIRED_CSV_COLUMNS = {
    "time_s",
    "step",
    "state",
    "phase_current_a",
    "phase_current_b",
    "phase_current_c",
    "id_ref_a",
    "iq_ref_a",
    "id_a",
    "iq_a",
    "vd_v",
    "vq_v",
    "dc_bus_voltage_v",
    "flags",
    "voltage_limited",
}

# Hardware-fault bits from foc/include/foc_platform.h. OUTPUT_ACTIVE and the
# configured/ready bits are intentionally not included because they are normal
# during a bounded run.
HAZARDOUS_PLATFORM_FLAGS = (
    (1 << 5)   # driver fault
    | (1 << 7) # ADC read error
    | (1 << 10) # hardware break
    | (1 << 11) # software current trip
    | (1 << 15) # deadline miss
    | (1 << 16) # control error
    | (1 << 17) # rejected output
)


class IdentificationError(ValueError):
    """Raised when a session or its evidence is unsafe or inconsistent."""


def exact_object(value: object, keys: set[str], context: str) -> dict:
    if not isinstance(value, dict):
        raise IdentificationError(f"{context} must be an object")
    actual = set(value)
    if actual != keys:
        raise IdentificationError(
            f"{context} keys mismatch: missing={sorted(keys - actual)}, "
            f"extra={sorted(actual - keys)}"
        )
    return value


def finite_number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise IdentificationError(f"{context} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise IdentificationError(f"{context} must be finite")
    return result


def positive_number(value: object, context: str) -> float:
    result = finite_number(value, context)
    if result <= 0.0:
        raise IdentificationError(f"{context} must be positive")
    return result


def positive_integer(value: object, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise IdentificationError(f"{context} must be a positive integer")
    return value


def hex_u32(value: object, context: str) -> int:
    if not isinstance(value, str) or re.fullmatch(r"0x[0-9A-Fa-f]{8}", value) is None:
        raise IdentificationError(f"{context} must use the form 0x1234ABCD")
    return int(value, 16)


def sha256_text(value: object, context: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9A-Fa-f]{64}", value) is None:
        raise IdentificationError(f"{context} must contain 64 hex digits")
    return value.upper()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(64 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def project_file(project_root: Path, relative: object, expected_sha: object, context: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise IdentificationError(f"{context} path is required")
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        raise IdentificationError(f"{context} path must stay inside the project")
    absolute = (project_root / path).resolve()
    try:
        absolute.relative_to(project_root.resolve())
    except ValueError as error:
        raise IdentificationError(f"{context} path escapes the project") from error
    if not absolute.is_file():
        raise IdentificationError(f"{context} file does not exist: {relative}")
    expected_sha = sha256_text(expected_sha, f"{context} SHA-256")
    actual_sha = sha256_file(absolute)
    if actual_sha != expected_sha:
        raise IdentificationError(
            f"{context} SHA-256 mismatch: expected={expected_sha}, actual={actual_sha}"
        )
    return absolute


def parse_csv_number(row: dict[str, str], name: str, row_index: int) -> float:
    try:
        result = float(row[name])
    except (KeyError, TypeError, ValueError) as error:
        raise IdentificationError(f"CSV row {row_index} has invalid {name}") from error
    if not math.isfinite(result):
        raise IdentificationError(f"CSV row {row_index} has non-finite {name}")
    return result


def parse_csv_integer(row: dict[str, str], name: str, row_index: int) -> int:
    try:
        return int(row[name], 0)
    except (KeyError, TypeError, ValueError) as error:
        raise IdentificationError(f"CSV row {row_index} has invalid {name}") from error


def load_trace(path: Path) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or [])
        missing = sorted(REQUIRED_CSV_COLUMNS - columns)
        if missing:
            raise IdentificationError(f"trace CSV is missing columns: {missing}")
        for index, raw in enumerate(reader, start=2):
            row: dict[str, float | int] = {
                name: parse_csv_number(raw, name, index)
                for name in REQUIRED_CSV_COLUMNS
                if name not in {"step", "state", "flags", "voltage_limited"}
            }
            for name in ("step", "state", "flags", "voltage_limited"):
                row[name] = parse_csv_integer(raw, name, index)
            rows.append(row)
    if not rows:
        raise IdentificationError("trace CSV contains no data rows")
    return rows


def parse_run_health(log_path: Path) -> dict[str, int]:
    text = log_path.read_text(encoding="utf-8", errors="replace")
    patterns = {
        "dropped_samples": r"dropped=(\d+)",
        "realtime_errors": r"errors=(\d+)",
        "deadline_misses": r"misses=(\d+)",
        "last_control_status": r"FOC last status=(\d+)",
        "rust_fault_flags": r"rust_fault=0x([0-9a-fA-F]+)",
    }
    result: dict[str, int] = {}
    for name, pattern in patterns.items():
        matches = re.findall(pattern, text)
        if not matches:
            raise IdentificationError(f"raw log does not contain final {name}")
        base = 16 if name == "rust_fault_flags" else 10
        result[name] = int(matches[-1], base)
    if any(result.values()):
        raise IdentificationError(f"run health is not clean: {result}")
    return result


def validate_session_shape(session: object) -> tuple[dict, dict, dict, dict, dict]:
    session = exact_object(session, TOP_KEYS, "session")
    if session["format_version"] != 1:
        raise IdentificationError("format_version must be 1")
    if not isinstance(session["session_id"], str) or re.fullmatch(
        r"[a-z0-9][a-z0-9-]{4,63}", session["session_id"]
    ) is None:
        raise IdentificationError("session_id must be a lowercase kebab-case identifier")
    if not isinstance(session["created_at_utc"], str) or re.fullmatch(
        r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z", session["created_at_utc"]
    ) is None:
        raise IdentificationError("created_at_utc must be an explicit UTC timestamp")
    if session["purpose"] != "screening-only":
        raise IdentificationError("purpose must remain screening-only for format v1")

    target = exact_object(session["target"], TARGET_KEYS, "target")
    conditions = exact_object(session["conditions"], CONDITION_KEYS, "conditions")
    acquisition = exact_object(session["acquisition"], ACQUISITION_KEYS, "acquisition")
    capabilities = exact_object(session["capabilities"], CAPABILITY_KEYS, "capabilities")
    safety = exact_object(session["safety"], SAFETY_KEYS, "safety")

    hex_u32(target["board_id"], "target.board_id")
    hex_u32(target["motor_id"], "target.motor_id")
    hex_u32(target["runtime_config_crc32"], "target.runtime_config_crc32")
    positive_integer(target["pole_pairs"], "target.pole_pairs")
    positive_integer(target["profile_revision"], "target.profile_revision")
    if target["firmware_profile"] != "Diagnostic":
        raise IdentificationError("identification capture requires a Diagnostic image")
    firmware_image_path = target["firmware_image_path"]
    if firmware_image_path is not None and (
        not isinstance(firmware_image_path, str) or not firmware_image_path
    ):
        raise IdentificationError("target.firmware_image_path must be a path or null")
    sha256_text(target["firmware_sha256"], "target.firmware_sha256")
    for name in ("board_name", "motor_name"):
        if not isinstance(target[name], str) or not target[name].strip():
            raise IdentificationError(f"target.{name} is required")

    for name in (
        "supply_voltage_nominal_v",
        "supply_current_limit_a",
        "software_current_trip_a",
    ):
        positive_number(conditions[name], f"conditions.{name}")
    if conditions["closed_loop_enabled"] is not False:
        raise IdentificationError("screening capture must keep closed_loop_enabled=false")
    if not isinstance(conditions["load"], str) or not conditions["load"].strip():
        raise IdentificationError("conditions.load is required")
    for name in ("ambient_temperature_c", "winding_temperature_c"):
        if conditions[name] is not None:
            finite_number(conditions[name], f"conditions.{name}")

    control_hz = positive_integer(acquisition["control_frequency_hz"], "acquisition.control_frequency_hz")
    trace_hz = positive_integer(acquisition["trace_rate_hz"], "acquisition.trace_rate_hz")
    if trace_hz > 50 or control_hz < trace_hz:
        raise IdentificationError("trace rate must be <= 50 Hz and no faster than control")
    positive_number(acquisition["duration_s"], "acquisition.duration_s")
    positive_number(acquisition["target_speed_rpm"], "acquisition.target_speed_rpm")

    for name in CAPABILITY_KEYS:
        if not isinstance(capabilities[name], bool):
            raise IdentificationError(f"capabilities.{name} must be boolean")
    if safety["allow_motor_run_confirmed"] is not True:
        raise IdentificationError("allow_motor_run_confirmed must be true for captured data")
    min_bus = positive_number(safety["minimum_bus_voltage_v"], "safety.minimum_bus_voltage_v")
    max_bus = positive_number(safety["maximum_bus_voltage_v"], "safety.maximum_bus_voltage_v")
    if min_bus >= max_bus:
        raise IdentificationError("minimum bus voltage must be below maximum")
    positive_number(safety["max_allowed_phase_current_a"], "safety.max_allowed_phase_current_a")
    if safety["emergency_stop_command"] != "foc_stop":
        raise IdentificationError("emergency_stop_command must be foc_stop")
    return session, target, conditions, acquisition, capabilities


def analyse_session(session_path: Path, project_root: Path = PROJECT_ROOT) -> dict:
    try:
        session = json.loads(session_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise IdentificationError(f"cannot read session: {error}") from error
    session, target, conditions, acquisition, capabilities = validate_session_shape(session)
    safety = session["safety"]

    firmware_path = None
    firmware_digest = sha256_text(target["firmware_sha256"], "target.firmware_sha256")
    if target["firmware_image_path"] is not None:
        firmware_path = project_file(
            project_root,
            target["firmware_image_path"],
            firmware_digest,
            "firmware image",
        )
    trace_path = project_file(
        project_root,
        acquisition["trace_csv_path"],
        acquisition["trace_csv_sha256"],
        "trace CSV",
    )
    log_path = project_file(
        project_root,
        acquisition["raw_log_path"],
        acquisition["raw_log_sha256"],
        "raw log",
    )
    rows = load_trace(trace_path)
    run_health = parse_run_health(log_path)

    previous_step = -1
    hazardous_rows = 0
    voltage_limited_rows = 0
    maximum_phase_current = 0.0
    bus_values: list[float] = []
    state_counts: dict[str, int] = {}
    for index, row in enumerate(rows, start=2):
        step = int(row["step"])
        if step <= previous_step:
            raise IdentificationError(f"trace step is not strictly increasing at CSV row {index}")
        previous_step = step
        state = int(row["state"])
        if state == 8:
            raise IdentificationError("trace entered the FOC fault state")
        state_counts[str(state)] = state_counts.get(str(state), 0) + 1
        flags = int(row["flags"])
        hazardous_rows += int((flags & HAZARDOUS_PLATFORM_FLAGS) != 0)
        voltage_limited_rows += int(int(row["voltage_limited"]) != 0)
        maximum_phase_current = max(
            maximum_phase_current,
            abs(float(row["phase_current_a"])),
            abs(float(row["phase_current_b"])),
            abs(float(row["phase_current_c"])),
        )
        bus_values.append(float(row["dc_bus_voltage_v"]))
    if hazardous_rows:
        raise IdentificationError(f"trace contains {hazardous_rows} hazardous platform samples")
    if maximum_phase_current > float(safety["max_allowed_phase_current_a"]):
        raise IdentificationError(
            f"phase current {maximum_phase_current:.6g} A exceeds session limit "
            f"{safety['max_allowed_phase_current_a']} A"
        )
    if min(bus_values) < float(safety["minimum_bus_voltage_v"]) or max(bus_values) > float(
        safety["maximum_bus_voltage_v"]
    ):
        raise IdentificationError("trace bus voltage left the declared safety window")

    alignment_rows = [
        row
        for row in rows
        if int(row["state"]) == 3
        and int(row["voltage_limited"]) == 0
        and abs(float(row["iq_a"])) >= 0.25
        and abs(float(row["iq_ref_a"]) - float(row["iq_a"])) <= 0.10
    ]
    if alignment_rows:
        last_alignment_time = max(float(row["time_s"]) for row in alignment_rows)
        alignment_rows = [
            row
            for row in alignment_rows
            if float(row["time_s"]) >= last_alignment_time - 0.25
        ]
    resistance_samples = [
        abs(float(row["vq_v"]) / float(row["iq_a"]))
        for row in alignment_rows
        if abs(float(row["iq_a"])) > 1.0e-6
    ]
    if len(resistance_samples) < 5:
        raise IdentificationError(
            "not enough settled alignment samples for resistance screening (need at least 5)"
        )
    resistance_median = statistics.median(resistance_samples)
    resistance_mad = statistics.median(
        abs(value - resistance_median) for value in resistance_samples
    )

    blocking_reasons = [
        "trace voltage is a controller command, not measured motor-terminal voltage",
    ]
    if firmware_path is None:
        blocking_reasons.append(
            "firmware image is not archived; recorded SHA-256 cannot be re-verified"
        )
    if not capabilities["winding_temperature_measured"]:
        blocking_reasons.append("winding temperature was not measured")
    if not capabilities["high_rate_current_capture"]:
        blocking_reasons.append("50 Hz trace cannot resolve the sub-millisecond RL transient")
    if not capabilities["actual_phase_voltage_measured"]:
        blocking_reasons.append("actual phase voltage is unavailable")
    if not capabilities["independent_speed_truth"]:
        blocking_reasons.append("independent rotor speed/angle truth is unavailable")

    session_digest = hashlib.sha256(session_path.read_bytes()).hexdigest().upper()
    return {
        "format_version": 1,
        "session_id": session["session_id"],
        "session_sha256": session_digest,
        "result": "insufficient-for-revision2",
        "candidate_generation_allowed": False,
        "target": {
            "profile_revision": target["profile_revision"],
            "runtime_config_crc32": target["runtime_config_crc32"],
            "firmware_sha256": firmware_digest,
            "firmware_image_archived": firmware_path is not None,
        },
        "run_health": run_health,
        "trace_summary": {
            "sample_count": len(rows),
            "state_counts": state_counts,
            "maximum_phase_current_a": round(maximum_phase_current, 6),
            "minimum_bus_voltage_v": round(min(bus_values), 6),
            "maximum_bus_voltage_v": round(max(bus_values), 6),
            "voltage_limited_samples": voltage_limited_rows,
            "hazardous_samples": hazardous_rows,
        },
        "estimates": {
            "stator_resistance_screening_ohm": {
                "value": round(resistance_median, 9),
                "median_absolute_deviation_ohm": round(resistance_mad, 9),
                "sample_count": len(resistance_samples),
                "method": "median(abs(vq_command_v / iq_measured_a)) in settled alignment",
                "eligible_for_profile": False,
            },
            "stator_inductance_h": {
                "value": None,
                "eligible_for_profile": False,
                "reason": "high-rate current-step capture and measured terminal voltage are required",
            },
            "flux_linkage_wb": {
                "value": None,
                "eligible_for_profile": False,
                "reason": "measured terminal voltage and independent rotor speed/angle truth are required",
            },
        },
        "blocking_reasons": blocking_reasons,
        "conditions": {
            "closed_loop_enabled": conditions["closed_loop_enabled"],
            "load": conditions["load"],
            "supply_current_limit_a": conditions["supply_current_limit_a"],
        },
    }


def render_report(report: dict) -> str:
    return json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"


def command_analyse(args: argparse.Namespace) -> int:
    report = analyse_session(args.session.resolve())
    rendered = render_report(report)
    output = args.output.resolve()
    if args.check:
        if not output.is_file() or output.read_text(encoding="utf-8") != rendered:
            raise IdentificationError(f"identification report is stale: {output}")
        print(f"CURRENT: {output}")
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8", newline="\n")
        print(f"WROTE: {output}")
    print(
        "SCREENING_ONLY "
        f"Rs={report['estimates']['stator_resistance_screening_ohm']['value']:.6f}ohm "
        "revision2=REFUSED"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    analyse = subparsers.add_parser("analyze", help="validate evidence and write screening report")
    analyse.add_argument("session", type=Path)
    analyse.add_argument("--output", type=Path, required=True)
    analyse.add_argument("--check", action="store_true", help="fail when the report is stale")
    analyse.set_defaults(handler=command_analyse)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.handler(args)
    except IdentificationError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
