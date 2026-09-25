#!/usr/bin/env python3
"""Verify and render FluxRT production-profile candidates.

The tool is deliberately offline and one-way: it validates a candidate and may
render a C initializer fragment, but it never edits firmware sources and never
turns an approval bit on by itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import zlib


PROFILE_MAGIC = 0x46505246
PROFILE_STRUCT_SIZE = 40
PROFILE_SCHEMA_VERSION = 1
RUNTIME_STRUCT_SIZE = 284
RUNTIME_CONFIG_VERSION = 8

PARAMETER_APPROVAL = 1 << 0
CLOSED_LOOP_APPROVAL = 1 << 1

PARAMETER_EVIDENCE_KINDS = {
    "parameter-identification-raw",
    "parameter-identification-report",
    "review",
}
CLOSED_LOOP_EVIDENCE_KINDS = {
    "independent-truth",
    "fault-injection",
    "long-run",
    "review",
}

TOP_KEYS = {
    "format_version",
    "name",
    "profile",
    "runtime_config",
    "approval",
    "expected",
}
PROFILE_KEYS = {"revision", "board_id", "board_name", "motor_id", "motor_name"}
APPROVAL_KEYS = {"parameters", "closed_loop"}
APPROVAL_SCOPE_KEYS = {
    "approved",
    "approved_runtime_config_crc32",
    "reviewer",
    "approved_at_utc",
    "evidence",
}
EVIDENCE_KEYS = {"kind", "path", "sha256"}
EXPECTED_KEYS = {"runtime_config_crc32", "record_crc32"}
PI_FIELDS = (
    "kp",
    "ki",
    "ts",
    "out_min",
    "out_max",
    "integrator_min",
    "integrator_max",
)
PI_KEYS = set(PI_FIELDS)
RUN_RELIABILITY_KEYS = {"minimum_speed_rpm", "maximum_phase_error_rad"}
ANGLE_COMPENSATION_KEYS = {"park_prediction_ticks", "reverse_park_prediction_ticks"}
INVERTER_MODEL_KEYS = {
    "enabled",
    "observer_voltage_correction_enable",
    "pwm_feedforward_enable",
    "pwm_carrier_frequency_hz",
    "dead_time_s",
    "compensation_gain",
    "current_zero_band_a",
    "current_sign_filter_alpha",
    "device_drop_v",
}

# Canonical C ABI order. Each entry contributes exactly one little-endian word.
RUNTIME_LAYOUT = [
    ("struct_size", "u32"),
    ("config_version", "u32"),
    ("observer_backend", "u32"),
    ("observer_enable", "u32"),
    ("closed_loop_enable", "u32"),
    ("observer_update_divider", "u32"),
    *[(f"id_pi.{name}", "f32") for name in PI_FIELDS],
    *[(f"iq_pi.{name}", "f32") for name in PI_FIELDS],
    *[(f"speed_pi.{name}", "f32") for name in PI_FIELDS],
    ("pole_pairs", "u32"),
    ("pwm_frequency_hz", "u32"),
    ("speed_loop_frequency_hz", "u32"),
    ("stator_resistance_ohm", "f32"),
    ("stator_inductance_h", "f32"),
    ("flux_linkage_wb", "f32"),
    ("rated_current_a", "f32"),
    ("max_speed_rpm", "f32"),
    ("nominal_bus_voltage_v", "f32"),
    ("voltage_utilization", "f32"),
    ("default_target_speed_rpm", "f32"),
    ("alignment_duration_s", "f32"),
    ("open_loop_ramp_duration_s", "f32"),
    ("observer_transition_duration_s", "f32"),
    ("startup_final_speed_rpm", "f32"),
    ("startup_current_a", "f32"),
    ("observer_smo_k_slide_v", "f32"),
    ("observer_smo_boundary_a", "f32"),
    ("observer_emf_filter_alpha", "f32"),
    ("observer_pll_kp", "f32"),
    ("observer_acquisition_pll_kp_ratio", "f32"),
    ("observer_pll_ki", "f32"),
    ("observer_minimum_speed_rpm", "f32"),
    ("observer_minimum_bemf_v", "f32"),
    ("observer_speed_variance_ratio", "f32"),
    ("observer_consecutive_samples", "u32"),
    ("observer_acquisition_timeout_s", "f32"),
    ("observer_loss_timeout_s", "f32"),
    ("closed_loop_speed_ramp_rpm_per_s", "f32"),
    ("speed_pi_preload_ratio", "f32"),
    ("closed_loop_current_slew_a_per_s", "f32"),
    ("observer_run_reliability.minimum_speed_rpm", "f32"),
    ("observer_run_reliability.maximum_phase_error_rad", "f32"),
    ("angle_compensation.park_prediction_ticks", "f32"),
    ("angle_compensation.reverse_park_prediction_ticks", "f32"),
    ("inverter_voltage_model.enabled", "u32"),
    ("inverter_voltage_model.observer_voltage_correction_enable", "u32"),
    ("inverter_voltage_model.pwm_feedforward_enable", "u32"),
    ("inverter_voltage_model.pwm_carrier_frequency_hz", "u32"),
    ("inverter_voltage_model.dead_time_s", "f32"),
    ("inverter_voltage_model.compensation_gain", "f32"),
    ("inverter_voltage_model.current_zero_band_a", "f32"),
    ("inverter_voltage_model.current_sign_filter_alpha", "f32"),
    ("inverter_voltage_model.device_drop_v", "f32"),
]

RUNTIME_TOP_KEYS = {
    path.split(".", 1)[0] for path, _kind in RUNTIME_LAYOUT
}


class CandidateError(ValueError):
    """Raised when a candidate cannot be trusted."""


def require_exact_keys(value: object, expected: set[str], context: str) -> dict:
    if not isinstance(value, dict):
        raise CandidateError(f"{context} must be an object")
    actual = set(value)
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    if missing or extra:
        raise CandidateError(f"{context} keys mismatch: missing={missing}, extra={extra}")
    return value


def require_u32(value: object, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFF:
        raise CandidateError(f"{context} must be a uint32")
    return value


def require_finite_number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CandidateError(f"{context} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise CandidateError(f"{context} must be finite")
    return result


def parse_hex_u32(value: object, context: str) -> int:
    if not isinstance(value, str) or re.fullmatch(r"0x[0-9A-Fa-f]{8}", value) is None:
        raise CandidateError(f"{context} must use the form 0x1234ABCD")
    return int(value, 16)


def nested_value(root: dict, path: str) -> object:
    value: object = root
    for component in path.split("."):
        if not isinstance(value, dict) or component not in value:
            raise CandidateError(f"runtime_config.{path} is missing")
        value = value[component]
    return value


def canonical_runtime_bytes(runtime: dict) -> bytes:
    output = bytearray()
    for path, kind in RUNTIME_LAYOUT:
        value = nested_value(runtime, path)
        if kind == "u32":
            output.extend(struct.pack("<I", require_u32(value, f"runtime_config.{path}")))
        else:
            output.extend(struct.pack("<f", require_finite_number(value, f"runtime_config.{path}")))
    if len(output) != RUNTIME_STRUCT_SIZE:
        raise CandidateError(
            f"internal layout error: expected {RUNTIME_STRUCT_SIZE} bytes, got {len(output)}"
        )
    return bytes(output)


def validate_runtime_shape(runtime: object) -> dict:
    runtime = require_exact_keys(runtime, RUNTIME_TOP_KEYS, "runtime_config")
    require_exact_keys(runtime["id_pi"], PI_KEYS, "runtime_config.id_pi")
    require_exact_keys(runtime["iq_pi"], PI_KEYS, "runtime_config.iq_pi")
    require_exact_keys(runtime["speed_pi"], PI_KEYS, "runtime_config.speed_pi")
    require_exact_keys(
        runtime["observer_run_reliability"],
        RUN_RELIABILITY_KEYS,
        "runtime_config.observer_run_reliability",
    )
    require_exact_keys(
        runtime["angle_compensation"],
        ANGLE_COMPENSATION_KEYS,
        "runtime_config.angle_compensation",
    )
    require_exact_keys(
        runtime["inverter_voltage_model"],
        INVERTER_MODEL_KEYS,
        "runtime_config.inverter_voltage_model",
    )
    canonical_runtime_bytes(runtime)
    if runtime["struct_size"] != RUNTIME_STRUCT_SIZE:
        raise CandidateError(f"runtime_config.struct_size must be {RUNTIME_STRUCT_SIZE}")
    if runtime["config_version"] != RUNTIME_CONFIG_VERSION:
        raise CandidateError(
            f"runtime_config.config_version must be {RUNTIME_CONFIG_VERSION}"
        )
    if runtime["observer_enable"] not in (0, 1) or runtime["closed_loop_enable"] not in (0, 1):
        raise CandidateError("runtime enable fields must be 0 or 1")
    if runtime["closed_loop_enable"] and not runtime["observer_enable"]:
        raise CandidateError("closed loop requires observer_enable=1")
    if runtime["observer_backend"] not in (0, 1, 2):
        raise CandidateError("observer_backend is unknown")
    if not 1 <= runtime["observer_update_divider"] <= 32:
        raise CandidateError("observer_update_divider must be in 1..32")
    inverter = runtime["inverter_voltage_model"]
    for field in (
        "enabled",
        "observer_voltage_correction_enable",
        "pwm_feedforward_enable",
    ):
        if inverter[field] not in (0, 1):
            raise CandidateError(f"inverter_voltage_model.{field} must be 0 or 1")
    if not inverter["enabled"] and (
        inverter["observer_voltage_correction_enable"]
        or inverter["pwm_feedforward_enable"]
    ):
        raise CandidateError("inverter model sub-gates require enabled=1")
    carrier_hz = inverter["pwm_carrier_frequency_hz"]
    control_hz = runtime["pwm_frequency_hz"]
    if not 1_000 <= carrier_hz <= 1_000_000:
        raise CandidateError("inverter PWM carrier must be in 1000..1000000 Hz")
    if carrier_hz < control_hz or carrier_hz % control_hz != 0:
        raise CandidateError("inverter PWM carrier must be an integer multiple of control rate")
    return runtime


def validate_evidence(
    scope: dict,
    context: str,
    required_kinds: set[str],
    project_root: Path,
    runtime_crc: int,
) -> None:
    approved = scope["approved"]
    if not isinstance(approved, bool):
        raise CandidateError(f"{context}.approved must be boolean")
    reviewer = scope["reviewer"]
    approved_at = scope["approved_at_utc"]
    approved_runtime_crc = scope["approved_runtime_config_crc32"]
    evidence = scope["evidence"]
    if not isinstance(evidence, list):
        raise CandidateError(f"{context}.evidence must be an array")
    if not approved:
        if (
            approved_runtime_crc is not None
            or reviewer is not None
            or approved_at is not None
            or evidence
        ):
            raise CandidateError(f"{context} is unapproved and must not carry approval evidence")
        return
    if parse_hex_u32(
        approved_runtime_crc, f"{context}.approved_runtime_config_crc32"
    ) != runtime_crc:
        raise CandidateError(f"{context} is bound to a different runtime CRC")
    if not isinstance(reviewer, str) or not reviewer.strip():
        raise CandidateError(f"{context}.reviewer is required for approval")
    if not isinstance(approved_at, str) or re.fullmatch(
        r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z", approved_at
    ) is None:
        raise CandidateError(f"{context}.approved_at_utc must be an explicit UTC timestamp")

    observed_kinds: set[str] = set()
    for index, raw_item in enumerate(evidence):
        item = require_exact_keys(raw_item, EVIDENCE_KEYS, f"{context}.evidence[{index}]")
        kind = item["kind"]
        relative = item["path"]
        digest = item["sha256"]
        if not isinstance(kind, str) or not kind:
            raise CandidateError(f"{context}.evidence[{index}].kind is required")
        if not isinstance(relative, str) or not relative:
            raise CandidateError(f"{context}.evidence[{index}].path is required")
        path = Path(relative)
        if path.is_absolute() or ".." in path.parts:
            raise CandidateError(f"{context}.evidence[{index}].path must stay inside the project")
        if not isinstance(digest, str) or re.fullmatch(r"[0-9A-Fa-f]{64}", digest) is None:
            raise CandidateError(f"{context}.evidence[{index}].sha256 must contain 64 hex digits")
        absolute = project_root / path
        if not absolute.is_file():
            raise CandidateError(f"approval evidence does not exist: {relative}")
        try:
            absolute.resolve().relative_to(project_root.resolve())
        except ValueError as error:
            raise CandidateError(
                f"approval evidence escapes the project: {relative}"
            ) from error
        actual_digest = hashlib.sha256(absolute.read_bytes()).hexdigest()
        if actual_digest.lower() != digest.lower():
            raise CandidateError(f"approval evidence hash mismatch: {relative}")
        observed_kinds.add(kind)
    missing_kinds = sorted(required_kinds - observed_kinds)
    if missing_kinds:
        raise CandidateError(f"{context} is missing evidence kinds: {missing_kinds}")


def approval_flags(
    approval: dict, runtime: dict, project_root: Path, runtime_crc: int
) -> int:
    approval = require_exact_keys(approval, APPROVAL_KEYS, "approval")
    parameters = require_exact_keys(
        approval["parameters"], APPROVAL_SCOPE_KEYS, "approval.parameters"
    )
    closed_loop = require_exact_keys(
        approval["closed_loop"], APPROVAL_SCOPE_KEYS, "approval.closed_loop"
    )
    if not isinstance(parameters["approved"], bool):
        raise CandidateError("approval.parameters.approved must be boolean")
    if not isinstance(closed_loop["approved"], bool):
        raise CandidateError("approval.closed_loop.approved must be boolean")
    if closed_loop["approved"] and not parameters["approved"]:
        raise CandidateError("closed-loop approval requires parameter approval")
    validate_evidence(
        parameters,
        "approval.parameters",
        PARAMETER_EVIDENCE_KINDS,
        project_root,
        runtime_crc,
    )
    validate_evidence(
        closed_loop,
        "approval.closed_loop",
        CLOSED_LOOP_EVIDENCE_KINDS,
        project_root,
        runtime_crc,
    )
    flags = 0
    if parameters["approved"]:
        flags |= PARAMETER_APPROVAL
    if closed_loop["approved"]:
        flags |= CLOSED_LOOP_APPROVAL
    expected_closed_loop_enable = 1 if closed_loop["approved"] else 0
    if runtime["closed_loop_enable"] != expected_closed_loop_enable:
        raise CandidateError(
            "runtime_config.closed_loop_enable must exactly match closed-loop approval"
        )
    return flags


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def compute_record_crc(profile: dict, runtime: dict, runtime_crc: int, flags: int) -> int:
    words = (
        PROFILE_MAGIC,
        PROFILE_STRUCT_SIZE,
        PROFILE_SCHEMA_VERSION,
        require_u32(profile["revision"], "profile.revision"),
        parse_hex_u32(profile["board_id"], "profile.board_id"),
        parse_hex_u32(profile["motor_id"], "profile.motor_id"),
        require_u32(runtime["config_version"], "runtime_config.config_version"),
        runtime_crc,
        flags,
    )
    return crc32(b"".join(struct.pack("<I", word) for word in words))


def load_and_verify(candidate_path: Path) -> tuple[dict, int, int, int]:
    try:
        candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CandidateError(f"cannot read candidate: {error}") from error
    candidate = require_exact_keys(candidate, TOP_KEYS, "candidate")
    if candidate["format_version"] != 1:
        raise CandidateError("format_version must be 1")
    if not isinstance(candidate["name"], str) or not candidate["name"].strip():
        raise CandidateError("name is required")

    profile = require_exact_keys(candidate["profile"], PROFILE_KEYS, "profile")
    if require_u32(profile["revision"], "profile.revision") == 0:
        raise CandidateError("profile.revision must be non-zero")
    parse_hex_u32(profile["board_id"], "profile.board_id")
    parse_hex_u32(profile["motor_id"], "profile.motor_id")
    for field in ("board_name", "motor_name"):
        if not isinstance(profile[field], str) or not profile[field].strip():
            raise CandidateError(f"profile.{field} is required")

    runtime = validate_runtime_shape(candidate["runtime_config"])
    runtime_crc = crc32(canonical_runtime_bytes(runtime))
    flags = approval_flags(
        candidate["approval"], runtime, candidate_path.resolve().parents[2], runtime_crc
    )
    record_crc = compute_record_crc(profile, runtime, runtime_crc, flags)

    expected = require_exact_keys(candidate["expected"], EXPECTED_KEYS, "expected")
    if parse_hex_u32(expected["runtime_config_crc32"], "expected.runtime_config_crc32") != runtime_crc:
        raise CandidateError(
            f"runtime CRC mismatch: candidate={expected['runtime_config_crc32']}, "
            f"computed=0x{runtime_crc:08X}"
        )
    if parse_hex_u32(expected["record_crc32"], "expected.record_crc32") != record_crc:
        raise CandidateError(
            f"record CRC mismatch: candidate={expected['record_crc32']}, "
            f"computed=0x{record_crc:08X}"
        )
    return candidate, runtime_crc, record_crc, flags


def render_initializer(
    candidate_path: Path, candidate: dict, runtime_crc: int, record_crc: int, flags: int
) -> str:
    profile = candidate["profile"]
    source_digest = hashlib.sha256(candidate_path.read_bytes()).hexdigest().upper()
    project_root = candidate_path.resolve().parents[2]
    source_name = candidate_path.resolve().relative_to(project_root).as_posix()
    approval_state = "CLOSED_LOOP_APPROVED" if flags & CLOSED_LOOP_APPROVAL else (
        "PARAMETERS_APPROVED" if flags & PARAMETER_APPROVAL else "UNAPPROVED"
    )
    return f"""/* AUTO-GENERATED CANDIDATE; DO NOT EDIT.
 * Source: {source_name}
 * Source SHA-256: {source_digest}
 * Approval state: {approval_state}
 *
 * This fragment is not compiled automatically. A reviewer must verify the
 * evidence, then explicitly apply it to foc_production_profile.c.
 */
{{
    0x{PROFILE_MAGIC:08X}UL,
    {PROFILE_STRUCT_SIZE}U,
    {PROFILE_SCHEMA_VERSION}U,
    {profile['revision']}U,
    0x{parse_hex_u32(profile['board_id'], 'profile.board_id'):08X}UL,
    0x{parse_hex_u32(profile['motor_id'], 'profile.motor_id'):08X}UL,
    {RUNTIME_CONFIG_VERSION}U,
    0x{runtime_crc:08X}UL,
    0x{flags:08X}UL,
    0x{record_crc:08X}UL,
}}
"""


def command_verify(args: argparse.Namespace) -> int:
    candidate_path = args.candidate.resolve()
    candidate, runtime_crc, record_crc, flags = load_and_verify(candidate_path)
    print(f"VALID: {candidate['name']}")
    print(f"runtime_config_crc32=0x{runtime_crc:08X}")
    print(f"approval_flags=0x{flags:08X}")
    print(f"record_crc32=0x{record_crc:08X}")
    return 0


def command_generate(args: argparse.Namespace) -> int:
    candidate_path = args.candidate.resolve()
    candidate, runtime_crc, record_crc, flags = load_and_verify(candidate_path)
    rendered = render_initializer(candidate_path, candidate, runtime_crc, record_crc, flags)
    output = args.output.resolve()
    if args.check:
        if not output.is_file() or output.read_text(encoding="utf-8") != rendered:
            raise CandidateError(f"generated output is stale: {output}")
        print(f"CURRENT: {output}")
        return 0
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered, encoding="utf-8", newline="\n")
    print(f"WROTE: {output}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    verify = subparsers.add_parser("verify", help="validate CRCs and approval evidence")
    verify.add_argument("candidate", type=Path)
    verify.set_defaults(handler=command_verify)
    generate = subparsers.add_parser("generate", help="render a non-integrated C candidate")
    generate.add_argument("candidate", type=Path)
    generate.add_argument("--output", type=Path, required=True)
    generate.add_argument("--check", action="store_true", help="fail if output is stale")
    generate.set_defaults(handler=command_generate)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except CandidateError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
