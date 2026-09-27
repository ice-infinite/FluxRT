#!/usr/bin/env python3
"""Build a deterministic host-only phase-voltage fault scenario matrix.

This script defines the *software contract* expected by A22/A24.  It does not
model STM32 registers, open a serial port, flash a target, or claim that the
nominal divider is calibrated.  The speed gate is assumed to have already
declared measured voltage eligible, so the matrix can isolate data-quality and
source-selection behaviour:

* ``CommandModel`` always uses the compensated command-voltage model.
* ``Measured`` accepts only a valid calibrated sample; otherwise it is
  unavailable (fail closed, with no silent command-model substitution).
* ``Hybrid`` uses the measured candidate while it is valid and falls back to
  ``CommandModel`` for every invalid quality state.

The numerical limits below are explicit *host-test fixtures*, not approved
target calibration or production thresholds.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
from typing import Iterable


SCHEMA_VERSION = 1
MODES = ("CommandModel", "Measured", "Hybrid")


@dataclass(frozen=True)
class QualityConfig:
    adc_max_code: int = 4095
    phase_full_scale_v: float = 18.3
    raw_low_rail_max: int = 8
    raw_high_rail_min: int = 4087
    max_age_ticks: int = 2
    line_residual_limit_v: float = 0.35
    open_measured_span_max_v: float = 0.04
    open_expected_span_min_v: float = 0.50


@dataclass(frozen=True)
class Frame:
    expected_v: tuple[float, float, float]
    measured_raw: tuple[int, int, int]


@dataclass(frozen=True)
class Scenario:
    name: str
    category: str
    description: str
    frames: tuple[Frame, ...]
    configured: bool = True
    calibrated: bool = True
    sample_available: bool = True
    age_ticks: int = 0


def _base_expected_frames() -> tuple[tuple[float, float, float], ...]:
    """Return a balanced, changing terminal-voltage fixture.

    Every row has the same common-mode sum.  Quality comparison is performed
    on line-to-line values, so a harmless common-mode offset is not confused
    with a three-phase mismatch.
    """

    rows: list[tuple[float, float, float]] = []
    for step in range(8):
        angle = (2.0 * math.pi * step) / 8.0
        rows.append(
            tuple(
                6.15 + 2.0 * math.sin(angle + phase)
                for phase in (0.0, -2.0 * math.pi / 3.0, 2.0 * math.pi / 3.0)
            )
        )
    return tuple(rows)


def _volts_to_raw(volts: float, config: QualityConfig) -> int:
    clipped = min(config.phase_full_scale_v, max(0.0, volts))
    return int(round(clipped * config.adc_max_code / config.phase_full_scale_v))


def _raw_to_volts(raw: int, config: QualityConfig) -> float:
    return raw * config.phase_full_scale_v / config.adc_max_code


def _quantize_effective_bits(raw: int, adc_bits: int, effective_bits: int) -> int:
    if not (0 < effective_bits <= adc_bits):
        raise ValueError("effective_bits must be in 1..adc_bits")
    quantum = 1 << (adc_bits - effective_bits)
    return int(round(raw / quantum)) * quantum


def _frames_from_transform(
    expected_rows: Iterable[tuple[float, float, float]],
    config: QualityConfig,
    transform,
) -> tuple[Frame, ...]:
    frames: list[Frame] = []
    for index, expected in enumerate(expected_rows):
        measured = transform(index, expected)
        frames.append(
            Frame(
                expected_v=expected,
                measured_raw=tuple(_volts_to_raw(value, config) for value in measured),
            )
        )
    return tuple(frames)


def build_scenarios(config: QualityConfig | None = None) -> tuple[Scenario, ...]:
    """Create all normal and faulty fixtures without external input."""

    cfg = config or QualityConfig()
    expected = _base_expected_frames()
    identity = _frames_from_transform(expected, cfg, lambda _index, row: row)

    offset = _frames_from_transform(
        expected, cfg, lambda _index, row: tuple(value + 0.08 for value in row)
    )
    gain = _frames_from_transform(
        expected, cfg, lambda _index, row: tuple(value * 1.01 for value in row)
    )

    def quantized_transform(_index: int, row: tuple[float, float, float]):
        coarse_raw = tuple(
            _quantize_effective_bits(_volts_to_raw(value, cfg), 12, 9)
            for value in row
        )
        return tuple(_raw_to_volts(raw, cfg) for raw in coarse_raw)

    quantization = _frames_from_transform(expected, cfg, quantized_transform)

    # A delayed sample remains valid when its age is within max_age_ticks.  Its
    # command comparison is aligned to the original timestamp in the evaluator.
    delay_age = 2
    delayed_expected = expected[:-delay_age]
    delayed_measurement = expected[:-delay_age]
    delayed_frames = _frames_from_transform(
        delayed_expected, cfg, lambda index, _row: delayed_measurement[index]
    )

    open_w_value = expected[0][2]
    open_frames = _frames_from_transform(
        expected,
        cfg,
        lambda _index, row: (row[0], row[1], open_w_value),
    )
    inconsistent = _frames_from_transform(
        expected,
        cfg,
        lambda _index, row: (row[0], row[1] + 0.80, row[2]),
    )

    low_saturation = list(identity)
    low_saturation[-1] = replace(
        low_saturation[-1],
        measured_raw=(0, low_saturation[-1].measured_raw[1], low_saturation[-1].measured_raw[2]),
    )
    high_saturation = list(identity)
    high_saturation[-1] = replace(
        high_saturation[-1],
        measured_raw=(
            high_saturation[-1].measured_raw[0],
            cfg.adc_max_code,
            high_saturation[-1].measured_raw[2],
        ),
    )

    return (
        Scenario("baseline", "normal", "quantized 12-bit calibrated samples", identity),
        Scenario("offset", "nonideality", "+80 mV common-mode offset", offset),
        Scenario("gain", "nonideality", "+1 percent common gain error", gain),
        Scenario("quantization", "nonideality", "12-bit samples reduced to 9 effective bits", quantization),
        Scenario("delay", "nonideality", "two-control-tick delay within freshness limit", delayed_frames, age_ticks=delay_age),
        Scenario("stale", "fault", "sample age exceeds freshness limit", identity, age_ticks=cfg.max_age_ticks + 1),
        Scenario("dropout", "fault", "no phase-voltage sample available", identity, sample_available=False),
        Scenario("low_saturation", "fault", "one phase reaches the ADC low rail", tuple(low_saturation)),
        Scenario("high_saturation", "fault", "one phase reaches the ADC high rail", tuple(high_saturation)),
        Scenario("open_suspicion", "fault", "one measured phase is stuck while its command changes", open_frames),
        Scenario("three_phase_inconsistency", "fault", "one phase has an excessive line-to-line residual", inconsistent),
        Scenario("unconfigured", "configuration", "phase-voltage acquisition is not configured", identity, configured=False),
        Scenario("uncalibrated", "configuration", "only nominal, non-observer-eligible scaling exists", identity, calibrated=False),
    )


def _phase_spans(frames: tuple[Frame, ...], config: QualityConfig) -> tuple[list[float], list[float]]:
    expected_spans: list[float] = []
    measured_spans: list[float] = []
    for phase in range(3):
        expected_values = [frame.expected_v[phase] for frame in frames]
        measured_values = [
            _raw_to_volts(frame.measured_raw[phase], config) for frame in frames
        ]
        expected_spans.append(max(expected_values) - min(expected_values))
        measured_spans.append(max(measured_values) - min(measured_values))
    return expected_spans, measured_spans


def _maximum_line_residual_v(frame: Frame, config: QualityConfig) -> float:
    measured = tuple(_raw_to_volts(value, config) for value in frame.measured_raw)
    pairs = ((0, 1), (1, 2), (2, 0))
    return max(
        abs((measured[left] - measured[right]) -
            (frame.expected_v[left] - frame.expected_v[right]))
        for left, right in pairs
    )


def evaluate_quality(scenario: Scenario, config: QualityConfig | None = None) -> dict[str, object]:
    """Evaluate one fixture with an explicit, stable fault priority."""

    cfg = config or QualityConfig()
    if not scenario.configured:
        state, reason = "unconfigured", "phase_voltage_path_not_configured"
    elif not scenario.calibrated:
        state, reason = "uncalibrated", "board_calibration_not_observer_eligible"
    elif not scenario.sample_available:
        state, reason = "dropout", "sample_not_available"
    elif scenario.age_ticks > cfg.max_age_ticks:
        state, reason = "stale", "sample_age_exceeds_limit"
    elif not scenario.frames:
        state, reason = "dropout", "empty_sample_window"
    else:
        latest_raw = scenario.frames[-1].measured_raw
        if any(value <= cfg.raw_low_rail_max for value in latest_raw):
            state, reason = "low_saturation", "adc_code_at_low_rail"
        elif any(value >= cfg.raw_high_rail_min for value in latest_raw):
            state, reason = "high_saturation", "adc_code_at_high_rail"
        else:
            expected_spans, measured_spans = _phase_spans(scenario.frames, cfg)
            stuck_phases = [
                index
                for index, (expected_span, measured_span) in enumerate(
                    zip(expected_spans, measured_spans)
                )
                if expected_span >= cfg.open_expected_span_min_v
                and measured_span <= cfg.open_measured_span_max_v
            ]
            if stuck_phases:
                state, reason = "open_suspicion", "excited_phase_is_stuck"
            else:
                max_residual = max(
                    _maximum_line_residual_v(frame, cfg) for frame in scenario.frames
                )
                if max_residual > cfg.line_residual_limit_v:
                    state, reason = (
                        "three_phase_inconsistency",
                        "line_to_line_residual_exceeds_limit",
                    )
                else:
                    state, reason = "valid", "quality_gate_passed"

    measured_eligible = state == "valid"
    return {
        "state": state,
        "reason": reason,
        "measured_eligible": measured_eligible,
        "age_ticks": scenario.age_ticks,
    }


def expected_source_selection(mode: str, quality: dict[str, object]) -> dict[str, str]:
    """Return the selection contract for one requested observer-voltage mode."""

    if mode not in MODES:
        raise ValueError(f"unknown voltage-source mode: {mode}")
    state = str(quality["state"])
    eligible = bool(quality["measured_eligible"])
    if mode == "CommandModel":
        return {
            "selected_source": "CommandModel",
            "reason": "mode_forces_command_model",
        }
    if mode == "Measured":
        if eligible:
            return {"selected_source": "Measured", "reason": "measured_quality_valid"}
        return {
            "selected_source": "Unavailable",
            "reason": f"measured_rejected_{state}",
        }
    if eligible:
        return {
            "selected_source": "Measured",
            "reason": "hybrid_measured_candidate_valid",
        }
    return {
        "selected_source": "CommandModel",
        "reason": f"hybrid_fallback_{state}",
    }


def run_analysis(config: QualityConfig | None = None) -> dict[str, object]:
    cfg = config or QualityConfig()
    rows: list[dict[str, object]] = []
    for scenario in build_scenarios(cfg):
        quality = evaluate_quality(scenario, cfg)
        rows.append(
            {
                "name": scenario.name,
                "category": scenario.category,
                "description": scenario.description,
                "quality": quality,
                "selection": {
                    mode: expected_source_selection(mode, quality) for mode in MODES
                },
            }
        )
    result: dict[str, object] = {
        "schema": "fluxrt.phase-voltage-fault-scenarios",
        "schema_version": SCHEMA_VERSION,
        "scope": "host-only deterministic contract; no hardware authorization",
        "operating_assumption": "Hybrid speed gate already declares measured candidate eligible",
        "thresholds_are_production_approved": False,
        "thresholds": {
            "adc_max_code": cfg.adc_max_code,
            "phase_full_scale_v": cfg.phase_full_scale_v,
            "raw_low_rail_max": cfg.raw_low_rail_max,
            "raw_high_rail_min": cfg.raw_high_rail_min,
            "max_age_ticks": cfg.max_age_ticks,
            "line_residual_limit_v": cfg.line_residual_limit_v,
            "open_measured_span_max_v": cfg.open_measured_span_max_v,
            "open_expected_span_min_v": cfg.open_expected_span_min_v,
        },
        "mode_semantics": {
            "CommandModel": "always select compensated command voltage",
            "Measured": "accept valid calibrated measured voltage only; otherwise unavailable",
            "Hybrid": "select valid measured candidate, otherwise fall back to CommandModel",
        },
        "scenarios": rows,
    }
    # Validate strict-JSON compatibility at the producer boundary.
    json.dumps(result, allow_nan=False, sort_keys=True)
    return result


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        help="write strict JSON to this path instead of stdout",
    )
    parser.add_argument("--compact", action="store_true", help="omit indentation")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    result = run_analysis()
    payload = json.dumps(
        result,
        allow_nan=False,
        ensure_ascii=False,
        indent=None if args.compact else 2,
        sort_keys=True,
    ) + "\n"
    if args.output is None:
        print(payload, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
