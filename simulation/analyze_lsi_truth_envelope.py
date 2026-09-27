#!/usr/bin/env python3
"""Audit the S5.4 Ls fit against S5.5 LCR bounds without touching hardware.

The tool has two independent jobs:

1. Re-scale the recorded S5.4 current and applied-voltage channels to quantify
   which calibration errors could move the dynamic fit into the LCR diagnostic
   envelope.
2. Optionally run the already-built Rust ``foc-bringup-sim`` binary over the
   same inductance envelope, once with the current 1.058 mH model and once with
   a matched model.

It never opens a serial port, flashes a target, or grants a powered-run or
parameter-approval permission.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ANALYZER_PATH = PROJECT_ROOT / "simulation" / "analyze_lsi_hardware_run.py"
SPEC = importlib.util.spec_from_file_location("lsi_hardware_analysis", ANALYZER_PATH)
assert SPEC is not None and SPEC.loader is not None
lsi = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = lsi
SPEC.loader.exec_module(lsi)


REFERENCE_MODEL_L_H = 0.001058
MEASURED_RS_OHM = 4.9666667
SCALE_PLAUSIBILITY_LIMIT = 0.05


def load_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def build_diagnostic_envelope(
    s5_5a: dict[str, object], s5_5b: dict[str, object]
) -> dict[str, object]:
    pair_summary = s5_5a["pair_summary"]
    assert isinstance(pair_summary, dict)
    pair_means_mh = [float(row["mean_l_mh"]) for row in pair_summary.values()]
    cross_pair = s5_5a["cross_pair_summary"]
    assert isinstance(cross_pair, dict)
    coarse = s5_5b["user_reported_coarse_range"]
    assert isinstance(coarse, dict)

    lower_h = float(coarse["min_line_to_line_l_mh"]) / 2_000.0
    coarse_upper_h = float(coarse["max_line_to_line_l_mh"]) / 2_000.0
    fixed_upper_h = max(pair_means_mh) / 2_000.0
    upper_h = max(coarse_upper_h, fixed_upper_h)
    fixed_mean_h = float(cross_pair["diagnostic_half_of_line_mean_mh"]) / 1_000.0
    points_h = sorted(
        {
            lower_h,
            REFERENCE_MODEL_L_H,
            fixed_mean_h,
            coarse_upper_h,
            upper_h,
        }
    )
    return {
        "lower_h": lower_h,
        "upper_h": upper_h,
        "simulation_points_h": points_h,
        "source": "S5.5A fixed-angle pair means plus S5.5B manual coarse extrema",
        "interpretation": (
            "line-to-line values divided by two form a diagnostic uncertainty "
            "envelope only; they are not approved Ld/Lq values"
        ),
    }


def _transition_rows(
    samples: list[lsi.ConvertedSample],
) -> list[tuple[float, float, float]]:
    return [
        (left.current_u_a, right.current_u_a, left.applied_phase_u_v)
        for left, right in zip(samples, samples[1:])
        if right.control_tick == left.control_tick + 1
    ]


def fit_scaled_conversion(
    transition_rows: list[tuple[float, float, float]],
    current_multiplier: float,
    voltage_multiplier: float,
) -> dict[str, float | int]:
    """Fit an exact discrete RL decay after scaling current and voltage.

    ``current_multiplier`` means true amperes equal nominal amperes times the
    multiplier.  ``voltage_multiplier`` has the analogous volts definition.
    Resistance remains the independently measured DC value.
    """
    if not transition_rows:
        raise ValueError("no contiguous pulse transitions")
    if current_multiplier <= 0.0 or voltage_multiplier <= 0.0:
        raise ValueError("scale multipliers must be positive")

    regressors: list[float] = []
    targets: list[float] = []
    for current, next_current, voltage in transition_rows:
        scaled_current = current * current_multiplier
        scaled_next = next_current * current_multiplier
        equilibrium = voltage * voltage_multiplier / MEASURED_RS_OHM
        regressors.append(scaled_current - equilibrium)
        targets.append(scaled_next - equilibrium)
    denominator = sum(value * value for value in regressors)
    if denominator <= 1.0e-18:
        raise ValueError("scaled RL regression is singular")
    decay = sum(x * y for x, y in zip(regressors, targets)) / denominator
    if not (0.0 < decay < 1.0):
        return {
            "transition_count": len(transition_rows),
            "current_multiplier": current_multiplier,
            "voltage_multiplier": voltage_multiplier,
            "discrete_decay": decay,
            "inductance_h": math.nan,
            "rmse_a": math.nan,
        }
    inductance_h = -MEASURED_RS_OHM * lsi.DT_S / math.log(decay)
    errors = [target - decay * regressor for regressor, target in zip(regressors, targets)]
    return {
        "transition_count": len(transition_rows),
        "current_multiplier": current_multiplier,
        "voltage_multiplier": voltage_multiplier,
        "discrete_decay": decay,
        "inductance_h": inductance_h,
        "rmse_a": math.sqrt(sum(error * error for error in errors) / len(errors)),
    }


def _inside(value: float, lower: float, upper: float) -> bool:
    return math.isfinite(value) and lower <= value <= upper


def _nearest_entry(rows: list[dict[str, float | int]], lower: float, upper: float) -> dict[str, float | int] | None:
    accepted = [row for row in rows if _inside(float(row["inductance_h"]), lower, upper)]
    if not accepted:
        return None
    return min(
        accepted,
        key=lambda row: max(
            abs(float(row["current_multiplier"]) - 1.0),
            abs(float(row["voltage_multiplier"]) - 1.0),
        ),
    )


def audit_scale_discrimination(
    pulse_samples: list[lsi.ConvertedSample], lower_h: float, upper_h: float
) -> dict[str, object]:
    rows = _transition_rows(pulse_samples)
    nominal = fit_scaled_conversion(rows, 1.0, 1.0)

    current_only = [
        fit_scaled_conversion(rows, step / 1_000.0, 1.0)
        for step in range(500, 1_501)
    ]
    voltage_only = [
        fit_scaled_conversion(rows, 1.0, step / 1_000.0)
        for step in range(500, 1_501)
    ]
    plausible = [
        fit_scaled_conversion(rows, current_step / 100.0, voltage_step / 100.0)
        for current_step in range(95, 106)
        for voltage_step in range(95, 106)
    ]
    # A common current/voltage multiplier cancels from the RL regression, so
    # only their relative ratio is observable.  Sweep that ratio broadly
    # without pretending that a 2-D scale grid contains independent evidence.
    relative_ratio = [
        fit_scaled_conversion(rows, 1.0, step / 1_000.0)
        for step in range(100, 3_001)
    ]

    plausible_l = [
        float(row["inductance_h"])
        for row in plausible
        if math.isfinite(float(row["inductance_h"]))
    ]
    valid_ratio = [
        row
        for row in relative_ratio
        if math.isfinite(float(row["inductance_h"]))
    ]
    closest_ratio = _nearest_entry(relative_ratio, lower_h, upper_h)
    closest_current = _nearest_entry(current_only, lower_h, upper_h)
    closest_voltage = _nearest_entry(voltage_only, lower_h, upper_h)
    minimum_required_deviation = (
        abs(float(closest_ratio["voltage_multiplier"]) - 1.0)
        if closest_ratio is not None
        else None
    )
    return {
        "scale_definition": {
            "current_multiplier": "true A = nominal converted A * multiplier",
            "voltage_multiplier": "true applied V = nominal reconstructed V * multiplier",
            "resistance_ohm": MEASURED_RS_OHM,
        },
        "nominal_exact_discrete_fit": nominal,
        "plausible_plus_minus_5pct": {
            "minimum_fit_h": min(plausible_l),
            "maximum_fit_h": max(plausible_l),
            "reaches_diagnostic_envelope": any(
                _inside(value, lower_h, upper_h) for value in plausible_l
            ),
        },
        "nearest_current_only": closest_current,
        "nearest_voltage_only": closest_voltage,
        "relative_voltage_to_current_scale_scan": {
            "minimum_ratio": 0.1,
            "maximum_ratio": 3.0,
            "minimum_fit": min(valid_ratio, key=lambda row: float(row["inductance_h"])),
            "maximum_fit": max(valid_ratio, key=lambda row: float(row["inductance_h"])),
            "nearest_inside_envelope": closest_ratio,
        },
        "minimum_required_relative_scale_deviation_pct": (
            minimum_required_deviation * 100.0
            if minimum_required_deviation is not None
            else None
        ),
        "decision": {
            "nominal_scale_fit_above_envelope": float(nominal["inductance_h"]) > upper_h,
            "plausible_5pct_scale_error_explains_gap": any(
                _inside(value, lower_h, upper_h) for value in plausible_l
            ),
            "tested_0p1_to_3_relative_scale_reaches_envelope": closest_ratio is not None,
            "current_and_voltage_scales_are_separately_identifiable_from_this_trace": False,
        },
    }


def parse_bringup_summary(output: str) -> dict[str, float | int]:
    line = next(
        (item for item in output.splitlines() if item.startswith("FOC_BRINGUP_SIM_PASS ")),
        None,
    )
    if line is None:
        raise ValueError("foc-bringup-sim output has no PASS summary")
    result: dict[str, float | int] = {}
    integer_fields = {
        "reliable_samples",
        "final_state",
        "samples",
        "pwm_ticks",
        "control_ticks",
        "applied_updates",
    }
    for token in line.split()[1:]:
        key, value = token.split("=", 1)
        result[key] = int(value) if key in integer_fields else float(value)
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(64 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def run_bringup_envelope(binary: Path, points_h: list[float]) -> dict[str, object]:
    if not binary.is_file():
        raise ValueError(f"bringup binary does not exist: {binary}")
    rows: list[dict[str, object]] = []
    for model_mode in ("reference_model", "matched_model"):
        for plant_l_h in points_h:
            model_l_h = REFERENCE_MODEL_L_H if model_mode == "reference_model" else plant_l_h
            command = [
                str(binary),
                "--closed-loop",
                "--duration",
                "10",
                "--target-rpm",
                "582",
                "--bus-voltage",
                "12.3",
                "--sample-every",
                "120",
                "--dead-time-compensation",
                "--dead-time-ns",
                "550",
                "--plant-rs",
                str(MEASURED_RS_OHM),
                "--model-rs",
                str(MEASURED_RS_OHM),
                "--plant-ls",
                f"{plant_l_h:.10f}",
                "--model-ls",
                f"{model_l_h:.10f}",
            ]
            completed = subprocess.run(
                command,
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                text=True,
                encoding="utf-8",
            )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"bringup simulation failed ({model_mode}, {plant_l_h} H): "
                    f"{completed.stderr.strip()}"
                )
            rows.append(
                {
                    "model_mode": model_mode,
                    "plant_l_h": plant_l_h,
                    "model_l_h": model_l_h,
                    **parse_bringup_summary(completed.stdout),
                }
            )
    peak_current_guard_a = 0.81
    functional_pass = all(
        int(row["final_state"]) == 7
        and int(row["reliable_samples"]) > 0
        and float(row["peak_phase_current_a"]) <= peak_current_guard_a
        for row in rows
    )
    return {
        "binary": str(binary.resolve()),
        "binary_sha256": _sha256(binary),
        "scenario": {
            "duration_s": 10.0,
            "target_rpm": 582.0,
            "bus_voltage_v": 12.3,
            "stator_resistance_ohm": MEASURED_RS_OHM,
            "dead_time_ns": 550.0,
            "dead_time_compensation": True,
            "reference_model_l_h": REFERENCE_MODEL_L_H,
        },
        "rows": rows,
        "summary": {
            "run_count": len(rows),
            "peak_current_guard_a": peak_current_guard_a,
            "maximum_peak_phase_current_a": max(
                float(row["peak_phase_current_a"]) for row in rows
            ),
            "minimum_reliable_samples": min(
                int(row["reliable_samples"]) for row in rows
            ),
            "maximum_steady_observer_rmse_rpm": max(
                float(row["steady_observer_rmse_rpm"]) for row in rows
            ),
            "maximum_steady_iq_rmse_a": max(
                float(row["steady_iq_rmse_a"]) for row in rows
            ),
            "maximum_steady_id_rmse_a": max(
                float(row["steady_id_rmse_a"]) for row in rows
            ),
        },
        "decision": {
            "all_runs_complete_reliable_and_below_0p81a_guard": functional_pass,
            "functional_envelope_gate": "pass" if functional_pass else "fail",
            "parameter_accuracy_proven": False,
            "hardware_behavior_proven": False,
        },
    }


def truth_measurement_plan() -> dict[str, object]:
    return {
        "purpose": "separate current conversion, applied voltage, and sample timing before another powered fit",
        "required_observables": [
            {
                "name": "independent_phase_current",
                "minimum": "capture the same 0.2 A bias and both pulse polarities with calibrated amplitude",
                "acceptable_instrument": "isolated current probe or calibrated series shunt plus differential oscilloscope channel",
                "not_acceptable": "bench-supply display or MCU ADC converted with the scale being tested",
            },
            {
                "name": "differential_phase_voltage",
                "minimum": "capture line-to-line PWM voltage over every pulse and integrate each control interval",
                "acceptable_instrument": "isolated differential probe with sufficient bandwidth and common-mode rating",
                "not_acceptable": "ground-referenced logic analyzer or handheld DMM across a switching phase",
            },
            {
                "name": "sample_and_update_timing",
                "minimum": "capture ADC trigger, control ISR marker, and PWM update marker on the same timebase",
                "acceptable_instrument": "oscilloscope or logic analyzer only on isolated 3.3 V timing markers",
                "not_acceptable": "logic analyzer connected to U/V/W power nodes",
            },
            {
                "name": "rotor_angle_index",
                "minimum": "mark or index the locked mechanical position for cross-run comparison",
                "acceptable_instrument": "mechanical index is sufficient for screening; encoder preferred for final Ld/Lq",
                "not_acceptable": "unrecorded manual position for final parameter approval",
            },
        ],
        "minimum_sessions": 3,
        "acceptance": {
            "cross_run_range_over_mean_max_percent": 10.0,
            "current_and_voltage_scale_each_bound_percent": SCALE_PLAUSIBILITY_LIMIT * 100.0,
            "temperature_record_required": True,
            "parameter_candidate_only_after_independent_truth": True,
        },
        "current_available_equipment_gap": (
            "LT1, DMM, bench supply, and LA1010 can support static checks and 3.3 V timing; "
            "they do not independently measure switching phase voltage and dynamic phase current"
        ),
    }


def run_analysis(
    hardware_log: Path,
    s5_5a_path: Path,
    s5_5b_path: Path,
    bringup_binary: Path | None = None,
) -> dict[str, object]:
    metadata, raw, _records = lsi.parse_log(hardware_log.read_text(encoding="utf-8"))
    offsets = [int(value) for value in metadata["current_offsets_u_v_w"].split(",")]
    converted = lsi.convert_samples(raw, offsets[0], offsets[1])
    pulse = lsi._pulse_samples(converted)
    envelope = build_diagnostic_envelope(load_json(s5_5a_path), load_json(s5_5b_path))
    bringup = (
        run_bringup_envelope(bringup_binary, list(envelope["simulation_points_h"]))
        if bringup_binary is not None
        else {
            "decision": {
                "functional_envelope_gate": "not-run",
                "parameter_accuracy_proven": False,
                "hardware_behavior_proven": False,
            }
        }
    )
    scale_audit = audit_scale_discrimination(
        pulse, float(envelope["lower_h"]), float(envelope["upper_h"])
    )
    return {
        "format_version": 1,
        "analysis_kind": "lsi-s5.5c-truth-and-uncertainty-envelope",
        "source": {
            "hardware_log": str(hardware_log),
            "s5_5a": str(s5_5a_path),
            "s5_5b": str(s5_5b_path),
        },
        "diagnostic_inductance_envelope": envelope,
        "scale_discrimination": scale_audit,
        "rust_bringup_envelope": bringup,
        "truth_measurement_plan": truth_measurement_plan(),
        "decision": {
            "manual_dense_angle_sweep_required_now": False,
            "parameter_update": "reject",
            "powered_repeat": "not-authorized",
            "next_gate": "obtain independent dynamic current and differential voltage truth before another powered fit",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware-log", type=Path, required=True)
    parser.add_argument("--s5-5a", type=Path, required=True)
    parser.add_argument("--s5-5b", type=Path, required=True)
    parser.add_argument("--bringup-bin", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = run_analysis(
        args.hardware_log, args.s5_5a, args.s5_5b, args.bringup_bin
    )
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
