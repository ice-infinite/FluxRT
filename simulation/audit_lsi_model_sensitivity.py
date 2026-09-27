#!/usr/bin/env python3
"""Audit EXP-B3 S5.0 conversion and model sensitivity without touching hardware."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ANALYZER_PATH = PROJECT_ROOT / "simulation" / "analyze_lsi_hardware_run.py"
SPEC = importlib.util.spec_from_file_location("lsi_hardware_analysis", ANALYZER_PATH)
assert SPEC is not None and SPEC.loader is not None
lsi = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = lsi
SPEC.loader.exec_module(lsi)


REFERENCE = {
    "source": "ST MCSDK 6.4.1 P-IHM03-Potentiometer generated project",
    "ioc_sha256": "77D8EF34453AE2050793425A08F72B302D68FBBAF2A672A5D15B3BBF3B6B9281",
    "parameters_conversion_sha256": "367AC8997AB2E7A487437980AE5B3F79FFB3AD508F64E7C7EE4711F7ABB4A882",
    "power_stage_parameters_sha256": "4030848606D0D63B5D2E6F7D0784E825416B7AA705108C7DE23379C91330F26B",
    "motor_parameters_sha256": "2E07D89F3383950E9127738BA128765200F35CFB7473EB7935B039FCC762B9C2",
    "rshunt_ohm": 0.33,
    "amplification_gain": 1.53,
    "bus_partitioning_factor": 0.0625,
    "hardware_dead_time_ns": 550.0,
    "software_dead_time_ns": 525.0,
    "database_stator_resistance_ohm": 5.29,
    "database_stator_inductance_h": 0.001058,
}
PWM_TIMER_CLOCK_HZ = 170_000_000.0


def _pulse(samples: list[lsi.ConvertedSample]) -> list[lsi.ConvertedSample]:
    return [
        sample
        for sample in samples
        if sample.flags & (lsi.FLAG_PULSE_POSITIVE | lsi.FLAG_PULSE_NEGATIVE)
    ]


def joint_fit_rs_l(samples: list[lsi.ConvertedSample]) -> dict[str, float | int]:
    pairs = list(zip(samples, samples[1:]))
    best: tuple[float, float, float, float] | None = None
    for resistance_milliohm in range(3500, 6501, 10):
        resistance = resistance_milliohm / 1000.0
        for inductance_microhenry in range(800, 2501, 2):
            inductance = inductance_microhenry * 1.0e-6
            decay = math.exp(-resistance * lsi.DT_S / inductance)
            squared_error = 0.0
            maximum_error = 0.0
            for left, right in pairs:
                voltage = left.applied_phase_u_v
                predicted = voltage / resistance + (
                    left.current_u_a - voltage / resistance
                ) * decay
                error = right.current_u_a - predicted
                squared_error += error * error
                maximum_error = max(maximum_error, abs(error))
            if best is None or squared_error < best[0]:
                best = (squared_error, resistance, inductance, maximum_error)
    assert best is not None
    return {
        "transition_count": len(pairs),
        "stator_resistance_ohm": best[1],
        "stator_inductance_h": best[2],
        "rmse_a": math.sqrt(best[0] / len(pairs)),
        "maximum_absolute_residual_a": best[3],
    }


def _solve_3x3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    augmented = [row[:] + [vector[index]] for index, row in enumerate(matrix)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        if abs(divisor) < 1.0e-12:
            raise ValueError("singular affine RL regression")
        for item in range(column, 4):
            augmented[column][item] /= divisor
        for row in range(3):
            if row == column:
                continue
            scale = augmented[row][column]
            for item in range(column, 4):
                augmented[row][item] -= scale * augmented[column][item]
    return [augmented[row][3] for row in range(3)]


def affine_rl_fit(samples: list[lsi.ConvertedSample]) -> dict[str, float | int]:
    """Fit i[k+1] = a*i[k] + b*v[k] + c and derive R/L/loss.

    The intercept absorbs one constant inverter-voltage error.  It is a
    diagnostic identifiability check, not a physical approval model.
    """
    rows = [
        (left.current_u_a, left.applied_phase_u_v, 1.0, right.current_u_a)
        for left, right in zip(samples, samples[1:])
    ]
    normal = [
        [sum(row[left] * row[right] for row in rows) for right in range(3)]
        for left in range(3)
    ]
    target = [sum(row[column] * row[3] for row in rows) for column in range(3)]
    decay, voltage_gain, intercept = _solve_3x3(normal, target)
    if not (0.0 < decay < 1.0) or voltage_gain <= 0.0:
        raise ValueError("affine RL fit produced a non-physical discrete model")
    resistance = (1.0 - decay) / voltage_gain
    inductance = -resistance * lsi.DT_S / math.log(decay)
    voltage_loss = -intercept / voltage_gain
    errors = [
        next_current - (decay * current + voltage_gain * voltage + intercept)
        for current, voltage, _one, next_current in rows
    ]
    return {
        "transition_count": len(rows),
        "discrete_decay": decay,
        "discrete_voltage_gain": voltage_gain,
        "stator_resistance_ohm": resistance,
        "stator_inductance_h": inductance,
        "constant_voltage_loss_v": voltage_loss,
        "rmse_a": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "maximum_absolute_residual_a": max(abs(error) for error in errors),
    }


def _evolve_rl(
    current_a: float, voltage_v: float, duration_s: float, inductance_h: float
) -> float:
    decay = math.exp(-lsi.RS_OHM * duration_s / inductance_h)
    return voltage_v / lsi.RS_OHM + (current_a - voltage_v / lsi.RS_OHM) * decay


def _predict_ideal_center_aligned_pwm(
    current_a: float, sample: lsi.ConvertedSample, inductance_h: float
) -> float:
    """Integrate ideal PWM1 leg states from one carrier top to the next."""
    period = sample.pwm_period_ticks
    compares = (sample.compare_u, sample.compare_v, sample.compare_w)
    events: list[tuple[float, int, int]] = []
    for phase, compare in sorted(
        enumerate(compares), key=lambda item: item[1], reverse=True
    ):
        events.append(((period - compare) / PWM_TIMER_CLOCK_HZ, phase, 1))
    for phase, compare in sorted(enumerate(compares), key=lambda item: item[1]):
        events.append(((period + compare) / PWM_TIMER_CLOCK_HZ, phase, 0))

    states = [0, 0, 0]
    previous_time = 0.0
    predicted = current_a
    for event_time, phase, state in events:
        leg_voltage = [value * sample.bus_voltage_v for value in states]
        phase_u_voltage = leg_voltage[0] - sum(leg_voltage) / 3.0
        predicted = _evolve_rl(
            predicted,
            phase_u_voltage,
            event_time - previous_time,
            inductance_h,
        )
        previous_time = event_time
        states[phase] = state
    final_time = 2.0 * period / PWM_TIMER_CLOCK_HZ
    leg_voltage = [value * sample.bus_voltage_v for value in states]
    phase_u_voltage = leg_voltage[0] - sum(leg_voltage) / 3.0
    return _evolve_rl(
        predicted, phase_u_voltage, final_time - previous_time, inductance_h
    )


def fit_ideal_center_aligned_pwm(
    samples: list[lsi.ConvertedSample],
) -> dict[str, float | int]:
    pairs = list(zip(samples, samples[1:]))
    best: tuple[float, float, float] | None = None
    for inductance_h in lsi.GRID_L_H:
        errors = [
            right.current_u_a
            - _predict_ideal_center_aligned_pwm(
                left.current_u_a, left, inductance_h
            )
            for left, right in pairs
        ]
        squared_error = sum(error * error for error in errors)
        if best is None or squared_error < best[0]:
            best = (
                squared_error,
                inductance_h,
                max(abs(error) for error in errors),
            )
    assert best is not None
    return {
        "transition_count": len(pairs),
        "inductance_h": best[1],
        "rmse_a": math.sqrt(best[0] / len(pairs)),
        "maximum_absolute_residual_a": best[2],
    }


def run_audit(text: str) -> dict[str, object]:
    metadata, raw, _records = lsi.parse_log(text)
    offsets = [int(value) for value in metadata["current_offsets_u_v_w"].split(",")]

    st_current_counts = (
        4096.0 * REFERENCE["rshunt_ohm"] * REFERENCE["amplification_gain"] / 3.3
    )
    st_bus_volts_per_count = 3.3 / (
        4096.0 * REFERENCE["bus_partitioning_factor"]
    )
    official_scaled = lsi.convert_samples(
        raw,
        offsets[0],
        offsets[1],
        st_current_counts,
        st_bus_volts_per_count,
        REFERENCE["hardware_dead_time_ns"] * 1.0e-9,
    )
    official_fit = lsi.fit_fixed_resistance(
        _pulse(official_scaled), "applied_phase_u_v"
    )

    dead_time_rows = []
    for dead_time_ns in (0.0, 250.0, 525.0, 550.0, 750.0, 1000.0):
        converted = lsi.convert_samples(
            raw,
            offsets[0],
            offsets[1],
            dead_time_s=dead_time_ns * 1.0e-9,
        )
        fit = lsi.fit_fixed_resistance(_pulse(converted), "applied_phase_u_v")
        dead_time_rows.append({"dead_time_ns": dead_time_ns, **fit})

    device_drop_rows = []
    for device_drop_v in (0.0, 0.05, 0.10, 0.20):
        converted = lsi.convert_samples(
            raw,
            offsets[0],
            offsets[1],
            device_drop_v=device_drop_v,
        )
        fit = lsi.fit_fixed_resistance(_pulse(converted), "applied_phase_u_v")
        device_drop_rows.append({"device_drop_v": device_drop_v, **fit})

    nominal = lsi.convert_samples(raw, offsets[0], offsets[1])
    bias = [
        sample
        for sample in raw
        if (sample.flags & lsi.FLAG_DRIVE_ACTIVE) != 0
        and (sample.flags & (lsi.FLAG_PULSE_POSITIVE | lsi.FLAG_PULSE_NEGATIVE)) == 0
    ]
    bias_tail_start = 32 if len(bias) > 48 else len(bias) // 2
    bias_tail = bias[bias_tail_start:]
    cooldown = [sample for sample in raw if (sample.flags & lsi.FLAG_DRIVE_ACTIVE) == 0]

    def standard_deviation(items: list[lsi.RawSample], field: str) -> float:
        return statistics.stdev(float(getattr(item, field)) for item in items)

    def detrended_standard_deviation(items: list[lsi.RawSample], field: str) -> float:
        values = [float(getattr(item, field)) for item in items]
        center_x = (len(values) - 1) / 2.0
        center_y = statistics.mean(values)
        slope = sum(
            (index - center_x) * (value - center_y)
            for index, value in enumerate(values)
        ) / sum((index - center_x) ** 2 for index in range(len(values)))
        residuals = [
            value - (center_y + slope * (index - center_x))
            for index, value in enumerate(values)
        ]
        return statistics.stdev(residuals)

    measured_noise = {
        "bias_tail_u_detrended_std_counts": detrended_standard_deviation(
            bias_tail, "current_u_raw"
        ),
        "bias_tail_v_detrended_std_counts": detrended_standard_deviation(
            bias_tail, "current_v_raw"
        ),
        "cooldown_u_std_counts": standard_deviation(cooldown, "current_u_raw"),
        "cooldown_v_std_counts": standard_deviation(cooldown, "current_v_raw"),
        "cooldown_bus_std_counts": standard_deviation(cooldown, "bus_voltage_raw"),
        "old_host_model_std_counts": 0.5,
    }
    joint = joint_fit_rs_l(_pulse(nominal))
    affine = affine_rl_fit(_pulse(nominal))
    switched_pwm = fit_ideal_center_aligned_pwm(_pulse(nominal))
    hardware_result, _converted = lsi.analyze(text)
    best_dead_time = min(dead_time_rows, key=lambda row: float(row["rmse_a"]))
    current_scale_difference = (
        lsi.CURRENT_COUNTS_PER_AMP / st_current_counts - 1.0
    ) * 100.0
    bus_scale_difference = (
        lsi.BUS_VOLTS_PER_COUNT / st_bus_volts_per_count - 1.0
    ) * 100.0
    return {
        "format_version": 1,
        "analysis_kind": "lsi-s5.1-model-sensitivity-audit",
        "reference": REFERENCE,
        "scale_cross_check": {
            "fluxrt_current_counts_per_amp": lsi.CURRENT_COUNTS_PER_AMP,
            "st_4096_current_counts_per_amp": st_current_counts,
            "difference_pct": current_scale_difference,
            "fluxrt_bus_volts_per_count": lsi.BUS_VOLTS_PER_COUNT,
            "st_4096_bus_volts_per_count": st_bus_volts_per_count,
            "bus_difference_pct": bus_scale_difference,
            "official_scale_fit": official_fit,
        },
        "dead_time_sensitivity": dead_time_rows,
        "best_tested_dead_time_ns": best_dead_time["dead_time_ns"],
        "device_drop_sensitivity": device_drop_rows,
        "measured_noise": measured_noise,
        "joint_rs_l_fit": joint,
        "affine_rl_with_constant_voltage_loss_fit": affine,
        "ideal_center_aligned_pwm_fit": switched_pwm,
        "within_run_repeatability": {
            "pulse_pair_count": hardware_result["fit"]["pulse_pair_count"],
            "pulse_samples_per_pair": hardware_result["fit"]["pulse_samples_per_pair"],
            "pulse_pair_range_over_mean_pct": hardware_result["fit"]["pulse_pair_range_over_mean_pct"],
            "screen_10pct_pass": hardware_result["fit"]["pulse_pair_range_over_mean_pct"] <= 10.0,
        },
        "decision": {
            "reference_constants_match": abs(current_scale_difference) < 0.1
            and abs(bus_scale_difference) < 0.1,
            "dead_time_model_supported": best_dead_time["dead_time_ns"] in (525.0, 550.0),
            "resistance_model_supported": abs(
                float(joint["stator_resistance_ohm"]) - lsi.RS_OHM
            )
            < 0.3,
            "conversion_constants_explain_l_mismatch": False,
            "ideal_subperiod_pwm_explains_l_mismatch": abs(
                float(switched_pwm["inductance_h"])
                - float(
                    hardware_result["fit"]["command_voltage_fixed_rs"][
                        "inductance_h"
                    ]
                )
            )
            / float(
                hardware_result["fit"]["command_voltage_fixed_rs"][
                    "inductance_h"
                ]
            )
            > 0.10,
            "old_host_noise_model_is_representative": False,
            "repeat_hardware": "not-authorized",
            "next_gate": "redesign or independently validate stationarity/current waveform before another powered run",
        },
        "remaining_hypotheses": [
            *(
                ["the unloaded rotor was not independently locked or angle-tracked during the DC-bias interval"]
                if "fixed" not in metadata.get("motor", "").lower()
                else ["one fixed rotor angle cannot bound angle-dependent incremental inductance"]
            ),
            "six short pulse pairs contain switching-correlated current variation above the host noise model",
            "the 1 kHz LCR equivalent and the 12 kHz PWM dynamic model may not share the same effective inductance",
            "STSPIN830 bridge loss is current dependent; one constant voltage-loss intercept cannot identify it independently",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = run_audit(args.input.read_text(encoding="utf-8"))
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
