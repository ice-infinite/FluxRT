#!/usr/bin/env python3
"""Parse and screen one EXP-B3 powered Ls(I) serial capture.

This tool is deliberately offline: it never opens a serial port and cannot
arm the target.  A successful bounded run and an acceptable parameter estimate
are reported separately so one safe pulse train cannot silently approve a new
motor parameter.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import re
import statistics


ADC_FULL_SCALE = 4095.0
ADC_REFERENCE_V = 3.3
BUS_DIVIDER = 0.0625
SHUNT_OHM = 0.33
AMPLIFIER_GAIN = 1.53
CURRENT_COUNTS_PER_AMP = ADC_FULL_SCALE * SHUNT_OHM * AMPLIFIER_GAIN / ADC_REFERENCE_V
BUS_VOLTS_PER_COUNT = ADC_REFERENCE_V / (ADC_FULL_SCALE * BUS_DIVIDER)
SAMPLE_RATE_HZ = 12_000.0
DT_S = 1.0 / SAMPLE_RATE_HZ
DEAD_TIME_S = 550.0e-9
CURRENT_ZERO_BAND_A = 0.005
RS_OHM = 4.9666667
LCR_PHASE_L_H = 0.0011133333
GRID_L_H = tuple(value * 1.0e-6 for value in range(200, 3001))

FLAG_DRIVE_ACTIVE = 1 << 4
FLAG_PULSE_POSITIVE = 1 << 5
FLAG_PULSE_NEGATIVE = 1 << 6
FLAG_ADC_VALID = 1 << 7
FLAG_HARDWARE_FAULT = 1 << 8
FLAG_SOFTWARE_TRIP = 1 << 9


@dataclass(frozen=True)
class RawSample:
    sequence: int
    control_tick: int
    current_u_raw: int
    current_v_raw: int
    bus_voltage_raw: int
    compare_u: int
    compare_v: int
    compare_w: int
    pwm_period_ticks: int
    flags: int


@dataclass(frozen=True)
class ConvertedSample:
    sequence: int
    control_tick: int
    current_u_a: float
    current_v_a: float
    current_w_a: float
    bus_voltage_v: float
    command_phase_u_v: float
    applied_phase_u_v: float
    compare_u: int
    compare_v: int
    compare_w: int
    pwm_period_ticks: int
    flags: int


def _parse_csv_ints(line: str, prefix: str, count: int) -> list[int]:
    fields = line.split(",")
    if fields[0] != prefix or len(fields) != count + 1:
        raise ValueError(f"malformed {prefix} line: {line}")
    return [int(value, 0) for value in fields[1:]]


def parse_log(text: str) -> tuple[dict[str, str], list[RawSample], dict[str, list[list[int]]]]:
    metadata: dict[str, str] = {}
    samples: list[RawSample] = []
    records: dict[str, list[list[int]]] = {
        "FLSI_STATUS": [],
        "FLSI_HW": [],
        "FLSI_START": [],
        "FLSI_CAP_STATUS": [],
        "FTIMING": [],
    }
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line.startswith("# ") and "=" in line:
            key, value = line[2:].split("=", 1)
            metadata[key.strip()] = value.strip()
        elif line.startswith("FLSI_RAW,"):
            samples.append(RawSample(*_parse_csv_ints(line, "FLSI_RAW", 10)))
        else:
            for prefix in records:
                if line.startswith(prefix + ","):
                    records[prefix].append(
                        [int(value, 0) for value in line.split(",")[1:]]
                    )
                    break
    return metadata, samples, records


def validate_raw(samples: list[RawSample]) -> dict[str, object]:
    if not samples:
        raise ValueError("no FLSI_RAW samples found")
    sequence_contiguous = all(sample.sequence == index for index, sample in enumerate(samples))
    tick_contiguous = all(
        right.control_tick == left.control_tick + 1
        for left, right in zip(samples, samples[1:])
    )
    periods = {sample.pwm_period_ticks for sample in samples}
    adc_valid = all((sample.flags & FLAG_ADC_VALID) != 0 for sample in samples)
    fault_free = all(
        (sample.flags & (FLAG_HARDWARE_FAULT | FLAG_SOFTWARE_TRIP)) == 0
        for sample in samples
    )
    positive = sum((sample.flags & FLAG_PULSE_POSITIVE) != 0 for sample in samples)
    negative = sum((sample.flags & FLAG_PULSE_NEGATIVE) != 0 for sample in samples)
    active_without_pulse = sum(
        (sample.flags & FLAG_DRIVE_ACTIVE) != 0
        and (sample.flags & (FLAG_PULSE_POSITIVE | FLAG_PULSE_NEGATIVE)) == 0
        for sample in samples
    )
    output_off = sum((sample.flags & FLAG_DRIVE_ACTIVE) == 0 for sample in samples)
    return {
        "sample_count": len(samples),
        "sequence_contiguous": sequence_contiguous,
        "tick_contiguous": tick_contiguous,
        "first_tick": samples[0].control_tick,
        "last_tick": samples[-1].control_tick,
        "pwm_period_ticks": sorted(periods),
        "adc_valid_all": adc_valid,
        "fault_free_all": fault_free,
        "bias_samples": active_without_pulse,
        "positive_pulse_samples": positive,
        "negative_pulse_samples": negative,
        "output_off_samples": output_off,
        "contract_ok": sequence_contiguous
        and tick_contiguous
        and periods == {7083}
        and adc_valid
        and fault_free,
    }


def _polarity(current_a: float) -> float:
    return max(-1.0, min(1.0, current_a / CURRENT_ZERO_BAND_A))


def convert_samples(
    samples: list[RawSample],
    offset_u: int,
    offset_v: int,
    current_counts_per_amp: float = CURRENT_COUNTS_PER_AMP,
    bus_volts_per_count: float = BUS_VOLTS_PER_COUNT,
    dead_time_s: float = DEAD_TIME_S,
    device_drop_v: float = 0.0,
) -> list[ConvertedSample]:
    converted: list[ConvertedSample] = []
    for sample in samples:
        current_u = (offset_u - sample.current_u_raw) / current_counts_per_amp
        current_v = (offset_v - sample.current_v_raw) / current_counts_per_amp
        current_w = -current_u - current_v
        bus_v = sample.bus_voltage_raw * bus_volts_per_count
        duties = (
            sample.compare_u / sample.pwm_period_ticks,
            sample.compare_v / sample.pwm_period_ticks,
            sample.compare_w / sample.pwm_period_ticks,
        )
        command_legs = tuple(duty * bus_v for duty in duties)
        command_common = sum(command_legs) / 3.0
        command_u = command_legs[0] - command_common
        loss_magnitude = 2.0 * dead_time_s / DT_S * bus_v + device_drop_v
        loss = tuple(
            loss_magnitude * _polarity(current)
            for current in (current_u, current_v, current_w)
        )
        applied_legs = tuple(
            command - phase_loss for command, phase_loss in zip(command_legs, loss)
        )
        applied_u = applied_legs[0] - sum(applied_legs) / 3.0
        converted.append(
            ConvertedSample(
                sample.sequence,
                sample.control_tick,
                current_u,
                current_v,
                current_w,
                bus_v,
                command_u,
                applied_u,
                sample.compare_u,
                sample.compare_v,
                sample.compare_w,
                sample.pwm_period_ticks,
                sample.flags,
            )
        )
    return converted


def _pulse_samples(samples: list[ConvertedSample]) -> list[ConvertedSample]:
    return [
        sample
        for sample in samples
        if (sample.flags & (FLAG_PULSE_POSITIVE | FLAG_PULSE_NEGATIVE)) != 0
    ]


def _pulse_pairs(samples: list[ConvertedSample]) -> list[list[ConvertedSample]]:
    """Split +V/-V injection rows without assuming 4 or 6 ticks/polarity."""
    pairs: list[list[ConvertedSample]] = []
    current_pair: list[ConvertedSample] = []
    previous_polarity = 0
    for sample in _pulse_samples(samples):
        polarity = 1 if (sample.flags & FLAG_PULSE_POSITIVE) != 0 else -1
        if polarity > 0 and previous_polarity < 0:
            pairs.append(current_pair)
            current_pair = []
        current_pair.append(sample)
        previous_polarity = polarity
    if current_pair:
        pairs.append(current_pair)

    for pair in pairs:
        polarities = [
            1 if (sample.flags & FLAG_PULSE_POSITIVE) != 0 else -1
            for sample in pair
        ]
        first_negative = next(
            (index for index, polarity in enumerate(polarities) if polarity < 0),
            len(polarities),
        )
        if (
            first_negative == 0
            or first_negative == len(polarities)
            or any(polarity < 0 for polarity in polarities[:first_negative])
            or any(polarity > 0 for polarity in polarities[first_negative:])
        ):
            raise ValueError("pulse pair is not one positive run followed by one negative run")
    return pairs


def fit_fixed_resistance(
    samples: list[ConvertedSample], voltage_field: str, voltage_shift: int = 0
) -> dict[str, float | int]:
    pairs: list[tuple[float, float, float]] = []
    for index, (left, right) in enumerate(zip(samples, samples[1:])):
        voltage_index = index + voltage_shift
        if (
            right.control_tick != left.control_tick + 1
            or voltage_index < 0
            or voltage_index >= len(samples)
        ):
            continue
        pairs.append(
            (
                left.current_u_a,
                right.current_u_a,
                float(getattr(samples[voltage_index], voltage_field)),
            )
        )
    if len(pairs) < 3:
        raise ValueError("not enough contiguous pulse transitions")
    best: tuple[float, float, float] | None = None
    for inductance_h in GRID_L_H:
        decay = math.exp(-RS_OHM * DT_S / inductance_h)
        errors = [
            next_current
            - (voltage / RS_OHM + (current - voltage / RS_OHM) * decay)
            for current, next_current, voltage in pairs
        ]
        squared_error = sum(error * error for error in errors)
        if best is None or squared_error < best[0]:
            best = (squared_error, inductance_h, max(abs(error) for error in errors))
    assert best is not None
    return {
        "transition_count": len(pairs),
        "inductance_h": best[1],
        "rmse_a": math.sqrt(best[0] / len(pairs)),
        "maximum_absolute_residual_a": best[2],
    }


def residual_for_candidate(
    samples: list[ConvertedSample], voltage_field: str, inductance_h: float
) -> dict[str, float]:
    decay = math.exp(-RS_OHM * DT_S / inductance_h)
    errors = []
    for left, right in zip(samples, samples[1:]):
        voltage = float(getattr(left, voltage_field))
        predicted = voltage / RS_OHM + (left.current_u_a - voltage / RS_OHM) * decay
        errors.append(right.current_u_a - predicted)
    return {
        "inductance_h": inductance_h,
        "rmse_a": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "maximum_absolute_residual_a": max(abs(error) for error in errors),
    }


def analyze(text: str) -> tuple[dict[str, object], list[ConvertedSample]]:
    metadata, raw_samples, records = parse_log(text)
    contract = validate_raw(raw_samples)
    offsets = [int(value) for value in metadata["current_offsets_u_v_w"].split(",")]
    converted = convert_samples(raw_samples, offsets[0], offsets[1])
    pulse = _pulse_samples(converted)
    pulse_pairs = _pulse_pairs(converted)
    if len(pulse_pairs) != 6:
        raise ValueError(f"expected 6 pulse pairs, got {len(pulse_pairs)}")
    samples_per_pair = [len(pair) for pair in pulse_pairs]
    if len(set(samples_per_pair)) != 1:
        raise ValueError(f"pulse pairs have inconsistent lengths: {samples_per_pair}")

    applied_fit = fit_fixed_resistance(pulse, "applied_phase_u_v")
    command_fit = fit_fixed_resistance(pulse, "command_phase_u_v")
    alignment = {
        str(shift): fit_fixed_resistance(pulse, "applied_phase_u_v", shift)
        for shift in (-1, 0, 1)
    }
    pair_fits = [
        fit_fixed_resistance(pair, "applied_phase_u_v") for pair in pulse_pairs
    ]
    pair_estimates = [float(item["inductance_h"]) for item in pair_fits]
    pair_mean = statistics.mean(pair_estimates)
    pair_range_over_mean = (max(pair_estimates) - min(pair_estimates)) / pair_mean
    candidate_residuals = [
        residual_for_candidate(pulse, "applied_phase_u_v", inductance_h)
        for inductance_h in (0.000893, 0.001058, LCR_PHASE_L_H, float(applied_fit["inductance_h"]))
    ]

    start = records["FLSI_START"][-1]
    final_status = records["FLSI_STATUS"][-1]
    capture = records["FLSI_CAP_STATUS"][-1]
    timing = records["FTIMING"][-1]
    isr_cycles = timing[4]
    isr_budget = timing[13]
    maximum_current = max(
        max(abs(sample.current_u_a), abs(sample.current_v_a), abs(sample.current_w_a))
        for sample in converted
    )
    minimum_bus_voltage = min(sample.bus_voltage_v for sample in converted)
    maximum_bus_voltage = max(sample.bus_voltage_v for sample in converted)
    bounded_run_pass = (
        start[1] == 0
        and final_status[3] == 7
        and final_status[4] == 0
        and final_status[5] == 1
        and final_status[6] == 0
        and final_status[12] == 1
        and final_status[13] == 1
        and capture[1] == 2
        and capture[4] == len(raw_samples)
        and capture[6:] == [0, 0, 0, 0]
        and timing[14] == 0
        and bool(contract["contract_ok"])
    )
    lcr_difference = float(applied_fit["inductance_h"]) / LCR_PHASE_L_H - 1.0
    result: dict[str, object] = {
        "format_version": 1,
        "analysis_kind": "lsi-powered-one-shot-screen",
        "source_metadata": metadata,
        "nominal_conversion": {
            "current_counts_per_amp": CURRENT_COUNTS_PER_AMP,
            "bus_volts_per_count": BUS_VOLTS_PER_COUNT,
            "sample_rate_hz": SAMPLE_RATE_HZ,
            "dead_time_s": DEAD_TIME_S,
            "current_zero_band_a": CURRENT_ZERO_BAND_A,
            "stator_resistance_ohm": RS_OHM,
        },
        "raw_contract": contract,
        "safety": {
            "bounded_run_pass": bounded_run_pass,
            "final_state": final_status[3],
            "abort_reason": final_status[4],
            "force_safe_output": final_status[5],
            "final_drive_request": final_status[6],
            "start_count": final_status[12],
            "start_consumed": final_status[13],
            "capture_state": capture[1],
            "capture_samples": capture[4],
            "capture_overflow_count": capture[6],
            "contract_error_count": capture[8],
            "storage_error_count": capture[9],
            "deadline_miss_count": timing[14],
            "session_control_ticks": timing[1],
            "session_control_duration_ms": timing[1] / SAMPLE_RATE_HZ * 1000.0,
            "maximum_isr_cycles": isr_cycles,
            "isr_budget_cycles": isr_budget,
            "isr_budget_utilization_pct": isr_cycles / isr_budget * 100.0,
            "isr_budget_margin_pct": (isr_budget - isr_cycles) / isr_budget * 100.0,
            "maximum_reconstructed_phase_current_a": maximum_current,
            "minimum_synchronous_bus_voltage_v": minimum_bus_voltage,
            "maximum_synchronous_bus_voltage_v": maximum_bus_voltage,
        },
        "fit": {
            "alignment_definition": "voltage at row k predicts current at row k+1",
            "applied_voltage_fixed_rs": applied_fit,
            "command_voltage_fixed_rs": command_fit,
            "alignment_sensitivity": alignment,
            "pulse_pair_fixed_rs": pair_fits,
            "pulse_pair_count": len(pulse_pairs),
            "pulse_samples_per_pair": samples_per_pair,
            "pulse_pair_mean_h": pair_mean,
            "pulse_pair_range_over_mean_pct": pair_range_over_mean * 100.0,
            "candidate_residuals": candidate_residuals,
            "lcr_phase_inductance_h": LCR_PHASE_L_H,
            "dynamic_vs_lcr_difference_pct": lcr_difference * 100.0,
        },
        "decision": {
            "hardware_execution": "pass" if bounded_run_pass else "fail",
            "parameter_screen": "reject-update",
            "parameter_approval": "not-granted",
            "reasons": [
                "only one separately authorised powered session exists for this exact sequence; the experiment requires at least three",
                *(
                    ["six within-run pulse-pair fits exceed the 10 percent range-over-mean screen"]
                    if pair_range_over_mean > 0.10
                    else []
                ),
                *(
                    ["the dynamic estimate differs materially from the LCR-derived phase inductance"]
                    if abs(lcr_difference) > 0.20
                    else []
                ),
                "current and inverter voltage scales are nominal rather than independently calibrated",
            ],
            "next_gate": "review scaling and fit assumptions before requesting any separately authorised repeat; do not update production parameters",
        },
        "limitations": [
            "single powered run at one 0.2 A bias point",
            "nominal 626.535 counts/A and nominal bus-divider conversion",
            "550 ns dead-time model with zero device-drop term",
            "fixed measured Rs; no joint temperature-aware Rs/L fit",
            "no independent current probe, oscilloscope, encoder or torque truth",
        ],
    }
    return result, converted


def write_csv(path: Path, samples: list[ConvertedSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(samples[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in samples)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--csv-output", type=Path)
    args = parser.parse_args()
    result, converted = analyze(args.input.read_text(encoding="utf-8"))
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(encoded, encoding="utf-8")
    if args.csv_output is not None:
        write_csv(args.csv_output, converted)
    print(encoded, end="")
    return 0 if result["decision"]["hardware_execution"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
