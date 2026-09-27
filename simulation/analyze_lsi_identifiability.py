#!/usr/bin/env python3
"""Screen whether 12 kHz phase-current samples can distinguish Ls candidates.

This is a host-only observability model for EXP-B3.  It never opens a serial
port and never drives hardware.  The RL plant is integrated exactly over each
sample and the current is quantised with the STM32G431/IHM16M1 nominal scale.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import statistics


SAMPLE_RATE_HZ = 12_000.0
DT_S = 1.0 / SAMPLE_RATE_HZ
RS_OHM = 4.9666667
BUS_V = 12.3
DEAD_TIME_S = 550.0e-9
CURRENT_COUNTS_PER_AMP = 4095.0 * 0.33 * 1.53 / 3.3
CURRENT_NOISE_STD_COUNTS = 0.5
DELTA_V = 0.4
PULSE_PATTERN_V = ([DELTA_V] * 4 + [-DELTA_V] * 4) * 6
TRUTH_LS_H = (0.000893, 0.001058, 0.0011133333)
BIAS_CURRENT_A = (0.2, 0.4, 0.6)
GRID_LS_H = tuple(value * 1.0e-6 for value in range(700, 1301))


def quantise_current(
    current_a: float,
    rng: random.Random,
    noise: bool = True,
    noise_std_counts: float = CURRENT_NOISE_STD_COUNTS,
) -> float:
    noise_a = rng.gauss(0.0, noise_std_counts / CURRENT_COUNTS_PER_AMP) if noise else 0.0
    return round((current_a + noise_a) * CURRENT_COUNTS_PER_AMP) / CURRENT_COUNTS_PER_AMP


def simulate_trace(
    ls_h: float,
    bias_current_a: float,
    rng: random.Random,
    noise: bool = True,
    noise_std_counts: float = CURRENT_NOISE_STD_COUNTS,
    pulse_pattern_v: tuple[float, ...] | list[float] = PULSE_PATTERN_V,
) -> tuple[list[float], list[float], list[float]]:
    """Return quantised current, reconstructed voltage and command voltage."""
    dead_time_loss_v = 2.0 * DEAD_TIME_S * SAMPLE_RATE_HZ * BUS_V
    current_a = bias_current_a
    measured = [quantise_current(current_a, rng, noise, noise_std_counts)]
    applied_volts: list[float] = []
    command_volts: list[float] = []
    decay = math.exp(-RS_OHM * DT_S / ls_h)
    for perturbation_v in pulse_pattern_v:
        # The command includes the nominal positive-current dead-time loss so
        # the reconstructed applied voltage stays centred at Rs * I_bias.
        command_v = RS_OHM * bias_current_a + dead_time_loss_v + perturbation_v
        applied_v = command_v - dead_time_loss_v
        equilibrium_a = applied_v / RS_OHM
        current_a = equilibrium_a + (current_a - equilibrium_a) * decay
        command_volts.append(command_v)
        applied_volts.append(applied_v)
        measured.append(quantise_current(current_a, rng, noise, noise_std_counts))
    return measured, applied_volts, command_volts


def fit_inductance(measured_a: list[float], applied_v: list[float]) -> float:
    best_error = math.inf
    best_ls_h = 0.0
    for ls_h in GRID_LS_H:
        predicted_a = measured_a[0]
        decay = math.exp(-RS_OHM * DT_S / ls_h)
        squared_error = 0.0
        for voltage_v, observed_a in zip(applied_v, measured_a[1:]):
            equilibrium_a = voltage_v / RS_OHM
            predicted_a = equilibrium_a + (predicted_a - equilibrium_a) * decay
            squared_error += (predicted_a - observed_a) ** 2
        if squared_error < best_error:
            best_error = squared_error
            best_ls_h = ls_h
    return best_ls_h


def run_matrix(
    trials: int = 30,
    seed: int = 431,
    noise_std_counts: float = CURRENT_NOISE_STD_COUNTS,
    pulse_ticks_per_polarity: int = 4,
    pulse_pair_count: int = 6,
) -> dict:
    rng = random.Random(seed)
    pulse_pattern_v = (
        [DELTA_V] * pulse_ticks_per_polarity
        + [-DELTA_V] * pulse_ticks_per_polarity
    ) * pulse_pair_count
    rows = []
    maximum_current_a = 0.0
    for bias_a in BIAS_CURRENT_A:
        for truth_h in TRUTH_LS_H:
            estimates = []
            classifications = 0
            command_voltage_estimates = []
            for _ in range(trials):
                measured, applied, commanded = simulate_trace(
                    truth_h,
                    bias_a,
                    rng,
                    noise_std_counts=noise_std_counts,
                    pulse_pattern_v=pulse_pattern_v,
                )
                maximum_current_a = max(maximum_current_a, *(abs(value) for value in measured))
                estimate_h = fit_inductance(measured, applied)
                estimates.append(estimate_h)
                nearest = min(TRUTH_LS_H, key=lambda value: abs(value - estimate_h))
                classifications += int(nearest == truth_h)
                command_voltage_estimates.append(fit_inductance(measured, commanded))
            mean_h = statistics.mean(estimates)
            rows.append(
                {
                    "bias_current_a": bias_a,
                    "truth_h": truth_h,
                    "mean_estimate_h": mean_h,
                    "mean_error_pct": (mean_h / truth_h - 1.0) * 100.0,
                    "range_over_mean_pct": (max(estimates) - min(estimates)) / mean_h * 100.0,
                    "classification_rate": classifications / trials,
                    "command_voltage_only_mean_h": statistics.mean(command_voltage_estimates),
                    "command_voltage_only_error_pct":
                        (statistics.mean(command_voltage_estimates) / truth_h - 1.0) * 100.0,
                }
            )
    all_distinguished = all(row["classification_rate"] == 1.0 for row in rows)
    repeatable = all(row["range_over_mean_pct"] < 10.0 for row in rows)
    return {
        "format_version": 1,
        "analysis_kind": "lsi-host-identifiability",
        "model": {
            "sample_rate_hz": SAMPLE_RATE_HZ,
            "rs_ohm": RS_OHM,
            "bus_voltage_v": BUS_V,
            "dead_time_s": DEAD_TIME_S,
            "current_counts_per_amp": CURRENT_COUNTS_PER_AMP,
            "current_noise_std_counts": noise_std_counts,
            "perturbation_v": DELTA_V,
            "pulse_ticks_per_polarity": pulse_ticks_per_polarity,
            "pulse_pair_count": pulse_pair_count,
            "samples_per_trial": len(pulse_pattern_v),
            "trials_per_point": trials,
            "seed": seed,
        },
        "rows": rows,
        "maximum_quantised_current_a": maximum_current_a,
        "decision": {
            "all_three_candidates_distinguished_at_all_bias_points": all_distinguished,
            "all_range_over_mean_below_10_pct": repeatable,
            "host_identifiability_gate": "pass" if all_distinguished and repeatable else "fail",
            "hardware_permission": "not-granted",
            "next_gate": (
                "review the selected sequence through Host, target-build, and unpowered-board "
                "safety gates before any powered repeat"
            ),
        },
        "limitations": [
            "ideal single-axis linear RL plant; magnetic saturation is represented only by choosing a different truth L",
            "nominal ADC scale and Gaussian plus quantisation noise; no measured current-channel noise spectrum",
            "applied-voltage fit assumes the 550 ns dead-time loss is reconstructed; command-voltage-only bias is reported separately",
            "passing this host observability screen is not permission to inject the physical motor",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=431)
    parser.add_argument("--noise-std-counts", type=float, default=CURRENT_NOISE_STD_COUNTS)
    parser.add_argument("--pulse-ticks-per-polarity", type=int, default=4)
    parser.add_argument("--pulse-pair-count", type=int, default=6)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.trials < 1 or args.trials > 1000:
        parser.error("--trials must be in 1..1000")
    if not math.isfinite(args.noise_std_counts) or args.noise_std_counts < 0.0:
        parser.error("--noise-std-counts must be finite and non-negative")
    if args.pulse_ticks_per_polarity < 1 or args.pulse_ticks_per_polarity > 32:
        parser.error("--pulse-ticks-per-polarity must be in 1..32")
    if args.pulse_pair_count < 1 or args.pulse_pair_count > 16:
        parser.error("--pulse-pair-count must be in 1..16")
    result = run_matrix(
        args.trials,
        args.seed,
        args.noise_std_counts,
        args.pulse_ticks_per_polarity,
        args.pulse_pair_count,
    )
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if result["decision"]["host_identifiability_gate"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
