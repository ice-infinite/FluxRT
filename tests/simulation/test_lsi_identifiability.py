from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import random
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT_ROOT / "simulation" / "analyze_lsi_identifiability.py"
SPEC = importlib.util.spec_from_file_location("lsi", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
lsi = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(lsi)


class LsiIdentifiabilityTests(unittest.TestCase):
    def test_noise_free_fit_recovers_grid_value(self) -> None:
        measured, applied, _commanded = lsi.simulate_trace(
            0.001058, 0.4, random.Random(1), noise=False
        )
        estimate = lsi.fit_inductance(measured, applied)
        self.assertLessEqual(abs(estimate - 0.001058), 2.1e-6)

    def test_bounded_matrix_distinguishes_all_candidates(self) -> None:
        result = lsi.run_matrix(trials=5, seed=431)
        self.assertEqual(result["decision"]["host_identifiability_gate"], "pass")
        self.assertLess(result["maximum_quantised_current_a"], 0.8)
        self.assertTrue(
            all(math.isfinite(row["mean_estimate_h"]) for row in result["rows"])
        )

    def test_command_voltage_only_exposes_model_bias(self) -> None:
        result = lsi.run_matrix(trials=5, seed=431)
        self.assertTrue(
            any(abs(row["command_voltage_only_error_pct"]) > 1.0 for row in result["rows"])
        )

    def test_measured_noise_downgrades_the_old_host_gate(self) -> None:
        result = lsi.run_matrix(trials=20, seed=431, noise_std_counts=1.76)
        self.assertEqual(result["decision"]["host_identifiability_gate"], "fail")
        self.assertLess(min(row["classification_rate"] for row in result["rows"]), 1.0)

    def test_six_tick_redesign_restores_measured_noise_gate(self) -> None:
        result = lsi.run_matrix(
            trials=20,
            seed=431,
            noise_std_counts=1.76,
            pulse_ticks_per_polarity=6,
            pulse_pair_count=6,
        )
        self.assertEqual(result["decision"]["host_identifiability_gate"], "pass")
        self.assertEqual(result["model"]["samples_per_trial"], 72)


if __name__ == "__main__":
    unittest.main()
