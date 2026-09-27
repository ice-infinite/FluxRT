from __future__ import annotations

import importlib.util
from pathlib import Path
import json
import sys
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT_ROOT / "simulation" / "analyze_lsi_truth_envelope.py"
EVIDENCE_DIR = (
    PROJECT_ROOT / "profiles" / "identification" / "evidence" / "lsi-20260927"
)
SPEC = importlib.util.spec_from_file_location("lsi_truth_envelope", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = audit
SPEC.loader.exec_module(audit)


class LsiTruthEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = audit.run_analysis(
            EVIDENCE_DIR / "s5_4_locked_6x6_run_01.log",
            EVIDENCE_DIR / "s5_5a_lcr_locked_angle_a_repeat.json",
            EVIDENCE_DIR / "s5_5b_lcr_manual_angle_sweep.json",
        )

    def test_lcr_envelope_is_diagnostic_and_bounded(self) -> None:
        envelope = self.result["diagnostic_inductance_envelope"]
        self.assertAlmostEqual(envelope["lower_h"], 0.00098)
        self.assertAlmostEqual(envelope["upper_h"], 0.00141033333335)
        self.assertIn(0.001058, envelope["simulation_points_h"])

    def test_exact_discrete_fit_matches_existing_nominal_fit(self) -> None:
        nominal = self.result["scale_discrimination"]["nominal_exact_discrete_fit"]
        self.assertLess(abs(nominal["inductance_h"] - 0.00195), 0.00002)
        self.assertGreater(nominal["transition_count"], 60)

    def test_small_scale_error_does_not_silently_approve_parameter(self) -> None:
        scale = self.result["scale_discrimination"]
        self.assertFalse(scale["decision"]["plausible_5pct_scale_error_explains_gap"])
        self.assertFalse(
            scale["decision"]["tested_0p1_to_3_relative_scale_reaches_envelope"]
        )
        self.assertFalse(
            scale["decision"]["current_and_voltage_scales_are_separately_identifiable_from_this_trace"]
        )
        self.assertEqual(self.result["decision"]["parameter_update"], "reject")
        self.assertEqual(self.result["decision"]["powered_repeat"], "not-authorized")
        json.dumps(self.result, allow_nan=False)

    def test_bringup_summary_parser_is_strict(self) -> None:
        summary = audit.parse_bringup_summary(
            "FOC_BRINGUP_SIM_PASS true_rpm=582.10 observer_rpm=580.00 "
            "peak_phase_current_a=0.800 reliable_samples=123 hold_speed_rmse_rpm=3.00 "
            "hold_angle_rmse_rad=0.0100 transient_speed_rmse_rpm=4.000 "
            "transient_observer_rmse_rpm=5.000 steady_speed_mean_rpm=582.000 "
            "steady_speed_std_rpm=0.1000 steady_observer_rmse_rpm=6.000 "
            "steady_iq_rmse_a=0.001000 steady_id_rmse_a=0.002000 "
            "final_state=7 samples=100 pwm_ticks=200 control_ticks=100 applied_updates=100"
        )
        self.assertEqual(summary["final_state"], 7)
        self.assertAlmostEqual(summary["steady_iq_rmse_a"], 0.001)

    def test_truth_plan_requires_independent_dynamic_channels(self) -> None:
        plan = self.result["truth_measurement_plan"]
        names = {row["name"] for row in plan["required_observables"]}
        self.assertIn("independent_phase_current", names)
        self.assertIn("differential_phase_voltage", names)
        self.assertIn("sample_and_update_timing", names)
        self.assertTrue(plan["acceptance"]["parameter_candidate_only_after_independent_truth"])


if __name__ == "__main__":
    unittest.main()
