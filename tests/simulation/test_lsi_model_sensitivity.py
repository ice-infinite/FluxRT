from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT_ROOT / "simulation" / "audit_lsi_model_sensitivity.py"
EVIDENCE = (
    PROJECT_ROOT
    / "profiles"
    / "identification"
    / "evidence"
    / "lsi-20260927"
    / "s5_0_run_0p2a_01.log"
)
LOCKED_6X6_EVIDENCE = (
    PROJECT_ROOT
    / "profiles"
    / "identification"
    / "evidence"
    / "lsi-20260927"
    / "s5_4_locked_6x6_run_01.log"
)
SPEC = importlib.util.spec_from_file_location("lsi_model_audit", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = audit
SPEC.loader.exec_module(audit)


class LsiModelSensitivityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = audit.run_audit(EVIDENCE.read_text(encoding="utf-8"))

    def test_st_reference_scale_matches_project_nominal_scale(self) -> None:
        scale = self.result["scale_cross_check"]
        self.assertLess(abs(scale["difference_pct"]), 0.1)
        self.assertLess(abs(scale["bus_difference_pct"]), 0.1)
        self.assertTrue(self.result["decision"]["reference_constants_match"])

    def test_reference_dead_time_and_measured_resistance_are_supported(self) -> None:
        self.assertIn(self.result["best_tested_dead_time_ns"], (525.0, 550.0))
        joint = self.result["joint_rs_l_fit"]
        self.assertLess(abs(joint["stator_resistance_ohm"] - 4.9666667), 0.3)
        self.assertGreater(joint["stator_inductance_h"], 0.0015)

    def test_audit_does_not_authorize_repeat_or_parameter_update(self) -> None:
        decision = self.result["decision"]
        self.assertFalse(decision["conversion_constants_explain_l_mismatch"])
        self.assertEqual(decision["repeat_hardware"], "not-authorized")


class LsiLocked6x6ModelSensitivityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = audit.run_audit(LOCKED_6X6_EVIDENCE.read_text(encoding="utf-8"))

    def test_locked_run_noise_and_pair_repeatability(self) -> None:
        noise = self.result["measured_noise"]
        self.assertLess(noise["bias_tail_u_detrended_std_counts"], 2.0)
        self.assertLess(noise["cooldown_u_std_counts"], 2.0)
        repeatability = self.result["within_run_repeatability"]
        self.assertEqual(repeatability["pulse_pair_count"], 6)
        self.assertTrue(repeatability["screen_10pct_pass"])

    def test_constant_loss_does_not_reconcile_lcr_inductance(self) -> None:
        fit = self.result["affine_rl_with_constant_voltage_loss_fit"]
        self.assertGreater(fit["stator_inductance_h"], 0.0015)
        switched = self.result["ideal_center_aligned_pwm_fit"]
        self.assertLess(abs(switched["inductance_h"] - 0.002026), 0.0001)
        self.assertFalse(
            self.result["decision"]["ideal_subperiod_pwm_explains_l_mismatch"]
        )
        self.assertEqual(self.result["decision"]["repeat_hardware"], "not-authorized")


if __name__ == "__main__":
    unittest.main()
