from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT_ROOT / "simulation" / "analyze_lsi_hardware_run.py"
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
SPEC = importlib.util.spec_from_file_location("lsi_hardware", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
lsi_hardware = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = lsi_hardware
SPEC.loader.exec_module(lsi_hardware)


class LsiHardwareRunTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result, cls.samples = lsi_hardware.analyze(EVIDENCE.read_text(encoding="utf-8"))

    def test_raw_contract_and_safety_result(self) -> None:
        self.assertEqual(len(self.samples), 192)
        self.assertTrue(self.result["raw_contract"]["contract_ok"])
        self.assertTrue(self.result["safety"]["bounded_run_pass"])
        self.assertEqual(self.result["safety"]["start_consumed"], 1)
        self.assertEqual(self.result["safety"]["deadline_miss_count"], 0)

    def test_segments_match_frozen_experiment(self) -> None:
        contract = self.result["raw_contract"]
        self.assertEqual(contract["bias_samples"], 120)
        self.assertEqual(contract["positive_pulse_samples"], 24)
        self.assertEqual(contract["negative_pulse_samples"], 24)
        self.assertEqual(contract["output_off_samples"], 24)

    def test_fit_is_screening_only(self) -> None:
        fit = self.result["fit"]["applied_voltage_fixed_rs"]
        self.assertGreater(fit["inductance_h"], 0.0015)
        self.assertLess(fit["inductance_h"], 0.0020)
        self.assertEqual(self.result["decision"]["parameter_screen"], "reject-update")
        self.assertEqual(self.result["decision"]["parameter_approval"], "not-granted")


class LsiLocked6x6HardwareRunTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result, cls.samples = lsi_hardware.analyze(
            LOCKED_6X6_EVIDENCE.read_text(encoding="utf-8")
        )

    def test_raw_contract_and_bounded_run(self) -> None:
        self.assertEqual(len(self.samples), 120)
        self.assertTrue(self.result["raw_contract"]["contract_ok"])
        self.assertTrue(self.result["safety"]["bounded_run_pass"])
        self.assertEqual(self.result["safety"]["session_control_ticks"], 377)
        self.assertEqual(self.result["safety"]["deadline_miss_count"], 0)

    def test_segments_match_6x6_sequence(self) -> None:
        contract = self.result["raw_contract"]
        self.assertEqual(contract["bias_samples"], 24)
        self.assertEqual(contract["positive_pulse_samples"], 36)
        self.assertEqual(contract["negative_pulse_samples"], 36)
        self.assertEqual(contract["output_off_samples"], 24)
        fit = self.result["fit"]
        self.assertEqual(fit["pulse_pair_count"], 6)
        self.assertEqual(fit["pulse_samples_per_pair"], [12] * 6)

    def test_one_run_cannot_approve_parameters(self) -> None:
        self.assertEqual(self.result["decision"]["hardware_execution"], "pass")
        self.assertEqual(self.result["decision"]["parameter_screen"], "reject-update")
        self.assertEqual(self.result["decision"]["parameter_approval"], "not-granted")


if __name__ == "__main__":
    unittest.main()
