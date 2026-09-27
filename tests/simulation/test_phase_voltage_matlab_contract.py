from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PYTHON_ANALYZER = PROJECT_ROOT / "simulation" / "analyze_phase_voltage_fault_scenarios.py"
MATLAB_SCRIPT = PROJECT_ROOT / "simulation" / "matlab" / "run_phase_voltage_fault_scenarios.m"
MATLAB_EVIDENCE = (
    PROJECT_ROOT
    / "profiles"
    / "identification"
    / "evidence"
    / "phase-voltage-20260927"
    / "a24_matlab_fault_scenario_matrix.json"
)

SPEC = importlib.util.spec_from_file_location("phase_voltage_fault_scenarios", PYTHON_ANALYZER)
assert SPEC is not None and SPEC.loader is not None
analysis = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analysis
SPEC.loader.exec_module(analysis)


class PhaseVoltageMatlabContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.python_result = analysis.run_analysis()
        cls.matlab_result = json.loads(MATLAB_EVIDENCE.read_text(encoding="utf-8"))
        cls.python_rows = {
            row["name"]: row for row in cls.python_result["scenarios"]
        }
        cls.matlab_rows = {
            row["name"]: row for row in cls.matlab_result["scenarios"]
        }

    def test_matlab_run_is_recorded_as_host_only_and_matching(self) -> None:
        self.assertEqual(
            self.matlab_result["schema"],
            "fluxrt.phase-voltage-fault-scenarios.matlab",
        )
        self.assertEqual(self.matlab_result["scenario_count"], 13)
        self.assertEqual(self.matlab_result["mismatch_count"], 0)
        self.assertFalse(self.matlab_result["thresholds_are_production_approved"])
        self.assertIn("PC-only", self.matlab_result["scope"])
        self.assertTrue(self.matlab_result["matlab_version"])

    def test_matlab_and_python_states_and_selections_match(self) -> None:
        self.assertEqual(set(self.matlab_rows), set(self.python_rows))
        for name, matlab_row in self.matlab_rows.items():
            with self.subTest(name=name):
                python_row = self.python_rows[name]
                self.assertEqual(
                    matlab_row["quality_state"], python_row["quality"]["state"]
                )
                self.assertEqual(
                    matlab_row["command_model_selection"],
                    python_row["selection"]["CommandModel"]["selected_source"],
                )
                self.assertEqual(
                    matlab_row["measured_selection"],
                    python_row["selection"]["Measured"]["selected_source"],
                )
                self.assertEqual(
                    matlab_row["hybrid_selection"],
                    python_row["selection"]["Hybrid"]["selected_source"],
                )

    def test_matlab_source_contains_independent_quality_formulas(self) -> None:
        source = MATLAB_SCRIPT.read_text(encoding="utf-8")
        for token in (
            "voltsToRaw",
            "rawToVolts",
            "measuredLine",
            "expectedLine",
            "openMeasuredSpanMaxV",
            "rawLowRailMax",
            "rawHighRailMin",
        ):
            self.assertIn(token, source)
        lowered = source.lower()
        self.assertNotIn("serialport(", lowered)
        self.assertNotIn("foc_start", lowered)


if __name__ == "__main__":
    unittest.main()
