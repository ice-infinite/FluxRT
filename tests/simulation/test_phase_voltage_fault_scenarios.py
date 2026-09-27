from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT_ROOT / "simulation" / "analyze_phase_voltage_fault_scenarios.py"
SPEC = importlib.util.spec_from_file_location("phase_voltage_fault_scenarios", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
analysis = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analysis
SPEC.loader.exec_module(analysis)


class PhaseVoltageFaultScenarioTests(unittest.TestCase):
    def setUp(self) -> None:
        self.result = analysis.run_analysis()
        self.rows = {row["name"]: row for row in self.result["scenarios"]}

    def test_required_nonidealities_and_faults_are_covered(self) -> None:
        self.assertTrue(
            {
                "offset",
                "gain",
                "quantization",
                "delay",
                "stale",
                "dropout",
                "low_saturation",
                "high_saturation",
                "open_suspicion",
                "three_phase_inconsistency",
            }.issubset(self.rows)
        )

    def test_valid_nonidealities_remain_measured_candidates(self) -> None:
        for name in ("baseline", "offset", "gain", "quantization", "delay"):
            with self.subTest(name=name):
                row = self.rows[name]
                self.assertEqual(row["quality"]["state"], "valid")
                self.assertEqual(
                    row["selection"]["Measured"]["selected_source"], "Measured"
                )
                self.assertEqual(
                    row["selection"]["Hybrid"]["selected_source"], "Measured"
                )

    def test_each_bad_input_has_the_expected_deterministic_state(self) -> None:
        expected = {
            "stale": "stale",
            "dropout": "dropout",
            "low_saturation": "low_saturation",
            "high_saturation": "high_saturation",
            "open_suspicion": "open_suspicion",
            "three_phase_inconsistency": "three_phase_inconsistency",
            "unconfigured": "unconfigured",
            "uncalibrated": "uncalibrated",
        }
        for name, quality_state in expected.items():
            with self.subTest(name=name):
                self.assertEqual(self.rows[name]["quality"]["state"], quality_state)

    def test_bad_measured_data_fails_closed_and_hybrid_falls_back(self) -> None:
        for name, row in self.rows.items():
            if row["quality"]["state"] == "valid":
                continue
            with self.subTest(name=name):
                self.assertEqual(
                    row["selection"]["CommandModel"]["selected_source"],
                    "CommandModel",
                )
                self.assertEqual(
                    row["selection"]["Measured"]["selected_source"], "Unavailable"
                )
                self.assertEqual(
                    row["selection"]["Hybrid"]["selected_source"], "CommandModel"
                )
                self.assertIn(
                    row["quality"]["state"], row["selection"]["Hybrid"]["reason"]
                )

    def test_result_is_deterministic_and_strict_json(self) -> None:
        first = json.dumps(self.result, allow_nan=False, sort_keys=True)
        second = json.dumps(analysis.run_analysis(), allow_nan=False, sort_keys=True)
        self.assertEqual(first, second)
        self.assertFalse(self.result["thresholds_are_production_approved"])

    def test_cli_writes_the_same_strict_json(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "fault-scenarios.json"
            completed = subprocess.run(
                [sys.executable, str(SCRIPT), "--compact", "--output", str(output)],
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            decoded = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(decoded, self.result)


if __name__ == "__main__":
    unittest.main()
