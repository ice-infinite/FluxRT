from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "simulation/scenarios/lsi_h2_replay_v1.json"
COMPARER = ROOT / "simulation/compare_lsi_h2_replay_results.py"


class LsiH2ReplayContractTests(unittest.TestCase):
    def test_historical_fixture_is_structural_only(self) -> None:
        scenario = json.loads(SCENARIO.read_text(encoding="utf-8"))
        self.assertEqual(scenario["contract"], "fluxrt-lsi-h2-replay")
        self.assertEqual(scenario["version"], 1)
        self.assertEqual(len(scenario["cases"]), 1)
        case = scenario["cases"][0]
        self.assertEqual(case["angle_index_valid"], 0)
        self.assertEqual(case["angle_source"], "unavailable-historical")
        self.assertEqual(case["angle_sample_count"], 0)
        trace = ROOT / case["trace_path"]
        self.assertTrue(trace.is_file())
        with trace.open(newline="", encoding="utf-8-sig") as stream:
            fields = set(csv.DictReader(stream).fieldnames or [])
        self.assertTrue(
            {"control_tick", "current_u_a", "applied_phase_u_v", "flags"} <= fields
        )

    def test_comparer_accepts_identical_independent_tables(self) -> None:
        spec = importlib.util.spec_from_file_location("lsi_h2_compare", COMPARER)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader if spec else None)
        text = COMPARER.read_text(encoding="utf-8")
        self.assertIn("BLOCKED_UNTIL_VALID_INDEX", text)
        self.assertNotIn("subprocess", text)

    def test_matlab_and_rust_do_not_call_each_other(self) -> None:
        rust = (ROOT / "rust/crates/foc-sim/src/lsi_h2_replay_contract.rs").read_text(
            encoding="utf-8"
        ).lower()
        matlab = (ROOT / "simulink/run_lsi_h2_replay_contract.m").read_text(
            encoding="utf-8"
        ).lower()
        self.assertNotIn("matlab-lsi-h2-replay", rust)
        self.assertNotIn("rust-lsi-h2-replay", matlab)
        self.assertNotIn("cargo", matlab)


if __name__ == "__main__":
    unittest.main()
