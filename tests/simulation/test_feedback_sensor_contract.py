"""Static and parser checks for the P4.1B2 dual feedback-sensor contract."""

from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "simulation/scenarios/feedback_sensor_fault_matrix_v1.json"
TRACE_SCHEMA = ROOT / "simulation/contracts/trace-schema-v1.json"
COMPARISON = ROOT / "simulation/contracts/comparison-gates-v1.json"
COMPARER = ROOT / "simulation/compare_feedback_sensor_results.py"


def load_comparer():
    spec = importlib.util.spec_from_file_location("feedback_compare", COMPARER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FeedbackSensorContractTests(unittest.TestCase):
    def test_fault_matrix_is_si_versioned_and_complete(self) -> None:
        scenario = json.loads(SCENARIO.read_text(encoding="utf-8"))
        self.assertEqual(scenario["contract"], "fluxrt-feedback-sensor-scenario")
        self.assertEqual(scenario["version"], 1)
        self.assertEqual(scenario["units"], "SI")
        self.assertEqual(scenario["evaluation_ticks"], 16)
        case_ids = {case["case_id"] for case in scenario["cases"]}
        self.assertEqual(
            case_ids,
            {
                "feature_off",
                "nominal",
                "encoder_drop_fallback",
                "encoder_index_missing",
                "hall_wrong_order_and_primary_drop",
            },
        )
        self.assertEqual(scenario["hall"]["sector_map"], list(range(6)))

    def test_trace_channels_and_numeric_tolerances_are_declared(self) -> None:
        trace = json.loads(TRACE_SCHEMA.read_text(encoding="utf-8"))
        comparison = json.loads(COMPARISON.read_text(encoding="utf-8"))
        channels = {item["name"] for item in trace["channels"]}
        tolerances = {item["channel"] for item in comparison["numeric_tolerances"]}
        self.assertTrue(load_comparer().EXACT_COLUMNS | load_comparer().NUMERIC_COLUMNS <= channels | {"case_id", "truth_position_rad", "truth_velocity_rad_s"})
        self.assertTrue(
            {
                "feedback_encoder_position_rad",
                "feedback_hall_electrical_angle_rad",
                "feedback_selected_position_rad",
                "feedback_selected_velocity_rad_s",
                "feedback_selected_electrical_angle_rad",
                "feedback_sample_age_s",
            }
            <= tolerances
        )

    def test_rust_and_matlab_models_do_not_call_each_other(self) -> None:
        rust = (ROOT / "rust/crates/foc-sim/src/simulation_contract/feedback_sensor.rs").read_text(encoding="utf-8").lower()
        matlab = (ROOT / "simulink/run_feedback_sensor_contract.m").read_text(encoding="utf-8").lower()
        self.assertNotIn("matlab-feedback-trace", rust)
        self.assertNotIn("rust-feedback-trace", matlab)
        self.assertNotIn("cargo", matlab)


if __name__ == "__main__":
    unittest.main()
