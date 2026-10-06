from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/foc_h3_time_map.py"
SPEC = importlib.util.spec_from_file_location("foc_h3_time_map", TOOL)
assert SPEC is not None and SPEC.loader is not None
time_map = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = time_map
SPEC.loader.exec_module(time_map)


class H3TimeMapTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.anchors = self.root / "anchors.csv"
        self.truth = self.root / "truth.csv"
        self.queries = self.root / "queries.csv"
        self.manifest_path = self.root / "manifest.json"
        self.reference_start = 100
        self.write_anchors([0, 100_000, 200_000], [100, 1300, 2500])
        self.write_truth()
        self.write_queries([50_000, 100_000, 150_000])
        self.write_manifest()

    @staticmethod
    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest().upper()

    @staticmethod
    def angle_at(truth_tick: float) -> float:
        return (6.2 + truth_tick * 1.0e-5) % (2.0 * math.pi)

    def write_csv(self, path: Path, fields: list[str], rows: list[dict]) -> None:
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def write_anchors(self, truth_ticks: list[int], reference_ticks: list[int]) -> None:
        self.write_csv(
            self.anchors,
            ["edge_sequence", "reference_control_tick", "truth_tick"],
            [
                {
                    "edge_sequence": index + 1,
                    "reference_control_tick": reference,
                    "truth_tick": truth,
                }
                for index, (truth, reference) in enumerate(zip(truth_ticks, reference_ticks))
            ],
        )

    def write_truth(self, sequence_gap: bool = False, invalid: bool = False) -> None:
        rows = []
        for index, tick in enumerate(range(0, 200_001, 10_000)):
            sequence = index + 1 + (1 if sequence_gap and index >= 10 else 0)
            rows.append(
                {
                    "sequence": sequence,
                    "truth_tick": tick,
                    "mechanical_angle_rad": self.angle_at(tick),
                    "valid": 0 if invalid and index == 10 else 1,
                    "read_failures": 0,
                }
            )
        self.write_csv(
            self.truth,
            ["sequence", "truth_tick", "mechanical_angle_rad", "valid", "read_failures"],
            rows,
        )

    def write_queries(
        self,
        truth_ticks: list[int],
        reference_start: int | None = None,
        controller_context: bool = False,
    ) -> None:
        start = self.reference_start if reference_start is None else reference_start
        rows = []
        for index, tick in enumerate(truth_ticks):
            reference = (start + round(0.012 * tick)) & 0xFFFFFFFF
            row = {
                    "sequence": index + 1,
                    "reference_control_tick": reference,
                    "estimated_electrical_angle_rad": self.angle_at(tick),
                    "reliable": 1,
                }
            if controller_context:
                row.update({
                    "controller_state": 3,
                    "control_electrical_angle_rad": 1.25,
                    "forced_electrical_angle_rad": 1.25,
                })
            rows.append(row)
        fields = [
            "sequence",
            "reference_control_tick",
            "estimated_electrical_angle_rad",
            "reliable",
        ]
        if controller_context:
            fields[2:2] = [
                "controller_state",
                "control_electrical_angle_rad",
                "forced_electrical_angle_rad",
            ]
        self.write_csv(
            self.queries,
            fields,
            rows,
        )

    def manifest(self) -> dict:
        return {
            "contract": "fluxrt-h3-time-map",
            "version": 1,
            "motor_id": "M0-test",
            "reference_firmware_sha256": "A" * 64,
            "truth_firmware_sha256": "B" * 64,
            "pole_pairs": 1,
            "mechanical_direction": 1,
            "electrical_offset_rad": 0.0,
            "reference_tick_hz": 12_000,
            "truth_tick_hz": 1_000_000,
            "requirements": {
                "minimum_anchor_count": 3,
                "maximum_anchor_residual_ticks": 0.25,
                "maximum_clock_drift_ppm": 100.0,
                "maximum_truth_gap_ticks": 10_000,
            },
            "anchors_csv_path": self.anchors.name,
            "anchors_csv_sha256": self.digest(self.anchors),
            "truth_csv_path": self.truth.name,
            "truth_csv_sha256": self.digest(self.truth),
            "queries_csv_path": self.queries.name,
            "queries_csv_sha256": self.digest(self.queries),
            "parameter_approval": "not-granted",
            "target_capability_approval": "not-granted",
        }

    def write_manifest(self) -> None:
        self.manifest_path.write_text(json.dumps(self.manifest()), encoding="utf-8")

    def refresh_manifest(self) -> None:
        self.write_manifest()

    def test_nominal_mapping_interpolates_through_angle_wrap(self) -> None:
        result = time_map.analyse_manifest(self.manifest_path, self.root)
        self.assertEqual(result["status"], "aligned-offline")
        self.assertEqual(result["aligned_sample_count"], 3)
        self.assertLess(result["reliable_maximum_angle_error_rad"], 1.0e-9)
        self.assertAlmostEqual(result["mapping"]["clock_drift_ppm"], 0.0, places=6)
        self.assertEqual(result["parameter_approval"], "not-granted")
        self.assertEqual(result["target_capability_approval"], "not-granted")

    def test_clock_drift_and_anchor_residual_fail_closed(self) -> None:
        self.write_anchors([0, 100_000, 200_000], [100, 1400, 2700])
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "clock drift"):
            time_map.analyse_manifest(self.manifest_path, self.root)

        self.write_anchors([0, 100_000, 200_000], [100, 1310, 2500])
        value = self.manifest()
        value["requirements"]["maximum_clock_drift_ppm"] = 10_000.0
        self.manifest_path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(time_map.TimeMapError, "anchor residual"):
            time_map.analyse_manifest(self.manifest_path, self.root)

    def test_anchor_sequence_gap_is_rejected(self) -> None:
        rows = [
            {"edge_sequence": 1, "reference_control_tick": 100, "truth_tick": 0},
            {"edge_sequence": 3, "reference_control_tick": 1300, "truth_tick": 100_000},
            {"edge_sequence": 4, "reference_control_tick": 2500, "truth_tick": 200_000},
        ]
        self.write_csv(
            self.anchors,
            ["edge_sequence", "reference_control_tick", "truth_tick"],
            rows,
        )
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "anchor edge sequence"):
            time_map.analyse_manifest(self.manifest_path, self.root)

    def test_truth_drop_invalid_sample_and_large_gap_are_rejected(self) -> None:
        self.write_truth(sequence_gap=True)
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "missing-sample count"):
            time_map.analyse_manifest(self.manifest_path, self.root)

        value = self.manifest()
        value["requirements"]["maximum_truth_missing_samples"] = 1
        self.manifest_path.write_text(json.dumps(value), encoding="utf-8")
        accepted_gap = time_map.analyse_manifest(self.manifest_path, self.root)
        self.assertEqual(accepted_gap["truth_missing_sample_count"], 1)
        self.assertEqual(accepted_gap["maximum_truth_missing_samples"], 1)

        self.write_truth(invalid=True)
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "invalid AS5600"):
            time_map.analyse_manifest(self.manifest_path, self.root)

        self.write_truth()
        value = self.manifest()
        value["requirements"]["maximum_truth_gap_ticks"] = 9_999
        self.manifest_path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(time_map.TimeMapError, "sample gap"):
            time_map.analyse_manifest(self.manifest_path, self.root)

    def test_query_extrapolation_is_rejected(self) -> None:
        self.write_queries([250_000])
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "anchor span"):
            time_map.analyse_manifest(self.manifest_path, self.root)

        self.write_anchors([50_000, 100_000, 150_000], [700, 1300, 1900])
        self.write_queries([25_000])
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "anchor span"):
            time_map.analyse_manifest(self.manifest_path, self.root)

    def test_manifest_may_bound_but_never_silently_drop_outside_evidence(self) -> None:
        self.write_anchors(
            [0, 100_000, 200_000, 300_000],
            [100, 1300, 2500, 3700],
        )
        self.write_queries([50_000, 250_000])
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "truth window"):
            time_map.analyse_manifest(self.manifest_path, self.root)

        value = self.manifest()
        value["requirements"].update({
            "maximum_anchors_outside_truth_window": 1,
            "maximum_queries_outside_anchor_span": 1,
        })
        self.manifest_path.write_text(json.dumps(value), encoding="utf-8")
        result = time_map.analyse_manifest(self.manifest_path, self.root)
        self.assertEqual(result["input_anchor_count"], 4)
        self.assertEqual(result["used_anchor_count"], 3)
        self.assertEqual(result["excluded_anchor_sequences"], [4])
        self.assertEqual(result["input_query_count"], 2)
        self.assertEqual(result["excluded_query_sequences"], [2])
        self.assertEqual(result["aligned_sample_count"], 1)

    def test_reference_u32_wrap_is_unwrapped(self) -> None:
        start = 0xFFFFFF00
        references = [(start + offset) & 0xFFFFFFFF for offset in (0, 1200, 2400)]
        self.reference_start = start
        self.write_anchors([0, 100_000, 200_000], references)
        self.write_queries([50_000, 150_000], reference_start=start)
        self.refresh_manifest()
        result = time_map.analyse_manifest(self.manifest_path, self.root)
        self.assertEqual(result["aligned_sample_count"], 2)
        self.assertLess(result["reliable_maximum_angle_error_rad"], 1.0e-9)

    def test_controller_context_is_propagated_and_partial_context_is_rejected(self) -> None:
        self.write_queries([50_000, 100_000], controller_context=True)
        self.refresh_manifest()
        result = time_map.analyse_manifest(self.manifest_path, self.root)
        self.assertEqual(result["aligned"][0]["controller_state"], 3)
        self.assertAlmostEqual(
            result["aligned"][0]["forced_electrical_angle_rad"], 1.25
        )

        rows = [{
            "sequence": 1,
            "reference_control_tick": 700,
            "controller_state": 3,
            "estimated_electrical_angle_rad": 1.0,
            "reliable": 0,
        }]
        self.write_csv(
            self.queries,
            [
                "sequence",
                "reference_control_tick",
                "controller_state",
                "estimated_electrical_angle_rad",
                "reliable",
            ],
            rows,
        )
        self.refresh_manifest()
        with self.assertRaisesRegex(time_map.TimeMapError, "incomplete controller context"):
            time_map.analyse_manifest(self.manifest_path, self.root)

    def test_tampered_file_and_approval_claim_are_rejected(self) -> None:
        self.queries.write_text("tampered\n", encoding="utf-8")
        with self.assertRaisesRegex(time_map.TimeMapError, "SHA-256 mismatch"):
            time_map.analyse_manifest(self.manifest_path, self.root)

        self.write_queries([50_000])
        value = self.manifest()
        value["parameter_approval"] = "approved"
        self.manifest_path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(time_map.TimeMapError, "must not claim approval"):
            time_map.analyse_manifest(self.manifest_path, self.root)

    def test_schema_freezes_hardware_sync_and_not_granted_fields(self) -> None:
        schema = json.loads(
            (ROOT / "profiles/schema/fluxrt-h3-time-map-v1.schema.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(schema["properties"]["contract"]["const"], "fluxrt-h3-time-map")
        self.assertEqual(schema["properties"]["parameter_approval"]["const"], "not-granted")
        self.assertIn("anchors_csv_sha256", schema["required"])


if __name__ == "__main__":
    unittest.main()
