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
TOOL = ROOT / "tools/foc_h3_alignment_offset.py"
SPEC = importlib.util.spec_from_file_location("foc_h3_alignment_offset", TOOL)
assert SPEC is not None and SPEC.loader is not None
offset = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = offset
SPEC.loader.exec_module(offset)


class H3AlignmentOffsetTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.aligned = self.root / "aligned.csv"
        self.manifest_path = self.root / "manifest.json"
        self.expected_additive = 0.65
        self.write_aligned()
        self.write_manifest()

    def write_aligned(self, outlier: bool = False, state: int = 3) -> None:
        fields = [
            "sequence",
            "reference_control_tick_unwrapped",
            "truth_mechanical_angle_rad",
            "controller_state",
            "forced_electrical_angle_rad",
            "estimated_electrical_angle_rad",
            "reliable",
        ]
        with self.aligned.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(8):
                mechanical = 0.1 + index * 0.0005
                forced = (7.0 * mechanical + self.expected_additive) % (2.0 * math.pi)
                if outlier and index == 7:
                    mechanical += 0.2
                writer.writerow({
                    "sequence": index,
                    "reference_control_tick_unwrapped": 1000 + index * 20,
                    "truth_mechanical_angle_rad": mechanical,
                    "controller_state": state,
                    "forced_electrical_angle_rad": forced,
                    "estimated_electrical_angle_rad": 5.9 - index,
                    "reliable": index & 1,
                })

    def manifest(self) -> dict:
        return {
            "contract": "fluxrt-h3-alignment-offset",
            "version": 1,
            "motor_id": "M0-test",
            "pole_pairs": 7,
            "mechanical_direction": 1,
            "alignment_state": 3,
            "requirements": {
                "minimum_alignment_samples": 4,
                "settled_tail_sample_count": 5,
                "maximum_candidate_deviation_rad": 0.05,
                "maximum_forced_angle_deviation_rad": 0.05,
            },
            "aligned_csv_path": self.aligned.name,
            "aligned_csv_sha256": hashlib.sha256(self.aligned.read_bytes()).hexdigest().upper(),
            "parameter_approval": "not-granted",
            "target_capability_approval": "not-granted",
        }

    def write_manifest(self) -> None:
        self.manifest_path.write_text(json.dumps(self.manifest()), encoding="utf-8")

    def test_candidate_uses_forced_angle_and_truth_not_observer(self) -> None:
        result = offset.analyse_manifest(self.manifest_path, self.root)
        self.assertEqual(result["status"], "alignment-offset-candidate-not-approved")
        self.assertAlmostEqual(result["additive_time_map_offset_rad"], self.expected_additive)
        self.assertAlmostEqual(
            result["as5600_subtractive_offset_rad"],
            (-self.expected_additive) % (2.0 * math.pi),
        )
        self.assertFalse(result["observer_angle_used"])
        self.assertEqual(result["parameter_approval"], "not-granted")

    def test_unstable_candidate_and_missing_alignment_fail_closed(self) -> None:
        self.write_aligned(outlier=True)
        self.write_manifest()
        with self.assertRaisesRegex(offset.AlignmentOffsetError, "candidate is not stable"):
            offset.analyse_manifest(self.manifest_path, self.root)

        self.write_aligned(state=4)
        self.write_manifest()
        with self.assertRaisesRegex(offset.AlignmentOffsetError, "not enough alignment"):
            offset.analyse_manifest(self.manifest_path, self.root)

    def test_tamper_and_approval_claim_fail_closed(self) -> None:
        self.aligned.write_text("tampered\n", encoding="utf-8")
        with self.assertRaisesRegex(offset.AlignmentOffsetError, "SHA-256 mismatch"):
            offset.analyse_manifest(self.manifest_path, self.root)

        self.write_aligned()
        value = self.manifest()
        value["parameter_approval"] = "approved"
        self.manifest_path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(offset.AlignmentOffsetError, "must not claim approval"):
            offset.analyse_manifest(self.manifest_path, self.root)

    def test_schema_freezes_non_approval_contract(self) -> None:
        schema = json.loads(
            (ROOT / "profiles/schema/fluxrt-h3-alignment-offset-v1.schema.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(schema["properties"]["contract"]["const"], "fluxrt-h3-alignment-offset")
        self.assertEqual(schema["properties"]["parameter_approval"]["const"], "not-granted")


if __name__ == "__main__":
    unittest.main()
