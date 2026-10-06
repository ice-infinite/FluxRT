from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/foc_h3_bidirectional_compare.py"
SCHEMA = ROOT / "profiles/schema/fluxrt-h3-bidirectional-consistency-v1.schema.json"
SPEC = importlib.util.spec_from_file_location("foc_h3_bidirectional_compare", TOOL)
assert SPEC is not None and SPEC.loader is not None
compare = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = compare
SPEC.loader.exec_module(compare)


class H3BidirectionalCompareTests(unittest.TestCase):
    @staticmethod
    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest().upper()

    def write_json(self, path: Path, value: dict) -> None:
        path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")

    def write_side(
        self,
        root: Path,
        name: str,
        direction: int,
        speed: int,
        offset: float,
        rms_error: float = 0.20,
        maximum_error: float = 0.50,
    ) -> dict:
        directory = root / name
        directory.mkdir()
        raw = directory / "raw-lines.csv"
        with raw.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["source", "raw_line"])
            writer.writerow(["g431", "FOC st=0 rf=00000000 duty=0/0/0"])
            writer.writerow([
                "g431",
                "FADVP,state=4,result=1,ticks=1200/85000,first=83801,"
                "epoch=0/0,miss=0/0,features=00,status=00000000,snap=1,"
                f"ctrlstate=7,orel=1,closed=1,ogates=7f,phase=0,speed={speed},"
                "lossus=0,wcet=12000,ctrl=9900,diagmiss=0,finish=0",
            ])
            writer.writerow(["g431", "FH3R,16,50,1"])
            writer.writerow(["g431", "FOC st=0 rf=00000000 duty=0/0/0"])
        capture = directory / "capture.summary.json"
        self.write_json(capture, {
            "contract": "fluxrt-h3-dynamic-board-capture",
            "version": 1,
            "status": "captured-not-approved",
            "anchor_count": 16,
            "query_count": 350,
            "commanded_mechanical_direction": direction,
            "parameter_approval": "not-granted",
            "target_capability_approval": "not-granted",
            "artifacts": {"raw-lines.csv": self.digest(raw)},
        })
        time_manifest = directory / "time-map.manifest.json"
        self.write_json(time_manifest, {
            "contract": "fluxrt-h3-time-map",
            "version": 1,
            "motor_id": "test-motor",
            "pole_pairs": 7,
            "mechanical_direction": 1,
            "parameter_approval": "not-granted",
            "target_capability_approval": "not-granted",
        })
        time_result = directory / "time-map.result.json"
        self.write_json(time_result, {
            "contract": "fluxrt-h3-time-map-result",
            "version": 1,
            "status": "aligned-offline",
            "motor_id": "test-motor",
            "mapping": {
                "clock_drift_ppm": 25.0,
                "maximum_anchor_residual_ticks": 0.6,
            },
            "reliable_sample_count": 30,
            "reliable_rms_angle_error_rad": rms_error,
            "reliable_maximum_angle_error_rad": maximum_error,
            "parameter_approval": "not-granted",
            "target_capability_approval": "not-granted",
        })
        alignment = directory / "alignment-offset.result.json"
        self.write_json(alignment, {
            "contract": "fluxrt-h3-alignment-offset-result",
            "version": 1,
            "status": "alignment-offset-candidate-not-approved",
            "motor_id": "test-motor",
            "additive_time_map_offset_rad": offset,
            "maximum_candidate_deviation_rad": 0.005,
            "parameter_approval": "not-granted",
            "target_capability_approval": "not-granted",
        })
        return {
            "commanded_direction": direction,
            "capture_summary_path": capture.relative_to(root).as_posix(),
            "capture_summary_sha256": self.digest(capture),
            "time_map_manifest_path": time_manifest.relative_to(root).as_posix(),
            "time_map_manifest_sha256": self.digest(time_manifest),
            "time_map_result_path": time_result.relative_to(root).as_posix(),
            "time_map_result_sha256": self.digest(time_result),
            "alignment_offset_result_path": alignment.relative_to(root).as_posix(),
            "alignment_offset_result_sha256": self.digest(alignment),
        }

    @staticmethod
    def requirements() -> dict:
        return {
            "minimum_anchor_count": 15,
            "minimum_query_count": 300,
            "minimum_reliable_sample_count": 20,
            "maximum_absolute_clock_drift_ppm": 100.0,
            "maximum_anchor_residual_ticks": 1.0,
            "maximum_rms_angle_error_rad": 0.30,
            "maximum_angle_error_rad": 0.70,
            "maximum_alignment_candidate_deviation_rad": 0.02,
            "maximum_alignment_offset_delta_rad": 0.05,
            "maximum_terminal_speed_magnitude_delta_rpm": 30.0,
            "maximum_rms_angle_error_delta_rad": 0.05,
            "maximum_angle_error_delta_rad": 0.10,
        }

    def make_manifest(self, root: Path, reverse_speed: int = -590) -> Path:
        forward = self.write_side(root, "forward", 1, 592, 0.965)
        reverse = self.write_side(root, "reverse", -1, reverse_speed, 0.970, 0.22, 0.55)
        manifest = root / "bidirectional.manifest.json"
        self.write_json(manifest, {
            "contract": "fluxrt-h3-bidirectional-consistency",
            "version": 1,
            "forward": forward,
            "reverse": reverse,
            "requirements": self.requirements(),
            "parameter_approval": "not-granted",
            "target_capability_approval": "not-granted",
        })
        return manifest

    def test_schema_declares_non_approving_bounded_contract(self) -> None:
        schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        self.assertEqual(
            schema["properties"]["parameter_approval"]["const"], "not-granted"
        )
        self.assertEqual(
            schema["properties"]["target_capability_approval"]["const"],
            "not-granted",
        )
        self.assertIn("maximum_alignment_offset_delta_rad", schema["properties"]["requirements"]["required"])

    def test_matching_forward_reverse_evidence_yields_candidate_only(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.make_manifest(root)
            result = compare.analyse_manifest(manifest, root)
        self.assertEqual(
            result["status"], "bidirectional-consistency-candidate-not-approved"
        )
        self.assertEqual(result["forward"]["commanded_direction"], 1)
        self.assertEqual(result["reverse"]["commanded_direction"], -1)
        self.assertFalse(result["automatic_retry"])
        self.assertEqual(result["target_capability_approval"], "not-granted")

    def test_reverse_speed_sign_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.make_manifest(root, reverse_speed=590)
            with self.assertRaisesRegex(
                compare.BidirectionalCompareError, "speed sign"
            ):
                compare.analyse_manifest(manifest, root)

    def test_hash_and_consistency_bounds_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.make_manifest(root)
            value = json.loads(manifest.read_text(encoding="utf-8"))
            value["forward"]["capture_summary_sha256"] = "0" * 64
            self.write_json(manifest, value)
            with self.assertRaisesRegex(
                compare.BidirectionalCompareError, "SHA-256 mismatch"
            ):
                compare.analyse_manifest(manifest, root)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.make_manifest(root)
            value = json.loads(manifest.read_text(encoding="utf-8"))
            value["requirements"]["maximum_alignment_offset_delta_rad"] = 0.001
            self.write_json(manifest, value)
            with self.assertRaisesRegex(
                compare.BidirectionalCompareError, "alignment_offset_delta_rad"
            ):
                compare.analyse_manifest(manifest, root)


if __name__ == "__main__":
    unittest.main()
