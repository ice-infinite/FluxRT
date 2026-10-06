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
TOOL = ROOT / "tools/foc_lsi_h2_matrix.py"
SPEC = importlib.util.spec_from_file_location("foc_lsi_h2_matrix", TOOL)
assert SPEC is not None and SPEC.loader is not None
matrix = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = matrix
SPEC.loader.exec_module(matrix)


class H2MatrixTests(unittest.TestCase):
    def test_repository_schema_declares_identity_hashes_and_windows(self) -> None:
        schema = json.loads(
            (ROOT / "profiles/schema/fluxrt-lsi-h2-matrix-v1.schema.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            schema["properties"]["contract"]["const"], "fluxrt-lsi-h2-matrix"
        )
        run_required = set(schema["properties"]["runs"]["items"]["required"])
        self.assertTrue(
            {
                "trace_csv_sha256",
                "analysis_json_sha256",
                "capture_summary_sha256",
                "combined_lines_sha256",
                "angle_csv_sha256",
                "angle_window_start_s",
                "angle_window_end_s",
            }
            <= run_required
        )

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.trace = self.root / "trace.csv"
        self.analysis = self.root / "analysis.json"
        self.angle = self.root / "angle.csv"
        self.capture_summary = self.root / "capture.summary.json"
        self.combined_lines = self.root / "combined-lines.csv"
        self.firmware = "A" * 64
        self.write_trace()
        self.write_analysis()
        self.write_angle([1.0, 1.0, 1.0])
        self.write_capture_evidence()

    @staticmethod
    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest().upper()

    def write_trace(self) -> None:
        fields = [
            "control_tick",
            "current_u_a",
            "current_v_a",
            "current_w_a",
            "applied_phase_u_v",
            "flags",
        ]
        rows = []
        for index in range(4):
            rows.append(
                {
                    "control_tick": index,
                    "current_u_a": 0.01 * index,
                    "current_v_a": -0.005 * index,
                    "current_w_a": -0.005 * index,
                    "applied_phase_u_v": 0.1,
                    "flags": (1 << 4) | (1 << 5),
                }
            )
        for index in range(4, 8):
            rows.append(
                {
                    "control_tick": index,
                    "current_u_a": 0.06 - 0.01 * index,
                    "current_v_a": -(0.06 - 0.01 * index) / 2,
                    "current_w_a": -(0.06 - 0.01 * index) / 2,
                    "applied_phase_u_v": -0.1,
                    "flags": (1 << 4) | (1 << 6),
                }
            )
        with self.trace.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def write_analysis(self) -> None:
        self.analysis.write_text(
            json.dumps(
                {
                    "analysis_kind": "lsi-powered-one-shot-screen",
                    "source_metadata": {"hex_sha256": self.firmware},
                    "decision": {
                        "hardware_execution": "pass",
                        "parameter_approval": "not-granted",
                    },
                    "fit": {
                        "applied_voltage_fixed_rs": {
                            "inductance_h": 0.0012,
                            "rmse_a": 0.002,
                            "maximum_absolute_residual_a": 0.004,
                        }
                    },
                }
            ),
            encoding="utf-8",
        )

    def write_capture_evidence(self) -> None:
        self.combined_lines.write_text(
            "source,line_sequence,host_time_ns,host_time_s,raw_line\n"
            "g431,1,1000000000,1.0,FLSI_RAW sample\n"
            "dengfoc,1,1100000000,1.1,axis sample\n",
            encoding="utf-8",
        )
        self.capture_summary.write_text(
            json.dumps(
                {
                    "contract": "fluxrt-dual-serial-capture",
                    "version": 1,
                    "status": "capture-complete",
                    "read_only": True,
                    "commands_sent": 0,
                    "clock": {
                        "source": "time.monotonic_ns",
                        "shared_origin": True,
                        "origin_ns": 0,
                        "end_ns": 2_000_000_000,
                        "elapsed_s": 2.0,
                    },
                    "sources": {
                        "g431": {"flsi_raw_line_count": 1},
                        "dengfoc": {"parsed_angle_sample_count": 3},
                    },
                    "static_h2_association": {
                        "kind": "common-host-static-window-only",
                        "dynamic_alignment_valid": False,
                        "g431_raw_window_start_s": 1.0,
                        "g431_raw_window_end_s": 1.2,
                    },
                    "artifacts": {
                        "combined_lines_path": self.combined_lines.name,
                        "combined_lines_sha256": self.digest(self.combined_lines),
                        "angle_csv_path": self.angle.name,
                        "angle_csv_sha256": self.digest(self.angle),
                    },
                    "errors": [],
                    "parameter_approval": "not-granted",
                    "target_capability_approval": "not-granted",
                }
            ),
            encoding="utf-8",
        )

    def bind_capture_to_angle(self, angle_path: Path, summary_path: Path) -> None:
        summary = json.loads(self.capture_summary.read_text(encoding="utf-8"))
        summary["artifacts"]["angle_csv_path"] = angle_path.name
        summary["artifacts"]["angle_csv_sha256"] = self.digest(angle_path)
        summary_path.write_text(json.dumps(summary), encoding="utf-8")

    def write_angle(
        self, angles: list[float], failure: int = 0, path: Path | None = None
    ) -> None:
        fields = [
            "host_time_s",
            "axis",
            "valid",
            "sequence",
            "mechanical_position_rad",
            "read_failures",
        ]
        destination = path or self.angle
        with destination.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index, angle in enumerate(angles):
                writer.writerow(
                    {
                        "host_time_s": 1.0 + index * 0.1,
                        "axis": 0,
                        "valid": 1,
                        "sequence": index + 1,
                        "mechanical_position_rad": angle,
                        "read_failures": failure,
                    }
                )

    def manifest(self) -> dict:
        return {
            "contract": "fluxrt-lsi-h2-matrix",
            "version": 1,
            "motor_id": "M0-test",
            "firmware_sha256": self.firmware,
            "pole_pairs": 7,
            "requirements": {
                "minimum_runs": 6,
                "angle_bin_count": 6,
                "minimum_angle_bins": 6,
                "minimum_angle_samples": 3,
                "maximum_angle_span_counts": 2,
                "maximum_phase_current_a": 0.25,
                "maximum_positive_negative_asymmetry": 0.05,
                "minimum_response_to_noise_ratio": 1.0,
            },
            "runs": [
                {
                    "id": "run-1",
                    "motor_id": "M0-test",
                    "firmware_sha256": self.firmware,
                    "trace_csv_path": "trace.csv",
                    "trace_csv_sha256": self.digest(self.trace),
                    "analysis_json_path": "analysis.json",
                    "analysis_json_sha256": self.digest(self.analysis),
                    "capture_summary_path": "capture.summary.json",
                    "capture_summary_sha256": self.digest(self.capture_summary),
                    "combined_lines_path": "combined-lines.csv",
                    "combined_lines_sha256": self.digest(self.combined_lines),
                    "angle_csv_path": "angle.csv",
                    "angle_csv_sha256": self.digest(self.angle),
                    "angle_axis": 0,
                    "angle_window_start_s": 0.9,
                    "angle_window_end_s": 1.3,
                }
            ],
        }

    def test_valid_single_run_is_collecting_not_approved(self) -> None:
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(self.manifest()), encoding="utf-8")
        result = matrix.analyse_manifest(manifest, self.root)
        self.assertEqual(result["result"], "collecting")
        self.assertFalse(result["matrix_complete"])
        self.assertEqual(result["parameter_approval"], "not-granted")
        self.assertEqual(result["runs"][0]["angle_span_counts"], 0)
        self.assertAlmostEqual(result["runs"][0]["mechanical_angle_rad"], 1.0)

    def test_hash_tampering_is_rejected(self) -> None:
        value = self.manifest()
        value["runs"][0]["trace_csv_sha256"] = "0" * 64
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "SHA-256 mismatch"):
            matrix.analyse_manifest(manifest, self.root)

        value = self.manifest()
        value["runs"][0]["capture_summary_sha256"] = "0" * 64
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "SHA-256 mismatch"):
            matrix.analyse_manifest(manifest, self.root)

    def test_capture_must_be_read_only_common_clock_evidence(self) -> None:
        summary = json.loads(self.capture_summary.read_text(encoding="utf-8"))
        summary["commands_sent"] = 1
        self.capture_summary.write_text(json.dumps(summary), encoding="utf-8")
        value = self.manifest()
        value["runs"][0]["capture_summary_sha256"] = self.digest(self.capture_summary)
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "not read-only"):
            matrix.analyse_manifest(manifest, self.root)

    def test_capture_artifact_replacement_is_rejected(self) -> None:
        self.combined_lines.write_text("replaced\n", encoding="utf-8")
        value = self.manifest()
        value["runs"][0]["combined_lines_sha256"] = self.digest(self.combined_lines)
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "artifact binding mismatch"):
            matrix.analyse_manifest(manifest, self.root)

    def test_angle_window_must_contain_raw_dump_window(self) -> None:
        value = self.manifest()
        value["runs"][0]["angle_window_start_s"] = 1.05
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "does not contain"):
            matrix.analyse_manifest(manifest, self.root)

    def test_six_distinct_electrical_bins_complete_screening_without_approval(self) -> None:
        value = self.manifest()
        value["runs"] = []
        for angle_bin in range(6):
            angle = (angle_bin + 0.5) * 2.0 * math.pi / 6.0 / 7.0
            angle_path = self.root / f"angle-{angle_bin}.csv"
            self.write_angle([angle, angle, angle], path=angle_path)
            run = dict(self.manifest()["runs"][0])
            run["id"] = f"run-{angle_bin}"
            run["angle_csv_path"] = angle_path.name
            run["angle_csv_sha256"] = self.digest(angle_path)
            capture_summary = self.root / f"capture-{angle_bin}.summary.json"
            self.bind_capture_to_angle(angle_path, capture_summary)
            run["capture_summary_path"] = capture_summary.name
            run["capture_summary_sha256"] = self.digest(capture_summary)
            value["runs"].append(run)
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(value), encoding="utf-8")

        result = matrix.analyse_manifest(manifest, self.root)

        self.assertTrue(result["matrix_complete"])
        self.assertEqual(result["result"], "screening-complete")
        self.assertEqual(result["covered_electrical_angle_bins"], list(range(6)))
        self.assertEqual(result["parameter_approval"], "not-granted")

    def test_unstable_or_failed_angle_window_is_rejected(self) -> None:
        count = 12
        self.write_angle([1.0, 1.0 + count * 2.0 * math.pi / 4096.0, 1.0])
        self.bind_capture_to_angle(self.angle, self.capture_summary)
        value = self.manifest()
        value["runs"][0]["capture_summary_sha256"] = self.digest(self.capture_summary)
        value["runs"][0]["angle_csv_sha256"] = self.digest(self.angle)
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "spans"):
            matrix.analyse_manifest(manifest, self.root)

        self.write_angle([1.0, 1.0, 1.0], failure=1)
        self.bind_capture_to_angle(self.angle, self.capture_summary)
        value = self.manifest()
        value["runs"][0]["capture_summary_sha256"] = self.digest(self.capture_summary)
        value["runs"][0]["angle_csv_sha256"] = self.digest(self.angle)
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "failure count"):
            matrix.analyse_manifest(manifest, self.root)

    def test_identity_and_current_limit_are_fail_closed(self) -> None:
        value = self.manifest()
        value["runs"][0]["motor_id"] = "other-motor"
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "motor_id mismatch"):
            matrix.analyse_manifest(manifest, self.root)

        value = self.manifest()
        value["requirements"]["maximum_phase_current_a"] = 0.02
        manifest.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(matrix.MatrixError, "current limit"):
            matrix.analyse_manifest(manifest, self.root)


if __name__ == "__main__":
    unittest.main()
