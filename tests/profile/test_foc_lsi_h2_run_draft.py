from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from foc_lsi_h2_run_draft import DraftError, build_draft  # noqa: E402


class H2RunDraftTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.capture = self.root / "capture"
        self.capture.mkdir()
        self.trace = self.root / "trace.csv"
        self.analysis = self.root / "analysis.json"
        self.firmware = "B" * 64
        self.write_capture()
        self.trace.write_text("control_tick,current_u_a\n0,0\n", encoding="utf-8")
        self.analysis.write_text(
            json.dumps(
                {
                    "analysis_kind": "lsi-powered-one-shot-screen",
                    "source_metadata": {"hex_sha256": self.firmware},
                    "decision": {
                        "hardware_execution": "pass",
                        "parameter_approval": "not-granted",
                    },
                }
            ),
            encoding="utf-8",
        )

    @staticmethod
    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest().upper()

    def write_capture(self, commands_sent: int = 0, raw_count: int = 2) -> None:
        combined = self.capture / "combined-lines.csv"
        angle = self.capture / "dengfoc-angle.csv"
        combined.write_text(
            "source,line_sequence,host_time_ns,host_time_s,raw_line\n"
            "g431,1,100,1.0,FLSI_RAW sample\n",
            encoding="utf-8",
        )
        with angle.open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "host_time_s",
                    "axis",
                    "valid",
                    "sequence",
                    "mechanical_position_rad",
                    "read_failures",
                ],
            )
            writer.writeheader()
            for index in range(3):
                writer.writerow(
                    {
                        "host_time_s": 1.0 + index * 0.1,
                        "axis": 0,
                        "valid": 1,
                        "sequence": index + 1,
                        "mechanical_position_rad": 0.5,
                        "read_failures": 0,
                    }
                )
        (self.capture / "capture.summary.json").write_text(
            json.dumps(
                {
                    "contract": "fluxrt-dual-serial-capture",
                    "version": 1,
                    "status": "capture-complete",
                    "read_only": True,
                    "commands_sent": commands_sent,
                    "clock": {
                        "source": "time.monotonic_ns",
                        "shared_origin": True,
                    },
                    "sources": {
                        "g431": {"flsi_raw_line_count": raw_count},
                        "dengfoc": {"parsed_angle_sample_count": 3},
                    },
                    "static_h2_association": {
                        "kind": "common-host-static-window-only",
                        "dynamic_alignment_valid": False,
                        "g431_raw_window_start_s": 1.05,
                        "g431_raw_window_end_s": 1.15,
                    },
                    "artifacts": {
                        "combined_lines_path": combined.name,
                        "combined_lines_sha256": self.digest(combined),
                        "angle_csv_path": angle.name,
                        "angle_csv_sha256": self.digest(angle),
                    },
                    "errors": [],
                }
            ),
            encoding="utf-8",
        )

    def build(self) -> dict:
        return build_draft(
            self.root,
            "run-1",
            "M0-test",
            self.firmware,
            Path("capture"),
            Path("trace.csv"),
            Path("analysis.json"),
            0,
            0.25,
        )

    def test_valid_bundle_binds_every_artifact_without_approval(self) -> None:
        draft = self.build()
        run = draft["matrix_run"]
        self.assertEqual(draft["status"], "evidence-bound-unapproved")
        self.assertEqual(draft["parameter_approval"], "not-granted")
        self.assertEqual(run["capture_summary_sha256"], self.digest(self.capture / "capture.summary.json"))
        self.assertEqual(run["combined_lines_sha256"], self.digest(self.capture / "combined-lines.csv"))
        self.assertEqual(run["angle_csv_sha256"], self.digest(self.capture / "dengfoc-angle.csv"))
        self.assertAlmostEqual(run["angle_window_start_s"], 0.8)
        self.assertAlmostEqual(run["angle_window_end_s"], 1.4)

    def test_capture_with_commands_is_rejected(self) -> None:
        self.write_capture(commands_sent=1)
        with self.assertRaisesRegex(DraftError, "not read-only"):
            self.build()

    def test_capture_without_raw_or_angle_evidence_is_rejected(self) -> None:
        self.write_capture(raw_count=0)
        with self.assertRaisesRegex(DraftError, "no G431"):
            self.build()

    def test_firmware_identity_mismatch_is_rejected(self) -> None:
        with self.assertRaisesRegex(DraftError, "firmware SHA"):
            build_draft(
                self.root,
                "run-1",
                "M0-test",
                "C" * 64,
                Path("capture"),
                Path("trace.csv"),
                Path("analysis.json"),
                0,
                0.25,
            )

    def test_replaced_capture_artifact_is_rejected(self) -> None:
        (self.capture / "combined-lines.csv").write_text("replaced\n", encoding="utf-8")
        with self.assertRaisesRegex(DraftError, "artifact binding mismatch"):
            self.build()

    def test_evidence_outside_project_root_is_rejected(self) -> None:
        outside = Path(self.temporary.name).parent / "outside-trace.csv"
        outside.write_text("unsafe", encoding="utf-8")
        self.addCleanup(lambda: outside.unlink(missing_ok=True))
        with self.assertRaisesRegex(DraftError, "inside project root"):
            build_draft(
                self.root,
                "run-1",
                "M0-test",
                self.firmware,
                Path("capture"),
                outside,
                Path("analysis.json"),
                0,
                0.25,
            )


if __name__ == "__main__":
    unittest.main()
