from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/foc_h3_partial_diagnostic.py"
SPEC = importlib.util.spec_from_file_location("foc_h3_partial_diagnostic", TOOL)
assert SPEC is not None and SPEC.loader is not None
diagnostic = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = diagnostic
SPEC.loader.exec_module(diagnostic)


class H3PartialDiagnosticTests(unittest.TestCase):
    @staticmethod
    def anchor(sequence: int, control: int, truth: int):
        left = (
            "FH3_DYNAMIC_ANCHOR,196608,3,1,1,0,1,100,"
            f"{100000 + sequence * 170000},{control},7,9,{sequence},"
            f"{1000 + sequence},0,1,100,{100049 + sequence * 170000},49"
        )
        right = (
            "SYNC_ANCHOR_RX,session=9,"
            f"edge_sequence={sequence},edge_tag={1000 + sequence},"
            f"truth_tick_us={truth},source=2,flags=0x00000003"
        )
        return left, right

    @staticmethod
    def ftr(step: int) -> str:
        values = [0] * 31
        values[0] = step
        return "FTR," + ",".join(str(value) for value in values)

    def test_failed_capture_yields_only_non_approving_intersection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            capture_dir = Path(directory) / "capture"
            output = Path(directory) / "derived"
            capture_dir.mkdir()
            rows = []
            for sequence, control in zip(range(5, 9), (100, 200, 300, 400)):
                left, right = self.anchor(sequence, control, sequence * 1000)
                rows.append(("g431", left))
                if sequence != 5:
                    rows.append(("dengfoc", right))
            rows.extend((
                ("g431", self.ftr(250)),
                ("dengfoc", "SYNC_TRUTH,sequence=1,truth_tick_us=6000,"
                 "mechanical_angle_rad=1.0,valid=1,read_failures=0"),
                ("dengfoc", "SYNC_TRUTH,sequence=2,truth_tick_us=7000,"
                 "mechanical_angle_rad=1.1,valid=1,read_failures=0"),
                ("g431", "FH3R,4,50,1"),
            ))
            raw_path = capture_dir / "raw-lines.csv"
            with raw_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["source", "raw_line"])
                writer.writerows(rows)
            (capture_dir / "capture.summary.json").write_text(
                json.dumps({"status": "capture-failed"}), encoding="utf-8"
            )

            result = diagnostic.analyse_source(raw_path)
            summary = diagnostic.write_output(raw_path, output, result)

            self.assertEqual(result["g431_only_sequences"], [5])
            self.assertEqual(len(result["pairs"]), 3)
            self.assertEqual(summary["status"], "partial-diagnostic-not-approved")
            self.assertEqual(summary["query_alignment_status"], "bracketed")
            self.assertEqual(summary["truth_quality"]["rebase_sample_count"], 0)
            self.assertEqual(summary["target_capability_approval"], "not-granted")
            self.assertTrue((output / "valid-truth.csv").is_file())

    def test_non_failed_parent_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            capture_dir = Path(directory)
            raw_path = capture_dir / "raw-lines.csv"
            raw_path.write_text("source,raw_line\n", encoding="utf-8")
            (capture_dir / "capture.summary.json").write_text(
                json.dumps({"status": "captured-not-approved"}),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                diagnostic.PartialDiagnosticError, "not explicitly failed"
            ):
                diagnostic.analyse_source(raw_path)

    def test_explicit_bounded_gap_override_remains_non_approving(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            capture_dir = Path(directory) / "capture"
            output = Path(directory) / "derived"
            capture_dir.mkdir()
            rows = []
            for sequence, control in zip(range(5, 8), (100, 200, 300)):
                left, right = self.anchor(sequence, control, sequence * 1000)
                rows.extend((("g431", left), ("dengfoc", right)))
            rows.extend((
                ("g431", self.ftr(150)),
                ("dengfoc", "SYNC_TRUTH,sequence=1,truth_tick_us=5000,"
                 "mechanical_angle_rad=1.0,valid=1,read_failures=0"),
                ("dengfoc", "SYNC_TRUTH,sequence=2,truth_tick_,"
                 "mechanical_angle_rad=1.1,valid=1,read_failures=0"),
                ("dengfoc", "SYNC_TRUTH,sequence=8,truth_tick_us=19000,"
                 "mechanical_angle_rad=1.7,valid=1,read_failures=0"),
                ("g431", "FH3R,3,50,1"),
                ("dengfoc", "SYNC_TRUTH,sequence=9,truth_tick_us=21000,"),
            ))
            raw_path = capture_dir / "raw-lines.csv"
            with raw_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["source", "raw_line"])
                writer.writerows(rows)
            (capture_dir / "capture.summary.json").write_text(
                json.dumps({"status": "capture-failed"}), encoding="utf-8"
            )

            with self.assertRaisesRegex(
                diagnostic.PartialDiagnosticError, "malformed.*exceeds"
            ):
                diagnostic.analyse_source(raw_path)

            result = diagnostic.analyse_source(
                raw_path,
                maximum_valid_truth_gap_us=14_000,
                maximum_malformed_truth_lines=1,
            )
            summary = diagnostic.write_output(raw_path, output, result)
            self.assertEqual(
                summary["truth_quality"]["maximum_valid_gap_us"], 14_000
            )
            self.assertEqual(
                summary["truth_quality"]["malformed_truth_line_count"], 1
            )
            self.assertEqual(
                summary["truth_quality"]["terminal_partial_truth_line_count"],
                1,
            )
            self.assertEqual(summary["status"], "partial-diagnostic-not-approved")
            self.assertEqual(summary["target_capability_approval"], "not-granted")


if __name__ == "__main__":
    unittest.main()
