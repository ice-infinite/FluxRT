import importlib.util
import math
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "tools" / "foc_g431_as5600_alignment_capture.py"
SPEC = importlib.util.spec_from_file_location("g431_alignment_capture", MODULE_PATH)
capture = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(capture)


class G431As5600AlignmentCaptureTests(unittest.TestCase):
    def valid_end(self):
        return capture.parse_end(
            "FALEND,1000,0,0,0,4,1,12000,12000,00000001,0,0,40,20,20,0"
        )

    def test_parses_exact_firmware_records(self):
        row = capture.parse_row("FALR,2,40,1,3500,3")
        self.assertEqual(row["raw_count"], 3500)
        self.assertIsNotNone(self.valid_end())
        self.assertIsNone(capture.parse_end("FALEND,short"))

    def test_stable_historical_repeat_passes_without_approving(self):
        # raw ~= 4007 maps to the historical additive offset under
        # offset = -7 * mechanical_angle (mod 2*pi).
        rows = [capture.parse_row("FALR,0,0,1,4007,2")]
        rows += [
            capture.parse_row(f"FALR,{i},{i * 20},1,{4007 + (i % 2)},3")
            for i in range(1, 19)
        ]
        rows += [capture.parse_row("FALR,19,380,1,4007,4")]
        summary = capture.summarize(rows, self.valid_end())
        self.assertTrue(summary["requirements_pass"])
        self.assertTrue(summary["transport_capture_complete"])
        self.assertTrue(summary["tail_capture_complete"])
        self.assertEqual(summary["parameter_approval"], "not-granted")

    def test_early_transport_gap_does_not_invalidate_complete_tail(self):
        rows = [capture.parse_row("FALR,0,0,1,4007,2")]
        rows += [
            capture.parse_row(f"FALR,{i},{i * 20},1,{4007 + (i % 2)},3")
            for i in range(3, 19)
        ]
        rows += [capture.parse_row("FALR,19,380,1,4007,4")]
        summary = capture.summarize(rows, self.valid_end())
        self.assertTrue(summary["requirements_pass"])
        self.assertFalse(summary["transport_capture_complete"])
        self.assertTrue(summary["tail_capture_complete"])

    def test_unstable_or_incomplete_capture_fails(self):
        rows = [capture.parse_row("FALR,0,0,1,1000,2")]
        rows += [
            capture.parse_row(f"FALR,{i},{i * 20},1,{1000 + i * 30},3")
            for i in range(1, 19)
        ]
        rows += [capture.parse_row("FALR,19,380,1,1570,4")]
        summary = capture.summarize(rows, self.valid_end())
        self.assertFalse(summary["requirements_pass"])
        self.assertGreater(summary["maximum_candidate_deviation_rad"], 0.05)
        self.assertFalse(capture.summarize(rows[:10], None)["requirements_pass"])

    def test_missing_tail_row_fails_even_if_target_reports_all_valid(self):
        rows = [capture.parse_row("FALR,0,0,1,4007,2")]
        rows += [
            capture.parse_row(f"FALR,{i},{i * 20},1,{4007 + (i % 2)},3")
            for i in range(1, 18)
        ]
        rows += [capture.parse_row("FALR,19,380,1,4007,4")]
        summary = capture.summarize(rows, self.valid_end())
        self.assertFalse(summary["requirements_pass"])
        self.assertFalse(summary["tail_capture_complete"])


if __name__ == "__main__":
    unittest.main()
