import importlib.util
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "tools" / "foc_g431_as5600_capture.py"
SPEC = importlib.util.spec_from_file_location("g431_as5600_capture", MODULE_PATH)
capture = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(capture)


class G431As5600CaptureTests(unittest.TestCase):
    def test_parse_rejects_legacy_missing_sda_field(self):
        legacy = "FENC,0,1,100,4090,359000,1,1,0,0,00000000,1,1"
        self.assertIsNone(capture.parse_fenc(legacy))

    def test_bidirectional_wrap_summary(self):
        raw = [3900, 100, 500, 100, 3900, 3500]
        rows = []
        for index, count in enumerate(raw):
            rows.append(
                capture.parse_fenc(
                    f"FENC,{index},1,{index * 100},{count},0,1,{index + 1},"
                    "0,0,00000000,1,1,1"
                )
            )
        summary = capture.summarize(rows)
        self.assertEqual(summary["valid_sample_count"], len(raw))
        self.assertEqual(summary["wrap_event_count"], 2)
        self.assertGreater(summary["positive_motion_count"], 0)
        self.assertGreater(summary["negative_motion_count"], 0)
        self.assertTrue(summary["bus_healthy"])

    def test_stationary_capture_has_no_motion(self):
        rows = [
            capture.parse_fenc(
                f"FENC,{index},1,{index * 100},2000,0,1,{index + 1},"
                "0,0,00000000,1,1,1"
            )
            for index in range(5)
        ]
        summary = capture.summarize(rows)
        self.assertEqual(summary["total_motion_rev"], 0.0)
        self.assertTrue(summary["bus_healthy"])


if __name__ == "__main__":
    unittest.main()
