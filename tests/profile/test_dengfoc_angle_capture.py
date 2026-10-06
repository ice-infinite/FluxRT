import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from dengfoc_angle_capture import parse_telemetry_line, summarize  # noqa: E402


class DengFocAngleCaptureTests(unittest.TestCase):
    def test_parser_accepts_firmware_line(self):
        row = parse_telemetry_line(
            "axis=0,valid=1,seq=42,mech=1.0,multi=7.2,vel=-0.5,elec=0.7,fail=0",
            0.25,
        )
        self.assertIsNotNone(row)
        self.assertEqual(row["sequence"], 42)
        self.assertEqual(row["read_failures"], 0)
        self.assertAlmostEqual(row["host_time_s"], 0.25)

    def test_parser_rejects_boot_and_partial_lines(self):
        self.assertIsNone(parse_telemetry_line("axis0_init=1"))
        self.assertIsNone(parse_telemetry_line("axis=0,valid=1"))

    def test_summary_detects_bidirectional_wrap_and_failures(self):
        lines = [
            "axis=0,valid=1,seq=1,mech=6.1,multi=6.1,vel=1,elec=1,fail=0",
            "axis=0,valid=1,seq=2,mech=0.1,multi=6.4,vel=1,elec=1,fail=0",
            "axis=0,valid=1,seq=3,mech=6.2,multi=6.2,vel=-1,elec=1,fail=1",
        ]
        result = summarize(parse_telemetry_line(line) for line in lines)
        self.assertEqual(result["wrap_events"], 2)
        self.assertEqual(result["failure_count_max"], 1)
        self.assertAlmostEqual(result["positive_motion_rad"], 0.3)
        self.assertAlmostEqual(result["negative_motion_rad"], 0.2)

    def test_two_turn_summary_keeps_unwrapped_motion(self):
        rows = []
        for index in range(9):
            multi = index * (math.pi / 2.0)
            rows.append(
                {
                    "host_time_s": float(index),
                    "axis": 0,
                    "valid": 1,
                    "sequence": index + 1,
                    "mechanical_position_rad": multi % (2.0 * math.pi),
                    "multi_turn_position_rad": multi,
                    "mechanical_velocity_rad_s": 1.0,
                    "electrical_angle_rad": 0.0,
                    "read_failures": 0,
                }
            )
        result = summarize(rows)
        self.assertAlmostEqual(result["net_motion_rad"], 4.0 * math.pi)
        self.assertAlmostEqual(result["dominant_motion_rad"], 4.0 * math.pi)
        self.assertEqual(result["reverse_motion_rad"], 0.0)
        self.assertAlmostEqual(result["mechanical_span_rad"], 3.0 * math.pi / 2.0)
        self.assertGreater(result["velocity_rms_rad_s"], 0.0)


if __name__ == "__main__":
    unittest.main()
