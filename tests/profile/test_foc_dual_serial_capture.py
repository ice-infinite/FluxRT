import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from foc_dual_serial_capture import (  # noqa: E402
    CapturedLine,
    SourceConfig,
    build_summary,
    capture,
    validate_sources,
    write_artifacts,
)


class DualSerialCaptureTests(unittest.TestCase):
    def setUp(self) -> None:
        self.g431 = SourceConfig("g431", "COM6", 115200)
        self.dengfoc = SourceConfig("dengfoc", "COM13", 115200)

    def test_rejects_same_port_and_invalid_duration(self) -> None:
        with self.assertRaises(ValueError):
            validate_sources(self.g431, SourceConfig("dengfoc", "com6", 115200), 1.0)
        with self.assertRaises(ValueError):
            validate_sources(self.g431, self.dengfoc, 0.0)

    def test_summary_has_one_clock_and_never_grants_capability(self) -> None:
        records = [
            CapturedLine("dengfoc", 1, 1_100_000_000, 0.1,
                         "axis=0,valid=1,seq=8,mech=1.0,multi=1.0,vel=0,elec=0.7,fail=0"),
            CapturedLine("g431", 1, 1_200_000_000, 0.2,
                         "FLSI_RAW,1,20,2048,2048,100,200,200,200,7083,0"),
            CapturedLine("g431", 2, 1_300_000_000, 0.3, "FLSI_CAP_DUMP,1,1"),
        ]
        summary = build_summary(
            self.g431, self.dengfoc, 1_000_000_000, 1_500_000_000, records, []
        )
        self.assertEqual(summary["status"], "capture-complete")
        self.assertTrue(summary["read_only"])
        self.assertEqual(summary["commands_sent"], 0)
        self.assertTrue(summary["clock"]["shared_origin"])
        self.assertEqual(summary["sources"]["g431"]["flsi_raw_line_count"], 1)
        self.assertEqual(summary["sources"]["dengfoc"]["parsed_angle_sample_count"], 1)
        self.assertFalse(summary["static_h2_association"]["dynamic_alignment_valid"])
        self.assertEqual(summary["parameter_approval"], "not-granted")
        self.assertEqual(summary["target_capability_approval"], "not-granted")

    def test_reader_error_fails_closed(self) -> None:
        summary = build_summary(
            self.g431, self.dengfoc, 100, 200, [], ["g431: disconnected"]
        )
        self.assertEqual(summary["status"], "capture-failed")
        self.assertEqual(summary["commands_sent"], 0)

    def test_capture_opens_both_sources_and_only_reads(self) -> None:
        class FakeStream:
            def __init__(self, lines: list[bytes]) -> None:
                self.lines = lines
                self.closed = False

            def readline(self) -> bytes:
                if self.lines:
                    return self.lines.pop(0)
                return b""

            def close(self) -> None:
                self.closed = True

        streams = {
            "g431": FakeStream([b"FLSI_CAP_STATUS,1,2\n"]),
            "dengfoc": FakeStream(
                [b"axis=0,valid=1,seq=1,mech=1,multi=1,vel=0,elec=0,fail=0\n"]
            ),
        }

        def opener(config: SourceConfig) -> FakeStream:
            return streams[config.name]

        origin, end, records, errors = capture(
            self.g431, self.dengfoc, 0.01, opener=opener
        )
        self.assertGreaterEqual(end, origin)
        self.assertEqual({row.source for row in records}, {"g431", "dengfoc"})
        self.assertEqual(errors, [])
        self.assertTrue(all(stream.closed for stream in streams.values()))

    def test_artifacts_keep_common_time_for_matrix_angle_input(self) -> None:
        records = [
            CapturedLine("g431", 1, 10, 0.01, "boot ok"),
            CapturedLine(
                "dengfoc",
                1,
                20,
                0.02,
                "axis=0,valid=1,seq=9,mech=1.2,multi=1.2,vel=0,elec=2.1,fail=0",
            ),
        ]
        summary = build_summary(self.g431, self.dengfoc, 0, 30, records, [])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            write_artifacts(output, records, summary)
            with (output / "combined-lines.csv").open(newline="", encoding="utf-8") as stream:
                combined = list(csv.DictReader(stream))
            with (output / "dengfoc-angle.csv").open(newline="", encoding="utf-8") as stream:
                angles = list(csv.DictReader(stream))
            stored = json.loads((output / "capture.summary.json").read_text(encoding="utf-8"))
        self.assertEqual(len(combined), 2)
        self.assertEqual(angles[0]["host_time_s"], "0.02")
        self.assertEqual(angles[0]["sequence"], "9")
        self.assertEqual(stored["commands_sent"], 0)
        self.assertEqual(stored["artifacts"]["combined_lines_path"], "combined-lines.csv")
        self.assertEqual(len(stored["artifacts"]["combined_lines_sha256"]), 64)
        self.assertEqual(stored["artifacts"]["angle_csv_path"], "dengfoc-angle.csv")

    def test_tool_source_has_no_serial_write_call_or_start_command(self) -> None:
        source = (PROJECT_ROOT / "tools/foc_dual_serial_capture.py").read_text(encoding="utf-8")
        self.assertNotIn(".write(", source)
        self.assertNotIn("foc_lsi_start LSI1", source)
        self.assertNotIn("foc_lsi_capture_arm", source)


if __name__ == "__main__":
    unittest.main()
