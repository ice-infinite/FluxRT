from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/foc_h3_board_capture.py"
SPEC = importlib.util.spec_from_file_location("foc_h3_board_capture", TOOL)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)


class H3BoardCaptureTests(unittest.TestCase):
    @staticmethod
    def g431(sequence: int, tag: int, delta: int = 49) -> str:
        cycle = 100000 + (sequence - 5) * 170000
        return (
            f"FH3_SYNC_STATUS,1,196608,3,1,1,0,1,101,{cycle},200,7,"
            f"20261004,{sequence},{tag},0,1,100,{cycle + delta},{delta}"
        )

    @staticmethod
    def deng(sequence: int, tag: int, tick: int) -> str:
        return (
            "SYNC_ANCHOR_RX,session=20261004,"
            f"edge_sequence={sequence},edge_tag={tag},truth_tick_us={tick},"
            "source=2,flags=0x00000003"
        )

    def test_parses_and_pairs_verified_loopback_anchors(self) -> None:
        g431 = [capture.parse_g431_status(self.g431(i, 100 + i)) for i in range(5, 8)]
        deng = [capture.parse_deng_anchor(self.deng(i, 100 + i, 1000 * i)) for i in range(5, 8)]
        pairs = capture.pair_anchors(g431, deng)
        summary = capture.build_summary(pairs, [])
        self.assertEqual(summary["anchor_count"], 3)
        self.assertEqual(summary["loopback"]["absolute_max_us"], 49 / 170)
        self.assertAlmostEqual(summary["clock_mapping"]["clock_drift_ppm"], 0.0)
        self.assertAlmostEqual(summary["clock_mapping"]["maximum_anchor_residual_us"], 0.0)
        self.assertEqual(summary["target_capability_approval"], "not-granted")

    def test_missing_loopback_and_identity_mismatch_fail_closed(self) -> None:
        bad = self.g431(5, 105).replace(",7,20261004,", ",3,20261004,")
        with self.assertRaisesRegex(capture.CaptureError, "loopback"):
            capture.parse_g431_status(bad)
        left = [capture.parse_g431_status(self.g431(i, 100 + i)) for i in range(5, 8)]
        right = [capture.parse_deng_anchor(self.deng(i, 200 + i, 1000 * i)) for i in range(5, 8)]
        with self.assertRaisesRegex(capture.CaptureError, "identity mismatch"):
            capture.pair_anchors(left, right)

    def test_ready_and_busy_snapshots_are_ignored_but_failed_is_rejected(self) -> None:
        completed = self.g431(5, 105)
        self.assertIsNone(capture.parse_g431_status(completed.replace(",3,1,1,", ",1,1,1,")))
        self.assertIsNone(capture.parse_g431_status(completed.replace(",3,1,1,", ",2,1,1,")))
        with self.assertRaisesRegex(capture.CaptureError, "not a completed frame"):
            capture.parse_g431_status(completed.replace(",3,1,1,", ",4,1,1,"))

    def test_sequence_gap_and_invalid_truth_fail_closed(self) -> None:
        left = [capture.parse_g431_status(self.g431(i, 100 + i)) for i in (5, 7, 8)]
        right = [capture.parse_deng_anchor(self.deng(i, 100 + i, 1000 * i)) for i in (5, 7, 8)]
        with self.assertRaisesRegex(capture.CaptureError, "not contiguous"):
            capture.pair_anchors(left, right)
        pairs = capture.pair_anchors(
            [capture.parse_g431_status(self.g431(i, 100 + i)) for i in range(5, 8)],
            [capture.parse_deng_anchor(self.deng(i, 100 + i, 1000 * i)) for i in range(5, 8)],
        )
        with self.assertRaisesRegex(capture.CaptureError, "invalid sample"):
            capture.build_summary(
                pairs,
                [{"sequence": 1, "truth_tick": 1, "mechanical_angle_rad": 0.0,
                  "valid": 0, "read_failures": 0}],
            )

    def test_artifacts_bind_hashes_and_use_pb7_tick(self) -> None:
        pairs = capture.pair_anchors(
            [capture.parse_g431_status(self.g431(i, 100 + i)) for i in range(5, 8)],
            [capture.parse_deng_anchor(self.deng(i, 100 + i, 1000 * i)) for i in range(5, 8)],
        )
        summary = capture.build_summary(pairs, [])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            capture.write_artifacts(output, pairs, [], [], summary)
            anchors = (output / "anchors.csv").read_text(encoding="utf-8")
            stored = (output / "capture.summary.json").read_text(encoding="utf-8")
        self.assertIn("100049", anchors)
        self.assertIn("anchors.csv", stored)
        self.assertIn("captured-not-approved", stored)

    def test_clock_mapping_rejects_duplicate_pb7_tick(self) -> None:
        left = [capture.parse_g431_status(self.g431(i, 100 + i)) for i in range(5, 8)]
        right = [capture.parse_deng_anchor(self.deng(i, 100 + i, 1000 * i)) for i in range(5, 8)]
        left[1] = capture.G431Anchor(
            **{**capture.asdict(left[1]), "loopback_cycle_tick": left[0].loopback_cycle_tick}
        )
        with self.assertRaisesRegex(capture.CaptureError, "duplicate"):
            capture.build_summary(capture.pair_anchors(left, right), [])

    def test_failed_checkpoint_keeps_raw_and_denies_approval(self) -> None:
        raw = [("g431", "FH3_SYNC_INIT,0"), ("dengfoc", "SYNC_STATUS")]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            capture.write_raw_checkpoint(
                output,
                raw,
                "captured-pending-analysis",
            )
            capture.mark_checkpoint_failed(output, "init failed")
            summary = __import__("json").loads(
                (output / "capture.summary.json").read_text(encoding="utf-8")
            )
            stored = (output / "raw-lines.csv").read_text(encoding="utf-8")
        self.assertEqual(summary["status"], "capture-failed")
        self.assertEqual(summary["error"], "init failed")
        self.assertEqual(summary["motor_power_enabled"], False)
        self.assertEqual(summary["target_capability_approval"], "not-granted")
        self.assertIn("FH3_SYNC_INIT,0", stored)


if __name__ == "__main__":
    unittest.main()
