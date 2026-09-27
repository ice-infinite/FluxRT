from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = PROJECT_ROOT / "simulation" / "capture_timing_baseline.py"
SPEC = importlib.util.spec_from_file_location("capture_timing_baseline", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
capture_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_tool)


class TimingCaptureStartReplyTests(unittest.TestCase):
    def test_current_machine_protocol_is_accepted(self) -> None:
        self.assertEqual(
            capture_tool.classify_start_reply(
                ["msh >", "FSTART,armed,582,582,200,rust-smo-pll,1,1,0"]
            ),
            "armed",
        )

    def test_legacy_human_protocol_remains_accepted(self) -> None:
        self.assertEqual(
            capture_tool.classify_start_reply(["FOC ARMED target=582"]),
            "armed",
        )

    def test_current_and_legacy_refusals_are_detected(self) -> None:
        self.assertEqual(
            capture_tool.classify_start_reply(["FSTART,refused,2,0004231f,12301"]),
            "refused",
        )
        self.assertEqual(
            capture_tool.classify_start_reply(["FOC start REFUSED: unsafe"]),
            "refused",
        )

    def test_refusal_wins_over_mixed_or_stale_armed_lines(self) -> None:
        self.assertEqual(
            capture_tool.classify_start_reply(
                ["FSTART,armed,582,582,200,rust-smo-pll,1,1,0", "FSTART,refused,2,0,0"]
            ),
            "refused",
        )

    def test_unrelated_output_is_missing(self) -> None:
        self.assertEqual(
            capture_tool.classify_start_reply(["msh >", "FSTOP"]),
            "missing",
        )


if __name__ == "__main__":
    unittest.main()
