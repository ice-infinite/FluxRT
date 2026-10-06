from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/foc_h3_no_power_probe_capture.py"
SPEC = importlib.util.spec_from_file_location("foc_h3_no_power_probe_capture", TOOL)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)


class FixedNoPowerProbeCaptureTests(unittest.TestCase):
    def test_preflight_requires_zero_output_valid_bus_and_disabled_receiver(self) -> None:
        lines = [
            "FOC st=0 rf=00000000 duty=0/0/0",
            "FFAULT,00000000,00000000,0,0,ARM,0,0000",
            "FADC,1/2/3,1/2/3,10,129,2052",
            "SYNC_STATUS,frames=0,capture_failure=0,overflow=0,"
            "truth_tx_fail=0,session_rollovers=0,driver_disabled=1",
        ]
        self.assertEqual(capture.validate_preflight(lines), 129)
        lines[2] = "FADC,1/2/3,1/2/3,40,516,2052"
        with self.assertRaisesRegex(capture.FixedProbeError, "516 mV"):
            capture.validate_preflight(lines)
        lines[2] = "FADC,1/2/3,1/2/3,10,129,2052"
        lines[3] = lines[3].replace("truth_tx_fail=0,", "")
        with self.assertRaisesRegex(capture.FixedProbeError, "both targets disabled"):
            capture.validate_preflight(lines)

    def test_analysis_requires_one_pair_success_and_final_stop(self) -> None:
        dynamic = (
            "FH3_DYNAMIC_ANCHOR,196608,3,1,1,0,1,100,100000,200,7,"
            "2026100503,3000,700,0,1,100,100049,49"
        )
        deng = (
            "SYNC_ANCHOR_RX,session=2026100503,edge_sequence=3000,"
            "edge_tag=700,truth_tick_us=123456,source=2,flags=0x00000003"
        )
        raw = [
            ("g431", dynamic),
            ("dengfoc", deng),
            ("g431", "FH3_NOPWR,1"),
            ("g431", "FOC st=0 rf=00000000 duty=0/0/0"),
        ]
        result = capture.analyse(raw)
        self.assertEqual(result["anchor_count"], 1)
        self.assertEqual(result["edge_sequence"], 3000)
        self.assertEqual(result["loopback_delta_cycles"], 49)
        with self.assertRaisesRegex(capture.FixedProbeError, "final G431"):
            capture.analyse(raw[:-1])

    def test_failed_raw_checkpoint_is_explicitly_not_approved(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            capture._write_raw(output, [("g431", "FH3_NOPWR,0")])
            summary = capture._finish_summary(
                output, "capture-failed", error="probe failed"
            )
            stored = (output / "raw-lines.csv").read_text(encoding="utf-8")
        self.assertEqual(summary["status"], "capture-failed")
        self.assertEqual(summary["target_capability_approval"], "not-granted")
        self.assertIn("FH3_NOPWR,0", stored)


if __name__ == "__main__":
    unittest.main()
