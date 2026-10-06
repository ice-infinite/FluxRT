from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/foc_h3_dynamic_capture.py"
TARGET_MAIN = ROOT / "applications/main.c"
TARGET_DENGFOC = ROOT / "targets/esp32_dengfoc/src/time_sync_main.cpp"
SPEC = importlib.util.spec_from_file_location("foc_h3_dynamic_capture", TOOL)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)


class H3DynamicCaptureTests(unittest.TestCase):
    def test_reverse_candidate_is_compile_time_fixed_and_not_retryable(self) -> None:
        source = TARGET_MAIN.read_text(encoding="utf-8")
        trial_header = (
            ROOT / "foc/include/foc_advanced_power_trial.h"
        ).read_text(encoding="utf-8")
        platform = (
            ROOT / "foc/platform/stm32g431/foc_platform_stm32g431.c"
        ).read_text(encoding="utf-8")
        self.assertEqual(capture.COMMAND_TOKEN, "P55-H3-REV")
        self.assertEqual(capture.COMMAND_DIRECTION, -1)
        self.assertEqual(capture.COMMAND_TARGET_SPEED_RPM, -582.0)
        self.assertIn("P55-H3-REV", source)
        self.assertNotIn('strcmp(argv[1], "P55-H3-QUERY")', source)
        self.assertIn(
            "(-FOC_ADVANCED_POWER_TRIAL_TARGET_SPEED_RPM)", trial_header
        )
        self.assertIn(
            "FOC_ADVANCED_POWER_TRIAL_COMMAND_SPEED_RPM", platform
        )

    def test_target_terminal_anchor_keeps_management_thread_margin(self) -> None:
        source = TARGET_MAIN.read_text(encoding="utf-8")
        self.assertIn(
            "#define FOC_H3_DYNAMIC_TERMINAL_ANCHOR_ACTIVE_TICK (960U)",
            source,
        )
        self.assertIn("last 20 ms", source)
        self.assertNotIn(
            "#define FOC_H3_DYNAMIC_TERMINAL_ANCHOR_ACTIVE_TICK (1080U)",
            source,
        )

    def test_dengfoc_truth_transport_is_atomic_and_bandwidth_bounded(self) -> None:
        source = TARGET_DENGFOC.read_text(encoding="utf-8")
        self.assertIn("kTruthSamplePeriodUs = 4000U", source)
        self.assertIn("gTruthTransportFailures", source)
        self.assertIn("truth_tx_fail=%lu", source)
        self.assertIn("Serial.write(", source)
        self.assertIn("appendUnsigned64Decimal", source)
        self.assertNotIn("truth_tick_us=%llu", source)
        truth_body = source.split("void pollTruth()", 1)[1].split(
            "\n}\n\n}  // namespace", 1
        )[0]
        self.assertNotIn('Serial.print("SYNC_TRUTH', truth_body)

    @staticmethod
    def anchor(sequence: int, control: int, truth: int):
        dynamic = (
            "FH3_DYNAMIC_ANCHOR,196608,3,1,1,0,1,100,100000,"
            f"{control},7,9,{sequence},{1000 + sequence},0,1,100,100049,49"
        )
        deng = (
            "SYNC_ANCHOR_RX,session=9,"
            f"edge_sequence={sequence},edge_tag={1000 + sequence},"
            f"truth_tick_us={truth},source=2,flags=0x00000003"
        )
        return dynamic, deng

    @staticmethod
    def ftr(
        step: int,
        angle_mrad: int,
        reliable: int,
        state: int = 3,
        control_angle_mrad: int = 1200,
        forced_angle_mrad: int = 1250,
    ) -> str:
        values = [0] * 31
        values[0] = step
        values[1] = state
        values[15] = control_angle_mrad
        values[16] = forced_angle_mrad
        values[17] = angle_mrad
        values[19] = reliable
        return "FTR," + ",".join(str(value) for value in values)

    @staticmethod
    def completion(speed: int = -592) -> str:
        return (
            "FADVP,state=4,result=1,ticks=1200/85000,first=83801,"
            "epoch=0/0,miss=0/0,features=00,status=00000000,snap=1,"
            f"ctrlstate=7,orel=1,closed=1,ogates=7f,phase=0,speed={speed},"
            "lossus=0,wcet=12000,ctrl=9900,diagmiss=0,finish=0"
        )

    def test_parse_dynamic_anchor_and_query(self) -> None:
        left, _ = self.anchor(5, 100, 1000)
        parsed = capture.parse_dynamic_anchor(left)
        self.assertEqual(parsed.edge_sequence, 5)
        query = capture.parse_ftr_query(self.ftr(120, 3142, 1))
        self.assertEqual(query["reference_control_tick"], 120)
        self.assertEqual(query["controller_state"], 3)
        self.assertAlmostEqual(query["control_electrical_angle_rad"], 1.2)
        self.assertAlmostEqual(query["forced_electrical_angle_rad"], 1.25)
        self.assertAlmostEqual(query["estimated_electrical_angle_rad"], 3.142)

    def test_powered_preflight_requires_measured_bus_voltage(self) -> None:
        safe = [
            "FOC st=0 rf=00000000 duty=0/0/0",
            "FFAULT,00000000,00000000,0,0,ARM,0,0000",
            "SYNC_STATUS,frames=0,capture_failure=0,overflow=0,"
            "truth_tx_fail=0,session_rollovers=0,sync_level=0,driver_disabled=1",
            "FADC,1951/1932/1954,1951/1933/1953,954,12298,2053",
        ]
        self.assertEqual(capture.validate_powered_preflight(safe), 12298)
        legacy = list(safe)
        legacy[2] = (
            "SYNC_STATUS,frames=0,capture_failure=0,overflow=0,"
            "truth_tx_fail=0,sync_level=0,driver_disabled=1"
        )
        with self.assertRaisesRegex(capture.DynamicCaptureError, "both targets safe"):
            capture.validate_powered_preflight(legacy)
        safe[-1] = "FADC,1951/1932/1954,1951/1933/1953,0,0,2053"
        with self.assertRaisesRegex(capture.DynamicCaptureError, "0 mV"):
            capture.validate_powered_preflight(safe)
        with self.assertRaisesRegex(capture.DynamicCaptureError, "no FADC"):
            capture.validate_powered_preflight(safe[:-1])

        with self.assertRaisesRegex(capture.DynamicCaptureError, "schema"):
            capture.parse_fadc_bus_voltage_mv("FADC,1,2")

    def test_analysis_requires_matching_anchors_and_final_safe_state(self) -> None:
        raw = []
        for index, control in enumerate((100, 200, 300), start=5):
            left, right = self.anchor(index, control, index * 1000)
            raw.extend((("g431", left), ("dengfoc", right)))
        raw.extend((
            ("g431", self.ftr(150, 100, 0)),
            ("g431", self.ftr(250, 200, 1)),
            ("dengfoc", "SYNC_TRUTH,sequence=1,truth_tick_us=5000,mechanical_angle_rad=1.0,valid=1,read_failures=0"),
            ("dengfoc", "SYNC_TRUTH,sequence=2,truth_tick_us=6000,mechanical_angle_rad=1.1,valid=1,read_failures=0"),
            ("g431", self.completion()),
            ("g431", "FH3R,3,50,1"),
            ("g431", "FOC st=0 rf=00000000 duty=0/0/0"),
        ))
        pairs, truth, queries, truth_quality = capture.analyse(raw)
        self.assertEqual((len(pairs), len(truth), len(queries)), (3, 2, 2))
        self.assertEqual([row["sequence"] for row in queries], [0, 1])
        self.assertEqual(truth_quality["rebase_sample_count"], 0)

    def test_invalid_ftr_and_missing_final_stop_fail_closed(self) -> None:
        with self.assertRaisesRegex(capture.DynamicCaptureError, "V2"):
            capture.parse_ftr_query("FTR,1,2")
        raw = []
        for index, control in enumerate((100, 200, 300), start=5):
            left, right = self.anchor(index, control, index * 1000)
            raw.extend((("g431", left), ("dengfoc", right)))
        raw.extend((
            ("g431", self.ftr(150, 100, 0)),
            ("dengfoc", "SYNC_TRUTH,sequence=1,truth_tick_us=5000,mechanical_angle_rad=1.0,valid=1,read_failures=0"),
            ("dengfoc", "SYNC_TRUTH,sequence=2,truth_tick_us=6000,mechanical_angle_rad=1.1,valid=1,read_failures=0"),
            ("g431", self.completion()),
            ("g431", "FH3R,3,50,1"),
        ))
        with self.assertRaisesRegex(capture.DynamicCaptureError, "final G431"):
            capture.analyse(raw)

        raw.insert(0, ("g431", "FOC st=0 rf=00000000 duty=0/0/0"))
        with self.assertRaisesRegex(capture.DynamicCaptureError, "final G431"):
            capture.analyse(raw)

        raw.append(("g431", "FOC st=0 rf=00000010 duty=0/0/0"))
        with self.assertRaisesRegex(
            capture.DynamicCaptureError,
            "fail-closed: FOC st=0 rf=00000010 duty=0/0/0",
        ):
            capture.analyse(raw)

    def test_artifacts_remain_not_approved(self) -> None:
        raw = []
        for index, control in enumerate((100, 200, 300), start=5):
            left, right = self.anchor(index, control, index * 1000)
            raw.extend((("g431", left), ("dengfoc", right)))
        raw.extend((
            ("g431", self.ftr(150, 100, 0)),
            ("dengfoc", "SYNC_TRUTH,sequence=1,truth_tick_us=5000,mechanical_angle_rad=1.0,valid=1,read_failures=0"),
            ("dengfoc", "SYNC_TRUTH,sequence=2,truth_tick_us=6000,mechanical_angle_rad=1.1,valid=1,read_failures=0"),
            ("g431", self.completion()),
            ("g431", "FH3R,3,50,1"),
            ("g431", "FOC st=0 rf=00000000 duty=0/0/0"),
        ))
        pairs, truth, queries, truth_quality = capture.analyse(raw)
        with tempfile.TemporaryDirectory() as directory:
            summary = capture.write_artifacts(
                Path(directory), raw, pairs, truth, queries, truth_quality
            )
        self.assertEqual(summary["status"], "captured-not-approved")
        self.assertEqual(summary["target_capability_approval"], "not-granted")
        self.assertEqual(summary["commanded_mechanical_direction"], -1)
        self.assertEqual(summary["commanded_target_speed_rpm"], -582.0)
        self.assertEqual(summary["attempt_limit"], 1)
        self.assertFalse(summary["automatic_retry"])

    def test_forward_or_unsafe_completion_is_rejected(self) -> None:
        with self.assertRaisesRegex(capture.DynamicCaptureError, "reverse speed"):
            capture.validate_reverse_completion([("g431", self.completion(592))])
        unsafe = self.completion().replace("miss=0/0", "miss=0/1")
        with self.assertRaisesRegex(capture.DynamicCaptureError, "safe envelope"):
            capture.validate_reverse_completion([("g431", unsafe)])
        with self.assertRaisesRegex(capture.DynamicCaptureError, "exactly one"):
            capture.validate_reverse_completion([])

    def test_truth_rebase_is_accounted_but_read_failure_and_long_gap_fail(self) -> None:
        truth = [
            {"truth_tick": 1000, "valid": 1, "read_failures": 0},
            {"truth_tick": 7000, "valid": 0, "read_failures": 0},
            {"truth_tick": 8000, "valid": 1, "read_failures": 0},
        ]
        valid, quality = capture.prepare_truth_stream(truth)
        self.assertEqual(len(valid), 2)
        self.assertEqual(quality["rebase_sample_count"], 1)
        self.assertEqual(quality["maximum_valid_gap_us"], 7000)

        failed = [dict(row) for row in truth]
        failed[-1]["read_failures"] = 1
        with self.assertRaisesRegex(capture.DynamicCaptureError, "I2C read failure"):
            capture.prepare_truth_stream(failed)

        long_gap = [dict(truth[0]), dict(truth[-1])]
        long_gap[-1]["truth_tick"] = 12000
        with self.assertRaisesRegex(capture.DynamicCaptureError, "exceeds"):
            capture.prepare_truth_stream(long_gap)

        valid, quality = capture.prepare_truth_stream(
            long_gap,
            maximum_allowed_gap_us=11_000,
        )
        self.assertEqual(len(valid), 2)
        self.assertEqual(quality["maximum_allowed_gap_us"], 11_000)

    def test_raw_checkpoint_survives_analysis_failure(self) -> None:
        raw = [("g431", "FH3_DYNAMIC_ANCHOR,broken"), ("dengfoc", "truth")]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            capture.write_raw_checkpoint(
                output,
                raw,
                "captured-pending-analysis",
            )
            capture.mark_checkpoint_failed(output, "schema mismatch")
            summary = __import__("json").loads(
                (output / "capture.summary.json").read_text(encoding="utf-8")
            )
            stored = (output / "raw-lines.csv").read_text(encoding="utf-8")
        self.assertEqual(summary["status"], "capture-failed")
        self.assertEqual(summary["error"], "schema mismatch")
        self.assertIn("FH3_DYNAMIC_ANCHOR,broken", stored)
        self.assertEqual(summary["target_capability_approval"], "not-granted")

    def test_raw_checkpoint_refuses_existing_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            capture.write_raw_checkpoint(output, [], "captured-pending-analysis")
            with self.assertRaises(FileExistsError):
                capture.write_raw_checkpoint(output, [], "captured-pending-analysis")


if __name__ == "__main__":
    unittest.main()
