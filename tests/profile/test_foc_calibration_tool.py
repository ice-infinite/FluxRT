from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = PROJECT_ROOT / "tools" / "foc_calibration_tool.py"
SPEC = importlib.util.spec_from_file_location("foc_calibration_tool", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
calibration_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration_tool)


def draft_record() -> dict:
    return {
        "format_version": 1,
        "record_id": "a18-test-calibration",
        "state": "draft",
        "board": {
            "board_id": "NUCLEO-G431RB+X-NUCLEO-IHM16M1",
            "serial": "TEST-BOARD",
            "hardware_revision": "test",
        },
        "firmware": {"build_profile": "calibration", "bin_sha256": "1" * 64},
        "adc": {"reference_voltage_v": 3.3, "resolution_bits": 12, "samples_per_point": 64},
        "divider": {"enable_pin": "PC9", "active_level": "low", "upper_ohm": 10000.0, "lower_ohm": 2200.0},
        "acceptance": {"maximum_abs_error_v": 0.05, "minimum_r_squared": 0.999},
        "channels": {
            "u": {"pin": "PC0", "adc_input": "ADC12_IN6", "points": [], "fit": None},
            "v": {"pin": "PC3", "adc_input": "ADC12_IN9", "points": [], "fit": None},
            "w": {"pin": "PC1", "adc_input": "ADC12_IN7", "points": [], "fit": None},
        },
        "evidence": [],
        "approval": None,
    }


class CalibrationToolTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.root_directory.cleanup)
        self.root = Path(self.root_directory.name)

    def make_approved(self) -> dict:
        record = draft_record()
        points = [
            {"divider_mode": "on", "applied_voltage_v": 0.0, "raw_mean_counts": 8.0, "raw_stddev_counts": 0.5, "samples": 64, "temperature_c": 25.0},
            {"divider_mode": "on", "applied_voltage_v": 6.0, "raw_mean_counts": 1024.0, "raw_stddev_counts": 0.7, "samples": 64, "temperature_c": 25.2},
            {"divider_mode": "on", "applied_voltage_v": 12.0, "raw_mean_counts": 2040.0, "raw_stddev_counts": 0.8, "samples": 64, "temperature_c": 25.4},
        ]
        fit = {"slope_v_per_count": 0.0059055, "intercept_v": -0.0472, "maximum_abs_error_v": 0.02, "r_squared": 0.9999}
        for channel in record["channels"].values():
            channel["points"] = copy.deepcopy(points)
            channel["fit"] = copy.deepcopy(fit)
        evidence = []
        for kind in sorted(calibration_tool.REQUIRED_EVIDENCE):
            path = Path("evidence") / f"{kind}.txt"
            absolute = self.root / path
            absolute.parent.mkdir(parents=True, exist_ok=True)
            absolute.write_text(kind, encoding="utf-8")
            evidence.append({"kind": kind, "path": path.as_posix(), "sha256": hashlib.sha256(absolute.read_bytes()).hexdigest()})
        record["state"] = "approved"
        record["evidence"] = evidence
        record["approval"] = {"reviewer": "independent-reviewer", "approved_at_utc": "2026-09-24T08:00:00Z"}
        return record

    def test_valid_draft_is_accepted_without_measurements(self) -> None:
        calibration_tool.validate_record(draft_record(), self.root)

    def test_wrong_profile_or_channel_mapping_is_rejected(self) -> None:
        wrong_profile = draft_record()
        wrong_profile["firmware"]["build_profile"] = "diagnostic"
        with self.assertRaisesRegex(calibration_tool.CalibrationError, "must be calibration"):
            calibration_tool.validate_record(wrong_profile, self.root)
        wrong_mapping = draft_record()
        wrong_mapping["channels"]["u"]["pin"] = "PC1"
        with self.assertRaisesRegex(calibration_tool.CalibrationError, "mapping does not match"):
            calibration_tool.validate_record(wrong_mapping, self.root)

    def test_approval_requires_points_fit_evidence_and_review(self) -> None:
        incomplete = draft_record()
        incomplete["state"] = "approved"
        with self.assertRaisesRegex(calibration_tool.CalibrationError, "three distinct voltage points"):
            calibration_tool.validate_record(incomplete, self.root)

    def test_tampered_evidence_is_rejected(self) -> None:
        approved = self.make_approved()
        first = self.root / approved["evidence"][0]["path"]
        first.write_text("tampered", encoding="utf-8")
        with self.assertRaisesRegex(calibration_tool.CalibrationError, "SHA-256 mismatch"):
            calibration_tool.validate_record(approved, self.root)

    def test_complete_approved_record_is_accepted(self) -> None:
        calibration_tool.validate_record(self.make_approved(), self.root)

    def test_approval_ignores_divider_off_baseline_points(self) -> None:
        incomplete = draft_record()
        off_point = {
            "divider_mode": "off",
            "applied_voltage_v": 0.0,
            "raw_mean_counts": 12.0,
            "raw_stddev_counts": 0.4,
            "samples": 64,
            "temperature_c": 25.0,
        }
        for channel in incomplete["channels"].values():
            channel["points"] = [copy.deepcopy(off_point) for _ in range(3)]
        incomplete["state"] = "approved"
        with self.assertRaisesRegex(calibration_tool.CalibrationError, "three distinct voltage points"):
            calibration_tool.validate_record(incomplete, self.root)

    def test_csv_point_is_computed_and_hashed(self) -> None:
        record_path = self.root / "profiles" / "calibration" / "draft.json"
        record_path.parent.mkdir(parents=True)
        record_path.write_text(json.dumps(draft_record()), encoding="utf-8")
        capture_path = self.root / "evidence" / "capture.csv"
        capture_path.parent.mkdir(parents=True)
        rows = [
            "divider_mode,sequence,phase_u_raw,phase_v_raw,phase_w_raw,current_u_raw,current_v_raw,bus_last_raw"
        ]
        for index in range(64):
            rows.append(f"off,{index},{100 + index % 2},200,300,0,0,0")
        capture_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
        args = SimpleNamespace(
            record=record_path,
            project_root=self.root,
            capture_csv=capture_path,
            channel="u",
            applied_voltage_v=0.0,
            temperature_c=24.5,
        )

        record = calibration_tool.add_capture_point(args)
        point = record["channels"]["u"]["points"][0]
        self.assertEqual(point["divider_mode"], "off")
        self.assertEqual(point["samples"], 64)
        self.assertAlmostEqual(point["raw_mean_counts"], 100.5)
        self.assertAlmostEqual(point["raw_stddev_counts"], 0.5)
        self.assertEqual(record["evidence"][0]["path"], "evidence/capture.csv")

    def test_fit_uses_only_divider_on_points(self) -> None:
        points = [
            {"divider_mode": "off", "applied_voltage_v": 0.0, "raw_mean_counts": 4000.0},
            {"divider_mode": "on", "applied_voltage_v": 0.0, "raw_mean_counts": 10.0},
            {"divider_mode": "on", "applied_voltage_v": 6.0, "raw_mean_counts": 1010.0},
            {"divider_mode": "on", "applied_voltage_v": 12.0, "raw_mean_counts": 2010.0},
        ]
        fit = calibration_tool.linear_fit(points, "test")
        self.assertAlmostEqual(fit["slope_v_per_count"], 0.006)
        self.assertAlmostEqual(fit["intercept_v"], -0.06)
        self.assertAlmostEqual(fit["maximum_abs_error_v"], 0.0)
        self.assertAlmostEqual(fit["r_squared"], 1.0)


if __name__ == "__main__":
    unittest.main()
