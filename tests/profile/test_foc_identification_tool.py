from __future__ import annotations

import copy
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = PROJECT_ROOT / "tools" / "foc_identification_tool.py"
SPEC = importlib.util.spec_from_file_location("foc_identification_tool", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
identification_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(identification_tool)


CSV_FIELDS = [
    "time_s", "step", "state", "phase_current_a", "phase_current_b",
    "phase_current_c", "id_ref_a", "iq_ref_a", "id_a", "iq_a",
    "vd_v", "vq_v", "dc_bus_voltage_v", "flags", "voltage_limited",
]


class FocIdentificationToolTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "evidence").mkdir()
        self.firmware = self.root / "evidence" / "firmware.bin"
        self.trace = self.root / "evidence" / "trace.csv"
        self.log = self.root / "evidence" / "trace.log"
        self.session_path = self.root / "session.json"
        self.firmware.write_bytes(b"diagnostic-firmware")
        self.log.write_text(
            "FOC trace stopped; dropped=0.\n"
            "FOC flags=0 steps=100 errors=0 ISRmax=1/2 cycles misses=0.\n"
            "FOC last status=0 rust_fault=0x00000000 duty=500/500/500 per-mille.\n",
            encoding="utf-8",
        )
        self.rows = []
        for index in range(15):
            iq = 0.60 + index * 0.005
            self.rows.append(
                {
                    "time_s": 0.70 + index * 0.02,
                    "step": 8400 + index * 240,
                    "state": 3,
                    "phase_current_a": iq,
                    "phase_current_b": -iq / 2,
                    "phase_current_c": -iq / 2,
                    "id_ref_a": 0.0,
                    "iq_ref_a": iq,
                    "id_a": 0.0,
                    "iq_a": iq,
                    "vd_v": 0.0,
                    "vq_v": iq * 5.0,
                    "dc_bus_voltage_v": 12.3,
                    "flags": 0x0000731E,
                    "voltage_limited": 0,
                }
            )
        self.write_trace(self.rows)
        self.session = self.make_session()
        self.write_session(self.session)

    @staticmethod
    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest().upper()

    def write_trace(self, rows: list[dict]) -> None:
        with self.trace.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def make_session(self) -> dict:
        return {
            "format_version": 1,
            "session_id": "test-screening-session",
            "created_at_utc": "2026-09-24T07:30:00Z",
            "purpose": "screening-only",
            "target": {
                "board_id": "0x43116001",
                "board_name": "NUCLEO-G431RB + X-NUCLEO-IHM16M1",
                "motor_id": "0x28041007",
                "motor_name": "GBM2804H-100T",
                "pole_pairs": 7,
                "profile_revision": 1,
                "runtime_config_crc32": "0x8C7A8DF4",
                "firmware_profile": "Diagnostic",
                "firmware_image_path": "evidence/firmware.bin",
                "firmware_sha256": self.digest(self.firmware),
            },
            "conditions": {
                "supply_voltage_nominal_v": 12.3,
                "supply_current_limit_a": 2.0,
                "load": "no-load-free-spin",
                "software_current_trip_a": 1.15,
                "closed_loop_enabled": False,
                "ambient_temperature_c": None,
                "winding_temperature_c": None,
            },
            "acquisition": {
                "trace_csv_path": "evidence/trace.csv",
                "trace_csv_sha256": self.digest(self.trace),
                "raw_log_path": "evidence/trace.log",
                "raw_log_sha256": self.digest(self.log),
                "control_frequency_hz": 12000,
                "trace_rate_hz": 50,
                "duration_s": 5.0,
                "target_speed_rpm": 582.0,
            },
            "capabilities": {
                "actual_phase_voltage_measured": False,
                "independent_speed_truth": False,
                "high_rate_current_capture": False,
                "winding_temperature_measured": False,
            },
            "safety": {
                "allow_motor_run_confirmed": True,
                "max_allowed_phase_current_a": 1.15,
                "minimum_bus_voltage_v": 7.0,
                "maximum_bus_voltage_v": 16.0,
                "emergency_stop_command": "foc_stop",
            },
        }

    def write_session(self, session: dict) -> None:
        self.session_path.write_text(json.dumps(session), encoding="utf-8")

    def refresh_trace_hash(self, session: dict) -> None:
        session["acquisition"]["trace_csv_sha256"] = self.digest(self.trace)

    def test_valid_run_is_screening_only_and_estimates_resistance_trend(self) -> None:
        report = identification_tool.analyse_session(self.session_path, self.root)
        estimate = report["estimates"]["stator_resistance_screening_ohm"]
        self.assertEqual(report["result"], "insufficient-for-revision2")
        self.assertFalse(report["candidate_generation_allowed"])
        self.assertTrue(report["target"]["firmware_image_archived"])
        self.assertAlmostEqual(estimate["value"], 5.0, places=6)
        self.assertFalse(estimate["eligible_for_profile"])

    def test_unarchived_firmware_keeps_digest_and_adds_blocker(self) -> None:
        expected_digest = self.session["target"]["firmware_sha256"]
        self.session["target"]["firmware_image_path"] = None
        self.write_session(self.session)

        report = identification_tool.analyse_session(self.session_path, self.root)

        self.assertEqual(report["target"]["firmware_sha256"], expected_digest)
        self.assertFalse(report["target"]["firmware_image_archived"])
        self.assertIn(
            "firmware image is not archived; recorded SHA-256 cannot be re-verified",
            report["blocking_reasons"],
        )

    def test_unarchived_firmware_still_requires_valid_digest(self) -> None:
        self.session["target"]["firmware_image_path"] = None
        self.session["target"]["firmware_sha256"] = "not-a-digest"
        self.write_session(self.session)

        with self.assertRaisesRegex(
            identification_tool.IdentificationError, "firmware_sha256"
        ):
            identification_tool.analyse_session(self.session_path, self.root)

    def test_tampered_trace_hash_is_rejected(self) -> None:
        self.trace.write_text("tampered", encoding="utf-8")
        with self.assertRaisesRegex(identification_tool.IdentificationError, "SHA-256 mismatch"):
            identification_tool.analyse_session(self.session_path, self.root)

    def test_overcurrent_trace_is_rejected(self) -> None:
        changed = copy.deepcopy(self.rows)
        changed[-1]["phase_current_a"] = 1.20
        self.write_trace(changed)
        self.refresh_trace_hash(self.session)
        self.write_session(self.session)
        with self.assertRaisesRegex(identification_tool.IdentificationError, "exceeds session limit"):
            identification_tool.analyse_session(self.session_path, self.root)

    def test_hazardous_platform_flag_is_rejected(self) -> None:
        changed = copy.deepcopy(self.rows)
        changed[-1]["flags"] |= 1 << 15
        self.write_trace(changed)
        self.refresh_trace_hash(self.session)
        self.write_session(self.session)
        with self.assertRaisesRegex(identification_tool.IdentificationError, "hazardous platform"):
            identification_tool.analyse_session(self.session_path, self.root)

    def test_closed_loop_capture_is_rejected(self) -> None:
        self.session["conditions"]["closed_loop_enabled"] = True
        self.write_session(self.session)
        with self.assertRaisesRegex(identification_tool.IdentificationError, "closed_loop_enabled=false"):
            identification_tool.analyse_session(self.session_path, self.root)

    def test_missing_run_health_is_rejected(self) -> None:
        self.log.write_text("incomplete log", encoding="utf-8")
        self.session["acquisition"]["raw_log_sha256"] = self.digest(self.log)
        self.write_session(self.session)
        with self.assertRaisesRegex(identification_tool.IdentificationError, "final dropped_samples"):
            identification_tool.analyse_session(self.session_path, self.root)


if __name__ == "__main__":
    unittest.main()
