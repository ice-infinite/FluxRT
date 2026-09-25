from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = PROJECT_ROOT / "tools" / "foc_profile_tool.py"
CANDIDATE_PATH = (
    PROJECT_ROOT / "profiles" / "candidates" / "rev1-gbm2804h-unapproved.json"
)
GENERATED_PATH = (
    PROJECT_ROOT / "profiles" / "generated" / "rev1-gbm2804h-unapproved.inc"
)

SPEC = importlib.util.spec_from_file_location("foc_profile_tool", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
profile_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(profile_tool)


class FocProfileToolTests(unittest.TestCase):
    def setUp(self) -> None:
        self.candidate = json.loads(CANDIDATE_PATH.read_text(encoding="utf-8"))

    def write_temporary_candidate(self, candidate: dict) -> Path:
        temporary = tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", encoding="utf-8", delete=False
        )
        with temporary:
            json.dump(candidate, temporary)
        self.addCleanup(Path(temporary.name).unlink, missing_ok=True)
        return Path(temporary.name)

    def test_revision1_matches_firmware_crc_baseline(self) -> None:
        _candidate, runtime_crc, record_crc, flags = profile_tool.load_and_verify(
            CANDIDATE_PATH
        )
        self.assertEqual(runtime_crc, 0x46BB0507)
        self.assertEqual(record_crc, 0x83C4CF8A)
        self.assertEqual(flags, 0)

    def test_any_runtime_edit_requires_new_expected_crc(self) -> None:
        changed = copy.deepcopy(self.candidate)
        changed["runtime_config"]["voltage_utilization"] = 0.89
        path = self.write_temporary_candidate(changed)
        with self.assertRaisesRegex(profile_tool.CandidateError, "runtime CRC mismatch"):
            profile_tool.load_and_verify(path)

    def test_closed_loop_cannot_be_enabled_without_approval(self) -> None:
        changed = copy.deepcopy(self.candidate)
        changed["runtime_config"]["closed_loop_enable"] = 1
        path = self.write_temporary_candidate(changed)
        with self.assertRaisesRegex(profile_tool.CandidateError, "must exactly match"):
            profile_tool.load_and_verify(path)

    def test_inverter_sub_gate_requires_master_gate(self) -> None:
        changed = copy.deepcopy(self.candidate)
        changed["runtime_config"]["inverter_voltage_model"][
            "observer_voltage_correction_enable"
        ] = 1
        path = self.write_temporary_candidate(changed)
        with self.assertRaisesRegex(profile_tool.CandidateError, "sub-gates require"):
            profile_tool.load_and_verify(path)

    def test_closed_loop_approval_requires_parameter_approval(self) -> None:
        changed = copy.deepcopy(self.candidate)
        changed["approval"]["closed_loop"].update(
            {
                "approved": True,
                "reviewer": "reviewer",
                "approved_at_utc": "2026-09-24T00:00:00Z",
            }
        )
        changed["runtime_config"]["closed_loop_enable"] = 1
        path = self.write_temporary_candidate(changed)
        with self.assertRaisesRegex(profile_tool.CandidateError, "requires parameter approval"):
            profile_tool.load_and_verify(path)

    def test_parameter_approval_requires_complete_hashed_evidence(self) -> None:
        changed = copy.deepcopy(self.candidate)
        changed["approval"]["parameters"].update(
            {
                "approved": True,
                "approved_runtime_config_crc32": "0x46BB0507",
                "reviewer": "reviewer",
                "approved_at_utc": "2026-09-24T00:00:00Z",
            }
        )
        path = self.write_temporary_candidate(changed)
        with self.assertRaisesRegex(profile_tool.CandidateError, "missing evidence kinds"):
            profile_tool.load_and_verify(path)

    def test_generated_candidate_snapshot_is_current(self) -> None:
        candidate, runtime_crc, record_crc, flags = profile_tool.load_and_verify(
            CANDIDATE_PATH
        )
        rendered = profile_tool.render_initializer(
            CANDIDATE_PATH, candidate, runtime_crc, record_crc, flags
        )
        self.assertEqual(GENERATED_PATH.read_text(encoding="utf-8"), rendered)


if __name__ == "__main__":
    unittest.main()
