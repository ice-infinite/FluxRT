"""Regression checks for the shared Rust/MATLAB simulation contract."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUNDLE_PATH = ROOT / "simulation/contracts/legacy_speed_start.bundle.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_comparer():
    path = ROOT / "simulation/compare_contract_gate_results.py"
    spec = importlib.util.spec_from_file_location("fluxrt_contract_compare", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SimulationContractTests(unittest.TestCase):
    def test_bundle_hashes_and_identity_are_current(self) -> None:
        bundle = json.loads(BUNDLE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(bundle["units"], "SI")
        self.assertEqual(bundle["product_contract_version"], "0x00010000")
        self.assertEqual(bundle["bridge_abi_version"], "0x00150000")
        for path_key, hash_key in (
            ("schema_path", "schema_sha256"),
            ("profile_path", "profile_sha256"),
            ("scenario_path", "scenario_sha256"),
            ("trace_schema_path", "trace_schema_sha256"),
            ("comparison_path", "comparison_sha256"),
        ):
            self.assertEqual(sha256(ROOT / bundle[path_key]), bundle[hash_key])

    def test_legacy_target_is_524_rpm_in_si(self) -> None:
        scenario = json.loads(
            (ROOT / "simulation/scenarios/legacy_speed_start.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(scenario["scenario_id"], "fluxrt.legacy_speed_start.524rpm.v1")
        rpm = scenario["target_velocity_rad_s"] * 30.0 / math.pi
        self.assertAlmostEqual(rpm, 524.0, delta=1.0e-4)

    def test_strict_comparer_requires_distinct_engines_and_exact_events(self) -> None:
        comparer = load_comparer()
        shared = [
            "contract_gate_result_version=1",
            "bundle_id=test",
            "workspace_revision=abc+dirty",
            "d0=PASS",
            "d1_event_tick_tolerance=0",
            "d1_event_count=1",
            "d1_event_0=0,Disabled,Disabled,none,axis_ready,0,0",
            "d1=PASS",
        ]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rust = root / "rust.txt"
            matlab = root / "matlab.txt"
            rust.write_text(
                "\n".join([shared[0], "engine_id=fluxrt.rust.lifecycle.v1", *shared[1:]])
                + "\n",
                encoding="utf-8",
            )
            matlab.write_text(
                "\n".join(
                    [shared[0], "engine_id=fluxrt.matlab.lifecycle.v1", *shared[1:]]
                )
                + "\n",
                encoding="utf-8",
            )
            comparer.compare(rust, matlab)
            matlab.write_text(
                matlab.read_text(encoding="utf-8").replace(
                    "axis_ready,0,0", "axis_ready,1,0"
                ),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                comparer.compare(rust, matlab)


if __name__ == "__main__":
    unittest.main()
