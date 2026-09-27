#!/usr/bin/env python3
"""Host regression for the line-to-line BEMF analyzer."""

from __future__ import annotations

import csv
import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest
import importlib.util


ROOT = pathlib.Path(__file__).resolve().parents[2]
ANALYZER = ROOT / "simulation" / "analyze_bemf_measurement.py"
CAPTURE = ROOT / "simulation" / "capture_bemf_measurement.py"
SAMPLE_RATE_HZ = 12_000
SAMPLES = 256
UV_PER_COUNT = 4469
POLE_PAIRS = 7
RPM = 1000.0
FLUX_WB = 0.005529026


def load_capture_module():
    spec = importlib.util.spec_from_file_location("capture_bemf_measurement", CAPTURE)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load capture module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BemfAnalysisTests(unittest.TestCase):
    def test_capture_parses_runtime_open_loop_timing(self) -> None:
        capture = load_capture_module()
        parsed = capture.parse_runtime_timing([
            "noise",
            "C0,rust-smo-pll,1/1,0,582/800/800,2000/5000/50,900",
        ])
        self.assertEqual(parsed, (582.0, 2000, 5000))

    def test_capture_requires_explicit_motor_run_gate(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(CAPTURE), "--output", "unused.csv"],
            check=False, capture_output=True, text=True,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("without --allow-motor-run", completed.stdout)

    def test_capture_rejects_invalid_timing_before_opening_serial(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(CAPTURE), "--output", "unused.csv",
             "--allow-motor-run", "--alignment-s", "0"],
            check=False, capture_output=True, text=True,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("Refusing timing", completed.stdout)

    def test_line_to_line_rejects_common_mode_and_recovers_flux(self) -> None:
        electrical_hz = POLE_PAIRS * RPM / 60.0
        phase_peak_v = FLUX_WB * 2.0 * math.pi * electrical_hz
        phase_peak_counts = phase_peak_v * 1e6 / UV_PER_COUNT

        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = pathlib.Path(temp_dir) / "synthetic_bemf.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                fields = ["divider_mode", "sequence", "phase_u_raw", "phase_v_raw",
                          "phase_w_raw", "current_u_raw", "current_v_raw",
                          "bus_last_raw"]
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for index in range(SAMPLES):
                    t = index / SAMPLE_RATE_HZ
                    theta = 2.0 * math.pi * electrical_hz * t + 0.37
                    # Large time-varying common mode must disappear in line differences.
                    common = 2048.0 + 180.0 * math.sin(2.0 * math.pi * 31.0 * t) + 0.7 * index
                    values = [
                        common + phase_peak_counts * math.sin(theta),
                        common + phase_peak_counts * math.sin(theta - 2.0 * math.pi / 3.0),
                        common + phase_peak_counts * math.sin(theta + 2.0 * math.pi / 3.0),
                    ]
                    writer.writerow({
                        "divider_mode": "on",
                        "sequence": index,
                        "phase_u_raw": round(values[0]),
                        "phase_v_raw": round(values[1]),
                        "phase_w_raw": round(values[2]),
                        "current_u_raw": 0,
                        "current_v_raw": 0,
                        "bus_last_raw": 0,
                    })

            completed = subprocess.run(
                [sys.executable, str(ANALYZER), str(csv_path), "--rpm", str(RPM),
                 "--sc-ke", "4.964"],
                check=False, capture_output=True, text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            result = json.loads(csv_path.with_suffix(".analysis.json").read_text("utf-8"))
            self.assertEqual(result["format_version"], 5)
            self.assertEqual(result["validity"], "ok")
            self.assertEqual(result["decision"]["sc_ke_matches"], "ke-phph-rms")
            self.assertAlmostEqual(
                result["frequency_method"]["electrical_hz_median"],
                electrical_hz, delta=0.8,
            )
            self.assertAlmostEqual(
                result["flux_frequency_method"]["flux_si_wb"],
                FLUX_WB, delta=FLUX_WB * 0.015,
            )
            self.assertLess(
                result["line_to_line_amplitude"]["line_peak_spread_pct"], 1.0)


if __name__ == "__main__":
    unittest.main()
