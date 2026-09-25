from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = PROJECT_ROOT / "simulation" / "capture_phase_voltage_window.py"
SPEC = importlib.util.spec_from_file_location("capture_phase_voltage_window", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
capture_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_tool)


class PhaseVoltageCaptureScriptTests(unittest.TestCase):
    def test_parser_accepts_prefixed_fpv_line(self) -> None:
        sample = capture_tool.parse_sample(
            "msh > FPV,7,101,202,303,401,402,99"
        )
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample["sequence"], 7)
        self.assertEqual(sample["phase_w_raw"], 303)

    def test_parser_rejects_wrong_width_or_non_integer(self) -> None:
        self.assertIsNone(capture_tool.parse_sample("FPV,0,1,2"))
        self.assertIsNone(capture_tool.parse_sample("FPV,0,1,2,3,4,5,nope"))

    def test_csv_contract_places_mode_before_raw_fields(self) -> None:
        self.assertEqual(capture_tool.CSV_FIELDS[0], "divider_mode")
        self.assertEqual(capture_tool.CSV_FIELDS[1:], capture_tool.RAW_FIELDS)

    def test_parser_accepts_official_nominal_model(self) -> None:
        model = capture_tool.parse_model(
            "msh > FPV_MODEL,source=st-ihm16m1-nominal,"
            "quality=nominal-not-calibrated,observer=disabled,vref_mv=3300,"
            "adc_max=4095,upper_ohm=10000,lower_ohm=2200,"
            "full_scale_mv=18300,uv_per_count=4469"
        )
        self.assertIsNotNone(model)
        assert model is not None
        self.assertEqual(model["source"], "st-ihm16m1-nominal")
        self.assertEqual(model["full_scale_mv"], 18300)
        self.assertEqual(model["observer"], "disabled")

    def test_parser_rejects_incomplete_or_non_integer_model(self) -> None:
        self.assertIsNone(capture_tool.parse_model("FPV_MODEL,source=none"))
        self.assertIsNone(
            capture_tool.parse_model(
                "FPV_MODEL,source=x,quality=x,observer=disabled,vref_mv=bad,"
                "adc_max=4095,upper_ohm=10000,lower_ohm=2200,"
                "full_scale_mv=18300,uv_per_count=4469"
            )
        )


if __name__ == "__main__":
    unittest.main()
