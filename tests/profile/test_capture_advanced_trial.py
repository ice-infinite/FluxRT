from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = PROJECT_ROOT / "simulation" / "capture_advanced_trial.py"
SPEC = importlib.util.spec_from_file_location("capture_advanced_trial", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
capture_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_tool)


class AdvancedTrialCaptureTests(unittest.TestCase):
    def test_preflight_requires_every_safe_machine_fact(self) -> None:
        lines = [
            "FSTAT,disabled,rust-smo-pll,0,0,0,0",
            "FOC f=0004231f s=0 e=0 ISR=0/12750 miss=0 pk=0(0mA)",
            "FOC st=0 rf=00000000 duty=0/0/0",
            "FFAULT,00000000,00000000,0,0,ARM,0,0000",
            "FADC,1/2/3,1/2/3,953,12288,2045",
        ]
        self.assertEqual(
            capture_tool.parse_preflight(lines),
            {"safe": True, "bus_raw": 953, "bus_mv": 12288},
        )
        for index in range(4):
            altered = lines.copy()
            altered[index] = "unsafe"
            self.assertFalse(capture_tool.parse_preflight(altered)["safe"])

    def test_preflight_rejects_missing_or_out_of_window_bus(self) -> None:
        base = [
            "FSTAT,disabled,x",
            "FOC f=0 miss=0",
            "FOC st=0 rf=00000000 duty=0/0/0",
            "FFAULT,00000000,00000000,0,0",
        ]
        self.assertFalse(capture_tool.parse_preflight(base)["safe"])
        self.assertFalse(
            capture_tool.parse_preflight(base + ["FADC,1/2/3,1/2/3,3,39,2045"])[
                "safe"
            ]
        )

    def test_normal_completion_is_parsed_without_changing_values(self) -> None:
        parsed = capture_tool.parse_completion(
            "FADVP,state=5,result=7,ticks=26/85688,first=85662,"
            "snap=1,ctrlstate=4,orel=0,closed=0,finish=0"
        )
        assert parsed is not None
        self.assertEqual(parsed["result"], "7")
        self.assertEqual(parsed["ticks"], "26/85688")
        self.assertEqual(parsed["orel"], "0")

    def test_early_start_rejection_is_also_a_completion(self) -> None:
        parsed = capture_tool.parse_completion(
            "FADVP,start=4,state=5,result=9,finish=3,snap=0,"
            "ctrlstate=0,orel=0,closed=0"
        )
        assert parsed is not None
        self.assertEqual(parsed["start"], "4")
        self.assertEqual(parsed["result"], "9")

    def test_token_prompt_or_unrelated_text_is_not_completion(self) -> None:
        self.assertIsNone(capture_tool.parse_completion("FADVP,token"))
        self.assertIsNone(capture_tool.parse_completion("msh >"))

    def test_shutdown_proof_accepts_faulted_controller_only_when_outputs_are_off(self) -> None:
        self.assertTrue(
            capture_tool.shutdown_output_safe(
                ["FOC st=0 rf=00000040 duty=0/0/0"]
            )
        )
        self.assertFalse(
            capture_tool.shutdown_output_safe(
                ["FOC st=1 rf=00000000 duty=500/500/500"]
            )
        )

    def test_trial_command_has_exactly_one_send_site(self) -> None:
        source = SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertEqual(source.count("send(port, TRIAL_COMMAND)"), 1)

    def test_cleanup_always_requests_two_stops_before_status(self) -> None:
        class FakePort:
            def __init__(self) -> None:
                self.commands: list[str] = []

            def write(self, data: bytes) -> int:
                self.commands.append(data.decode("ascii").strip())
                return len(data)

            def flush(self) -> None:
                return None

        port = FakePort()
        phases: list[str] = []

        def collect(
            unused_port: object, seconds: float, raw: list[str], phase: str
        ) -> list[str]:
            del unused_port, seconds, raw
            phases.append(phase)
            return ["status"] if phase == "cleanup-status" else []

        result = capture_tool.safe_stop_and_status(port, [], collector=collect)
        self.assertEqual(port.commands, ["foc_stop", "foc_stop", "foc_status"])
        self.assertEqual(
            phases, ["cleanup-stop-1", "cleanup-stop-2", "cleanup-status"]
        )
        self.assertEqual(result, ["status"])


if __name__ == "__main__":
    unittest.main()
