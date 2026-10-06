import pathlib
import subprocess
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
PROBE = PROJECT_ROOT / "tests" / "profile" / "g431_product_profile_probe.cmake"


class G431ProductProfileTests(unittest.TestCase):
    def run_probe(self, profile: str, build_profile: str = "diagnostic", **features):
        values = {
            "ADVANCED": "OFF",
            "SENSORLESS": "OFF",
            "H3_DYNAMIC": "OFF",
            "AS5600_TRUTH": "OFF",
            "AS5600_ALIGNMENT": "OFF",
            "MOTION": "OFF",
            "POWER": "OFF",
            "EXTERNAL_IO": "OFF",
            "NATIVE": "OFF",
            "PWM_PULSE": "OFF",
            "ANALOG": "OFF",
            "STEP_DIR": "OFF",
        }
        values.update({name: "ON" if enabled else "OFF" for name, enabled in features.items()})
        command = [
            "cmake",
            f"-DFLUXRT_SOURCE_DIR={PROJECT_ROOT.as_posix()}",
            f"-DTEST_PRODUCT_PROFILE={profile}",
            f"-DTEST_BUILD_PROFILE={build_profile}",
        ]
        command.extend(f"-DTEST_{name}={value}" for name, value in values.items())
        command.extend(["-P", str(PROBE)])
        return subprocess.run(command, capture_output=True, text=True, check=False)

    def assert_valid(self, profile: str, build_profile: str = "diagnostic", **features):
        result = self.run_probe(profile, build_profile, **features)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"validated={profile}", result.stdout + result.stderr)

    def assert_rejected(self, profile: str, build_profile: str = "diagnostic", **features):
        result = self.run_probe(profile, build_profile, **features)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_valid_measured_envelopes(self):
        cases = (
            ("basic-drive", {}),
            ("basic-drive", {"AS5600_TRUTH": True}),
            (
                "basic-drive",
                {"AS5600_TRUTH": True, "AS5600_ALIGNMENT": True},
            ),
            ("advanced-lab", {"ADVANCED": True}),
            ("advanced-lab", {"SENSORLESS": True}),
            ("advanced-lab", {"ADVANCED": True, "H3_DYNAMIC": True}),
            ("motion-lab", {"MOTION": True}),
            ("power-lab", {"POWER": True}),
            (
                "connected-lab",
                {"EXTERNAL_IO": True, "NATIVE": True, "ANALOG": True},
            ),
        )
        for profile, features in cases:
            with self.subTest(profile=profile):
                self.assert_valid(profile, **features)

    def test_basic_drive_is_the_only_non_diagnostic_envelope(self):
        for build_profile in ("calibration", "identification", "production"):
            with self.subTest(build_profile=build_profile):
                self.assert_valid("basic-drive", build_profile)
        self.assert_rejected("advanced-lab", "production", ADVANCED=True)
        self.assert_rejected("advanced-lab", "production", SENSORLESS=True)

    def test_feature_cannot_escape_its_envelope(self):
        self.assert_rejected("basic-drive", ADVANCED=True)
        self.assert_rejected("basic-drive", SENSORLESS=True)
        self.assert_rejected("basic-drive", ADVANCED=True, H3_DYNAMIC=True)
        self.assert_rejected("advanced-lab", H3_DYNAMIC=True)
        self.assert_rejected("advanced-lab", POWER=True)
        self.assert_rejected("advanced-lab", ADVANCED=True, SENSORLESS=True)
        self.assert_rejected("connected-lab", EXTERNAL_IO=True, MOTION=True)
        self.assert_rejected("advanced-lab", AS5600_TRUTH=True)
        self.assert_rejected("basic-drive", AS5600_ALIGNMENT=True)
        self.assert_rejected("basic-drive", "production", AS5600_TRUTH=True)

    def test_protocol_and_input_require_external_io_master(self):
        self.assert_rejected("connected-lab", NATIVE=True)
        self.assert_rejected("connected-lab", ANALOG=True)

    def test_connected_lab_allows_only_one_simple_input(self):
        self.assert_rejected(
            "connected-lab",
            EXTERNAL_IO=True,
            NATIVE=True,
            ANALOG=True,
            PWM_PULSE=True,
        )

    def test_unknown_profile_fails_closed(self):
        self.assert_rejected("full-product")


if __name__ == "__main__":
    unittest.main()
