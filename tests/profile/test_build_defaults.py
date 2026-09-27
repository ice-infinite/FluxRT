from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class BuildDefaultTests(unittest.TestCase):
    def test_powershell_defaults_every_profile_to_size_optimization(self) -> None:
        script = (PROJECT_ROOT / "build.ps1").read_text(encoding="utf-8")
        self.assertIn("$RustOptLevel = 's'", script)
        self.assertNotIn(
            "$RustOptLevel = if ($Profile -eq 'Diagnostic') { 'z' } else { 's' }",
            script,
        )

    def test_direct_cmake_default_matches_build_script(self) -> None:
        cmake = (PROJECT_ROOT / "custom.cmake").read_text(encoding="utf-8")
        self.assertIn('set(FLUXRT_DEFAULT_RUST_OPT_LEVEL "s")', cmake)
        self.assertNotIn('set(FLUXRT_DEFAULT_RUST_OPT_LEVEL "z")', cmake)


if __name__ == "__main__":
    unittest.main()
