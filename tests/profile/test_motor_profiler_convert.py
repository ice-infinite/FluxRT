#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``tools/motor_profiler_convert.py`` 的测试。
Tests for ``tools/motor_profiler_convert.py``.

覆盖重点 / What is covered:

- 磁链换算公式与官方单位声明、MCSDK `MAX_BEMF_VOLTAGE` 表达式自洽
- **工程现存磁链与物理口径的 7 倍偏差**被显式检出（这是本批次最重要的发现）
- 导出字段校验：缺字段、负数、非整数极对数都必须报错而不是补默认值
- 惯量/摩擦的 1e6 缩放被正确还原
- 只改电机字段，绝不动整定量
- 候选必须保持未批准、CRC 必须被清空
"""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import motor_profiler_convert as mpc  # noqa: E402

BASELINE = (Path(__file__).resolve().parents[2]
            / "profiles" / "candidates" / "rev1-gbm2804h-unapproved.json")


def sample_export(**overrides) -> dict:
    """构造一份最小可用的 Motor Pilot 导出。 / A minimal valid Motor Pilot export."""
    payload = {
        "id": "GBM2804H-100T",
        "label": "GBM2804H-100T",
        "hardwareFamily": "MOTOR",
        "polePairs": 7,
        "nominalDCVoltage": 12.3,
        "nominalCurrent": 0.8,
        "maxRatedSpeed": 1500,
        "rs": 5.4,
        "ls": 0.00105,
        "magneticStructure": {"type": "SM-PMSM"},
        "BEmfConstant": 4.9,
        "inertia": 29.1,
        "friction": 9.37,
    }
    payload.update(overrides)
    return payload


class FluxConversionTests(unittest.TestCase):
    """磁链换算必须与官方单位声明自洽。 / Flux conversion must match the declared unit."""

    def test_ke_unit_is_line_to_line_rms(self) -> None:
        """`Ke` 的官方单位是线电压 RMS，换算必须包含 √3（线→相）与 √2（RMS→峰值）。
        The official `Ke` unit is line-to-line RMS, so the conversion must contain
        sqrt(3) and sqrt(2).
        """
        # 1000 rpm 时 Ke 就是相电压峰值，除以机械角速度得到磁链。
        ke = 5.0
        v_phase_peak = ke / math.sqrt(3) * math.sqrt(2)
        omega_mech = 1000.0 * 2.0 * math.pi / 60.0
        expected = v_phase_peak / omega_mech
        self.assertAlmostEqual(mpc.ke_to_flux_phase_peak(ke), expected, places=12)

    def test_matches_mcsdk_max_bemf_expression(self) -> None:
        """与 MCSDK 的 ``MAX_BEMF_VOLTAGE`` 表达式一致。
        Consistent with MCSDK's own ``MAX_BEMF_VOLTAGE`` expression.

        MCSDK: ``MAX_BEMF_VOLTAGE = RPM * 1.2 * Ke * sqrt(2) / (1000 * sqrt(3))``
        这是相电压峰值。磁链 = 该值 / 机械角速度。
        """
        ke, rpm = 5.0, 1000.0
        mcsdk_peak = rpm * 1.2 * ke * math.sqrt(2) / (1000.0 * math.sqrt(3))
        omega = rpm * 2.0 * math.pi / 60.0
        # 去掉 MCSDK 表达式里的 1.2 余量因子后应等于本工具的换算。
        self.assertAlmostEqual(
            mcsdk_peak / 1.2 / omega, mpc.ke_to_flux_phase_peak(ke), places=12
        )

    def test_two_conventions_differ_by_sqrt3(self) -> None:
        """两种口径差 √3，这正是"线电压还是相电压"这个问题的量纲。
        The two conventions differ by sqrt(3), which is exactly the line-versus-phase
        question.

        `ke-phph-rms` 把 Ke 当线电压，所以得到的磁链**更小**，比值是 1/√3。
        `ke-phph-rms` reads Ke as a line-to-line voltage, so it yields the smaller
        flux and the ratio is 1/sqrt(3).
        """
        ratio = mpc.ke_to_flux_phase_peak(1.0) / mpc.ke_to_flux_phase_rms(1.0)
        self.assertAlmostEqual(ratio, 1.0 / math.sqrt(3), places=12)
        self.assertLess(mpc.ke_to_flux_phase_peak(1.0),
                        mpc.ke_to_flux_phase_rms(1.0))

    def test_conversion_scales_linearly(self) -> None:
        for factor in (0.5, 1.0, 2.0):
            self.assertAlmostEqual(
                mpc.ke_to_flux_phase_peak(5.0 * factor),
                mpc.ke_to_flux_phase_peak(5.0) * factor,
                places=12,
            )

    def test_baseline_flux_is_about_seven_times_off_physics(self) -> None:
        """**核心发现**：工程现存磁链比 `Ke` 的物理含义小约 7 倍。
        **Core finding**: the flux value already in the firmware is about 7x smaller
        than the physical meaning of `Ke`.

        `params.rs` 写的是 ``0.034739897 / (2*pi) = 0.005529026 Wb``，而
        ``MOTOR_VOLTAGE_CONSTANT = 5.0 Vrms/kRPM(ph-ph)`` 按官方单位应为
        ``0.038985 Wb``。观测器的反电势幅值与滑模增益都按磁链标度，这个偏差会让
        无感观测器整体失准。

        这条测试的作用是**把这个偏差钉住**：如果哪天有人修好了它，测试会失败并
        提醒更新常量与文档，而不是让修好的值悄悄漂回去。
        This test pins the discrepancy: if it is ever fixed, the test fails and forces
        the constants and documentation to be updated rather than drifting back.
        """
        physical = mpc.ke_to_flux_phase_peak(5.0)
        self.assertAlmostEqual(physical, 0.038984840, places=8)
        factor = physical / mpc.EXISTING_FLUX_WB
        self.assertGreater(factor, 6.5)
        self.assertLess(factor, 7.5)
        # 现存值也不是"另一种口径"，说明它不是单位口径问题而是量纲/系数问题。
        self.assertLess(mpc.EXISTING_FLUX_WB / mpc.ke_to_flux_phase_rms(5.0), 0.3)

    def test_existing_flux_reproduces_from_documented_source(self) -> None:
        """现存值必须能由它自己注释里的出处复现，确认常量没抄错。
        The existing value must be reproducible from its own documented source."""
        self.assertAlmostEqual(
            0.034739897 / (2.0 * math.pi), mpc.EXISTING_FLUX_WB, places=9
        )


class ParseTests(unittest.TestCase):
    """导出解析与校验。 / Export parsing and validation."""

    def test_parses_valid_export(self) -> None:
        m = mpc.parse_profiler_export(sample_export())
        self.assertEqual(m["pole_pairs"], 7)
        self.assertAlmostEqual(m["stator_resistance_ohm"], 5.4)
        self.assertAlmostEqual(m["ke_vrms_per_krpm"], 4.9)
        self.assertEqual(m["magnetic_structure"], "SM-PMSM")

    def test_inertia_and_friction_are_descaled(self) -> None:
        """导出把惯量/摩擦乘了 1e6，解析后必须是 SI 单位。
        The export scales inertia/friction by 1e6; parsing must return SI units."""
        m = mpc.parse_profiler_export(sample_export(inertia=29.1, friction=9.37))
        self.assertAlmostEqual(m["inertia_kg_m2"], 29.1e-6, places=12)
        self.assertAlmostEqual(m["friction_nm_s"], 9.37e-6, places=12)

    def test_missing_required_field_raises(self) -> None:
        """缺任一必需字段必须报错，不能补默认值。
        A missing required field must raise rather than be defaulted."""
        for field in mpc.REQUIRED_EXPORT_FIELDS:
            with self.subTest(missing=field):
                payload = sample_export()
                del payload[field]
                with self.assertRaises(mpc.ConversionError):
                    mpc.parse_profiler_export(payload)

    def test_negative_physical_quantity_raises(self) -> None:
        """负电阻/电感/Ke 一定是辨识失败，必须拒绝。
        A negative resistance/inductance/Ke means a failed run and must be rejected."""
        for field in ("rs", "ls", "BEmfConstant", "maxRatedSpeed", "nominalCurrent"):
            with self.subTest(field=field):
                with self.assertRaises(mpc.ConversionError):
                    mpc.parse_profiler_export(sample_export(**{field: -1.0}))

    def test_non_integer_pole_pairs_raises(self) -> None:
        for bad in (0, 7.5, -3):
            with self.subTest(pole_pairs=bad):
                with self.assertRaises(mpc.ConversionError):
                    mpc.parse_profiler_export(sample_export(polePairs=bad))

    def test_non_finite_values_raise(self) -> None:
        """nan/inf 会静默污染整条控制链，必须拒绝。
        nan/inf would silently poison the control chain."""
        for bad in (float("nan"), float("inf")):
            with self.subTest(value=bad):
                with self.assertRaises(mpc.ConversionError):
                    mpc.parse_profiler_export(sample_export(rs=bad))

    def test_string_number_raises(self) -> None:
        """字符串数字不能被悄悄接受。 / Numeric strings must not be accepted."""
        with self.assertRaises(mpc.ConversionError):
            mpc.parse_profiler_export(sample_export(rs="5.4"))

    def test_ipmsm_carries_ld_lq_ratio(self) -> None:
        m = mpc.parse_profiler_export(sample_export(
            magneticStructure={"type": "I-PMSM", "ld_lq_ratio": 0.8}
        ))
        self.assertEqual(m["magnetic_structure"], "I-PMSM")
        self.assertAlmostEqual(m["ld_lq_ratio"], 0.8)


class BuildCandidateTests(unittest.TestCase):
    """候选合成。 / Candidate assembly."""

    def setUp(self) -> None:
        self.baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        self.measured = mpc.parse_profiler_export(sample_export())

    def test_only_motor_fields_change(self) -> None:
        """整定量必须原样保留：辨识改的是被控对象，增益要另行整定。
        Tuning must be preserved byte for byte: identifying the plant does not
        re-tune the controller.
        """
        candidate, _ = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 2, "test"
        )
        before = self.baseline["runtime_config"]
        after = candidate["runtime_config"]
        for key, value in before.items():
            if key in mpc.MOTOR_FIELDS:
                continue
            with self.subTest(field=key):
                self.assertEqual(after[key], value,
                                 f"{key} 不应被改动 / must not be changed")

    def test_motor_fields_take_measured_values(self) -> None:
        candidate, _ = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 2, "test"
        )
        runtime = candidate["runtime_config"]
        self.assertEqual(runtime["pole_pairs"], 7)
        self.assertAlmostEqual(runtime["stator_resistance_ohm"], 5.4)
        self.assertAlmostEqual(runtime["stator_inductance_h"], 0.00105)
        self.assertAlmostEqual(runtime["flux_linkage_wb"],
                               mpc.ke_to_flux_phase_peak(4.9), places=12)

    def test_candidate_stays_unapproved(self) -> None:
        """本工具绝不产出审批结论。 / This tool never approves."""
        self.baseline["approval"]["parameters"]["approved"] = True
        candidate, _ = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 2, "test"
        )
        for scope in ("parameters", "closed_loop"):
            self.assertFalse(candidate["approval"][scope]["approved"])
            self.assertIsNone(
                candidate["approval"][scope]["approved_runtime_config_crc32"]
            )

    def test_stale_crcs_are_cleared(self) -> None:
        """过期 CRC 必须清空，否则会被误当成已验证值。
        Stale CRCs must be cleared or they would look like verified values."""
        candidate, _ = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 2, "test"
        )
        self.assertNotIn("runtime_config_crc32", candidate["expected"])
        self.assertNotIn("record_crc32", candidate["expected"])

    def test_baseline_is_not_mutated(self) -> None:
        """基线对象不能被就地改写。 / The baseline must not be mutated in place."""
        snapshot = json.dumps(self.baseline, sort_keys=True)
        mpc.build_candidate(self.measured, self.baseline, "ke-phph-rms", 2, "test")
        self.assertEqual(json.dumps(self.baseline, sort_keys=True), snapshot)

    def test_revision_and_name_are_applied(self) -> None:
        candidate, _ = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 7, "rev7 trial"
        )
        self.assertEqual(candidate["profile"]["revision"], 7)
        self.assertEqual(candidate["name"], "rev7 trial")

    def test_unknown_convention_raises(self) -> None:
        with self.assertRaises(mpc.ConversionError):
            mpc.build_candidate(self.measured, self.baseline, "nope", 2, "x")

    def test_baseline_without_runtime_config_raises(self) -> None:
        with self.assertRaises(mpc.ConversionError):
            mpc.build_candidate(self.measured, {}, "ke-phph-rms", 2, "x")

    def test_report_flags_flux_and_rs_changes(self) -> None:
        """报告必须同时给出磁链倍数偏差与 Rs/Ls 变化警告。
        The report must carry both the flux factor and the Rs/Ls change warnings."""
        _, report = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 2, "test"
        )
        self.assertGreater(report["baseline_flux_factor"], 6.5)
        joined = " ".join(report["warnings"])
        self.assertIn("基线磁链", joined)
        self.assertIn("歧义", joined)

    def test_report_lists_every_changed_motor_field(self) -> None:
        _, report = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 2, "test"
        )
        # 样例导出与基线在这几项上都不同。
        for field in ("flux_linkage_wb", "stator_resistance_ohm",
                      "stator_inductance_h", "max_speed_rpm",
                      "nominal_bus_voltage_v"):
            with self.subTest(field=field):
                self.assertIn(field, report["changed"])
        # 极对数两边都是 7，不应出现在改动列表里。
        self.assertNotIn("pole_pairs", report["changed"])

    def test_ipmsm_adds_saliency_warning(self) -> None:
        measured = mpc.parse_profiler_export(sample_export(
            magneticStructure={"type": "I-PMSM", "ld_lq_ratio": 0.8}
        ))
        _, report = mpc.build_candidate(measured, self.baseline, "ke-phph-rms", 2, "t")
        self.assertTrue(any("I-PMSM" in w for w in report["warnings"]))

    def test_report_is_renderable(self) -> None:
        _, report = mpc.build_candidate(
            self.measured, self.baseline, "ke-phph-rms", 2, "test"
        )
        text = mpc.format_report(report)
        self.assertIn("conversion report", text)
        self.assertIn("ke-phph-rms", text)


class WorkedExampleTests(unittest.TestCase):
    """用仓库内的样例导出把整条流水线端到端钉住。
    Pin the whole pipeline end to end using the example export in the repo.

    样例用的是 ST Workbench 数据库参考值，**不是实物辨识结果**，所以它的作用是
    证明工具链可用与量纲口径可复现，而不是提供一份可用参数。
    The example uses ST Workbench database reference values, NOT identified data, so
    it proves the toolchain works and the convention is reproducible rather than
    providing usable parameters.
    """

    EXAMPLE = (Path(__file__).resolve().parents[2]
               / "profiles" / "examples" / "motor_pilot_export_reference.json")

    def test_example_export_exists_and_parses(self) -> None:
        """样例导出必须存在、可解析、且具备全部必需字段。
        The example export must exist, parse, and carry every required field."""
        self.assertTrue(self.EXAMPLE.is_file(), f"缺少样例 / missing: {self.EXAMPLE}")
        payload = json.loads(self.EXAMPLE.read_text(encoding="utf-8"))
        for field in mpc.REQUIRED_EXPORT_FIELDS:
            with self.subTest(field=field):
                self.assertIn(field, payload)
        measured = mpc.parse_profiler_export(payload)
        self.assertEqual(measured["pole_pairs"], 7)
        self.assertEqual(measured["magnetic_structure"], "SM-PMSM")

    def test_example_reproduces_the_seven_times_flux_factor(self) -> None:
        """样例必须精确复现 7.00 倍磁链偏差——这个数字是本次调查的核心结论。
        The example must reproduce the 7.00x flux factor exactly; that number is the
        core finding of this investigation.

        参考值 `Ke = 4.964 Vrms/kRPM(ph-ph)` 按官方单位应得 `0.0387041 Wb`，而
        工程现值为 `0.005529026 Wb`，比值 `7.0002`。
        The reference `Ke` yields `0.0387041 Wb` under the official unit while the
        firmware holds `0.005529026 Wb`, a ratio of `7.0002`.
        """
        payload = json.loads(self.EXAMPLE.read_text(encoding="utf-8"))
        measured = mpc.parse_profiler_export(payload)
        baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        _candidate, report = mpc.build_candidate(
            measured, baseline, "ke-phph-rms", 2, "worked example"
        )
        # 因子随 Ke 线性变化：Ke=4.964 得 7.0002，Ke=5.0 得 7.0509。
        # The factor scales linearly with Ke: 7.0002 at Ke=4.964, 7.0509 at Ke=5.0.
        self.assertAlmostEqual(report["baseline_flux_factor"], 7.0002, places=4)
        self.assertAlmostEqual(report["flux_wb"], 0.038704149, places=9)
        # 无论用哪个 Ke，偏差都在 7 倍量级，这才是要钉住的结论。
        # Either Ke lands the discrepancy in the 7x range, which is the point.
        self.assertGreater(report["baseline_flux_factor"], 6.9)
        self.assertLess(report["baseline_flux_factor"], 7.1)

    def test_worked_example_report_warns(self) -> None:
        """样例转换必须给出磁链偏差与口径歧义两条警告。
        Converting the example must emit both the flux-factor and the convention
        ambiguity warnings."""
        payload = json.loads(self.EXAMPLE.read_text(encoding="utf-8"))
        measured = mpc.parse_profiler_export(payload)
        baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        _candidate, report = mpc.build_candidate(
            measured, baseline, "ke-phph-rms", 2, "worked example"
        )
        joined = " ".join(report["warnings"])
        self.assertIn("7.00 倍", joined)
        self.assertIn("歧义", joined)


class CliTests(unittest.TestCase):
    """命令行接口。 / Command-line interface."""

    def test_dry_run_writes_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp) / "export.json"
            export.write_text(json.dumps(sample_export()), encoding="utf-8")
            rc = mpc.main(["convert", str(export), "--baseline", str(BASELINE),
                           "--flux-convention", "ke-phph-rms"])
            self.assertEqual(rc, 0)
            self.assertEqual(list(Path(tmp).iterdir()), [export])

    def test_convert_writes_loadable_candidate(self) -> None:
        """写出的候选必须是合法 JSON，且电机字段来自实测值。
        The written candidate must be valid JSON with the measured motor fields."""
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp) / "export.json"
            out = Path(tmp) / "candidate.json"
            export.write_text(json.dumps(sample_export()), encoding="utf-8")
            rc = mpc.main(["convert", str(export), "--baseline", str(BASELINE),
                           "--flux-convention", "ke-phph-rms",
                           "--revision", "3", "--output", str(out)])
            self.assertEqual(rc, 0)
            candidate = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(candidate["profile"]["revision"], 3)
            self.assertAlmostEqual(
                candidate["runtime_config"]["stator_resistance_ohm"], 5.4
            )
            # 仍与官方 schema 的键集合一致。
            self.assertEqual(
                set(candidate), {"format_version", "name", "profile",
                                 "runtime_config", "approval", "expected"}
            )

    def test_flux_convention_is_mandatory(self) -> None:
        """漏掉 --flux-convention 必须失败：默认值会让静默选口径成为可能。
        Omitting --flux-convention must fail, so a silent choice is impossible."""
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp) / "export.json"
            export.write_text(json.dumps(sample_export()), encoding="utf-8")
            with self.assertRaises(SystemExit):
                mpc.main(["convert", str(export), "--baseline", str(BASELINE)])

    def test_bad_export_returns_nonzero(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            export = Path(tmp) / "export.json"
            export.write_text(json.dumps({"polePairs": 7}), encoding="utf-8")
            rc = mpc.main(["convert", str(export), "--baseline", str(BASELINE),
                           "--flux-convention", "ke-phph-rms"])
            self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
