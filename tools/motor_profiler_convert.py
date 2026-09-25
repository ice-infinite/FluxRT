#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 ST Motor Pilot 的电机参数导出转换成 FluxRT 参数候选。
Convert an ST Motor Pilot motor-parameter export into a FluxRT profile candidate.

背景 / Background
-----------------
ST Motor Pilot 的 Profiler 会在实物电机上辨识 `Rs` / `Ls` / `Ke` / 惯量 / 摩擦，
并导出成 JSON。FluxRT 当前的 `Rs`/`Ls`/磁链 是**从 ST Workbench 数据库抄来的、
未经实物辨识**的值（见 `rust/crates/foc-control/src/params.rs` 的注释）。本工具
把 Profiler 的实测值灌进 FluxRT 的候选档案，从而关闭 A26 的"参数辨识"缺口。

The Motor Pilot Profiler identifies `Rs` / `Ls` / `Ke` / inertia / friction on the
physical motor and exports JSON. FluxRT's current `Rs`/`Ls`/flux come from the ST
Workbench database and are NOT identified (see `params.rs`). This tool feeds the
profiler's measured values into a FluxRT candidate, closing A26's identification
gap.

输入格式 / Input format
-----------------------
字段名取自官方 `GUI/profiler.qml` 的导出语句（`jsonExportData`），不是猜的：

- ``polePairs``          极对数 / pole pairs
- ``nominalDCVoltage``   母线电压 `[V]` / DC bus voltage
- ``nominalCurrent``     电流 `[A]` / current
- ``maxRatedSpeed``      实测最高转速 `[rpm]` / measured max speed
- ``rs``                 定子电阻 `[ohm]`
- ``ls``                 定子电感 `[H]`（导出时保留 5 位小数）
- ``BEmfConstant``       反电势常数 `[Vrms/kRPM 线电压]`
- ``inertia``            转动惯量 `[kg*m^2]`（导出时乘 1e6）
- ``friction``           粘滞摩擦 `[N*m*s]`（导出时乘 1e6）
- ``magneticStructure``  ``{"type": "SM-PMSM"}`` 或带 ``ld_lq_ratio`` 的 I-PMSM

**必须自己确认的一件事**：``ls`` 的口径。Profiler 只测一个 `Ld`（源码注释：
"Q-axis inductance is not seperately measured"）。表贴式电机 `Ld = Lq`，所以
FluxRT 的 `stator_inductance_h` 直接取 `ls` 合理；换内嵌式电机则不够。

One thing you must confirm yourself: the `ls` convention. The profiler measures a
single `Ld`; for a surface PMSM that equals `Lq`, which is what FluxRT stores.

磁链换算与一个已发现的严重风险 / Flux conversion and a serious finding
----------------------------------------------------------------------
`Ke` 的官方单位是**线电压 RMS**（`pmsm_motor_parameters.h` 注释
"Volts RMS ph-ph /kRPM"），换算成相磁链要用
``flux = Ke/sqrt(3)*sqrt(2) / (1000*2*pi/60)``。这个式子也被 MCSDK 自己的
``MAX_BEMF_VOLTAGE = RPM*1.2*Ke*sqrt(2)/(1000*sqrt(3))`` 印证。

但**工程现存的磁链值比这个物理口径小约 7 倍**：`params.rs` 写的是
``0.034739897 / (2*pi) = 0.005529026 Wb``，而 `Ke = 5.0` 按官方单位应得
``0.038985 Wb``。观测器的反电势幅值与滑模增益都按磁链标度，差 7 倍足以让
无感观测器整体失准——这与工程记录里"观测器无法收敛"的现象方向一致。

**必须用一次实物反电势测量判定**：在已知转速下测开路相电压，即可同时确定
`Ke` 与磁链。本工具因此**要求显式指定 ``--flux-convention``**，并把两种口径
与基线值的倍数差全部打印出来，绝不静默选一个。

The stored flux is roughly 7x smaller than the physical convention implies. A single
open-circuit back-EMF measurement at a known speed settles both `Ke` and the flux.
The tool therefore requires an explicit ``--flux-convention`` and prints both
conventions plus the factor against the baseline; it never silently picks one.

用法 / Usage
------------
    python tools/motor_profiler_convert.py convert export.json \\
        --baseline profiles/candidates/rev1-gbm2804h-unapproved.json \\
        --flux-convention ke-phph-rms \\
        --output profiles/candidates/rev2-gbm2804h-profiled.json

工具只写候选 JSON，**不**批准、**不**改固件、**不**生成集成代码。生成 `.inc` 请
继续用 `tools/foc_profile_tool.py generate`。
The tool only writes a candidate JSON: it never approves, never touches firmware and
never emits integration code.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# 常量 / Constants
# ---------------------------------------------------------------------------

#: 官方 `pmsm_motor_parameters.h` 里 `MOTOR_VOLTAGE_CONSTANT` 的单位是
#: "Volts RMS ph-ph /kRPM"，即**线电压 RMS**。换算到相磁链要：线→相除以 √3、
#: RMS→峰值乘 √2、再除以机械角速度（`rpm*2*pi/60`，即每 kRPM 是 `1000*2*pi/60`）。
#: The official unit is line-to-line RMS volts per kRPM. Reaching a phase flux
#: linkage means dividing by sqrt(3) (line to phase), multiplying by sqrt(2)
#: (RMS to peak) and dividing by the mechanical angular rate.
_SQRT3 = math.sqrt(3.0)
_SQRT2 = math.sqrt(2.0)


def ke_to_flux_phase_peak(ke_vrms_per_krpm: float) -> float:
    """[推荐] `Ke` 为**线电压 RMS** 时，换算成相磁链 `[Wb]`。
    [Preferred] Phase flux linkage `[Wb]` from a line-to-line RMS `Ke`.

    推导 / Derivation::

        V_phase_peak = Ke / sqrt(3) * sqrt(2)
        omega_mech   = rpm * 2*pi / 60
        flux         = V_phase_peak / omega_mech

    化简后对每 kRPM：``flux = Ke * sqrt(2) / (sqrt(3) * 1000 * 2*pi/60)``。
    Simplified per kRPM: ``flux = Ke * sqrt(2) / (sqrt(3) * 1000 * 2*pi/60)``。

    这个口径直接来自 `MOTOR_VOLTAGE_CONSTANT` 的单位声明，并已被 MCSDK 自己的
    ``MAX_BEMF_VOLTAGE = RPM * 1.2 * Ke * sqrt(2) / (1000 * sqrt(3))`` 印证。
    This follows directly from the declared unit and is corroborated by MCSDK's own
    ``MAX_BEMF_VOLTAGE`` expression.
    """
    return (ke_vrms_per_krpm / _SQRT3 * _SQRT2) / (1000.0 * 2.0 * math.pi / 60.0)


def ke_to_flux_phase_rms(ke_vrms_per_krpm: float) -> float:
    """`Ke` 被当作**相电压 RMS**（而非官方声明的线电压）时的换算。
    Alternative: treat `Ke` as a phase RMS voltage instead of the declared
    line-to-line value.

    相 RMS 要乘 √2 才是相峰值，再除以机械角速度。与
    :func:`ke_to_flux_phase_peak` 相差 √3 倍（线电压 vs 相电压）。
    A phase RMS value needs a sqrt(2) to reach a phase peak before dividing by the
    mechanical rate. The result differs from :func:`ke_to_flux_phase_peak` by
    sqrt(3), which is the line-versus-phase question.
    """
    return (ke_vrms_per_krpm * _SQRT2) / (1000.0 * 2.0 * math.pi / 60.0)


#: 可用口径 / available conventions。
FLUX_CONVENTIONS = {
    "ke-phph-rms": ke_to_flux_phase_peak,
    "ke-phase-rms": ke_to_flux_phase_rms,
}

#: 工程现存磁链值与其出处，用于一致性检查。
#: The flux value currently in the firmware and where it comes from.
EXISTING_FLUX_WB = 0.005529026
EXISTING_FLUX_SOURCE = "rust/crates/foc-control/src/params.rs (0.034739897 / 2*pi)"

#: 相对物理口径偏差超过这个倍数就要报警。 / Warn past this factor off the physics.
FLUX_FACTOR_WARN = 1.5


class ConversionError(RuntimeError):
    """输入不合法或缺少必要判定。 / Invalid input or a missing decision."""


# ---------------------------------------------------------------------------
# 输入解析 / Input parsing
# ---------------------------------------------------------------------------

#: Profiler 导出 JSON 里必须存在的字段。 / Required fields in the profiler export.
REQUIRED_EXPORT_FIELDS = (
    "polePairs", "nominalDCVoltage", "nominalCurrent", "maxRatedSpeed",
    "rs", "ls", "BEmfConstant",
)

#: 可选字段，缺失只记录不报错。 / Optional fields; absence is only recorded.
OPTIONAL_EXPORT_FIELDS = ("id", "label", "description", "inertia", "friction",
                          "magneticStructure")


def _finite(value: object, name: str) -> float:
    """要求是有限的数。 / Require a finite number."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConversionError(f"{name} 必须是数字 / must be a number: {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise ConversionError(f"{name} 必须是有限值 / must be finite: {value!r}")
    return result


def parse_profiler_export(payload: dict) -> dict:
    """校验并归一化 Motor Pilot 的导出。
    Validate and normalise the Motor Pilot export.

    缺字段直接报错而不是补默认值：参数辨识结果缺一项就整份不可用，补默认值只会
    把错误悄悄带进观测器。
    Missing fields raise instead of defaulting: a partial identification is unusable
    and a default would silently reach the observer.
    """
    if not isinstance(payload, dict):
        raise ConversionError("导出必须是 JSON 对象 / export must be a JSON object")

    missing = [f for f in REQUIRED_EXPORT_FIELDS if f not in payload]
    if missing:
        raise ConversionError(
            "导出缺少字段 / export is missing fields: " + ", ".join(missing)
        )

    structure = payload.get("magneticStructure") or {}
    if not isinstance(structure, dict):
        raise ConversionError("magneticStructure 必须是对象 / must be an object")
    structure_type = structure.get("type", "SM-PMSM")

    result = {
        "pole_pairs": _finite(payload["polePairs"], "polePairs"),
        "nominal_bus_voltage_v": _finite(payload["nominalDCVoltage"], "nominalDCVoltage"),
        "rated_current_a": _finite(payload["nominalCurrent"], "nominalCurrent"),
        "max_speed_rpm": _finite(payload["maxRatedSpeed"], "maxRatedSpeed"),
        "stator_resistance_ohm": _finite(payload["rs"], "rs"),
        "stator_inductance_h": _finite(payload["ls"], "ls"),
        "ke_vrms_per_krpm": _finite(payload["BEmfConstant"], "BEmfConstant"),
        "magnetic_structure": structure_type,
        "ld_lq_ratio": (None if structure.get("ld_lq_ratio") is None
                        else _finite(structure["ld_lq_ratio"], "ld_lq_ratio")),
        "source_name": payload.get("label") or payload.get("id") or "unnamed",
    }
    for key in ("inertia", "friction"):
        result[key] = None if payload.get(key) is None else _finite(payload[key], key)

    # 官方导出语句把惯量与摩擦各乘了 1e6（`profiler.qml`：`value*1000000`），
    # 所以这里除回去才是 SI 单位。
    # The official export multiplies inertia and friction by 1e6, so divide back to
    # reach SI units.
    if result["inertia"] is not None:
        result["inertia_kg_m2"] = result["inertia"] / 1e6
    else:
        result["inertia_kg_m2"] = None
    if result["friction"] is not None:
        result["friction_nm_s"] = result["friction"] / 1e6
    else:
        result["friction_nm_s"] = None

    # 极对数必须是正整数：0 或小数会让电角度换算失效。
    # Pole pairs must be a positive integer or the electrical-angle conversion breaks.
    if result["pole_pairs"] < 1 or result["pole_pairs"] != int(result["pole_pairs"]):
        raise ConversionError(
            f"polePairs 必须是 >=1 的整数 / must be a positive integer: "
            f"{result['pole_pairs']}"
        )
    result["pole_pairs"] = int(result["pole_pairs"])

    # 物理量必须为正：负电阻/电感/Ke 一定是辨识失败或接反。
    # Physical quantities must be positive; a negative one means a failed run.
    for key, label in (("stator_resistance_ohm", "rs"),
                       ("stator_inductance_h", "ls"),
                       ("ke_vrms_per_krpm", "BEmfConstant"),
                       ("max_speed_rpm", "maxRatedSpeed"),
                       ("rated_current_a", "nominalCurrent")):
        if result[key] <= 0.0:
            raise ConversionError(f"{label} 必须为正 / must be positive: {result[key]}")
    return result


# ---------------------------------------------------------------------------
# 候选合成 / Candidate assembly
# ---------------------------------------------------------------------------

#: 本工具允许改写的字段——只有电机本体参数。
#: 故意不含整定量：辨识改变的是被控对象，控制器增益必须另行重新整定。
#: Fields this tool may overwrite: motor parameters only. Tuning is deliberately
#: excluded, because identifying the plant means the controller gains must be
#: re-tuned separately.
MOTOR_FIELDS = (
    "pole_pairs",
    "stator_resistance_ohm",
    "stator_inductance_h",
    "flux_linkage_wb",
    "rated_current_a",
    "max_speed_rpm",
    "nominal_bus_voltage_v",
)

#: 磁链变化超过这个比例就要显式警告：观测器反电势幅值直接跟它走。
#: Warn above this relative change in flux; the observer's BEMF amplitude scales
#: with it directly.
FLUX_WARN_RATIO = 0.15


def build_candidate(measured: dict, baseline: dict, convention: str,
                    revision: int, name: str) -> tuple:
    """用实测值覆盖基线的电机字段，返回 ``(候选, 报告)``。
    Overwrite the baseline's motor fields with measured values; return
    ``(candidate, report)``.
    """
    if convention not in FLUX_CONVENTIONS:
        raise ConversionError(
            f"未知的磁链口径 / unknown flux convention: {convention!r}；"
            f"可选 / choices: {', '.join(sorted(FLUX_CONVENTIONS))}"
        )
    if not isinstance(baseline.get("runtime_config"), dict):
        raise ConversionError("基线缺少 runtime_config / baseline lacks runtime_config")

    candidate = json.loads(json.dumps(baseline))  # 深拷贝 / deep copy
    runtime = candidate["runtime_config"]

    flux = FLUX_CONVENTIONS[convention](measured["ke_vrms_per_krpm"])
    flux_other_key = ("ke-phase-rms" if convention == "ke-phph-rms" else "ke-phph-rms")
    flux_other = FLUX_CONVENTIONS[flux_other_key](measured["ke_vrms_per_krpm"])

    previous = {field: runtime.get(field) for field in MOTOR_FIELDS}
    runtime.update({
        "pole_pairs": measured["pole_pairs"],
        "stator_resistance_ohm": measured["stator_resistance_ohm"],
        "stator_inductance_h": measured["stator_inductance_h"],
        "flux_linkage_wb": flux,
        "rated_current_a": measured["rated_current_a"],
        "max_speed_rpm": measured["max_speed_rpm"],
        "nominal_bus_voltage_v": measured["nominal_bus_voltage_v"],
    })

    # 候选必须仍然未批准：本工具不产出审批结论。
    # The candidate stays unapproved; this tool never approves.
    for scope in ("parameters", "closed_loop"):
        scope_obj = candidate.get("approval", {}).get(scope)
        if isinstance(scope_obj, dict):
            scope_obj["approved"] = False
            scope_obj["approved_runtime_config_crc32"] = None

    candidate["name"] = name
    candidate["profile"]["revision"] = revision

    # CRC 由 foc_profile_tool.py 重算，这里清空以免留下过期值被误当真值。
    # CRCs are recomputed by foc_profile_tool.py; clear them so a stale value can
    # never be mistaken for a verified one.
    if isinstance(candidate.get("expected"), dict):
        candidate["expected"].pop("runtime_config_crc32", None)
        candidate["expected"].pop("record_crc32", None)

    report = {
        "convention": convention,
        "convention_other": flux_other_key,
        "flux_wb": flux,
        "flux_wb_other_convention": flux_other,
        "changed": {},
        "warnings": [],
        "measured": measured,
    }
    for field in MOTOR_FIELDS:
        old = previous.get(field)
        new = runtime.get(field)
        if old != new:
            entry = {"from": old, "to": new}
            if isinstance(old, (int, float)) and old not in (0, None):
                entry["relative_change"] = (new - old) / abs(old)
            report["changed"][field] = entry

    # 基线磁链与物理口径的一致性检查：这是本次发现的关键风险。
    # Consistency check between the baseline's flux and the physical convention.
    baseline_flux = previous.get("flux_linkage_wb")
    if isinstance(baseline_flux, (int, float)) and baseline_flux > 0:
        factor = flux / baseline_flux
        report["baseline_flux_factor"] = factor
        if factor > FLUX_FACTOR_WARN or factor < 1.0 / FLUX_FACTOR_WARN:
            report["warnings"].append(
                f"基线磁链与物理口径差 {factor:.2f} 倍：基线 {baseline_flux:.9f} Wb "
                f"（出处 {EXISTING_FLUX_SOURCE}），而本次辨识的 Ke="
                f"{measured['ke_vrms_per_krpm']:.4f} Vrms/kRPM(ph-ph) 按官方单位应为 "
                f"{flux:.9f} Wb。观测器的反电势幅值与其滑模增益都按磁链标度，"
                f"差 {factor:.2f} 倍会让无感观测器整体失准。"
                f" / baseline flux is {factor:.2f}x off the physical convention; the"
                f" observer's BEMF amplitude and sliding gain both scale with flux"
            )

    rs_change = report["changed"].get("stator_resistance_ohm", {}).get("relative_change")
    if rs_change is not None and abs(rs_change) > 0.15:
        report["warnings"].append(
            f"Rs 变化 {rs_change * 100:+.1f}%：观测器的 Rs*i 前馈与电流环增益都建立"
            f"在旧值上，需要重新整定。 / Rs changed {rs_change * 100:+.1f}%; both the"
            f" Rs*i feed-forward and the current-loop gains were built on the old value"
        )
    ls_change = report["changed"].get("stator_inductance_h", {}).get("relative_change")
    if ls_change is not None and abs(ls_change) > 0.15:
        report["warnings"].append(
            f"Ls 变化 {ls_change * 100:+.1f}%：电流环增益按 Ls 标度，需要重新整定。"
            f" / Ls changed {ls_change * 100:+.1f}%; the current-loop gains scale with Ls"
        )
    report["warnings"].append(
        f"磁链口径存在歧义：本结果用 {convention}（{flux:.9f} Wb），另一种口径给 "
        f"{flux_other:.9f} Wb。两者差 "
        f"{max(flux, flux_other) / min(flux, flux_other):.3f} 倍，起因是 `Ke` 的单位"
        f"究竟按官方的线电压 RMS 还是相电压 RMS。必须用一次实物反电势测量判定。"
        f" / Ke unit ambiguity; one physical back-EMF measurement must decide"
    )
    if measured["magnetic_structure"] != "SM-PMSM":
        report["warnings"].append(
            f"电机结构为 {measured['magnetic_structure']}：Profiler 不单独测 Lq，"
            f"内嵌式电机需要额外的凸极测量。 / Lq is not separately measured"
        )
    return candidate, report


def format_report(report: dict) -> str:
    """把报告渲染成可读文本。 / Render the report as readable text."""
    lines = [
        "=== Motor Pilot → FluxRT 参数转换 / conversion report ===",
        f"来源 / source: {report['measured']['source_name']}",
        "",
        "--- 实测值 / measured ---",
    ]
    m = report["measured"]
    lines += [
        f"  极对数 pole pairs      : {m['pole_pairs']}",
        f"  Rs                     : {m['stator_resistance_ohm']:.6f} ohm",
        f"  Ls (Ld)                : {m['stator_inductance_h'] * 1e3:.6f} mH",
        f"  Ke                     : {m['ke_vrms_per_krpm']:.6f} Vrms/kRPM(ph-ph)",
        f"  最高转速 max speed     : {m['max_speed_rpm']:.1f} rpm",
        f"  额定电流 rated current : {m['rated_current_a']:.3f} A",
        f"  母线 bus               : {m['nominal_bus_voltage_v']:.3f} V",
        f"  结构 structure         : {m['magnetic_structure']}",
    ]
    if m.get("inertia_kg_m2") is not None:
        lines.append(f"  惯量 inertia           : {m['inertia_kg_m2']:.6g} kg*m^2"
                     f"  (导出原值 {m['inertia']:.6g} /1e6)")
    if m.get("friction_nm_s") is not None:
        lines.append(f"  摩擦 friction          : {m['friction_nm_s']:.6g} N*m*s"
                     f"  (导出原值 {m['friction']:.6g} /1e6)")
    lines += [
        "",
        f"--- 磁链换算 / flux ({report['convention']}) ---",
        f"  flux_linkage_wb        : {report['flux_wb']:.9f}",
        f"  另一口径 / other       : {report['flux_wb_other_convention']:.9f}"
        f"  ({report['convention_other']})",
    ]
    factor = report.get("baseline_flux_factor")
    if factor is not None:
        lines.append(
            f"  相对基线磁链 / vs baseline: {factor:.3f}x"
            f"  ({EXISTING_FLUX_WB:.9f} Wb in {EXISTING_FLUX_SOURCE})"
        )
    lines += [
        "",
        "--- 相对基线的改动 / changes vs baseline ---",
    ]
    if not report["changed"]:
        lines.append("  (无 / none)")
    for field, entry in sorted(report["changed"].items()):
        rel = entry.get("relative_change")
        rel_txt = f"  ({rel * 100:+.2f}%)" if rel is not None else ""
        lines.append(f"  {field:26s} {entry['from']} -> {entry['to']}{rel_txt}")
    lines += ["", "--- 警告 / warnings ---"]
    lines += [f"  ! {w}" for w in report["warnings"]]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _cmd_convert(args: argparse.Namespace) -> int:
    try:
        export = json.loads(args.export.read_text(encoding="utf-8"))
        baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
        measured = parse_profiler_export(export)
        candidate, report = build_candidate(
            measured, baseline, args.flux_convention, args.revision, args.name
        )
    except (OSError, json.JSONDecodeError, ConversionError) as exc:
        print(f"转换失败 / conversion failed: {exc}", file=sys.stderr)
        return 2

    print(format_report(report))
    if args.output is None:
        print("\n(未指定 --output，只做检查 / no --output given, dry run only)")
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(candidate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"\n已写出候选 / candidate written: {args.output}")
    print("注意：expected 里的 CRC 已清空，请用 "
          "`tools/foc_profile_tool.py verify` 重算后再使用。")
    print("Note: the expected CRCs were cleared; recompute them with "
          "`tools/foc_profile_tool.py verify` before use.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。 / Build the command-line parser."""
    parser = argparse.ArgumentParser(
        description="ST Motor Pilot 参数导出 → FluxRT 候选转换 / "
                    "convert an ST Motor Pilot export into a FluxRT candidate"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    convert = sub.add_parser("convert", help="转换并写出候选 / convert and write a candidate")
    convert.add_argument("export", type=Path, help="Motor Pilot 导出的 JSON")
    convert.add_argument("--baseline", type=Path, required=True,
                         help="基线候选 JSON（提供整定值）")
    convert.add_argument("--flux-convention", required=True,
                         choices=sorted(FLUX_CONVENTIONS),
                         help="磁链换算口径，必须显式指定 / must be given explicitly")
    convert.add_argument("--revision", type=int, default=2,
                         help="新候选的 profile revision，默认 2")
    convert.add_argument("--name", default="Motor Pilot profiled candidate",
                         help="候选名称 / candidate name")
    convert.add_argument("--output", type=Path, default=None,
                         help="输出路径；省略则只打印报告 / omit for a dry run")
    convert.set_defaults(func=_cmd_convert)
    return parser


def main(argv: Optional[list] = None) -> int:
    """入口。 / Entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
